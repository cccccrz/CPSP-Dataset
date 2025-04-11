#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <memory>
#include <mutex>
#include <deque>

namespace fs = std::filesystem;

struct SensorConfig {
    std::string name;
    std::string type;
    std::string topic;
    double rate_hz;
    fs::path data_path;
    fs::path csv_path;
    YAML::Node yaml_config;
    std::ofstream csv_file;
    std::mutex mtx;
    rclcpp::Time last_save_time;
};

class SyncSaver : public rclcpp::Node {
public:
    SyncSaver() : Node("sync_saver"), first_sync_triggered_(false) {
        this->declare_parameter("config_path", "config");
        fs::path config_path = this->get_parameter("config_path").as_string();
        
        init_sensor("camL", config_path / "camL.yaml");
        init_sensor("camR", config_path / "camR.yaml");
        init_sensor("imu", config_path / "imu.yaml");
        init_sensor("dev", config_path / "dev.yaml"); // 预留DEV

        // 创建同步器
        setup_synchronization();
    }

private:
    void init_sensor(const std::string& name, const fs::path& config_file) {
        RCLCPP_INFO(this->get_logger(), "Loading config: %s", config_file.c_str());
        if (!fs::exists(config_file)) {
            if (name == "dev") return; // wait dev finished
            RCLCPP_ERROR(get_logger(), "Config file missing: %s", config_file.c_str());
            return;
        }

        YAML::Node config = YAML::LoadFile(config_file.string());
        SensorConfig sensor;
        sensor.name = name;
        sensor.type = config["sensor_type"].as<std::string>();
        sensor.topic = config["topic"].as<std::string>();
        sensor.rate_hz = config["rate_hz"].as<double>();
        
        // create path
        if (sensor.type == "camera") {
            sensor.data_path = fs::current_path() / "dataset" / name / "data";
            fs::create_directories(sensor.data_path);
        } else {
            sensor.data_path = fs::current_path() / "dataset" / name;
            fs::create_directories(sensor.data_path);
        }
        
        // init CSV
        sensor.csv_path = fs::current_path() / "dataset" / name / "data.csv";
        sensor.csv_file.open(sensor.csv_path);
        sensor.csv_file << "timestamp,filename\n";
        
        // all topics
        sensors_[name] = std::make_shared<SensorConfig>(sensor);
    }

    void setup_synchronization() {
        // get topics
        std::vector<message_filters::SubscriberBase*> subs;
        for (auto& [name, sensor] : sensors_) {
            if (sensor->type == "camera") {
                subs.push_back(&sensor->image_sub);
                sensor->image_sub.subscribe(this, sensor->topic);
            } else if (sensor->type == "imu") {
                subs.push_back(&sensor->imu_sub);
                sensor->imu_sub.subscribe(this, sensor->topic);
            }
        }

        // sync policy
        sync_ = std::make_shared<Sync>(
            SyncPolicy(20),
            *subs.begin(), *(subs.begin()+1),
            *(subs.begin()+2), *(subs.begin()+3));    
        sync_->setInterMessageLowerBound(0, rclcpp::Duration(0, 50000000)); // 50ms 20Hz
        sync_->registerCallback(std::bind(&SyncSaver::sync_callback, this, 
            std::placeholders::_1, std::placeholders::_2, 
            std::placeholders::_3, std::placeholders::_4));
    }

    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_img,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_img,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
        const sensor_msgs::msg::Image::ConstSharedPtr& dev_img = nullptr) 
    {
        try {
            // save camera img
            process_camera("camL", left_img);
            process_camera("camR", right_img);
            
            // save IMU data
            if (should_save("imu", imu->header.stamp)) {
                save_imu(imu);
            }

            // save DEV img
            if (dev_img && sensors_.count("dev")) {
                process_camera("dev", dev_img);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
        }
    }

    void process_camera(const std::string& name, const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        auto& sensor = sensors_[name];
        if (!should_save(name, msg->header.stamp)) return;

        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::string filename = std::to_string(msg->header.stamp.sec) + "_" + 
                              std::to_string(msg->header.stamp.nanosec) + ".png";
        
        std::lock_guard<std::mutex> lock(sensor->mtx);
        cv::imwrite((sensor->data_path / filename).string(), img);
        sensor->csv_file << msg->header.stamp.nanosec << "," << filename << "\n";
        sensor->last_save_time = msg->header.stamp;
    }

    void save_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        auto& sensor = sensors_["imu"];
        std::lock_guard<std::mutex> lock(sensor->mtx);
        sensor->csv_file << msg->header.stamp.nanosec << ","
                        << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
                        << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z << "\n";
        sensor->last_save_time = msg->header.stamp;
        // 批量保存到CSV
        if (!batch.empty()) {
            auto& sensor = sensors_["imu"];
            std::lock_guard<std::mutex> lock(sensor->mtx);
            for (const auto& imu : batch) {
                sensor->csv_file << imu->header.stamp.nanoseconds() << ","
                            << imu->angular_velocity.x << ","
                            << imu->angular_velocity.y << ","
                            << imu->angular_velocity.z << ","
                            << imu->linear_acceleration.x << ","
                            << imu->linear_acceleration.y << ","
                            << imu->linear_acceleration.z << "\n";
            }
            sensor->csv_file << "#SYNC_TS:" 
                        << msg->header.stamp.nanoseconds() << "\n";
        }
    #endif
    }

    bool should_save(const std::string& name, const builtin_interfaces::msg::Time& stamp) {
        auto& sensor = sensors_[name];
        rclcpp::Time current_time(stamp);
        rclcpp::Duration interval(1.0 / sensor->rate_hz);
        return (current_time - sensor->last_save_time) >= interval;
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,  // camL
        sensor_msgs::msg::Image,  // camR
        sensor_msgs::msg::Imu,    // IMU
        sensor_msgs::msg::Image>; // DEV 
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    std::map<std::string, std::shared_ptr<SensorConfig>> sensors_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::deque<sensor_msgs::msg::Imu::ConstPtr> imu_buffer_;
    std::mutex imu_mutex_;
    std::atomic<bool> first_sync_triggered_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}