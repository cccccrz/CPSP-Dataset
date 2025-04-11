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
        init_sensor("dev", config_path / "dev.yaml");

        // 独立IMU订阅
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/zed/zed_node/imu/data", 200,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                imu_buffer_.push_back(msg);
            });

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
        if (sensor.type == "camera") {
            sensor.csv_file << "#timestamp [ns],filename\n";
        } else if (sensor.type == "imu")  {
            sensor.csv_file << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1]
            ,a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
        }
        
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
        // 标记首次同步触发
        if (!first_sync_triggered_.exchange(true)) {
            // TODO check sync timestamp
            RCLCPP_INFO(this->get_logger(), "First sync， camL[%09ld], camR[%09ld], imu[%09ld]", 
                left_img->header.stamp.nanoseconds(),
                right_img->header.stamp.nanoseconds(),
                imu_img->header.stamp.nanoseconds());

            std::lock_guard<std::mutex> lock(imu_mutex_);
            // 清除首次同步前的历史数据
            auto first_valid = std::lower_bound(
                imu_buffer_.begin(), imu_buffer_.end(),
                left_img->header.stamp,
                [](const auto& imu, const auto& stamp) {
                    return imu->header.stamp < stamp;
                });
            imu_buffer_.erase(imu_buffer_.begin(), first_valid);
        }

        try {
            // save imgs
            process_camera("camL", left_img);
            process_camera("camR", right_img);
            // DEV
            if (dev_img && sensors_.count("dev")) {
                process_camera("dev", dev_img);
            }

            save_imu(imu);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
        }
    }

    void process_camera(const std::string& name, const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        auto& sensor = sensors_[name];
        // if (!should_save(name, msg->header.stamp)) return;

        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::string filename = std::to_string(msg->header.stamp.sec) + "_" + 
                              std::to_string(msg->header.stamp.nanosec) + ".png";
        
        std::lock_guard<std::mutex> lock(sensor->mtx);
        cv::imwrite((sensor->data_path / filename).string(), img);
        sensor->csv_file << msg->header.stamp.nanosec << "," << filename << "\n";
        sensor->last_save_time = msg->header.stamp;
    }

    void save_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    #if 0
        auto& sensor = sensors_["imu"];
        std::lock_guard<std::mutex> lock(sensor->mtx);
        sensor->csv_file << msg->header.stamp.nanosec << ","
                        << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
                        << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z << "\n";
        sensor->last_save_time = msg->header.stamp;
    #else
        // 获取时间窗口内的所有IMU数据
        // TODO check sync time
        const rclcpp::Time sync_time = msg->header.stamp; //left_img->header.stamp;
        std::vector<sensor_msgs::msg::Imu::ConstPtr> batch;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            auto it_start = std::lower_bound(
                imu_buffer_.begin(), imu_buffer_.end(),
                sync_time - rclcpp::Duration(0, 50'000'000), // 前50ms
                [](const auto& imu, const auto& time) {
                    return imu->header.stamp < time;
                });
            
            auto it_end = std::upper_bound(
                imu_buffer_.begin(), imu_buffer_.end(),
                sync_time + rclcpp::Duration(0, 50'000'000), // 后50ms
                [](const auto& time, const auto& imu) {
                    return time < imu->header.stamp;
                });

            if (it_start != imu_buffer_.end()) {
                batch.reserve(std::distance(it_start, it_end));
                std::move(it_start, it_end, std::back_inserter(batch));
                imu_buffer_.erase(imu_buffer_.begin(), it_end);
            }
        }

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