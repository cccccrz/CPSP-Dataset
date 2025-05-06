#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <memory>
#include <mutex>
#include <deque>
#include "sync_saver/config_parser.hpp"

#define NAME_CAM_ZED_L "camL"
#define NAME_CAM_ZED_R "camR"
#define NAME_CAM_DEV "camDev"
#define NAME_IMU "imu"

namespace fs = std::filesystem;
using namespace std::placeholders;

struct SensorConfig {
    std::string name;
    std::string type;
    std::string topic;
    double rate_hz;
    fs::path data_path;
    fs::path csv_path;
    std::ofstream csv_file;
    std::mutex mtx;
    rclcpp::Time last_save_time;
};

class SyncSaver : public rclcpp::Node {
public:
    SyncSaver() : Node("sync_saver"), first_sync_triggered_(false) {
        this->declare_parameter("config_path", "/home/menna/CPSP-Dataset/sync_saver/config");
        fs::path config_path = this->get_parameter("config_path").as_string();
        
        init_sensor(NAME_CAM_ZED_L, config_path / "camL.ini");
        init_sensor(NAME_CAM_ZED_R, config_path / "camR.ini");
        init_sensor(NAME_IMU, config_path / "imu.ini");
        // init_sensor(NAME_CAM_DEV, config_path / "dev.yaml");

        // 独立IMU订阅
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            sensors_[NAME_IMU]->topic, rclcpp::SensorDataQoS().keep_last(200)/*sensors_[NAME_IMU]->rate_hz*/,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                if (imu_buffer_.size() > 30) {
                    imu_buffer_.pop_front();
                    RCLCPP_WARN(get_logger(), "wait to sync, imu_buffer out size");
                  }
                imu_buffer_.push_back(msg);
            });

        setup_synchronization();
    }

    ~SyncSaver() {
        for (auto& [name, sensor] : sensors_) {
            if (sensor->csv_file.is_open()) {
                sensor->csv_file.close();
            }
        }
    }

private:
    void init_sensor(const std::string& name, const fs::path& config_file) {
        RCLCPP_INFO(this->get_logger(), "Loading config: %s", config_file.c_str());
        if (!fs::exists(config_file)) {
            if (name == NAME_CAM_DEV) return; // wait dev finished
            RCLCPP_ERROR(get_logger(), "Config file missing: %s", config_file.c_str());
            return;
        }

        ConfigParser config(config_file.string());
        auto sensor = std::make_shared<SensorConfig>();
        sensor->name = name;
        sensor->type = config.get("sensor.type");
        sensor->topic = config.get("sensor.topic");
        sensor->rate_hz = config.get_double("sensor.rate_hz");
        
        // create path
        if (sensor->type == "camera") {
            sensor->data_path = fs::path("/home/menna/dataset") / name / "data";
            fs::create_directories(sensor->data_path);
        } else {
            sensor->data_path = fs::path("/home/menna/dataset") / name;
            fs::create_directories(sensor->data_path);
        }
        
        // init CSV
        sensor->csv_path = fs::path("/home/menna/dataset") / name / "data.csv";
        sensor->csv_file.open(sensor->csv_path);
        if (sensor->type == "camera") {
            sensor->csv_file << "#timestamp [ns],filename\n";
        } else if (sensor->type == "imu")  {
            sensor->csv_file << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
        }
        
        // save sensor config
        sensors_[name] = sensor;
    }

    void setup_synchronization() {
        // init subscribe
        left_sync_sub_.subscribe(this, sensors_[NAME_CAM_ZED_L]->topic);
        right_sync_sub_.subscribe(this, sensors_[NAME_CAM_ZED_R]->topic);
        imu_sync_sub_.subscribe(this, sensors_[NAME_IMU]->topic);
        // dev_sync_sub_ TODO

        // sync policy
        sync_ = std::make_shared<Sync>(SyncPolicy(20), 
            left_sync_sub_, right_sync_sub_, imu_sync_sub_/*dev_sync_sub_*/);    
        sync_->setInterMessageLowerBound(0, rclcpp::Duration(0, 50000000)); // 50ms 20Hz
        sync_->registerCallback(std::bind(&SyncSaver::sync_callback, this, _1, _2, _3/*_4*/));
    }

    inline int64_t get_nanoseconds(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        return msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    }

    inline int64_t get_nanoseconds(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) 
    {
        return msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    }

    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_img,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_img,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu
        /*const sensor_msgs::msg::Image::ConstSharedPtr& dev_img = nullptr*/) 
    {
        // sync time 以ZED为准
        rclcpp::Time sync_time = left_img->header.stamp;
        // 标记首次同步触发
        if (!first_sync_triggered_.exchange(true)) {
            RCLCPP_INFO(this->get_logger(), "First sync， camL[%ld], camR[%ld], imu[%ld]", 
                get_nanoseconds(left_img), get_nanoseconds(right_img), get_nanoseconds(imu));

            std::lock_guard<std::mutex> lock(imu_mutex_);
            // 清除首次同步前的历史数据
            auto first_valid = std::lower_bound(
                imu_buffer_.begin(), imu_buffer_.end(),
                rclcpp::Time(left_img->header.stamp),
                [](const auto& imu_msg, const rclcpp::Time& stamp) {
                    return rclcpp::Time(imu_msg->header.stamp) < stamp;
                });
            imu_buffer_.erase(imu_buffer_.begin(), first_valid);
        }

        RCLCPP_INFO(this->get_logger(), "sync time[%ld]", sync_time.nanoseconds());

        try {
            // save imgs
            process_camera(NAME_CAM_ZED_L, left_img);
            process_camera(NAME_CAM_ZED_R, right_img);
            // DEV
            // if (dev_img && sensors_.count("dev")) {
            //     process_camera("dev", dev_img);
            // }

            // save_imu(imu);
            save_imu_window(sync_time);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
        }
    }

    void process_camera(const std::string& name, const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        auto& sensor = sensors_[name];
        // if (!should_save(name, msg->header.stamp)) return;

        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        int64_t nanoseconds = get_nanoseconds(msg);
        std::string filename = std::to_string(nanoseconds) + ".png";
        
        std::lock_guard<std::mutex> lock(sensor->mtx);
        cv::imwrite((sensor->data_path / filename).string(), img);
        sensor->csv_file << nanoseconds << "," << filename << "\n";
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
        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> batch;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            auto it_start = std::lower_bound(
                imu_buffer_.begin(), imu_buffer_.end(),
                sync_time - rclcpp::Duration(0, 50'000'000),
                [](const auto& imu_msg, const rclcpp::Time& time) {
                    return rclcpp::Time(imu_msg->header.stamp) < time;
                });
            
            auto it_end = std::upper_bound(
                imu_buffer_.begin(), imu_buffer_.end(),
                sync_time + rclcpp::Duration(0, 50'000'000),
                [](const rclcpp::Time& time, const auto& imu_msg) {
                    return time < rclcpp::Time(imu_msg->header.stamp);
                });

            if (it_start != imu_buffer_.end()) {
                batch.assign(it_start, it_end);
                imu_buffer_.erase(imu_buffer_.begin(), it_end);
            }
        }

        // 批量保存到CSV
        if (!batch.empty()) {
            auto& sensor = sensors_["imu"];
            std::lock_guard<std::mutex> lock(sensor->mtx);
            for (const auto& imu : batch) {
                sensor->csv_file << get_nanoseconds(imu) << ","
                            << imu->angular_velocity.x << ","
                            << imu->angular_velocity.y << ","
                            << imu->angular_velocity.z << ","
                            << imu->linear_acceleration.x << ","
                            << imu->linear_acceleration.y << ","
                            << imu->linear_acceleration.z << "\n";
            }
            sensor->csv_file << "#SYNC_TS:" << get_nanoseconds(msg) << "\n";
        }
    #endif
    }

    void save_imu_window(const rclcpp::Time& sync_time) {
        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> batch;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            // ±50 ms 窗口
            auto it_start = std::lower_bound(
                imu_buffer_.begin(), imu_buffer_.end(),
                sync_time - rclcpp::Duration(0, 50'000'000),
                [](auto const& msg, auto const& t){ return rclcpp::Time(msg->header.stamp) < t; });
            auto it_end = std::upper_bound(
                imu_buffer_.begin(), imu_buffer_.end(),
                sync_time + rclcpp::Duration(0, 50'000'000),
                [](auto const& t, auto const& msg){ return t < rclcpp::Time(msg->header.stamp); });
    
            batch.assign(it_start, it_end);
            imu_buffer_.erase(imu_buffer_.begin(), it_end);
        }
    
        if (batch.empty()) {
            RCLCPP_WARN(get_logger(), "No IMU in sync window at %ld", sync_time.nanoseconds());
            return;
        }
    
        // 找最接近 sync_time 的索引
        size_t best_idx = 0;
        {
            int64_t best_diff = std::llabs(
                rclcpp::Time(batch[0]->header.stamp).nanoseconds()
                - sync_time.nanoseconds());
            for (size_t i = 1; i < batch.size(); ++i) {
                int64_t ts = rclcpp::Time(batch[i]->header.stamp).nanoseconds();
                int64_t diff = std::llabs(ts - sync_time.nanoseconds());
                if (diff < best_diff) {
                    best_diff = diff;
                    best_idx = i;
                }
            }
        }
    
        // 写入 CSV：只有 best_idx 那条用 sync_time.nanoseconds()，其它保持原 stamp
        auto& sensor = sensors_[NAME_IMU];
        std::lock_guard<std::mutex> lock(sensor->mtx);
        for (size_t i = 0; i < batch.size(); ++i) {
            auto const& imu_msg = batch[i];
            int64_t ts_to_write = (i == best_idx)
                ? sync_time.nanoseconds()
                : rclcpp::Time(imu_msg->header.stamp).nanoseconds();
    
            sensor->csv_file
                << ts_to_write << ","
                << imu_msg->angular_velocity.x << ","
                << imu_msg->angular_velocity.y << ","
                << imu_msg->angular_velocity.z << ","
                << imu_msg->linear_acceleration.x << ","
                << imu_msg->linear_acceleration.y << ","
                << imu_msg->linear_acceleration.z
                << "\n";
        }
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,  // camL
        sensor_msgs::msg::Image,  // camR
        sensor_msgs::msg::Imu     // IMU
        /*sensor_msgs::msg::Image*/>; // DEV 
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    std::map<std::string, std::shared_ptr<SensorConfig>> sensors_;
    std::shared_ptr<Sync> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sync_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sync_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sync_sub_;
    // message_filters::Subscriber<sensor_msgs::msg::Image> dev_sync_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
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