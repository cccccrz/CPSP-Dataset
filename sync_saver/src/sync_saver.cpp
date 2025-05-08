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
#include <yaml-cpp/yaml.h>
#include "rmw/qos_profiles.h"

#define NAME_CAM_ZED_L "camZedL"
#define NAME_CAM_ZED_R "camZedR"
#define NAME_CAM_DEV "camDev"
#define NAME_IMU "imuZed"
#define DIRPATH_CONFIG "/home/menna/CPSP-Dataset/sync_saver/config"
#define DIRPATH_DATASET "/home/menna/dataset"
#define CONFIG_CAM_ZED_L "camL.yaml"
#define CONFIG_CAM_ZED_R "camR.yaml"
#define CONFIG_IMU "imu.yaml"
#define CONFIG_CAM_DEV "dev.yaml"

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
        this->declare_parameter("config_path", DIRPATH_CONFIG);
        fs::path config_path = this->get_parameter("config_path").as_string();
        
        init_sensor(NAME_CAM_ZED_L, config_path / CONFIG_CAM_ZED_L);
        init_sensor(NAME_CAM_ZED_R, config_path / CONFIG_CAM_ZED_R);
        init_sensor(NAME_IMU, config_path / CONFIG_IMU);
        init_sensor(NAME_CAM_DEV, config_path / CONFIG_CAM_DEV);

        // 独立IMU订阅
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            sensors_[NAME_IMU]->topic, rclcpp::SensorDataQoS().keep_last(200)/*sensors_[NAME_IMU]->rate_hz*/,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                if (imu_buffer_.size() > 100) {
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
            RCLCPP_ERROR(get_logger(), "Config file missing: %s", config_file.c_str());
            return;
        }
    
        // Load config YAML
        YAML::Node cfg = YAML::LoadFile(config_file.string());
        if (!cfg["sensor"]) {
            RCLCPP_ERROR(get_logger(), "YAML missing 'sensor' root node");
            return;
        }
    
        auto sensor = std::make_shared<SensorConfig>();
        sensor->name = name;
        sensor->type    = cfg["sensor"]["type"].as<std::string>();
        sensor->topic   = cfg["sensor"]["topic"].as<std::string>();
        sensor->rate_hz = cfg["sensor"]["rate_hz"].as<double>();
    
        // create dir、open CSV
        if (sensor->type == "camera") {
            sensor->data_path = fs::path(DIRPATH_DATASET) / name / "data";
            fs::create_directories(sensor->data_path);
            sensor->csv_path = fs::path(DIRPATH_DATASET) / name / "data.csv";
            sensor->csv_file.open(sensor->csv_path);
            sensor->csv_file << "#timestamp [ns],filename\n";
        } 
        else if (sensor->type == "imu") {
            sensor->data_path = fs::path(DIRPATH_DATASET) / name;
            fs::create_directories(sensor->data_path);
            sensor->csv_path = fs::path(DIRPATH_DATASET) / name / "data.csv";
            sensor->csv_file.open(sensor->csv_path);
            sensor->csv_file 
              << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
                 "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
        } 
        else {
            RCLCPP_WARN(get_logger(), "Unknown sensor.type '%s' in %s",
                        sensor->type.c_str(), config_file.c_str());
        }
    
        sensors_[name] = sensor;
    }    

    void setup_synchronization() {
        // init subscribe
        left_sync_sub_.subscribe(this, sensors_[NAME_CAM_ZED_L]->topic, rmw_qos_profile_sensor_data);
        right_sync_sub_.subscribe(this, sensors_[NAME_CAM_ZED_R]->topic, rmw_qos_profile_sensor_data);
        imu_sync_sub_.subscribe(this, sensors_[NAME_IMU]->topic, rmw_qos_profile_sensor_data);
        dev_sync_sub_.subscribe(this, sensors_[NAME_CAM_DEV]->topic, rmw_qos_profile_sensor_data);

        // test
        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/event_camera/image_raw", 
            rclcpp::SensorDataQoS().keep_last(10),
            [this](const sensor_msgs::msg::Image::SharedPtr msg){
                RCLCPP_INFO(get_logger(), "Got image! [%d x %d]", 
                    msg->width, msg->height);
            });

        // sync policy
        sync_ = std::make_shared<Sync>(SyncPolicy(40), 
            left_sync_sub_, right_sync_sub_, imu_sync_sub_, dev_sync_sub_);    
        sync_->setInterMessageLowerBound(0, rclcpp::Duration(0, 50000000)); // 50ms 20Hz
        sync_->registerCallback(std::bind(&SyncSaver::sync_callback, this, _1, _2, _3, _4));
    }

    inline int64_t get_nanoseconds(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        return rclcpp::Time(msg->header.stamp).nanoseconds();
    }

    inline int64_t get_nanoseconds(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) 
    {
        return rclcpp::Time(msg->header.stamp).nanoseconds();
    }

    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_img,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_img,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu,
        const sensor_msgs::msg::Image::ConstSharedPtr& dev_img) 
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
            process_camera(NAME_CAM_DEV, dev_img);

            save_imu_window(sync_time);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Processing error: %s", e.what());
        }
    }

    void process_camera(const std::string& name, const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        auto& sensor = sensors_[name];

        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        int64_t nanoseconds = get_nanoseconds(msg);
        std::string filename = std::to_string(nanoseconds) + ".png";
        
        std::lock_guard<std::mutex> lock(sensor->mtx);
        cv::imwrite((sensor->data_path / filename).string(), img);
        sensor->csv_file << nanoseconds << "," << filename << "\n";
        sensor->last_save_time = msg->header.stamp;
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
        sensor_msgs::msg::Imu,    // IMU
        sensor_msgs::msg::Image>; // DEV 
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    std::map<std::string, std::shared_ptr<SensorConfig>> sensors_;
    std::shared_ptr<Sync> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sync_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sync_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sync_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> dev_sync_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr img_sub_;
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