#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/zed_imu.hpp>
#include <dvs_msgs/msg/event_array.hpp>
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
#define NAME_CAM_DEV "camDavis"
#define NAME_ZED_IMU "imuZed"
#define NAME_DEV_IMU "imuDavis"
#define NAME_EVENT "eventDavis"

#define DIRPATH_CONFIG "/home/menna/CPSP-Dataset/sync_saver/config"
#define DIRPATH_DATASET "/home/menna/dataset"

#define CONFIG_CAM_ZED_L "camZedL.yaml"
#define CONFIG_CAM_ZED_R "camZedR.yaml"
#define CONFIG_ZED_IMU "imuZed.yaml"
#define CONFIG_CAM_DEV "camDavis.yaml"
#define CONFIG_DEV_IMU "imuDavis.yaml"
#define CONFIG_EVENT "event.yaml"

#define DEBUG

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
        init_sensor(NAME_ZED_IMU, config_path / CONFIG_ZED_IMU);
        init_sensor(NAME_CAM_DEV, config_path / CONFIG_CAM_DEV);
        init_sensor(NAME_DEV_IMU, config_path / CONFIG_DEV_IMU);
        init_sensor(NAME_EVENT, config_path / CONFIG_EVENT);

        // 独立IMU订阅
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            sensors_[NAME_ZED_IMU]->topic, rclcpp::SensorDataQoS().keep_last(200)/*sensors_[NAME_ZED_IMU]->rate_hz*/,
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
        if (sensor->type == "camera" || sensor->type == "event") {
            sensor->data_path = fs::path(DIRPATH_DATASET) / name / "data";
            fs::create_directories(sensor->data_path);
            sensor->csv_path = fs::path(DIRPATH_DATASET) / name / "data.csv";
            sensor->csv_file.open(sensor->csv_path);
            if (isFileEmpty(sensor->csv_path.string())) {
                sensor->csv_file << "#timestamp [ns],filename\n";
            }
        } else if (sensor->type == "imu") {
            sensor->data_path = fs::path(DIRPATH_DATASET) / name;
            fs::create_directories(sensor->data_path);
            sensor->csv_path = fs::path(DIRPATH_DATASET) / name / "data.csv";
            sensor->csv_file.open(sensor->csv_path);
            if (isFileEmpty(sensor->csv_path.string())) {
                sensor->csv_file 
                << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
                "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
            }
        } else {
            RCLCPP_WARN(get_logger(), "Unknown sensor.type '%s' in %s",
                        sensor->type.c_str(), config_file.c_str());
        }
    
        sensors_[name] = sensor;
    }    

    void setup_synchronization() {
        // init subscribe
        left_sync_sub_.subscribe(this, sensors_[NAME_CAM_ZED_L]->topic, rmw_qos_profile_sensor_data);
        right_sync_sub_.subscribe(this, sensors_[NAME_CAM_ZED_R]->topic, rmw_qos_profile_sensor_data);
        imu_sync_sub_.subscribe(this, sensors_[NAME_ZED_IMU]->topic, rmw_qos_profile_sensor_data);
        davis_sync_img_sub_.subscribe(this, sensors_[NAME_CAM_DEV]->topic, rmw_qos_profile_sensor_data);
        davis_sync_imu_sub_.subscribe(this, sensors_[NAME_DEV_IMU]->topic, rmw_qos_profile_sensor_data);
        davis_sync_event_sub_.subscribe(this, sensors_[NAME_EVENT]->topic, rmw_qos_profile_sensor_data);

#ifdef DEBUG
        d_img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/event_camera/image_raw", 
            rclcpp::SensorDataQoS().keep_last(10),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr msg){
                RCLCPP_INFO(get_logger(), "Got image! [%d x %d]", 
                    msg->width, msg->height);
                process_camera(NAME_CAM_DEV, msg);
            });
        d_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/event_camera/imu", 
            rclcpp::SensorDataQoS().keep_last(10),
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg){
                RCLCPP_INFO(get_logger(), "Got imu!");
                save_imu(NAME_DEV_IMU, msg, get_nanoseconds(msg));
            });
        d_event_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/event_camera/event", 
            rclcpp::SensorDataQoS().keep_last(10),
            [this](const dvs_msgs::msg::EventArray::ConstSharedPtr msg){
                RCLCPP_INFO(get_logger(), "Got event!");
                process_event(NAME_EVENT, msg);
            });
#else
        // sync policy
        sync_ = std::make_shared<Sync>(SyncPolicy(100), 
        left_sync_sub_, right_sync_sub_, imu_sync_sub_, davis_sync_img_sub_, davis_sync_imu_sub_, davis_sync_event_sub_);    
        sync_->setInterMessageLowerBound(0, rclcpp::Duration(0, 50000000)); // 50ms 20Hz
        sync_->registerCallback(std::bind(&SyncSaver::sync_callback, this, _1, _2, _3, _4, _5, _6));
#endif
    }

    inline int64_t get_nanoseconds(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        return rclcpp::Time(msg->header.stamp).nanoseconds();
    }

    inline int64_t get_nanoseconds(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) 
    {
        return rclcpp::Time(msg->header.stamp).nanoseconds();
    }

    inline int64_t get_nanoseconds(const dvs_msgs::msg::EventArray::ConstSharedPtr& msg) 
    {
        return rclcpp::Time(msg->header.stamp).nanoseconds();
    }
    
    bool isFileEmpty(const std::string& filename) {
        std::ifstream file(filename);
        return file.peek() == std::ifstream::traits_type::eof();
    }

    void sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_img,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_img,
        const sensor_msgs::msg::Imu::ConstSharedPtr& zed_imu,
        const sensor_msgs::msg::Image::ConstSharedPtr& dev_img,
        const sensor_msgs::msg::Imu::ConstSharedPtr& dev_imu,
        const dvs_msgs::msg::EventArray::ConstSharedPtr& dev_event) 
    {
        // sync time 以ZED为准
        auto sync_time = get_nanoseconds(left_img);
        // 标记首次同步触发
        if (!first_sync_triggered_.exchange(true)) {
            RCLCPP_INFO(this->get_logger(), "First sync， camL[%ld], camR[%ld], zed_img[%ld], dev_imu[%ld], dev_imu[%ld], dev_event[%ld]", 
                get_nanoseconds(left_img), get_nanoseconds(right_img), get_nanoseconds(zed_imu),
                get_nanoseconds(dev_img), get_nanoseconds(dev_imu), get_nanoseconds(dev_event));

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

        RCLCPP_INFO(this->get_logger(), "sync time[%ld]", sync_time);

        try {
            // save imgs
            process_camera(NAME_CAM_ZED_L, left_img);
            process_camera(NAME_CAM_ZED_R, right_img);
            process_camera(NAME_CAM_DEV, dev_img);

            save_imu_window(left_img->header.stamp);
            save_imu(NAME_DEV_IMU, dev_imu, sync_time)

            process_event(NAME_EVENT, dev_event);

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

    void process_event(const std::string& name, const dvs_msgs::msg::EventArray::ConstSharedPtr& msg) {
        auto& sensor = sensors_[name];

        int64_t nanoseconds = get_nanoseconds(msg);
        std::string filename = std::to_string(nanoseconds) + ".bt";
        
        std::lock_guard<std::mutex> lock(sensor->mtx);
        // cv::imwrite((sensor->data_path / filename).string(), img);
        // save as bin
        std::ofstream out(filename, std::ios::binary);

        for (const auto &e : msg->events) {
            out.write(reinterpret_cast<const char *>(&e.x), sizeof(e.x));
            out.write(reinterpret_cast<const char *>(&e.y), sizeof(e.y));
            double ts = e.ts.sec + 1e-9 * e.ts.nanosec;
            out.write(reinterpret_cast<const char *>(&ts), sizeof(ts));
            out.write(reinterpret_cast<const char *>(&e.polarity), sizeof(e.polarity));
        }
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
        auto& sensor = sensors_[NAME_ZED_IMU];
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

    void save_imu(const std::string& name, const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, const int64_t& ts_to_write) {
        auto& sensor = sensors_[name];
        std::lock_guard<std::mutex> lock(sensor->mtx);
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

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,  // zed camL
        sensor_msgs::msg::Image,  // zed camR
        sensor_msgs::msg::Imu,    // zed IMU
        sensor_msgs::msg::Image,  // Davis img
        sensor_msgs::msg::Imu,      // Davis zed_imu
        dvs_msgs::msg::EventArray>; // Davis event 
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    std::map<std::string, std::shared_ptr<SensorConfig>> sensors_;
    std::shared_ptr<Sync> sync_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sync_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sync_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sync_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> davis_sync_img_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> davis_sync_imu_sub_;
    message_filters::Subscriber<dvs_msgs::msg::EventArray> davis_sync_event_sub_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
#ifdef DEBUG
    rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr d_img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr d_imu_sub_;
    rclcpp::Subscription<dvs_msgs::msg::EventArray>::ConstSharedPtr d_event_sub_;
#endif
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