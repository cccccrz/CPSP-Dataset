#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <atomic>

// c++17
namespace fs = std::filesystem; 
using namespace std::placeholders;

class SyncSaver : public rclcpp::Node {
public:
    SyncSaver() : Node("sync_saver"), data_id_(0) {
        // init save dir
        save_dir_ = "/home/menna/dataset/stereo/stereo_data";
        fs::create_directories(save_dir_ + "/left");
        fs::create_directories(save_dir_ + "/right");
        fs::create_directories(save_dir_ + "/depth");
        fs::create_directories(save_dir_ + "/rgb");

        // init subscribe
        left_sub_.subscribe(this, "/zed/zed_node/left/image_rect_color");
        right_sub_.subscribe(this, "/zed/zed_node/right/image_rect_color");
        depth_sub_.subscribe(this, "/zed/zed_node/depth/depth_registered");
        rgb_sub_.subscribe(this, "/zed/zed_node/rgb/image_rect_color");
        //.subscribe(this, "/zed/zed_node/stereo/image_rect_color");
        // todo: DEV topic
        // ...

        // sync policy（10，±100ms duration）
        sync_ = std::make_shared<Sync>(SyncPolicy(10), left_sub_, right_sub_, depth_sub_, rgb_sub_);
        sync_->setInterMessageLowerBound(0, rclcpp::Duration(0, 100000000)); // foxy
        sync_->registerCallback(std::bind(&SyncSaver::syncCallback, this, _1, _2, _3, _4));
    }

private:
    void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg) 
    {
        try {
            // process data
            cv::Mat left_img = cv_bridge::toCvCopy(left_msg, "bgr8")->image;
            cv::Mat right_img = cv_bridge::toCvCopy(right_msg, "bgr8")->image;
            cv::Mat depth_img = cv_bridge::toCvCopy(depth_msg, "16UC1")->image;
            cv::Mat rgb_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;

            // atomic get id
            uint32_t current_id = data_id_.fetch_add(1);

            // save data
            cv::imwrite(save_dir_ + "/left/" + std::to_string(current_id) + ".png", left_img);
            cv::imwrite(save_dir_ + "/right/" + std::to_string(current_id) + ".png", right_img);
            cv::imwrite(save_dir_ + "/depth/" + std::to_string(current_id) + ".png", depth_img);
            cv::imwrite(save_dir_ + "/rgb/" + std::to_string(current_id) + ".png", rgb_img);

            RCLCPP_INFO(this->get_logger(), "Saved data: %u", current_id);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        }
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, 
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    std::shared_ptr<Sync> sync_;
    std::string save_dir_;
    std::atomic<uint32_t> data_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}