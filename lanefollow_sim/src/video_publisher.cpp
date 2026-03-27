#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher() : Node("video_publisher")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("video1", 10);

        timer_ = this->create_wall_timer(
            33ms, std::bind(&VideoPublisher::timer_callback, this));

        std::string path =
            "/home/linux/ros2_ws/simulation/lanefollow_100rpm_cw.mp4";
        cap_.open(path);

        if (!cap_.isOpened())
            RCLCPP_ERROR(this->get_logger(), "Failed to open: %s", path.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "Video opened: %s", path.c_str());
    }

private:
    void timer_callback()
    {
        if (!cap_.isOpened()) return;

        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "Video ended, restarting...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
