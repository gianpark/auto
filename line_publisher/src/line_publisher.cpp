#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher() : Node("video_publisher")
    {
        // --------- 이미지 퍼블리셔 생성 ---------
        // "video1" 토픽으로 sensor_msgs/msg/Image 메시지를 발행
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("video1", 10);

        // --------- 타이머 생성 (30ms = 약 33 FPS) ---------
        // timer_callback()이 30ms마다 실행됨 → 일정한 속도로 영상 프레임 발행
        timer_ = this->create_wall_timer(
            30ms,
            std::bind(&VideoPublisher::timer_callback, this)
        );

        // --------- 영상 파일 열기 ---------
        std::string video_path = "/home/linux/ros2_ws/simulation/7_lt_ccw_100rpm_in.mp4";
        cap_.open(video_path);

        // 영상 파일 오픈 실패 시 에러 출력
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
        }
    }

private:
    // ------------------ 타이머 콜백 함수 ------------------
    // 이 함수는 30ms마다 실행되어 영상 프레임을 읽고 퍼블리시함
    void timer_callback()
    {
        // 영상이 정상적으로 열리지 않은 경우
        if (!cap_.isOpened()) return;

        cv::Mat frame;

        // --------- 영상에서 한 프레임 읽기 ---------
        // 더 이상 읽을 프레임이 없으면 영상 종료 처리
        if (!cap_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "Video ended");
            rclcpp::shutdown();  // ROS 노드 종료
            return;
        }

        // --------- OpenCV Mat → ROS Image 메시지 변환 ---------
        // cv_bridge를 사용하여 BGR 포맷을 ROS 메시지로 변환
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // --------- 퍼블리시 ---------
        pub_->publish(*msg);
    }

    // --------- 클래스 멤버 변수들 ---------
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;  // 이미지 퍼블리셔
    rclcpp::TimerBase::SharedPtr timer_;                         // 타이머
    cv::VideoCapture cap_;                                       // 영상 파일 입력 객체
};

int main(int argc, char *argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // VideoPublisher 노드를 실행 (spin)
    rclcpp::spin(std::make_shared<VideoPublisher>());

    // 종료 처리
    rclcpp::shutdown();
    return 0;
}
