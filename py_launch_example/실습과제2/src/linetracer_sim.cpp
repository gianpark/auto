#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdlib>
#include <algorithm>

// ========= 비블로킹 키 입력 =========
int kbhit()
{
    struct termios oldt, newt;
    int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) { ungetc(ch, stdin); return 1; }
    return 0;
}

int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class line_subscriber : public rclcpp::Node
{
public:
    line_subscriber()
    : Node("line_detector"),
      prev_center_x_(-1.0),
      lost_count_(0),
      mode_(false),
      base_speed_(100),
      k_(0.5),
      min_speed_(50),
      vel1_(0), vel2_(0),
      goal1_(0), goal2_(0),
      display_ready_(false),
      running_(true)
    {
        // ========= 이미지 구독 =========
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video1", rclcpp::QoS(10),
            std::bind(&line_subscriber::image_callback, this,
                      std::placeholders::_1));

        // ========= 속도명령 퍼블리셔 =========
        vel_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "vel_cmd", rclcpp::QoS(10));
        vel_msg_.data.resize(2);

        // ========= 모터 시뮬레이션 스레드 (50ms) =========
        motor_thread_ = std::thread([this]() {
            while (running_) {
                {
                    std::lock_guard<std::mutex> lock(motor_mutex_);
                    if      (goal1_ > vel1_) vel1_ += 5;
                    else if (goal1_ < vel1_) vel1_ -= 5;
                    else                     vel1_  = goal1_;

                    if      (goal2_ > vel2_) vel2_ += 5;
                    else if (goal2_ < vel2_) vel2_ -= 5;
                    else                     vel2_  = goal2_;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });

        // ========= 디스플레이 스레드 =========
        display_thread_ = std::thread([this]() {
            const char* disp = std::getenv("DISPLAY");
            bool has_display = (disp != nullptr && disp[0] != '\0');
            if (!has_display) {
                RCLCPP_INFO(this->get_logger(), "No DISPLAY - skip imshow");
                return;
            }
            while (running_) {
                if (display_ready_.exchange(false)) {
                    std::lock_guard<std::mutex> lock(display_mutex_);
                    cv::imshow("Original Frame",      display_frame_);
                    cv::imshow("Binary with Overlay", display_binary_);
                    cv::waitKey(1);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
        });

        RCLCPP_INFO(this->get_logger(), "line_subscriber Node Started");
        RCLCPP_INFO(this->get_logger(),
            "base_speed:%d, k:%.2f, min_speed:%d",
            base_speed_, k_, min_speed_);
        RCLCPP_INFO(this->get_logger(), "'s':start 'q':stop");
    }

    ~line_subscriber()
    {
        running_ = false;
        if (motor_thread_.joinable())   motor_thread_.join();
        if (display_thread_.joinable()) display_thread_.join();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto start = std::chrono::steady_clock::now();

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) { return; }
        if (frame.empty()) return;

        int frame_w = frame.cols;
        int frame_h = frame.rows;

        // ========= ROI (하단 90px) =========
        int roi_height = 90;
        int roi_y      = frame_h - roi_height;
        cv::Rect roi_rect(0, roi_y, 640, roi_height);
        cv::Mat roi = frame(roi_rect).clone();

        // ========= 밝기 보정 =========
        double shift = 140.0 - cv::mean(roi)[0];
        roi.convertTo(roi, -1, 0.7, shift);

        // ========= 그레이스케일 + 이진화 =========
        cv::Mat gray, binary;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 140, 255, cv::THRESH_BINARY);

        cv::Mat display;
        cv::cvtColor(binary, display, cv::COLOR_GRAY2BGR);

        // ========= 컨투어 검출 =========
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        if (prev_center_x_ < 0)
            prev_center_x_ = frame_w / 2.0;

        double best_center_x = prev_center_x_;
        double min_dist      = 1e9;
        cv::Rect best_rect;
        bool found = false;

        for (const auto &cnt : contours)
        {
            cv::Rect rect = cv::boundingRect(cnt);
            double area   = rect.area();
            int cx        = rect.x + rect.width / 2;

            cv::rectangle(display, rect, cv::Scalar(255, 0, 0), 2);

            if (area < 60) continue;
            if (rect.width < 3 || rect.height < 5) continue;
            if ((float)rect.height / rect.width < 0.10f) continue;
            if (rect.y + rect.height < roi_height * 0.30) continue;

            double candidate_x = roi_rect.x + cx;
            double dist        = std::abs(candidate_x - prev_center_x_);

            if (dist < min_dist) {
                min_dist      = dist;
                best_center_x = candidate_x;
                best_rect     = rect;
                found         = true;
            }
        }

        const double max_jump       = 160.0;
        const int    max_lost       = 5;
        const double max_speed      = 12.0;
        const double reappear_speed = 6.0;

        if (found && min_dist < max_jump)
        {
            lost_count_ = 0;
            cv::rectangle(display, best_rect, cv::Scalar(0, 0, 255), 2);
            cv::circle(display,
                cv::Point(best_center_x - roi_rect.x,
                          best_rect.y + best_rect.height / 2),
                6, cv::Scalar(0, 0, 255), -1);

            double target = best_center_x;
            double dx     = target - prev_center_x_;
            double limit  = (lost_count_ > 0) ? reappear_speed : max_speed;

            if (std::abs(dx) > limit)
                target = prev_center_x_ + (dx > 0 ? limit : -limit);

            prev_center_x_ = prev_center_x_ * 0.7 + target * 0.3;
        }
        else
        {
            lost_count_++;
            if (lost_count_ > max_lost) lost_count_ = max_lost;

            double target_x;
            if (prev_center_x_ < frame_w * 0.3)
                target_x = roi_rect.x + (roi_rect.width * 3.0 / 5.0);
            else if (prev_center_x_ > frame_w * 0.7)
                target_x = roi_rect.x + (roi_rect.width * 1.0 / 5.0);
            else
                target_x = prev_center_x_;

            prev_center_x_ = prev_center_x_ * 0.85 + target_x * 0.15;

            cv::circle(display,
                cv::Point(prev_center_x_ - roi_rect.x, roi_height - 10),
                6, cv::Scalar(0, 0, 255), -1);
        }

        // ========= error 계산 =========
        double error = frame_w / 2.0 - prev_center_x_;

        // ========= P제어 속도 계산 =========
        int lvel = static_cast<int>( base_speed_ - k_ * error);
        int rvel = static_cast<int>(-(base_speed_ + k_ * error));

        // 속도 클램핑
        lvel = std::clamp(lvel, -200, 200);
        rvel = std::clamp(rvel, -200, 200);

        // ========= 안쪽 바퀴 최소 속도 보장 =========
        if (lvel > 0 && lvel < min_speed_)       lvel = min_speed_;
        else if (lvel < 0 && lvel > -min_speed_) lvel = -min_speed_;
        if (rvel > 0 && rvel < min_speed_)       rvel = min_speed_;
        else if (rvel < 0 && rvel > -min_speed_) rvel = -min_speed_;

        // ========= s/q 키 처리 =========
        if (kbhit())
        {
            int ch = getch();
            if (ch == 'q') {
                mode_ = false;
                std::lock_guard<std::mutex> lock(motor_mutex_);
                goal1_ = goal2_ = 0;
                RCLCPP_INFO(this->get_logger(), "STOP");
            } else if (ch == 's') {
                mode_ = true;
                RCLCPP_INFO(this->get_logger(), "START");
            }
        }

        // ========= goal 속도 업데이트 =========
        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            if (mode_) {
                goal1_ = lvel;
                goal2_ = rvel;
            } else {
                goal1_ = goal2_ = 0;
            }
        }

        // ========= vel_cmd 퍼블리시 =========
        vel_msg_.data[0] = vel1_;
        vel_msg_.data[1] = vel2_;
        vel_publisher_->publish(vel_msg_);

        // ========= 디스플레이 전달 =========
        {
            std::lock_guard<std::mutex> lock(display_mutex_);
            display_frame_  = frame.clone();
            display_binary_ = display.clone();
            display_ready_  = true;
        }

        // ========= 수행 시간 측정 및 출력 =========
        auto end = std::chrono::steady_clock::now();
        float t = std::chrono::duration<float, std::milli>(end - start).count();

        printf("err:%d, lvel:%d, rvel:%d, time:%.2fmsec\n",
            (int)error, vel1_, vel2_, t);
        fflush(stdout);
    }

    // ========= 멤버 변수 =========
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr vel_publisher_;

    double prev_center_x_;
    int    lost_count_;
    bool   mode_;
    int    base_speed_;   // 설계변수: 직진속도
    double k_;            // 설계변수: P제어 게인
    int    min_speed_;    // 설계변수: 최소속도

    int vel1_, vel2_;
    int goal1_, goal2_;

    std_msgs::msg::Int32MultiArray vel_msg_;

    std::mutex        motor_mutex_;
    std::thread       motor_thread_;

    cv::Mat           display_frame_;
    cv::Mat           display_binary_;
    std::mutex        display_mutex_;
    std::atomic<bool> display_ready_;
    std::atomic<bool> running_;
    std::thread       display_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<line_subscriber>());
    rclcpp::shutdown();
    return 0;
}
