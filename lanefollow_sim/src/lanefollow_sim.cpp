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

class lanefollow_sim : public rclcpp::Node
{
public:
    lanefollow_sim()
    : Node("lanefollow_sim"),
      left_center_x_(-1.0),
      right_center_x_(-1.0),
      left_lost_(0),
      right_lost_(0),
      mode_(false),
      base_speed_(100),
      k_(0.5),
      min_speed_(50),
      vel1_(0), vel2_(0),
      goal1_(0), goal2_(0),
      display_ready_(false),
      running_(true)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video1", rclcpp::QoS(10),
            std::bind(&lanefollow_sim::image_callback, this,
                      std::placeholders::_1));

        vel_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "vel_cmd", rclcpp::QoS(10));
        vel_msg_.data.resize(2);

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
                    cv::imshow("Original Frame",    display_frame_);
                    cv::imshow("Lane with Overlay", display_binary_);
                    cv::waitKey(1);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
        });

        RCLCPP_INFO(this->get_logger(), "lanefollow_sim Node Started");
        RCLCPP_INFO(this->get_logger(),
            "base_speed:%d, k:%.2f, min_speed:%d",
            base_speed_, k_, min_speed_);
        RCLCPP_INFO(this->get_logger(), "'s':start 'q':stop");
    }

    ~lanefollow_sim()
    {
        running_ = false;
        if (motor_thread_.joinable())   motor_thread_.join();
        if (display_thread_.joinable()) display_thread_.join();
    }

private:
    // ========= 라인 1개 추적 =========
    double track_line(
        const std::vector<std::vector<cv::Point>>& contours,
        double prev_x,
        int& lost_count,
        cv::Mat& display,
        int roi_offset_x,
        int roi_height)
    {
        double best_x   = prev_x;
        double min_dist = 1e9;
        cv::Rect best_rect;
        bool found = false;

        // ★ 정상 추적 시 이동 제한 (30px/frame)
        double move_limit = (lost_count > 0) ? 999.0 : 30.0;

        for (const auto &cnt : contours)
        {
            cv::Rect rect = cv::boundingRect(cnt);
            double area   = rect.area();
            int cx        = rect.x + rect.width / 2;

            cv::rectangle(display, rect, cv::Scalar(255, 0, 0), 2);

            if (area < 100) continue;
            if (rect.width < 5) continue;
            if (rect.height < 5) continue;
            if ((float)rect.height / rect.width < 0.15f) continue;

            double candidate_x = roi_offset_x + cx;
            double dist        = std::abs(candidate_x - prev_x);

            // ★ 정상 추적 시 너무 멀리 튀는 컨투어 제거
            if (dist > move_limit) continue;

            if (dist < min_dist) {
                min_dist  = dist;
                best_x    = candidate_x;
                best_rect = rect;
                found     = true;
            }
        }

        // lost 상태면 move_limit 없이 재탐색
        if (!found && lost_count > 0) {
            for (const auto &cnt : contours)
            {
                cv::Rect rect = cv::boundingRect(cnt);
                double area   = rect.area();
                int cx        = rect.x + rect.width / 2;

                if (area < 100) continue;
                if (rect.width < 5) continue;
                if (rect.height < 5) continue;
                if ((float)rect.height / rect.width < 0.15f) continue;

                double candidate_x = roi_offset_x + cx;
                double dist        = std::abs(candidate_x - prev_x);

                if (dist < min_dist) {
                    min_dist  = dist;
                    best_x    = candidate_x;
                    best_rect = rect;
                    found     = true;
                }
            }
        }

        if (found)
        {
            int prev_lost = lost_count;
            lost_count    = 0;

            cv::rectangle(display, best_rect, cv::Scalar(0, 0, 255), 2);
            cv::circle(display,
                cv::Point(best_x - roi_offset_x,
                          best_rect.y + best_rect.height / 2),
                6, cv::Scalar(0, 0, 255), -1);

            double alpha = (prev_lost > 0) ? 0.5 : 0.3;
            prev_x = prev_x * (1.0 - alpha) + best_x * alpha;
        }
        else
        {
            lost_count++;
            if (lost_count > 10) lost_count = 10;

            cv::circle(display,
                cv::Point((int)(prev_x - roi_offset_x), roi_height - 10),
                6, cv::Scalar(0, 255, 255), -1);
        }

        return prev_x;
    }

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
        cv::Mat roi = frame(cv::Rect(0, roi_y, frame_w, roi_height)).clone();

        // ========= 밝기 보정 =========
        double shift = 140.0 - cv::mean(roi)[0];
        roi.convertTo(roi, -1, 0.7, shift);

        // ========= 그레이스케일 + 블러 + 이진화 =========
        cv::Mat gray, blurred, binary;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        cv::threshold(blurred, binary, 150, 255, cv::THRESH_BINARY);

        cv::Mat display;
        cv::cvtColor(binary, display, cv::COLOR_GRAY2BGR);

        // ========= 초기위치 설정 =========
        if (left_center_x_ < 0)
            left_center_x_  = frame_w * 1.0 / 4.0;
        if (right_center_x_ < 0)
            right_center_x_ = frame_w * 3.0 / 4.0;

        // ========= 좌우 ROI 분할 =========
        int half_w = frame_w / 2;

        cv::Mat left_binary   = binary(cv::Rect(0,      0, half_w, roi_height)).clone();
        cv::Mat right_binary  = binary(cv::Rect(half_w, 0, half_w, roi_height)).clone();
        cv::Mat left_display  = display(cv::Rect(0,      0, half_w, roi_height));
        cv::Mat right_display = display(cv::Rect(half_w, 0, half_w, roi_height));

        std::vector<std::vector<cv::Point>> left_contours, right_contours;
        cv::findContours(left_binary,  left_contours,  cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(right_binary, right_contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        // ========= 좌우 라인 추적 =========
        left_center_x_ = track_line(
            left_contours, left_center_x_, left_lost_,
            left_display, 0, roi_height);

        right_center_x_ = track_line(
            right_contours, right_center_x_, right_lost_,
            right_display, half_w, roi_height);

        // ★ 한쪽 라인 사라졌을 때 반대쪽 기준으로 추정
        const double lane_width = frame_w / 2.0;

        if (left_lost_ > 2 && right_lost_ == 0) {
            // 좌측 사라짐 → 우측 기준 추정
            left_center_x_ = right_center_x_ - lane_width;
        }
        else if (right_lost_ > 2 && left_lost_ == 0) {
            // 우측 사라짐 → 좌측 기준 추정
            right_center_x_ = left_center_x_ + lane_width;
        }

        // ========= 두 라인 중점 =========
        double mid_x = (left_center_x_ + right_center_x_) / 2.0;
        cv::circle(display,
            cv::Point((int)mid_x, roi_height / 2),
            8, cv::Scalar(0, 255, 0), -1);

        // ========= error 계산 =========
        double error = frame_w / 2.0 - mid_x;

        // ========= P제어 속도 계산 =========
        int lvel = static_cast<int>( base_speed_ - k_ * error);
        int rvel = static_cast<int>(-(base_speed_ + k_ * error));

        lvel = std::clamp(lvel, -200, 200);
        rvel = std::clamp(rvel, -200, 200);

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

    double left_center_x_;
    double right_center_x_;
    int    left_lost_;
    int    right_lost_;

    bool   mode_;
    int    base_speed_;
    double k_;
    int    min_speed_;

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
    rclcpp::spin(std::make_shared<lanefollow_sim>());
    rclcpp::shutdown();
    return 0;
}
