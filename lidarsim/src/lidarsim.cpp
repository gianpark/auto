#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <csignal>
#include <thread>
#include <termios.h>
#include <unistd.h>

#define IMG_SIZE   500
#define WORLD_SIZE 2.0f
#define M2PIX      (IMG_SIZE / WORLD_SIZE)
#define CX         (IMG_SIZE / 2)
#define CY         (IMG_SIZE / 2)
#define KP         1
#define BASE_SPEED 100

bool g_running = true;
bool g_mode    = false;

void sigHandler(int sig)
{
    (void)sig;
    g_running = false;
    rclcpp::shutdown();
}

void keyboardThread()
{
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    printf("S: 주행시작  Q: 정지  Ctrl+C: 종료\n");
    while (g_running) {
        int ch = getchar();
        if (ch == 's' || ch == 'S') { g_mode = true;  printf("▶ 주행\n"); }
        if (ch == 'q' || ch == 'Q') { g_mode = false; printf("⏹ 정지\n"); }
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

class LidarSim : public rclcpp::Node
{
public:
    LidarSim() : Node("lidarsim"), prev_error_(0.0f)
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
                   "topic_dxlpub", 10);

        cap_.open("lidar_scan.mp4");
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "lidar_scan.mp4 열기 실패!");
            g_running = false;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "lidar_scan.mp4 열기 성공");

        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        writer_.open("lidarsim_result.mp4", fourcc, 10.0,
                     cv::Size(IMG_SIZE, IMG_SIZE));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarSim::timerCb, this));
    }

private:
    void timerCb()
    {
        if (!g_running) return;

        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "영상 재생 완료");
            sendVelCmd(0, 0);
            if (writer_.isOpened()) writer_.release();
            g_running = false;
            rclcpp::shutdown();
            return;
        }

        cv::resize(frame, frame, cv::Size(IMG_SIZE, IMG_SIZE));

        // ── 장애물 검출 ───────────────────────────────────
        cv::Mat mask;
        cv::inRange(frame,
                    cv::Scalar(0, 0, 200),
                    cv::Scalar(50, 50, 255),
                    mask);

        cv::Mat front_mask = mask(cv::Rect(0, 0, IMG_SIZE, CY));
        cv::Mat left_mask  = front_mask(cv::Rect(0, 0, CX, CY));
        cv::Mat right_mask = front_mask(cv::Rect(CX, 0, CX, CY));

        // 좌측 최단거리 장애물 검출
        int left_min_dist = INT_MAX;
        int left_min_x = -1, left_min_y = -1;

        for (int y = 0; y < left_mask.rows; y++) {
            for (int x = 0; x < left_mask.cols; x++) {
                if (left_mask.at<uchar>(y, x) > 0) {
                    int dx = x - CX, dy = y - CY;
                    int dist = dx*dx + dy*dy;
                    if (dist < left_min_dist) {
                        left_min_dist = dist;
                        left_min_x = x;
                        left_min_y = y;
                    }
                }
            }
        }

        // 우측 최단거리 장애물 검출
        int right_min_dist = INT_MAX;
        int right_min_x = -1, right_min_y = -1;

        for (int y = 0; y < right_mask.rows; y++) {
            for (int x = 0; x < right_mask.cols; x++) {
                if (right_mask.at<uchar>(y, x) > 0) {
                    int real_x = x + CX;
                    int dx = real_x - CX, dy = y - CY;
                    int dist = dx*dx + dy*dy;
                    if (dist < right_min_dist) {
                        right_min_dist = dist;
                        right_min_x = real_x;
                        right_min_y = y;
                    }
                }
            }
        }

        // ── error 계산 (각도, degree) ─────────────────────
        // 파란선(수평선) 기준
        // 중앙=0도, 왼쪽=-90도, 오른쪽=+90도
        float raw_error = 0.0f;

        bool left_found  = (left_min_x  != -1);
        bool right_found = (right_min_x != -1);

        if (left_found && right_found) {
            // 노란점 x좌표 기준으로 각도 계산
            int mid_x = (left_min_x + right_min_x) / 2;

            // 노란점은 y=10으로 고정
            // CX에서 mid_x까지의 거리를 CY 거리로 나눠 각도 계산
            float dx = static_cast<float>(mid_x - CX);  // 좌우 거리
            float dy = static_cast<float>(CY - 10);      // 수평선~노란점 거리

            // atan2(dx, dy): 수직=0도, 오른쪽=+90도, 왼쪽=-90도
            raw_error = std::atan2(dx, dy) * 180.0f / M_PI;

        } else if (left_found && !right_found) {
            raw_error = prev_error_;

        } else if (!left_found && right_found) {
            raw_error = prev_error_;

        } else {
            raw_error = 0.0f;
        }

        // 스무딩 (이전값 70% + 현재값 30%)
        float error = prev_error_ * 0.7f + raw_error * 0.3f;
        prev_error_ = error;

        // ── P제어 속도 계산 ───────────────────────────────
        // 좌측휠 = 직진속도 - k*error
        // 우측휠 = -(직진속도 + k*error)
        int left_vel  = BASE_SPEED - static_cast<int>(KP * error);
        int right_vel = -(BASE_SPEED + static_cast<int>(KP * error));
        left_vel  = std::max(-300, std::min(300, left_vel));
        right_vel = std::max(-300, std::min(300, right_vel));

        // ── 결과 영상 시각화 ──────────────────────────────
        cv::Mat result = frame.clone();

        // 전방/후방 경계선 (파란색 가로선)
        cv::line(result, cv::Point(0, CY),
                 cv::Point(IMG_SIZE, CY),
                 cv::Scalar(255, 0, 0), 1);

        // 좌/우 경계선 (초록색 세로선)
        cv::line(result, cv::Point(CX, 0),
                 cv::Point(CX, CY),
                 cv::Scalar(0, 255, 0), 1);

        // 좌측 최단거리 장애물 (초록 원)
        if (left_found)
            cv::circle(result, cv::Point(left_min_x, left_min_y),
                       8, cv::Scalar(0, 255, 0), -1);

        // 우측 최단거리 장애물 (파란 원)
        if (right_found)
            cv::circle(result, cv::Point(right_min_x, right_min_y),
                       8, cv::Scalar(255, 0, 0), -1);

        // 중앙점 계산 및 표시 (노란 원) - 위쪽 끝에 표시
        int mid_x = CX;   // 기본값: 정면
        int mid_y = 10;   // 위쪽 끝 고정

        if (left_found && right_found) {
            mid_x = (left_min_x + right_min_x) / 2;
            cv::circle(result, cv::Point(mid_x, mid_y),
                       6, cv::Scalar(0, 255, 255), -1);
        }

        // ── 나아가야 할 방향 화살표 ──────────────────────
        // 시작점: 라이다 중심
        cv::Point arrow_start(CX, CY);

        // 노란점 방향으로 고정 길이 80픽셀 화살표
        float dx  = static_cast<float>(mid_x - CX);
        float dy  = static_cast<float>(mid_y - CY);
        float len = std::sqrt(dx * dx + dy * dy);

        int arrow_len = 80;
        cv::Point arrow_end;

        if (len > 0) {
            // 방향은 노란점, 길이는 80픽셀로 고정
            arrow_end.x = CX + static_cast<int>(arrow_len * dx / len);
            arrow_end.y = CY + static_cast<int>(arrow_len * dy / len);
        } else {
            // 장애물 없음 → 정면
            arrow_end = cv::Point(CX, CY - arrow_len);
        }

        cv::Scalar arrow_color = g_mode
            ? cv::Scalar(0, 0, 255)
            : cv::Scalar(150, 150, 150);

        cv::arrowedLine(result,
                        arrow_start,
                        arrow_end,
                        arrow_color,
                        3, cv::LINE_AA, 0, 0.3);

        // ── 텍스트 표시 ──────────────────────────────────
        cv::putText(result,
                    "error: " + std::to_string(
                        static_cast<int>(error)) + "deg",
                    cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 0), 2);

        cv::putText(result,
                    "L: " + std::to_string(left_vel) +
                    "  R: " + std::to_string(right_vel),
                    cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 0), 2);

        cv::putText(result,
                    g_mode ? "RUNNING" : "STOPPED",
                    cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    g_mode ? cv::Scalar(0, 200, 0)
                           : cv::Scalar(0, 0, 200),
                    2);

        cv::imshow("lidarsim", result);
        cv::waitKey(1);

        if (writer_.isOpened())
            writer_.write(result);

        // mode에 따라 속도명령 전송
        if (g_mode)
            sendVelCmd(left_vel, right_vel);
        else
            sendVelCmd(0, 0);

        printf("error=%.1fdeg  L=%d  R=%d  mode=%s\n",
               error, left_vel, right_vel,
               g_mode ? "RUN" : "STOP");
    }

    void sendVelCmd(int left_vel, int right_vel)
    {
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = static_cast<double>(left_vel);
        msg.y = static_cast<double>(right_vel);
        pub_->publish(msg);
    }

    // 멤버 변수
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    cv::VideoWriter  writer_;
    float prev_error_;   // 이전 프레임 error 값 (degree)
};

int main(int argc, char **argv)
{
    signal(SIGINT, sigHandler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSim>();

    std::thread kb_thread(keyboardThread);
    kb_thread.detach();

    while (rclcpp::ok() && g_running) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10));
    }

    cv::destroyAllWindows();
    return 0;
}
