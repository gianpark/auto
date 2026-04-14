#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <csignal>
#include <thread>
#include <termios.h>
#include <unistd.h>

// 이미지 크기 (픽셀): 500x500
#define IMG_SIZE    500
// 실제 세계 크기: 3mx3m (반경 1.5m 영역)
#define WORLD_SIZE  3.0f
// 1m당 픽셀 수
#define M2PIX       (IMG_SIZE / WORLD_SIZE)
// 이미지 중심 좌표 (라이다 위치)
#define CX          (IMG_SIZE / 2)   // 250
#define CY          (IMG_SIZE / 2)   // 250
// P제어 게인
#define KP          1
// 기본 직진 속도
#define BASE_SPEED  50
// 좌측 바퀴 보정값
#define LEFT_OFFSET  0
// 우측 바퀴 보정값
#define RIGHT_OFFSET 0
// 근거리 블라인드존 (50cm = 125픽셀)
#define BLIND_ZONE  125
// 최소 장애물 픽셀 수 (노이즈 제거)
#define MIN_PIXELS  5

// Ctrl+C 종료 플래그
bool g_running = true;
// 주행/정지 모드
bool g_mode    = false;

void sigHandler(int sig)
{
    (void)sig;
    g_running = false;
    rclcpp::shutdown();
}

// 키보드 입력 스레드 (S: 주행시작, Q: 정지)
void keyboardThread()
{
    printf("S: 주행시작  Q: 정지  Ctrl+C: 종료\n");
    fflush(stdout);

    char ch;
    while (g_running) {
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == 's' || ch == 'S') {
                g_mode = true;
                printf("▶ 주행\n");
                fflush(stdout);
            }
            if (ch == 'q' || ch == 'Q') {
                g_mode = false;
                printf("⏹ 정지\n");
                fflush(stdout);
            }
        }
    }
}

class LidarDrive : public rclcpp::Node
{
public:
    LidarDrive() : Node("lidardrive"), prev_error_(0.0f)
    {
        // /scan 토픽 구독
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                   "scan", rclcpp::SensorDataQoS(),
                   std::bind(&LidarDrive::scanCb, this,
                             std::placeholders::_1));

        // /topic_dxlpub 토픽 퍼블리셔 생성
        pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
                   "topic_dxlpub", 10);

        // 결과 영상 저장용 VideoWriter 초기화
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        writer_.open("lidardrive_result.mp4", fourcc, 10.0,
                     cv::Size(IMG_SIZE, IMG_SIZE));

        if (writer_.isOpened())
            RCLCPP_INFO(this->get_logger(),
                        "lidardrive_result.mp4 저장 시작");

        RCLCPP_INFO(this->get_logger(),
                    "lidardrive 시작. /scan 토픽 대기 중...");
        RCLCPP_INFO(this->get_logger(),
                    "S: 주행시작  Q: 정지  Ctrl+C: 종료");
    }

    ~LidarDrive()
    {
        sendVelCmd(0, 0);
        if (writer_.isOpened()) writer_.release();
        cv::destroyAllWindows();
    }

private:
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (!g_running) return;

        // 총 측정 포인트 수 계산
        int count = static_cast<int>(
            scan->scan_time / scan->time_increment);

        // 흰색 배경 이미지 생성
        cv::Mat img(IMG_SIZE, IMG_SIZE, CV_8UC3,
                    cv::Scalar(255, 255, 255));

        // 중심 십자가 표시
        cv::drawMarker(img, cv::Point(CX, CY),
                       cv::Scalar(0, 0, 0),
                       cv::MARKER_CROSS, 10, 1);

        // 각 측정 포인트를 이미지에 빨간 점으로 그리기
        for (int i = 0; i < count; i++) {

            // i번째 측정 각도 계산 (rad)
            float angle_rad = scan->angle_min
                              + scan->angle_increment
                              * static_cast<float>(i);

            // i번째 거리값 (m)
            float range = scan->ranges[i];

            // 유효하지 않은 값 제외
            if (!std::isfinite(range) ||
                range < scan->range_min ||
                range > scan->range_max)
                continue;

            // 반경 1.5m 이내 장애물만 표시
            if (range > WORLD_SIZE / 2.0f)
                continue;

            // 극좌표 → 이미지 픽셀 좌표 변환
            int px = static_cast<int>(
                CX + range * M2PIX * std::sin(angle_rad));
            int py = static_cast<int>(
                CY + range * M2PIX * std::cos(angle_rad));

            // 이미지 범위 벗어난 포인트 제외
            if (px < 0 || px >= IMG_SIZE ||
                py < 0 || py >= IMG_SIZE)
                continue;

            // 장애물 위치에 빨간 점 그리기
            cv::circle(img, cv::Point(px, py), 2,
                       cv::Scalar(0, 0, 255), -1);
        }

        // ── 장애물 검출 ───────────────────────────────────

        // 빨간색 픽셀 마스크 생성
        cv::Mat mask;
        cv::inRange(img,
                    cv::Scalar(0, 0, 200),
                    cv::Scalar(50, 50, 255),
                    mask);

        // 전방 영역 (근거리 블라인드존 제외)
        int front_height = CY - BLIND_ZONE;
        if (front_height <= 0) front_height = 10;

        cv::Mat front_mask = mask(cv::Rect(0, 0, IMG_SIZE, front_height));

        // 좌측 영역 (x: 0 ~ CX)
        cv::Mat left_mask  = front_mask(cv::Rect(0, 0, CX, front_height));
        // 우측 영역 (x: CX ~ IMG_SIZE)
        cv::Mat right_mask = front_mask(cv::Rect(CX, 0, CX, front_height));

        // 좌측 최단거리 장애물 검출
        int left_min_dist    = INT_MAX;
        int left_min_x       = -1;
        int left_min_y       = -1;
        int left_pixel_count = 0;

        for (int y = 0; y < front_height; y++) {
            for (int x = 0; x < left_mask.cols; x++) {
                if (left_mask.at<uchar>(y, x) > 0) {
                    left_pixel_count++;
                    int dx   = x - CX;
                    int dy   = y - CY;
                    int dist = dx * dx + dy * dy;
                    if (dist < left_min_dist) {
                        left_min_dist = dist;
                        left_min_x    = x;
                        left_min_y    = y;
                    }
                }
            }
        }

        // 우측 최단거리 장애물 검출
        int right_min_dist    = INT_MAX;
        int right_min_x       = -1;
        int right_min_y       = -1;
        int right_pixel_count = 0;

        for (int y = 0; y < front_height; y++) {
            for (int x = 0; x < right_mask.cols; x++) {
                if (right_mask.at<uchar>(y, x) > 0) {
                    right_pixel_count++;
                    int real_x = x + CX;
                    int dx     = real_x - CX;
                    int dy     = y - CY;
                    int dist   = dx * dx + dy * dy;
                    if (dist < right_min_dist) {
                        right_min_dist = dist;
                        right_min_x    = real_x;
                        right_min_y    = y;
                    }
                }
            }
        }

        // 픽셀 수 적으면 노이즈로 판단 → 무시
        if (left_pixel_count  < MIN_PIXELS) {
            left_min_x  = -1;
            left_min_y  = -1;
        }
        if (right_pixel_count < MIN_PIXELS) {
            right_min_x = -1;
            right_min_y = -1;
        }

        // ── error 계산 (각도, degree) ─────────────────────
        float raw_error = 0.0f;

        bool left_found  = (left_min_x  != -1);
        bool right_found = (right_min_x != -1);

        // 좌우 장애물이 실제로 다른 벽인지 확인
        bool both_valid = left_found && right_found &&
                          (right_min_x - left_min_x) > 100 &&
                          left_min_dist  < right_min_dist * 3 &&
                          right_min_dist < left_min_dist  * 3;

        if (both_valid) {
            // 양쪽 장애물 → 중앙점 방향 각도 계산
            int mid_x = (left_min_x + right_min_x) / 2;
            float dx  = static_cast<float>(mid_x - CX);
            float dy  = static_cast<float>(CY - IMG_SIZE / 4);
            raw_error = std::atan2(dx, dy) * 180.0f / M_PI;

        } else if (left_found) {
            // 좌측만 장애물 → 이전값 유지
            raw_error = prev_error_;

        } else if (right_found) {
            // 우측만 장애물 → 이전값 유지
            raw_error = prev_error_;

        } else {
            // 장애물 없음 → 0도 직진
            raw_error = 0.0f;
        }

        // 스무딩 (이전값 90% + 현재값 10%)
        float error = prev_error_ * 0.9f + raw_error * 0.1f;
        prev_error_ = error;

        // ── P제어 속도 계산 ───────────────────────────────
        // error > 0: 장애물 중앙 우측 → 우측으로 이동
        // error < 0: 장애물 중앙 좌측 → 좌측으로 이동
        int left_vel  = BASE_SPEED + static_cast<int>(KP * error)
                        + LEFT_OFFSET;
        int right_vel = -(BASE_SPEED - static_cast<int>(KP * error)
                        + RIGHT_OFFSET);

        // 속도 범위 제한 (-300 ~ 300)
        left_vel  = std::max(-300, std::min(300, left_vel));
        right_vel = std::max(-300, std::min(300, right_vel));

        // ── 결과 영상 시각화 ──────────────────────────────
        cv::Mat result = img.clone();

        // 전방/후방 경계선 (파란색 가로선)
        cv::line(result, cv::Point(0, CY),
                 cv::Point(IMG_SIZE, CY),
                 cv::Scalar(255, 0, 0), 1);

        // 블라인드존 경계선 (회색 가로선)
        cv::line(result, cv::Point(0, CY - BLIND_ZONE),
                 cv::Point(IMG_SIZE, CY - BLIND_ZONE),
                 cv::Scalar(150, 150, 150), 1);

        // 좌/우 경계선 (초록색 세로선)
        cv::line(result, cv::Point(CX, 0),
                 cv::Point(CX, CY),
                 cv::Scalar(0, 255, 0), 1);

        // 좌측 최단거리 장애물 (초록 원)
        if (left_found)
            cv::circle(result,
                       cv::Point(left_min_x, left_min_y),
                       8, cv::Scalar(0, 255, 0), -1);

        // 우측 최단거리 장애물 (파란 원)
        if (right_found)
            cv::circle(result,
                       cv::Point(right_min_x, right_min_y),
                       8, cv::Scalar(255, 0, 0), -1);

        // 중앙점 (노란 원) - 화면 1/4 지점
        int mid_x = CX;
        int mid_y = IMG_SIZE / 4;

        if (both_valid) {
            mid_x = (left_min_x + right_min_x) / 2;
            cv::circle(result, cv::Point(mid_x, mid_y),
                       6, cv::Scalar(0, 255, 255), -1);
        }

        // ── 나아가야 할 방향 화살표 ──────────────────────
        cv::Point arrow_start(CX, CY);

        float adx = static_cast<float>(mid_x - CX);
        float ady = static_cast<float>(mid_y - CY);
        float len = std::sqrt(adx * adx + ady * ady);

        int arrow_len = 80;
        cv::Point arrow_end;

        if (len > 0) {
            arrow_end.x = CX + static_cast<int>(
                arrow_len * adx / len);
            arrow_end.y = CY + static_cast<int>(
                arrow_len * ady / len);
        } else {
            arrow_end = cv::Point(CX, CY - arrow_len);
        }

        // 화살표 (빨간색 고정)
        cv::arrowedLine(result,
                        arrow_start, arrow_end,
                        cv::Scalar(0, 0, 255),
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

        cv::imshow("lidardrive", result);
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
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr    pub_;
    cv::VideoWriter writer_;
    float prev_error_;
};

int main(int argc, char **argv)
{
    // 터미널 설정: 입력 버퍼링, 에코 끄기
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    signal(SIGINT, sigHandler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarDrive>();

    // 키보드 입력 스레드 시작
    std::thread kb_thread(keyboardThread);
    kb_thread.detach();

    while (rclcpp::ok() && g_running) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10));
    }

    // 터미널 복원
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    cv::destroyAllWindows();
    return 0;
}