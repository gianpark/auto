#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>

class line_subscriber : public rclcpp::Node
{
public:
    line_subscriber()
    : Node("line_detector"), prev_center_x_(-1.0), lost_count_(0)
    {
        // video1 토픽으로부터 Image 메시지를 구독
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video1", rclcpp::QoS(10),
            std::bind(&line_subscriber::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "line_subscriber Node Started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto start = std::chrono::steady_clock::now();
        cv::Mat frame;

        // ROS 이미지를 OpenCV BGR8 포맷으로 변환
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) { return; }
        if (frame.empty()) return;

        int frame_w = frame.cols;
        int frame_h = frame.rows;

        // ========= ROI 영역 설정 =========
        // ROI: 프레임 하단 약 90px만 사용 (노면 라인이 대부분 아래에 있으므로)
        int roi_width = 640;
        int roi_height = 90;
        int roi_x = 0;
        int roi_y = frame_h - roi_height;
        cv::Rect roi_rect(roi_x, roi_y, roi_width, roi_height);
        cv::Mat roi = frame(roi_rect).clone();

        // ========= 밝기 보정 =========
        // ROI 평균 밝기를 target_mean에 맞추기 위해 Shift 적용
        const double target_mean = 140.0;
        double shift = target_mean - cv::mean(roi)[0];
        roi.convertTo(roi, -1, 0.7, shift);

        // ========= 흑백 + 이진화 =========
        cv::Mat gray, binary;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, 140, 255, cv::THRESH_BINARY);

        // 이진화 이미지를 디스플레이용 BGR로 변환
        cv::Mat display;
        cv::cvtColor(binary, display, cv::COLOR_GRAY2BGR);

        // ========= 컨투어 검출 =========
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // prev_center_x 초기화 (첫 프레임)
        if (prev_center_x_ < 0)
            prev_center_x_ = frame_w / 2.0;

        double best_center_x = prev_center_x_;
        double min_dist = 1e9;
        cv::Rect best_rect;
        bool found = false;

        // ========= 각 컨투어 분석 =========
        for (const auto &cnt : contours)
        {
            cv::Rect rect = cv::boundingRect(cnt);
            double area = rect.area();
            int cx = rect.x + rect.width / 2;

            // 모든 컨투어는 파란색 박스로 표시
            cv::rectangle(display, rect, cv::Scalar(255, 0, 0), 2);

            // 컨투어 필터링 기준
            if (area < 60) continue;
            if (rect.width < 3 || rect.height < 5) continue;
            if ((float)rect.height / rect.width < 0.10f) continue;

            // ROI 상단 30% 근처에서 사라지는 잡음 제거
            if (rect.y + rect.height < roi_height * 0.30)
                continue;

            double candidate_x = roi_rect.x + cx;
            double dist = std::abs(candidate_x - prev_center_x_);

            // 가장 이전 중심점과 가까운 후보 컨투어 선택
            if (dist < min_dist) {
                min_dist = dist;
                best_center_x = candidate_x;
                best_rect = rect;
                found = true;
            }
        }

        // ========= 추적 상태 관련 파라미터 =========
        const double max_jump = 160.0;   // 이전 좌표에서 너무 멀리 뛰는 갑작스러운 변화는 무시
        const int max_lost = 5;          // 최대 연속 유실 허용 프레임
        const double max_speed = 12.0;   // 정상 추적 시 1프레임 이동 최대값
        const double reappear_speed = 6.0; // 재등장 시 속도 제한

        // ========= 정상적으로 라인을 찾았을 때 =========
        if (found && min_dist < max_jump)
        {
            lost_count_ = 0;

            // 선택된 컨투어는 빨간색 박스로 표시
            cv::rectangle(display, best_rect, cv::Scalar(0, 0, 255), 2);

            // 중심점을 빨간색 원으로 표시
            cv::circle(display,
                cv::Point(best_center_x - roi_rect.x,
                          best_rect.y + best_rect.height / 2),
                6, cv::Scalar(0, 0, 255), -1);

            double target = best_center_x;
            double dx = target - prev_center_x_;
            double limit = (lost_count_ > 0) ? reappear_speed : max_speed;

            // 이동 폭 제한(너무 빨리 이동하면 흔들림 발생)
            if (std::abs(dx) > limit)
                target = prev_center_x_ + (dx > 0 ? limit : -limit);

            // 지수평활(EMA) 적용 → 부드러운 움직임
            double alpha = 0.3;
            prev_center_x_ = prev_center_x_ * (1.0 - alpha) + target * alpha;
        }
        else
        {
            // ========= 라인을 못 찾는 경우 =========
            lost_count_++;
            if (lost_count_ > max_lost)
                lost_count_ = max_lost;

            // 라인이 화면 위쪽으로 사라졌을 때 방향 판단
            double target_x;

            if (prev_center_x_ < frame_w * 0.3)
            {
                // 왼쪽으로 사라짐 → 아래 오른쪽 3/5 위치로 이동
                target_x = roi_rect.x + (roi_rect.width * 3.0 / 5.0);
            }
            else if (prev_center_x_ > frame_w * 0.7)
            {
                // 오른쪽으로 사라짐 → 아래 왼쪽 1/5 위치로 이동
                target_x = roi_rect.x + (roi_rect.width * 1.0 / 5.0);
            }
            else
            {
                // 중앙 부근에서 사라짐 → 기존 방향 유지
                target_x = prev_center_x_;
            }

            // 부드럽게 보간 이동
            prev_center_x_ = prev_center_x_ * 0.85 + target_x * 0.15;

            // 추정 중심점 표시
            cv::circle(display,
                cv::Point(prev_center_x_ - roi_rect.x, roi_rect.height - 10),
                6, cv::Scalar(0, 0, 255), -1);
        }

        // 중앙 라인 대비 error 계산 (왼쪽 +, 오른쪽 -)
        double error = frame_w / 2.0 - prev_center_x_;

        // 수행 시간 측정
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        RCLCPP_INFO(this->get_logger(),
            "error:%d, time:%.4f sec",
            (int)error, elapsed.count());

        // 영상 디스플레이
        cv::imshow("Original Frame", frame);
        cv::imshow("Binary with Overlay", display);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    double prev_center_x_;  // 이전 프레임에서의 라인 중심 x좌표
    int lost_count_;        // 라인 유실된 연속 프레임 수
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<line_subscriber>());
    rclcpp::shutdown();
    return 0;
}
