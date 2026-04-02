#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

// 이미지 크기 (픽셀): 500x500
#define IMG_SIZE   500
// 실제 세계 크기 (m): 10m x 10m
#define WORLD_SIZE 10.0f
// 1m당 픽셀 수: 500픽셀 / 10m = 50픽셀/m
#define M2PIX      (IMG_SIZE / WORLD_SIZE)
// 이미지 중심 좌표 (라이다 위치)
#define CX         (IMG_SIZE / 2)   // 250
#define CY         (IMG_SIZE / 2)   // 250

// 동영상 저장용 VideoWriter (전역 선언)
cv::VideoWriter g_writer;
bool            g_writer_opened = false;

// /scan 토픽 수신 시 호출되는 콜백 함수
static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // ── 측정 포인트 수 계산 ──────────────────────────────
    // scan_time: 1회전 소요 시간(초)
    // time_increment: 포인트 간 시간 간격(초)
    // count: 1회전당 총 측정 포인트 수
    int count = static_cast<int>(scan->scan_time / scan->time_increment);

    // ── 흰색 배경 이미지 생성 (500x500, BGR 3채널) ───────
    cv::Mat img(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));

    // ── 중심 십자가 표시 (라이다 위치, 검정색) ───────────
    cv::drawMarker(img, cv::Point(CX, CY),
                   cv::Scalar(0, 0, 0),        // 검정색
                   cv::MARKER_CROSS, 10, 1);   // 크기 10, 두께 1

    // ── 각 측정 포인트를 이미지에 빨간 점으로 그리기 ─────
    for (int i = 0; i < count; i++) {

        // i번째 측정 각도 계산 (rad)
        // 공식: angle_min + angle_increment * i
        float angle_rad = scan->angle_min
                          + scan->angle_increment * static_cast<float>(i);

        // i번째 거리값 (m)
        float range = scan->ranges[i];

        // 유효하지 않은 값 제외
        // - inf: 장애물 없음
        // - range_min ~ range_max 범위 밖: 측정 오류
        if (!std::isfinite(range) ||
            range < scan->range_min ||
            range > scan->range_max)
            continue;

        // 극좌표 → 이미지 픽셀 좌표 변환
        // 라이다 좌표계: x축이 위쪽(↑), angle=0 이 정면
        // 이미지 좌표계: 원점이 좌상단, y축이 아래쪽(↓)
        //
        // px = CX + range * M2PIX * sin(angle)  ← 좌우 성분
        // py = CY - range * M2PIX * cos(angle)  ← 상하 성분 (부호 반전)
        int px = static_cast<int>(CX + range * M2PIX * std::sin(angle_rad));
        int py = static_cast<int>(CY - range * M2PIX * std::cos(angle_rad));

        // 이미지 범위(0~499) 벗어난 포인트 제외
        if (px < 0 || px >= IMG_SIZE || py < 0 || py >= IMG_SIZE)
            continue;

        // 장애물 위치에 빨간 점 그리기 (반지름 1픽셀, 채워진 원)
        cv::circle(img, cv::Point(px, py), 1, cv::Scalar(0, 0, 255), -1);
    }

    // ── 이미지 화면 출력 ─────────────────────────────────
    cv::imshow("lidar", img);
    cv::waitKey(1);   // 1ms 대기 (GUI 이벤트 처리)

    // ── VideoWriter 초기화 (첫 프레임에서 한 번만 실행) ──
    if (!g_writer_opened) {
        // FPS: scan_time 기반 계산 (기본값 10fps)
        double fps = (scan->scan_time > 0.0f)
                     ? (1.0 / scan->scan_time) : 10.0;

        // mp4v 코덱으로 lidar_scan.mp4 파일 생성
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        g_writer.open("lidar_scan.mp4", fourcc, fps,
                      cv::Size(IMG_SIZE, IMG_SIZE));

        if (g_writer.isOpened()) {
            g_writer_opened = true;
            RCLCPP_INFO(rclcpp::get_logger("lidarplot"),
                        "VideoWriter opened: lidar_scan.mp4 (%.1f fps)", fps);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("lidarplot"),
                        "VideoWriter open FAILED.");
        }
    }

    // ── 현재 프레임을 동영상 파일에 저장 ─────────────────
    if (g_writer_opened)
        g_writer.write(img);
}

int main(int argc, char **argv)
{
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // "lidarplot" 노드 생성
    auto node = rclcpp::Node::make_shared("lidarplot");

    // /scan 토픽 구독 (SensorDataQoS: 센서 데이터용 QoS)
    auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                   "scan", rclcpp::SensorDataQoS(), scanCb);

    RCLCPP_INFO(node->get_logger(), "lidarplot started. Waiting for /scan ...");

    // 콜백 대기 루프 (Ctrl+C 로 종료)
    rclcpp::spin(node);

    // 종료 처리
    rclcpp::shutdown();

    // VideoWriter 해제 (mp4 파일 정상 저장)
    if (g_writer_opened) g_writer.release();

    // OpenCV 창 닫기
    cv::destroyAllWindows();

    return 0;
}