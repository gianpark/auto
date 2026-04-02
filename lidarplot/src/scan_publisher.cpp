#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

int main(int argc, char **argv)
{
    // ROS2 초기화
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scan_publisher");

    // /scan 토픽 퍼블리셔 생성
    auto pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
                   "scan", rclcpp::SensorDataQoS());

    // 10Hz 타이머 (100ms마다 토픽 발행)
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&]() {
            sensor_msgs::msg::LaserScan msg;

            // 헤더 설정
            msg.header.stamp    = node->now();
            msg.header.frame_id = "laser";

            // 각도 범위: -180도 ~ +180도 (rad)
            msg.angle_min       = -M_PI;
            msg.angle_max       =  M_PI;
            // 약 719포인트: 2π / 719
            msg.angle_increment = (2.0f * M_PI) / 719.0f;
            msg.time_increment  = 0.1f / 719.0f;
            msg.scan_time       = 0.1f;
            msg.range_min       = 0.15f;
            msg.range_max       = 16.0f;

            // 가짜 장애물 데이터 생성 (719포인트)
            int count = 719;
            msg.ranges.resize(count);

            for (int i = 0; i < count; i++) {
                float angle = msg.angle_min
                              + msg.angle_increment * static_cast<float>(i);

                // 기본: 5m 거리 원형 장애물
                float range = 5.0f;

                // 정면(0도) 근처에 2m 장애물
                if (std::fabs(angle) < 0.3f)
                    range = 2.0f;

                // 우측(+90도) 근처에 3m 장애물
                if (std::fabs(angle - M_PI / 2) < 0.3f)
                    range = 3.0f;

                // 좌측(-90도) 근처에 3m 장애물
                if (std::fabs(angle + M_PI / 2) < 0.3f)
                    range = 3.0f;

                msg.ranges[i] = range;
            }

            pub->publish(msg);
            RCLCPP_INFO(node->get_logger(), "scan published");
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
