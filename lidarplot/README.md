# ROS2 LIDAR 실습과제 2

## 과제 목표
RPLIDAR C1의 /scan 토픽 데이터를 분석하여 다음 항목을 조사한다.

---

## 1. RPLIDAR C1 좌표축
```
        x축 (0rad, 정면↓)
            ↓
  좌(-) ← [라이다] → 우(+)

- x축 : 라이다 정면 방향(↓), 0rad 기준
- 우측 : 양수각도 (0 ~ +3.14rad)
- 좌측 : 음수각도 (0 ~ -3.14rad)
- 모터 회전방향 : 시계방향
```

---

## 2. 조사 결과

### 확인 명령어
```bash
# 토픽 전송 주기 확인
ros2 topic hz /scan

# 토픽 메시지 크기 확인
ros2 topic bw /scan

# 토픽 내용 확인
ros2 topic echo /scan
```

### 결과표

| 항목 | 값 | 확인 방법 |
|------|-----|----------|
| 토픽 전송 주기 | **10 Hz** | `ros2 topic hz /scan` |
| 메시지 크기 | **5.82 KB** | `ros2 topic bw /scan` |
| 측정값 개수/메시지 | **약 719개** | `scan_time / time_increment` |
| 1회전 측정 횟수 | **약 719번** | 메시지 1개 = 1회전 |
| angle_min | **-3.1241 rad (-179도)** | `ros2 topic echo /scan` |
| angle_increment | **0.008714 rad (0.508도)** | `ros2 topic echo /scan` |

---

## 3. 측정값 계산 방법

### 측정 포인트 수 계산
```
count = scan_time / time_increment
      = 0.1001 / 0.0001393
      ≈ 719개
```

### i번째 측정 각도 계산
```
angle = angle_min + angle_increment * i
      = -3.1241 + 0.008714 * i  (rad)
```

### 실측 데이터 (ros2 topic echo /scan)
```
angle_min       : -3.1241390705108643 rad
angle_max       :  3.1415927410125732 rad
angle_increment :  0.0087145509196579456 rad
time_increment  :  0.00013931671679131928 s
scan_time       :  0.10016877204179764 s
range_min       :  0.15 m
range_max       :  16.0 m
```

---

## 4. 코드로 확인하는 방법
```cpp
static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // 측정 포인트 수 출력
    int count = static_cast<int>(scan->scan_time / scan->time_increment);
    printf("count          = %d\n", count);

    // angle_min, angle_increment 출력
    printf("angle_min      = %f rad\n", scan->angle_min);
    printf("angle_increment= %f rad\n", scan->angle_increment);

    // 메시지 크기 출력
    printf("ranges size    = %zu\n", scan->ranges.size());
}
```

---

## 5. 실행 환경

| 항목 | 내용 |
|------|------|
| 하드웨어 | RPLIDAR C1 |
| 퍼블리셔 | Raspberry Pi5 (`sllidar_ros2` 패키지) |
| 구독자 | WSL2 Ubuntu 24.04 (`sllidar_client`) |
| ROS2 버전 | Jazzy |
| 토픽 | `/scan` (`sensor_msgs/msg/LaserScan`) |
