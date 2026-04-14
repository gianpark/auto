# lidardrive 패키지

## 개요
RPLIDAR C1의 실시간 `/scan` 토픽을 구독하여 장애물을 회피하면서 자율주행하는 ROS2 패키지.  
좌우 최단거리 장애물을 검출하고 P제어로 다이나믹셀 속도명령을 발행한다.

---

## 시스템 구성

```
[Raspberry Pi5]                      [WSL2]
RPLIDAR C1 → sllidar_node → /scan → lidardrive → /topic_dxlpub → dxl sub → Dynamixel
                                               → PC Monitor
                                               → lidardrive_result.mp4
```

---

## 실행 환경

| 항목 | 내용 |
|------|------|
| OS | WSL2 Ubuntu 24.04 |
| ROS2 | Jazzy |
| 언어 | C++17 |
| 의존성 | rclcpp, sensor_msgs, geometry_msgs, OpenCV |
| 구독 토픽 | `/scan` (`sensor_msgs/msg/LaserScan`) |
| 발행 토픽 | `/topic_dxlpub` (`geometry_msgs/msg/Vector3`) |

---

## 패키지 생성

```bash
cd ~/ros2_ws/src
ros2 pkg create lidardrive --build-type ament_cmake \
    --dependencies rclcpp sensor_msgs geometry_msgs OpenCV
```

---

## 빌드 및 실행

```bash
# 빌드
cd ~/ros2_ws
colcon build --packages-select lidardrive
source install/local_setup.bash

# 실행
export LIBGL_ALWAYS_SOFTWARE=1
ros2 run lidardrive lidardrive
```

---

## 전체 실행 순서

**Pi 터미널 1 — 라이다 퍼블리셔:**
```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run sllidar_ros2 sllidar_node
```

**Pi 터미널 2 — 다이나믹셀 구동:**
```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run dxl sub
```

**WSL2 터미널 — lidardrive 실행:**
```bash
source ~/ros2_ws/install/local_setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 run lidardrive lidardrive
```

---

## 키보드 조작

| 키 | 기능 |
|----|------|
| `S` | 주행 시작 |
| `Q` | 정지 |
| `Ctrl+C` | 종료 |

---

## 상수 정의

```cpp
#define IMG_SIZE    500    // 이미지 크기 (픽셀): 500x500
#define WORLD_SIZE  3.0f   // 실제 세계 크기: 3mx3m (반경 1.5m)
#define M2PIX       (IMG_SIZE / WORLD_SIZE)  // 1m당 픽셀 수
#define CX          (IMG_SIZE / 2)   // 이미지 중심 x = 250
#define CY          (IMG_SIZE / 2)   // 이미지 중심 y = 250
#define KP          2      // P제어 게인 (클수록 방향전환 빠름)
#define BASE_SPEED  70     // 기본 직진 속도 (rpm)
#define LEFT_OFFSET  0     // 좌측 바퀴 보정값
#define RIGHT_OFFSET 10    // 우측 바퀴 보정값 (우측이 느린 경우 양수)
#define BLIND_ZONE  125    // 근거리 블라인드존 (50cm = 125픽셀)
#define MIN_PIXELS  20     // 최소 장애물 픽셀 수 (노이즈 제거)
```

---

## 소스코드 설명 (lidardrive.cpp)

### sigHandler() — Ctrl+C 종료 처리

```cpp
void sigHandler(int sig)
{
    (void)sig;
    g_running = false;   // 루프 종료 플래그
    rclcpp::shutdown();  // ROS2 종료
}
```

---

### keyboardThread() — 키보드 입력 스레드

```cpp
void keyboardThread()
{
    char ch;
    while (g_running) {
        // 표준입력에서 한 글자 읽기
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            if (ch == 's' || ch == 'S') g_mode = true;   // S: 주행 시작
            if (ch == 'q' || ch == 'Q') g_mode = false;  // Q: 정지
        }
    }
}
```

---

### LidarDrive 클래스 생성자

```cpp
LidarDrive() : Node("lidardrive"), prev_error_(0.0f)
{
    // /scan 토픽 구독 (SensorDataQoS: 센서 데이터용 QoS)
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
               "scan", rclcpp::SensorDataQoS(), ...);

    // /topic_dxlpub 퍼블리셔 생성
    // geometry_msgs/Vector3: x=좌바퀴, y=우바퀴
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
               "topic_dxlpub", 10);

    // 결과 영상 저장 (10fps, mp4v 코덱)
    writer_.open("lidardrive_result.mp4", fourcc, 10.0,
                 cv::Size(IMG_SIZE, IMG_SIZE));
}
```

---

### scanCb() — /scan 토픽 수신 콜백

#### 1단계: 스캔 영상 생성

```cpp
// 총 측정 포인트 수 계산
// count = scan_time / time_increment
int count = static_cast<int>(scan->scan_time / scan->time_increment);

// 극좌표 → 이미지 픽셀 좌표 변환
// px = CX + range * M2PIX * sin(angle)  ← 좌우 성분
// py = CY + range * M2PIX * cos(angle)  ← 상하 성분
int px = static_cast<int>(CX + range * M2PIX * std::sin(angle_rad));
int py = static_cast<int>(CY + range * M2PIX * std::cos(angle_rad));

// 반경 1.5m 이내 장애물만 표시
if (range > WORLD_SIZE / 2.0f) continue;
```

#### 2단계: 장애물 검출

```cpp
// 빨간색 픽셀(장애물) 마스크 생성
cv::inRange(img,
            cv::Scalar(0, 0, 200),    // 하한값 BGR
            cv::Scalar(50, 50, 255),  // 상한값 BGR
            mask);

// 전방 영역 (근거리 블라인드존 50cm 제외)
// 너무 가까운 장애물은 인식하지 않음
int front_height = CY - BLIND_ZONE;   // 250 - 125 = 125픽셀
cv::Mat front_mask = mask(cv::Rect(0, 0, IMG_SIZE, front_height));

// 좌측 영역 (x: 0 ~ CX)
cv::Mat left_mask  = front_mask(cv::Rect(0, 0, CX, front_height));
// 우측 영역 (x: CX ~ IMG_SIZE)
cv::Mat right_mask = front_mask(cv::Rect(CX, 0, CX, front_height));
```

#### 3단계: 최단거리 장애물 검출

```cpp
// 중심(CX, CY)에서 가장 가까운 빨간 픽셀 검출
// dist = dx^2 + dy^2 (유클리드 거리의 제곱)
int dx   = x - CX, dy = y - CY;
int dist = dx * dx + dy * dy;

// 픽셀 수가 MIN_PIXELS 미만이면 노이즈로 판단 → 무시
if (left_pixel_count  < MIN_PIXELS) left_min_x  = -1;
if (right_pixel_count < MIN_PIXELS) right_min_x = -1;
```

#### 4단계: 양쪽 벽 유효성 검사

```cpp
// 같은 벽을 양쪽에서 인식하는 문제 방지
// x 차이 > 100    : 서로 다른 위치에 있음
// 거리 차이 < 3배 : 비슷한 거리에 있어야 다른 벽
bool both_valid = left_found && right_found &&
                  (right_min_x - left_min_x) > 100 &&
                  left_min_dist  < right_min_dist * 3 &&
                  right_min_dist < left_min_dist  * 3;
```

#### 5단계: error 계산 (각도, degree)

```cpp
// 중앙=0도, 왼쪽=-90도, 오른쪽=+90도
if (both_valid) {
    // 양쪽 장애물 → 중앙점 방향 각도 계산
    int mid_x = (left_min_x + right_min_x) / 2;
    float dx  = static_cast<float>(mid_x - CX);
    float dy  = static_cast<float>(CY - IMG_SIZE / 4);
    raw_error = std::atan2(dx, dy) * 180.0f / M_PI;

} else if (left_found || right_found) {
    // 한쪽만 장애물 → 이전값 유지 (급격한 변화 방지)
    raw_error = prev_error_;

} else {
    // 장애물 없음 → 0도 직진
    raw_error = 0.0f;
}

// 스무딩 (이전값 90% + 현재값 10%)
float error = prev_error_ * 0.9f + raw_error * 0.1f;
prev_error_ = error;
```

| 상황 | error |
|------|-------|
| 정면 직진 | 0도 |
| 우측 45도 장애물 | +45도 |
| 좌측 45도 장애물 | -45도 |
| 우측 90도 장애물 | +90도 |
| 좌측 90도 장애물 | -90도 |

#### 6단계: P제어 속도 계산

```cpp
// error > 0: 장애물 중앙 우측 → 우측으로 이동
//   좌바퀴 빠르게, 우바퀴 느리게
// error < 0: 장애물 중앙 좌측 → 좌측으로 이동
//   좌바퀴 느리게, 우바퀴 빠르게
int left_vel  = BASE_SPEED + static_cast<int>(KP * error) + LEFT_OFFSET;
int right_vel = -(BASE_SPEED - static_cast<int>(KP * error) + RIGHT_OFFSET);

// 속도 범위 제한 (-300 ~ 300)
left_vel  = std::max(-300, std::min(300, left_vel));
right_vel = std::max(-300, std::min(300, right_vel));
```

#### 7단계: 시각화

```cpp
// 파란색 가로선  : 전방/후방 경계 (파란선)
// 회색 가로선   : 블라인드존 경계 (50cm)
// 초록색 세로선  : 좌/우 경계
// 초록 원       : 좌측 최단거리 장애물
// 파란 원       : 우측 최단거리 장애물
// 노란 원       : 좌우 장애물 중앙점 (화면 1/4 지점)
// 빨간 화살표   : 나아가야 할 방향
```

#### 8단계: 속도명령 발행

```cpp
// geometry_msgs/Vector3
// x = 좌바퀴 속도 (rpm)
// y = 우바퀴 속도 (rpm)
if (g_mode)
    sendVelCmd(left_vel, right_vel);  // 주행
else
    sendVelCmd(0, 0);                 // 정지
```

---

## 전체 처리 흐름

```
/scan 토픽 수신
       ↓
극좌표 → 픽셀 좌표 변환 → 스캔 영상 생성
       ↓
빨간색 픽셀(장애물) 검출
       ↓
전방 180도 → 블라인드존 제외 → 좌/우 영역 분리
       ↓
각 영역 최단거리 장애물 검출 (MIN_PIXELS 이상만)
       ↓
양쪽 벽 유효성 검사 (같은 벽 인식 방지)
       ↓
중앙점 방향 각도 계산 → error (degree)
       ↓
스무딩 (이전값 90% + 현재값 10%)
       ↓
P제어: left_vel  = BASE + KP * error + LEFT_OFFSET
       right_vel = -(BASE - KP * error + RIGHT_OFFSET)
       ↓
시각화 (화살표, 텍스트, 원, 경계선)
       ↓
/topic_dxlpub 토픽 발행 → 다이나믹셀 구동
       ↓
lidardrive_result.mp4 저장
```

---

## 파라미터 조절 가이드

| 파라미터 | 기본값 | 효과 |
|---------|--------|------|
| `BASE_SPEED` | 70 | 직진 속도 (낮출수록 느림) |
| `KP` | 2 | 방향전환 속도 (높을수록 빠름, 진동 주의) |
| `LEFT_OFFSET` | 0 | 좌측 바퀴 보정 |
| `RIGHT_OFFSET` | 10 | 우측 바퀴 보정 |
| `WORLD_SIZE` | 3.0f | 라이다 인식 범위 (반경 1.5m) |
| `BLIND_ZONE` | 125 | 근거리 제외 거리 (125픽셀 = 50cm) |
| `MIN_PIXELS` | 20 | 최소 장애물 크기 (작을수록 민감) |

---

## 실행 결과

| 항목 | 내용 |
|------|------|
| 화면 출력 | 스캔영상 + 장애물 표시 + 방향 화살표 |
| 터미널 출력 | `error=XX.Xdeg  L=XXX  R=-XXX  mode=RUN/STOP` |
| 저장 파일 | `lidardrive_result.mp4` |
