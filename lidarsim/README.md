# lidarsim 패키지

## 시뮬레이션 영상 : https://youtu.be/qE3hkfbxIKo

## 개요
라이다 스캔 영상(mp4)을 입력으로 장애물 회피 알고리즘을 시뮬레이션하는 ROS2 패키지.  
좌우 최단거리 장애물을 검출하고 P제어로 다이나믹셀 속도명령을 발행한다.

---

## 시스템 구성

```
[lidar_scan.mp4] → lidarsim → /topic_dxlpub → dxl sub → Dynamixel
                            → PC Monitor
                            → lidarsim_result.mp4
```

---

## 실행 환경

| 항목 | 내용 |
|------|------|
| OS | WSL2 Ubuntu 24.04 |
| ROS2 | Jazzy |
| 언어 | C++17 |
| 의존성 | rclcpp, geometry_msgs, OpenCV |
| 토픽 | `/topic_dxlpub` (`geometry_msgs/msg/Vector3`) |

---

## 패키지 생성

```bash
cd ~/ros2_ws/src
ros2 pkg create lidarsim --build-type ament_cmake \
    --dependencies rclcpp sensor_msgs geometry_msgs OpenCV
```

---

## 빌드 및 실행

```bash
# 빌드
cd ~/ros2_ws
colcon build --packages-select lidarsim
source install/local_setup.bash

# 실행 (lidar_scan.mp4가 ~/ros2_ws에 있어야 함)
cd ~/ros2_ws
export LIBGL_ALWAYS_SOFTWARE=1
ros2 run lidarsim lidarsim
```

---

## 키보드 조작

| 키 | 기능 |
|----|------|
| `S` | 주행 시작 |
| `Q` | 정지 |
| `Ctrl+C` | 종료 |

---

## 소스코드 설명 (lidarsim.cpp)

### 상수 정의

```cpp
#define IMG_SIZE   500    // 이미지 크기 (픽셀): 500x500
#define WORLD_SIZE 2.0f   // 실제 세계 크기: 2mx2m (반경 1m)
#define M2PIX      (IMG_SIZE / WORLD_SIZE)  // 1m당 픽셀 수: 250픽셀/m
#define CX         (IMG_SIZE / 2)   // 이미지 중심 x = 250
#define CY         (IMG_SIZE / 2)   // 이미지 중심 y = 250
#define KP         1      // P제어 게인
#define BASE_SPEED 100    // 기본 직진 속도 (rpm)
```

---

### 전역 변수

```cpp
bool g_running = true;   // 노드 실행 플래그 (Ctrl+C로 false)
bool g_mode    = false;  // 주행/정지 모드 (S: true, Q: false)
```

---

### sigHandler() — Ctrl+C 종료 처리

```cpp
void sigHandler(int sig)
{
    (void)sig;
    g_running = false;   // 루프 종료 플래그 설정
    rclcpp::shutdown();  // ROS2 종료
}
```

---

### keyboardThread() — 키보드 입력 스레드

```cpp
void keyboardThread()
{
    // 터미널 설정: 입력 버퍼링, 에코 끄기
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (g_running) {
        int ch = getchar();
        if (ch == 's' || ch == 'S') g_mode = true;   // S: 주행 시작
        if (ch == 'q' || ch == 'Q') g_mode = false;  // Q: 정지
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 터미널 복원
}
```

---

### LidarSim 클래스 생성자

```cpp
LidarSim() : Node("lidarsim"), prev_error_(0.0f)
{
    // /topic_dxlpub 토픽 퍼블리셔 생성
    // geometry_msgs/Vector3: x=좌바퀴, y=우바퀴
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
               "topic_dxlpub", 10);

    // mp4 파일 열기
    cap_.open("lidar_scan.mp4");

    // 결과 영상 저장용 VideoWriter 초기화 (10fps, mp4v 코덱)
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    writer_.open("lidarsim_result.mp4", fourcc, 10.0,
                 cv::Size(IMG_SIZE, IMG_SIZE));

    // 10fps 타이머 (100ms마다 timerCb 호출)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LidarSim::timerCb, this));
}
```

---

### timerCb() — 메인 처리 루프 (100ms마다 실행)

#### 1단계: mp4 프레임 읽기

```cpp
cv::Mat frame;
cap_ >> frame;                                      // mp4에서 프레임 읽기
cv::resize(frame, frame, cv::Size(IMG_SIZE, IMG_SIZE)); // 500x500으로 리사이즈
```

#### 2단계: 장애물 검출

```cpp
// 빨간색 픽셀(장애물) 마스크 생성 (BGR 기준)
cv::Mat mask;
cv::inRange(frame,
            cv::Scalar(0, 0, 200),    // 하한값
            cv::Scalar(50, 50, 255),  // 상한값
            mask);

// 전방 180도 = 이미지 상단 절반 (y: 0 ~ CY)
cv::Mat front_mask = mask(cv::Rect(0, 0, IMG_SIZE, CY));

// 좌측 영역 (x: 0 ~ CX)
cv::Mat left_mask  = front_mask(cv::Rect(0, 0, CX, CY));

// 우측 영역 (x: CX ~ IMG_SIZE)
cv::Mat right_mask = front_mask(cv::Rect(CX, 0, CX, CY));
```

#### 3단계: 최단거리 장애물 검출

```cpp
// 중심(CX, CY)에서 가장 가까운 빨간 픽셀 검출
// dist = dx^2 + dy^2 (유클리드 거리의 제곱)
int dx = x - CX, dy = y - CY;
int dist = dx*dx + dy*dy;
```

#### 4단계: error 계산 (각도, degree)

```cpp
// 파란선(수평선) 기준
// 중앙=0도, 왼쪽=-90도, 오른쪽=+90도
if (left_found && right_found) {
    int mid_x = (left_min_x + right_min_x) / 2;  // 중앙점 x좌표
    float dx = static_cast<float>(mid_x - CX);    // 좌우 거리
    float dy = static_cast<float>(CY - 10);        // 수평선~노란점 거리
    raw_error = std::atan2(dx, dy) * 180.0f / M_PI; // 각도 계산
}

// 스무딩: 이전값 70% + 현재값 30% (급격한 변화 완화)
float error = prev_error_ * 0.7f + raw_error * 0.3f;
prev_error_ = error;
```

| 상황 | error |
|------|-------|
| 정면 직진 | 0도 |
| 우측 45도 | +45도 |
| 좌측 45도 | -45도 |
| 우측 90도 | +90도 |
| 좌측 90도 | -90도 |

#### 5단계: P제어 속도 계산

```cpp
// 강의자료 공식 (26_ROS2_모션제어 14페이지)
// 좌측휠 속도명령 = 직진속도 - k*error
// 우측휠 속도명령 = -(직진속도 + k*error)
int left_vel  = BASE_SPEED - static_cast<int>(KP * error);
int right_vel = -(BASE_SPEED + static_cast<int>(KP * error));

// 속도 범위 제한 (-300 ~ 300)
left_vel  = std::max(-300, std::min(300, left_vel));
right_vel = std::max(-300, std::min(300, right_vel));
```

#### 6단계: 시각화

```cpp
// 전방/후방 경계선 (파란색 가로선)
cv::line(result, cv::Point(0, CY), cv::Point(IMG_SIZE, CY),
         cv::Scalar(255, 0, 0), 1);

// 좌/우 경계선 (초록색 세로선)
cv::line(result, cv::Point(CX, 0), cv::Point(CX, CY),
         cv::Scalar(0, 255, 0), 1);

// 좌측 최단거리 장애물 → 초록 원
// 우측 최단거리 장애물 → 파란 원
// 중앙점 (노란 원) → y=10 위쪽 끝 고정

// 나아가야 할 방향 화살표
// 시작: 라이다 중심(CX, CY)
// 끝: 노란점 방향으로 80픽셀
cv::arrowedLine(result, arrow_start, arrow_end,
                arrow_color, 3, cv::LINE_AA, 0, 0.3);
```

#### 7단계: 속도명령 발행

```cpp
// geometry_msgs/Vector3
// x = 좌바퀴 속도 (rpm)
// y = 우바퀴 속도 (rpm)
msg.x = static_cast<double>(left_vel);
msg.y = static_cast<double>(right_vel);
pub_->publish(msg);
```

---

## 전체 처리 흐름

```
mp4 프레임 읽기
       ↓
빨간색 픽셀(장애물) 검출
       ↓
전방 180도 → 좌/우 영역 분리
       ↓
각 영역 최단거리 장애물 검출
       ↓
중앙점 방향 각도 계산 → error (degree)
       ↓
스무딩 (이전값 70% + 현재값 30%)
       ↓
P제어: left_vel  = BASE - KP * error
       right_vel = -(BASE + KP * error)
       ↓
시각화 (화살표, 텍스트, 원)
       ↓
/topic_dxlpub 토픽 발행 → 다이나믹셀 구동
       ↓
lidarsim_result.mp4 저장
```

---

## 실행 결과

| 항목 | 내용 |
|------|------|
| 화면 출력 | 스캔영상 + 장애물 표시 + 방향 화살표 |
| 터미널 출력 | `error=XX.Xdeg  L=XXX  R=-XXX  mode=RUN/STOP` |
| 저장 파일 | `lidarsim_result.mp4` |
