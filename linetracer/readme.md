라인검출 과제

실습과제 1 영상
https://youtu.be/9R3EVqAXw5Q

실습과제 2 영상
https://youtu.be/r4xM6_mLgQw

# ROS2 Line Detection Package

ROS2 기반의 영상 라인 검출 패키지입니다.
MP4 영상을 프레임 단위로 발행하는 **퍼블리셔**와, 수신된 프레임에서 주행 라인을 검출하는 **서브스크라이버**로 구성됩니다.

---

## 패키지 구성

```
ros2_ws/
├── src/
│   ├── line_publisher/          # 영상 파일 → ROS2 토픽 발행
│   │   └── src/line_publisher.cpp
│   └── line_subscriber/         # 토픽 수신 → 라인 검출
│       └── src/line_subscriber.cpp
└── simulation/
    └── 7_lt_ccw_100rpm_in.mp4   # 테스트용 영상 파일
```

---

## 노드 설명

### 1. `line_publisher` — 영상 퍼블리셔

MP4 파일을 열어 30ms(≈33 FPS) 주기로 프레임을 `/video1` 토픽에 발행합니다.

| 항목 | 내용 |
|------|------|
| 노드명 | `video_publisher` |
| 발행 토픽 | `/video1` |
| 메시지 타입 | `sensor_msgs/msg/Image` |
| 발행 주기 | 30ms (≈ 33 FPS) |
| 영상 포맷 | BGR8 |

영상이 끝나면 `rclcpp::shutdown()`을 호출하여 자동 종료됩니다.

---

### 2. `line_subscriber` — 라인 검출 서브스크라이버

`/video1` 토픽을 구독하여 각 프레임에서 흰색 라인의 중심 x좌표를 추적하고, 화면 중앙 대비 오차(error)를 출력합니다.

| 항목 | 내용 |
|------|------|
| 노드명 | `line_detector` |
| 구독 토픽 | `/video1` |
| 메시지 타입 | `sensor_msgs/msg/Image` |
| 출력 | `error` (픽셀 단위), 처리 시간 |

---

## 라인 검출 알고리즘

```
원본 프레임 수신
    │
    ▼
ROI 추출 (프레임 하단 90px)
    │
    ▼
밝기 보정 (평균 밝기 → 140 목표값으로 Shift)
    │
    ▼
그레이스케일 변환 → 이진화 (threshold: 140)
    │
    ▼
외곽선(Contour) 검출
    │
    ▼
필터링 (면적 < 60, 너비 < 3, 높이 < 5, 종횡비 < 0.1 제거)
    │
    ▼
이전 중심점과 가장 가까운 후보 선택
    │
    ▼
EMA 평활화 (α = 0.3) → 중심점 갱신
    │
    ▼
error = frame_width/2 - center_x 출력
```

### 주요 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `roi_height` | 90 px | ROI 높이 (프레임 하단 기준) |
| `target_mean` | 140.0 | 밝기 보정 목표값 |
| `threshold` | 140 | 이진화 임계값 |
| `max_jump` | 160.0 px | 허용 최대 순간 이동 거리 |
| `max_lost` | 5 frames | 최대 연속 유실 허용 프레임 |
| `max_speed` | 12.0 px/frame | 정상 추적 시 최대 이동 속도 |
| `alpha (EMA)` | 0.3 | 지수평활 계수 (클수록 빠른 반응) |

### 라인 유실 처리

라인을 찾지 못했을 때 이전 위치에서 방향을 추정합니다.

- 이전 중심이 **왼쪽 30% 이내** → 오른쪽(3/5 지점)으로 보간
- 이전 중심이 **오른쪽 70% 이상** → 왼쪽(1/5 지점)으로 보간
- **중앙 부근** → 기존 방향 유지

---

## 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 라이브러리 |
| `sensor_msgs` | Image 메시지 타입 |
| `cv_bridge` | ROS ↔ OpenCV 이미지 변환 |
| `OpenCV` | 영상 처리 (이진화, 컨투어, 디스플레이) |

---

## 빌드 및 실행

```bash
# 1. 워크스페이스로 이동
cd ~/ros2_ws

# 2. ROS2 환경 소싱
source /opt/ros/humble/setup.bash

# 3. 빌드
colcon build --symlink-install

# 4. 빌드 결과 소싱
source install/setup.bash

# 5. 퍼블리셔 실행 (터미널 1)
ros2 run line_publisher line_publisher

# 6. 서브스크라이버 실행 (터미널 2)
ros2 run line_subscriber line_subscriber
```

---

## 출력 예시

### 터미널 로그

```
[INFO] [line_detector]: error:23, time:0.0031 sec
[INFO] [line_detector]: error:-5, time:0.0028 sec
[INFO] [line_detector]: error:11, time:0.0030 sec
```

- `error`: 화면 중앙 대비 라인 중심의 픽셀 오프셋 (양수 = 라인이 왼쪽, 음수 = 라인이 오른쪽)
- `time`: 프레임 1장 처리 시간 (초)

### 디스플레이 창

| 창 이름 | 내용 |
|---------|------|
| `Original Frame` | 원본 프레임 전체 |
| `Binary with Overlay` | 이진화 결과 + 컨투어 박스(파란색) + 선택된 라인(빨간색) |

---

## 라이선스

MIT License
