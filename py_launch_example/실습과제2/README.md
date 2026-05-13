# linetracer_sim

ROS2 Jazzy 기반 라인트레이서 시뮬레이션 패키지

---

## 패키지 구조
```
linetracer_sim/
├── src/
│   ├── video_publisher.cpp    # 동영상 발행 노드
│   └── linetracer_sim.cpp     # 라인검출 + P제어 노드
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 알고리즘 설명

### 자율주행 3대 기능
```
인지 → 판단 → 제어
```
- **인지**: 카메라 영상에서 라인 위치 검출
- **판단**: 라인 위치오차(error) 계산
- **제어**: P제어로 좌우 바퀴 속도명령 계산

---

## 코드 설명

### 1. video_publisher.cpp

동영상 파일을 읽어 ROS2 Image 토픽으로 발행하는 노드
```
Mp4파일 → VideoCapture → sensor_msgs/Image → "video1" 토픽
```

- 발행 주기: 33ms (30fps)
- 토픽명: `/video1`
- 동영상 종료 시 자동 반복

### 2. linetracer_sim.cpp

라인을 검출하고 P제어로 속도명령을 계산하는 노드

#### 전처리
1. **ROI 선정**: 입력영상(640x360) 하단 90px를 관심영역으로 선정
2. **밝기 보정**: 평균 밝기를 140으로 보정 (조명 변화 대응)
3. **그레이스케일 변환**: BGR → Gray
4. **이진화**: 임계값 140으로 흰색 라인 분리

#### 라인 검출
- `findContours`로 라인 후보 영역 검출
- 노이즈 제거 기준:
  - 면적 < 60 제거
  - width < 3 또는 height < 5 제거
  - height/width 비율 < 0.10 제거
  - ROI 상단 30% 이내 제거
- 이전 위치에서 가장 가까운 후보를 라인으로 선택

#### 위치오차 계산
```
error = 영상 중심 x좌표 - 라인 무게중심 x좌표
error > 0 : 라인이 왼쪽에 있음 → 좌회전 필요
error < 0 : 라인이 오른쪽에 있음 → 우회전 필요
error = 0 : 라인이 중앙에 있음 → 직진
```

#### P제어 속도명령 계산
```
좌측휠 속도 = 직진속도 - k * error
우측휠 속도 = -(직진속도 + k * error)
```

#### 설계변수
| 변수 | 기본값 | 설명 |
|------|--------|------|
| base_speed | 100 rpm | 직진속도, error=0일 때 속도 |
| k | 0.5 | P제어 게인, 반응속도 조절 |
| min_speed | 50 rpm | 안쪽 바퀴 최소속도 |

#### 가감속 처리
- 모터 스레드 (50ms 주기)
- 목표속도까지 ±5rpm씩 증가/감소
- 급격한 속도 변화 방지

---

## 실행 방법
```bash
# 환경변수 설정
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 터미널 1 - 동영상 발행
ros2 run linetracer_sim video_publisher

# 터미널 2 - 라인 검출 + 속도 계산
ros2 run linetracer_sim linetracer_sim
```

### 키 조작
| 키 | 기능 |
|----|------|
| s | 주행 시작 |
| q | 정지 |

---

## 터미널 출력
```
err:-10, lvel:95, rvel:-105, time:3.50msec
err:-5,  lvel:97, rvel:-102, time:3.20msec
err:0,   lvel:100, rvel:-100, time:3.10msec
err:5,   lvel:102, rvel:-97, time:3.30msec
err:10,  lvel:105, rvel:-95, time:3.40msec
```

- `err`: 위치오차 (영상 중심 - 라인 중심)
- `lvel`: 좌측휠 현재 속도 (rpm)
- `rvel`: 우측휠 현재 속도 (rpm)
- `time`: 영상 1장 처리 시간 (msec, 30msec 이하 목표)

---

## 게인 튜닝 결과

### k = 0.1 (게인 너무 작음)
- 반응이 느려서 커브 구간에서 라인 이탈
- error가 커도 속도 변화가 작음

### k = 0.5 (적정 게인)
- 직선 구간: 안정적으로 라인 추종
- 커브 구간: 라인을 잘 따라감
- 진동 없음

### k = 2.0 (게인 너무 큼)
- 좌우 진동 심함
- error가 작아도 과도한 속도 변화 발생

### base_speed = 150, k = 1.0 (고속)
- 속도 증가 시 k도 같이 증가해야 함
- k가 작으면 커브에서 이탈

---

## 디스플레이

| 색상 | 의미 |
|------|------|
| 파란색 박스 | 모든 라인 후보 |
| 빨간색 박스 | 선택된 라인 |
| 빨간색 점 | 라인 무게중심 |

---

## 토픽 구조
```
video_publisher  →  /video1  →  linetracer_sim
                                      ↓
                               /vel_cmd (Int32MultiArray)
                               data[0]: 좌측휠 속도
                               data[1]: 우측휠 속도
```
