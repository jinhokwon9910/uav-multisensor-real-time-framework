# 시스템 아키텍처

[← 프로젝트 README로 돌아가기](../README.md)

> 이 문서는 ROS 2 통합 과정에서 구현 결과에 맞춰 갱신합니다.

## 1. 설계 목표

- Unity에서 생성되는 sensor frame의 source timestamp 보존
- Camera·IMU·ground truth의 sampling rate 분리
- 좌표계 변환을 명시적 frame tree로 관리
- perception, localization, fusion을 독립 ROS 2 node로 분리
- frame sequence와 timestamp를 이용한 latency·누락률 측정

## 2. 구성요소

| 구성요소 | 역할 |
|---|---|
| Unity UAV Simulator | UAV dynamics, Camera/IMU, ground truth 생성 |
| Unity ROS Bridge | ROS message 변환 및 송수신 |
| YOLO Perception Node | 기지국 검출과 camera bearing 생성 |
| MSCKF Localization Node | Camera–IMU 기반 pose·velocity 추정 |
| Direction Fusion Node | 기하 추정, localization, 신경망 보정 결합 |
| Evaluation Node | ground truth 비교와 frame 단위 성능 측정 |

## 3. Topic 초안

| Topic | Message | Producer → Consumer |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Unity → ROS 2 nodes |
| `/uav/imu/data` | `sensor_msgs/Imu` | Unity → MSCKF |
| `/uav/camera/image` | `sensor_msgs/Image` | Unity → YOLO, MSCKF |
| `/uav/camera/camera_info` | `sensor_msgs/CameraInfo` | Unity → YOLO, MSCKF |
| `/uav/ground_truth/odom` | `nav_msgs/Odometry` | Unity → Evaluation |
| `/perception/bearing` | custom message | YOLO → Fusion |
| `/localization/msckf/odom` | `nav_msgs/Odometry` | MSCKF → Fusion, Evaluation |
| `/fusion/direction` | custom message | Fusion → Evaluation, Unity |

## 4. Coordinate Frame 초안

```text
map
 └─ uav/base_link
     ├─ uav/imu_link
     └─ uav/camera_link
         └─ uav/camera_optical_frame
```

- `map`: ROS ENU 기준 world frame
- `uav/base_link`: UAV body frame
- `uav/imu_link`: IMU measurement frame
- `uav/camera_optical_frame`: 영상처리용 optical frame

Unity와 ROS의 handedness 차이는 단일 변환 모듈에서 처리하고, unit test로 basis vector와 quaternion 변환을 검증합니다.

## 5. 처리 단위

모든 파생 결과는 최초 sensor frame의 timestamp와 sequence ID를 유지합니다.

```text
capture
  → ROS bridge transmit
  → node receive
  → perception/localization complete
  → fusion complete
  → result receive
```

각 시점을 기록해 transport latency, processing latency, end-to-end latency와 dropped frame을 구분합니다.

## 6. 검증 항목

- timestamp 역행 및 sequence 누락 여부
- Unity–ROS 좌표변환 round-trip error
- Camera/IMU synchronization tolerance
- node별 처리시간 p50/p95
- MSCKF ATE/RPE 및 orientation RMSE
- 장시간 실행 시 message drop과 memory 증가
