# Unity–ROS 2–Python 기반 UAV Multi-Sensor Real-Time Framework

> **프로젝트 상태: 진행 중**
>
> Real-Time System Integration | Sensor Fusion | State Estimation

Unity 6에서 생성되는 UAV의 Camera·IMU·state 데이터를 ROS 2로 전달하고, Python 기반 인식·센서융합·상태추정 알고리즘을 **frame 단위로 실행**하기 위한 실시간 통합 프레임워크입니다.

## 1. 프로젝트 개요

기존 연구에서는 Unity에서 센서 데이터를 수집한 뒤 파일 단위로 Python 후처리를 수행했습니다. 이 프로젝트는 검증된 데이터 생성·좌표변환·센서융합 과정을 ROS 2 node로 전환해, 각 sensor frame이 생성되는 시점에 처리되는 구조로 확장합니다.

```text
[기존 구현]
Unity 데이터 생성 → CSV/Image 저장 → Python 후처리 → 성능평가

[현재 통합 중]
Unity sensor frame → ROS 2 → Python perception/localization/fusion → 실시간 평가
```

## 2. 현재 구현된 기능

### Unity UAV–기지국 시뮬레이터

- Unity 6 기반 도시 환경 UAV–기지국 시나리오
- LTI 및 non-linear time-varying UAV trajectory
- 이동속도 변화, 자세 jitter, Gaussian noise 모델
- Camera image와 UAV pose·velocity·attitude 생성
- Camera FoV 및 occlusion을 반영한 YOLO label 생성

### 영상 인식과 좌표변환

- YOLO fine-tuning 및 기지국 bounding box 추론
- bounding box 중심과 Camera FoV를 이용한 camera-local bearing 계산
- Camera local → UAV local → World frame 회전변환
- 기지국 방향벡터와 azimuth/elevation 추정

### Sensor Fusion과 평가

- 기하학적 방향 추정값과 Camera bearing을 결합한 residual FNN
- IMU 자세 시퀀스를 처리하는 CNN-LSTM 기반 회전 보정
- 방향 오차 및 antenna 크기별 beamforming efficiency 평가
- LTI/N-LTV 시나리오별 성능 비교

## 3. 현재 구현 중인 기능

- Unity ↔ ROS 2 TCP 연결과 frame 단위 message 전달
- Camera·IMU·UAV state publisher
- Python perception·fusion node와 Unity 반환 경로
- timestamp, sequence, coordinate frame, QoS 규약
- Camera–IMU 기반 MSCKF localization
- end-to-end latency, frame 누락률, ATE/RPE 평가

## 4. 시스템 구조

```text
Unity 6
 ├─ UAV Dynamics
 ├─ Camera Sensor
 ├─ IMU Sensor
 └─ Ground-Truth State
          │
          │ ROS-TCP
          ▼
ROS 2
 ├─ unity_uav_bridge
 ├─ yolo_perception
 ├─ msckf_localization
 ├─ direction_fusion
 └─ evaluation
```

세부 설계는 [시스템 아키텍처](docs/ARCHITECTURE.md)에서 확인할 수 있습니다.

## 5. 직접 기여

- Unity 기반 UAV–기지국 시뮬레이터와 동적 시나리오 개발
- Camera/IMU 및 UAV state 데이터 생성
- YOLO 학습·추론 파이프라인 구현
- Camera·UAV·World 좌표계 변환
- 기지국 방향 추정과 beam alignment 성능평가
- FNN·CNN-LSTM 기반 sensor fusion
- 실험 설계, 성능평가 및 논문 작성
- ROS 2 양방향 데이터 흐름과 MSCKF localization 통합 설계·구현(진행 중)

## 6. 주요 입출력

| 구성요소 | 입력 | 출력 |
|---|---|---|
| Unity simulation | trajectory·noise·sensor 설정 | Camera frame, IMU, UAV ground truth |
| YOLO perception | Camera frame | Bounding box, confidence, camera bearing |
| Coordinate transform | Camera bearing, UAV/Camera attitude | World-frame direction |
| MSCKF localization | Timestamped Camera·IMU | UAV pose, velocity, covariance |
| Neural fusion | Geometry·perception·state estimate | 보정 방향벡터, azimuth/elevation |
| Evaluation | Estimate, ground truth, timing trace | 각도오차, ATE/RPE, latency, drop rate |

## 7. 기술 스택

- **Simulation:** Unity 6, C#
- **Middleware:** ROS 2, ROS-TCP-Connector, ROS-TCP-Endpoint
- **Perception/Fusion:** Python, PyTorch, TensorFlow, Ultralytics YOLO
- **Estimation:** EKF, MSCKF, quaternion/rotation matrix
- **Evaluation:** NumPy, pandas, Matplotlib
- **Development:** Windows, WSL2, Ubuntu, VS Code, Git

## 8. 구현 로드맵

- [x] Unity UAV 동적 시나리오
- [x] Camera/IMU/UAV state dataset 생성
- [x] YOLO 학습·추론
- [x] Camera–UAV–World 좌표변환
- [x] 신경망 기반 방향 보정
- [x] Offline 성능평가
- [ ] ROS 2 sensor message 정의
- [ ] Unity → ROS 2 frame streaming
- [ ] ROS 2 → Unity 결과 반환
- [ ] MSCKF localization
- [ ] 실시간 통합시험 및 benchmark

## 9. 관련 연구 성과

- J. Kwon, J. Jung, et al., “A Multi-Modal Simulator for Aerial Communication with Applications to Beam Search,” ICTC 2025. **(제1저자)**
- J. Jung, J. Kwon, et al., “A Multi-Sensor Simulator for UAV Localization: Kalman Filter-Based Approach,” ICTC 2025.

## 10. 공개 범위

이 저장소는 직접 작성한 코드와 재현 가능한 최소 예제만 공개합니다. 상용 Unity asset, 외부 데이터셋, 대용량 학습 산출물 및 개인 개발환경 정보는 포함하지 않습니다.
