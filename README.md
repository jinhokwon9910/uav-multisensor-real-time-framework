# Unity–ROS 2–Python UAV Sensor-Frame Integration

> Unity에서 생성한 UAV Camera·자세 데이터를 파일 후처리가 아니라 **frame 단위 ROS 2 경로**로 연결하고, Python의 빔 방향 결과를 원본 frame과 대응시켜 Unity에 돌려보낸 프로젝트다.

이 프로젝트는 여러 포트폴리오 중 **online heterogeneous-runtime 통합과 sensor-frame 계약**을 보여주는 기술 사례 연구다. 핵심은 특정 AI 모델의 정확도보다 Unity, ROS 2, Linux Python, Windows GPU 환경 사이에서 한 sensor frame의 의미와 처리 결과를 잃지 않는 구조를 설계하고 실제로 왕복시킨 경험에 있다.

## 기술 스택과 책임

| 기술 | 이 프로젝트에서 맡은 책임 |
|---|---|
| Unity 6 · C# | Camera 강제 렌더링, single-message correlated snapshot, sequence 관리, lockstep, timeout, 결과 시각화 |
| ROS-TCP-Connector · Endpoint | Unity의 TCP 연결과 ROS 2 graph 사이의 transport 경계 |
| ROS 2 Jazzy · rclpy · DDS | custom message, topic/QoS, Python callback, source frame 추적 |
| Windows Conda · PyTorch · Ultralytics YOLO | 기존 GPU 추론 환경을 유지한 persistent detector worker |
| Python geometry | bbox 중심의 camera ray 변환, quaternion 합성, world 방향·azimuth·elevation 계산 |
| pytest · ROS graph test | geometry 예외조건과 message roundtrip 검증 |

## Offline 파이프라인을 Online 구조로 확장한 이유

기존 연구 구현은 Unity가 Camera image와 state를 CSV/PNG로 저장하고 Python이 batch 후처리하는 구조였다.

```text
[기존 offline]
Unity Camera/state 생성
  → CSV/PNG 저장
  → YOLO batch inference
  → Camera/UAV/World 좌표변환
  → FNN·CNN-LSTM 방향 보정
  → 성능평가
```

이 구조는 알고리즘 비교에는 적합하지만, 파일 이름과 행 번호에 frame 대응을 의존한다. 데이터 생성과 처리 사이의 지연, 누락, stale 결과, 실패 응답도 실행 중에 관찰하기 어렵다. 이를 보완하기 위해 message-assembly wall-clock stamp, Camera image, 자세, FoV, sequence를 하나의 message로 묶고, Python 결과가 같은 source sequence를 되돌려주는 online 경로를 설계했다.

```text
[현재 online vertical slice]
Unity SensorFrame(N)
  → TCP Endpoint
  → ROS 2 Jazzy Python
  → persistent YOLO
  → bearing/geometry
  → BeamDirectionEstimate(source_sequence=N)
  → Unity lockstep 해제 + cyan ray
```

현재 반환값은 시각화와 통합 검증에만 사용한다. UAV의 `Rigidbody`나 actuator에는 적용하지 않는다.

## 핵심 설계 판단

### 1. TCP와 DDS의 역할을 분리했다

Unity–Endpoint 구간은 ROS-TCP-Connector의 TCP 연결이고, Endpoint–Python node 구간은 ROS 2 DDS graph다. `.msg`는 데이터 계약을 정의하며 transport protocol 자체가 아니다. 연결 구간을 나누어 설명하면 TCP 접속 오류, ROS graph discovery, detector 오류를 서로 다른 문제로 진단할 수 있다.

### 2. WSL ROS와 Windows YOLO runtime을 억지로 합치지 않았다

ROS 2 Jazzy node는 WSL system Python에서 실행하고, 기존 PyTorch·CUDA·Ultralytics 환경은 Windows Conda worker로 유지했다. ROS node는 PNG와 sequence를 binary pipe로 보내며 worker는 model을 한 번만 적재하고 반복 추론한다. 이 분리는 `rclpy` ABI와 기존 GPU environment의 충돌을 피하면서 매 frame model reload도 제거한다. 저장소에는 이 경계를 구성한 [WSL adapter](runtime/windows_conda/ros_beam_estimator_node.py)와 [persistent worker](runtime/windows_conda/conda_yolo_worker.py)를 구현 근거로 남겼다.

### 3. 한 publish cycle을 `SensorFrame` 하나로 고정했다

`SensorFrame`은 compressed PNG, noisy UAV quaternion, camera-local quaternion, vertical FoV, message-assembly timestamp, uint32 sequence를 함께 담는다. 이 값들은 Unity의 한 publish cycle에서 한 message로 조립되며, 서로 독립적으로 저장한 파일을 사후 결합하지 않는다. PNG 렌더링과 Transform sampling은 순차 실행되므로 hardware-level simultaneous acquisition을 주장하지 않는다.

### 4. Timestamp만이 아니라 sequence로 결과 freshness를 판단했다

Python은 입력 `sequence=N`을 결과의 `source_sequence=N`으로 되돌려준다. Unity는 현재 기다리는 sequence와 일치하는 결과만 수락하고 stale 또는 unsolicited 결과를 무시한다. timestamp는 message assembly의 wall-clock context를 보존하지만, 서로 다른 clock을 직접 빼서 freshness를 판단하지 않는다.

### 5. Lockstep은 sequence validation과 함께 사용했다

Python estimator의 publisher와 subscription은 `RELIABLE + VOLATILE + KEEP_LAST(1)`을 사용하고, Unity의 `SensorFrame` publisher도 queue size 1로 등록했다. Unity로 돌아오는 subscription queue는 ROS-TCP Endpoint가 관리하므로 동일한 depth가 end-to-end로 강제된다고 주장하지 않는다. Unity는 frame을 보낸 뒤 `Time.timeScale=0`으로 simulation 진행을 멈추고 같은 sequence의 결과를 기다리므로, backlog가 생겨도 sequence 검증이 결과 freshness의 최종 경계가 된다. 응답시간과 timeout은 scaled game time이 아니라 wall-clock으로 측정해 pause 중에도 진행된다.

### 6. 실패도 반드시 명시적인 결과로 반환했다

정상 검출 외에 `NO_DETECTION`, `INVALID_INPUT`, `INFERENCE_ERROR` status를 정의했다. matching sequence의 실패 응답도 Unity lockstep을 해제하며, 응답 자체가 없을 때는 timeout이 원래 `Time.timeScale`을 복원한다. 이 구조는 detector 실패가 simulation의 영구 정지로 번지는 것을 막는다.

### 7. 좌표계 변환을 독립 geometry로 분리했다

bbox 중심을 FoV 기반 pinhole ray로 바꾼 뒤 `q_world_uav × q_uav_camera`로 Unity-world 방향을 계산한다. quaternion과 방향벡터는 정규화하고 0-norm·NaN·Inf를 거부한다. 현재 output frame은 `unity_world`이며 ROS ENU나 차량 좌표계로 변환했다고 주장하지 않는다.

상태기계와 message field는 [시스템 아키텍처](docs/ARCHITECTURE.md)에 정리했다.

## 내가 구현하고 검증한 범위

- 기존에 구축한 Unity UAV–기지국 Camera/state 생성과 offline YOLO·좌표변환 파이프라인을 ROS message 기반 online 구조로 재설계
- `SensorFrame`과 `BeamDirectionEstimate` custom message 설계
- Unity Camera PNG capture, 자세 noise, timestamp·sequence 발행 C# bridge 구현
- same-sequence 응답 수락, stale 거부, unscaled wall-clock timeout, `Time.timeScale` 복원 구현
- ROS 2 Python detector adapter와 pixel-to-bearing/quaternion geometry 구현
- 기존 Windows Conda YOLO를 한 번 적재해 반복 사용하는 persistent worker 경계 구성
- ROS graph roundtrip test와 Unity–ROS 2–YOLO–Unity 통합 smoke run 수행
- 결과 방향을 Unity cyan ray로 표시해 반환 경로 확인

FNN·CNN-LSTM 보정은 기존 offline 연구 결과이며 현재 live ROS 경로에는 포함하지 않았다.

## 실제 실행 증거

| 검증 | 확인 결과 | 해석 범위 |
|---|---|---|
| ROS graph roundtrip | synthetic `SensorFrame(sequence=42)`에 대해 같은 `source_sequence`, copied frame ID, 정상 status와 방향값 수신 | ROS message/node 경로만 검증; Unity TCP와 YOLO 미포함 |
| Unity–ROS 2–YOLO–Unity smoke run | [`STATUS_OK` 응답 3회](docs/evidence/sensor-frame-smoke.json) | 실제 통합 경로가 응답한 단일 실행 |
| Frame correlation | 마지막 published sequence `4`, accepted sequence `4` | 마지막 결과가 원본 frame과 일치 |
| 마지막 시간 관찰값 | roundtrip `170.7 ms`, Python processing `108.9 ms` | 한 표본이며 평균·p95·deadline이 아님 |

`processing_ms`는 Python callback 진입부터 결과 publish 호출 직전까지의 elapsed time이다. Windows Conda 경로에서는 worker IPC, PNG 확인, inference, geometry, 결과 구성이 포함되며, publish 이후의 DDS·TCP 전달시간은 포함되지 않는다. roundtrip은 Unity가 unscaled wall-clock으로 측정한 전체 대기시간이다. 이 수치는 benchmark, 처리율 또는 hard real-time 보장이 아니다.

공개 `center` backend는 model 없이 transport와 lockstep을 확인하는 deterministic smoke double이다. 실제 검증 실행은 기존 Windows Conda persistent YOLO와 비공개 weight를 사용했다. 검출 정확도 자체는 이 smoke run의 평가 대상이 아니다.

## 이 프로젝트를 통해 설명할 수 있는 것

- ROS message와 transport protocol을 구분하고 TCP–DDS 경계를 추적하는 방법
- heterogeneous runtime을 하나로 합치지 않고 process boundary로 연결한 이유
- timestamp와 sequence가 해결하는 문제가 어떻게 다른지
- ROS estimator-side queue depth 1과 lockstep·sequence 검증이 backlog와 freshness에 미치는 영향
- simulation pause 중 timeout을 scaled time으로 측정하면 안 되는 이유
- no detection, inference error, stale response, timeout을 서로 다른 실패로 다루는 방법
- callback processing time과 end-to-end roundtrip을 분리해 측정하는 방법
- 좌표 frame과 quaternion convention을 interface contract로 고정해야 하는 이유

## 전장·방산 직무와의 연결

| 직무 관점 | 이 프로젝트에서 제시하는 근거 |
|---|---|
| 무인체계 SW 통합 | simulation sensor, middleware, inference, visualization을 frame 단위로 연결 |
| Sensor data integrity | single-message correlation, source sequence echo, stale 결과 거부 |
| 실패 대응 | explicit status, timeout, simulation state 복원 |
| 이기종 SW 연동 | Unity C#, WSL ROS 2, Linux Python, Windows GPU runtime의 책임 분리 |
| 좌표·시간 계약 | Camera/UAV/World quaternion 변환과 message-assembly/processing/roundtrip 시간 구분 |
| 검증 설계 | ROS graph test와 실제 Unity integration proof의 범위를 분리해 보고 |

자동차 전장 관점에서도 interface, freshness, timeout, cross-runtime 검증 경험은 연결되지만, 이 프로젝트를 CAN, AUTOSAR 또는 ASIL 구현 경험으로 표현하지 않는다.

## 한계와 Further Work

- 실제 proof에 사용한 Unity scene에는 공개할 수 없는 third-party asset이 있어 저장소에는 bridge 핵심 source만 포함했다.
- 기존 YOLO weight와 개인 Conda/launcher 경로는 공개하지 않는다. 공개 baseline의 `center` backend는 detection 성능을 입증하지 않는다.
- 현재 통합 evidence는 `STATUS_OK` 3회의 단일 smoke run이다. 반복 횟수, drop, timeout, p50/p95가 포함된 benchmark는 아직 없다.
- 반환 빔 방향은 `Rigidbody`나 actuator에 적용되지 않는다. hard real-time, 기능안전, 인증·보안도 범위 밖이다.
- FNN·CNN-LSTM은 offline 상태이며 live regression이 필요하다. Camera–IMU MSCKF와 ATE/RPE 평가는 후속 범위다.
- 현재 `camera_orientation_local`은 Unity `Transform.localRotation`을 사용하므로 Camera의 direct parent와 basis가 UAV frame이라는 scene hierarchy를 전제한다. 일반적인 nested hierarchy에서는 명시적인 UAV-relative quaternion 계산이 필요하다.
- 상태–명령 P-controller/Rigidbody 실험은 로컬 `further_work`로 분리해 보존했으며 공개 대표 구현에는 포함하지 않았다.

다음 단계는 failure injection을 포함한 반복 통합시험, offline–online 수치 회귀, live fusion, Camera–IMU localization 순으로 evidence를 추가하는 것이다.

## 코드 지도

```text
.
├─ unity/Assets/UavBeamBridge/
│  ├─ SensorFrameCoordinator.cs          # capture, publish, sequence, timeout, cyan ray
│  └─ RosMessages/                       # Unity C# custom messages
├─ ros2_ws/
│  ├─ src/uav_interfaces/                # SensorFrame, BeamDirectionEstimate
│  ├─ src/beam_estimator/
│  │  ├─ beam_estimator/
│  │  │  ├─ beam_estimator_node.py       # ROS adapter and status response
│  │  │  ├─ detectors.py                 # center/Ultralytics detector adapters
│  │  │  └─ beam_geometry.py             # pixel ray and quaternion geometry
│  │  └─ test/                           # geometry and ROS graph roundtrip
│  └─ ros-tcp-endpoint.repos             # Endpoint source revision pin
├─ runtime/windows_conda/
│  ├─ ros_beam_estimator_node.py          # WSL ROS node ↔ Conda adapter
│  ├─ conda_yolo_worker.py                # persistent Windows GPU inference
│  └─ test_protocol.py                    # sequence-aware binary contract
├─ docs/
│  ├─ ARCHITECTURE.md
│  └─ evidence/                           # sanitized smoke-run evidence
```

## 공개 범위

직접 작성한 최소 코드와 공개 가능한 contract만 포함한다. third-party Unity asset, 외부 dataset, raw image, 학습 weight, 개인 절대경로, 대용량 산출물과 비공개 연구 내용은 저장소에서 제외한다.
