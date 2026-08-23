# 시스템 아키텍처

[← 프로젝트 README로 돌아가기](../README.md)

> 이 문서는 실제로 검증한 `SensorFrame`–`BeamDirectionEstimate` 경로와 아직 통합하지 않은 연구·제어 기능을 분리한다.

## 1. 구현 경계

현재 대표 경로는 Unity가 만든 한 correlated sensor-frame message를 Python의 빔 방향 결과와 sequence로 연결한다.

```text
┌──────────────────────────── Unity 6 ────────────────────────────┐
│ Camera.Render → PNG                                             │
│ noisy UAV q + camera-local q + vertical FoV                    │
│ message-assembly wall-clock stamp + sequence N                 │
│                  SensorFrameCoordinator                         │
└──────────────────────────────┬──────────────────────────────────┘
                               │ /unity/sensor_frame
                               │ ROS-TCP Connector
                               ▼
┌────────────────────── ROS-TCP Endpoint ────────────────────────┐
│ TCP transport boundary between Unity and ROS 2                 │
└──────────────────────────────┬──────────────────────────────────┘
                               │ ROS 2 Jazzy / DDS
                               ▼
┌────────────────────────── Python runtime ───────────────────────┐
│ detection → pixel-to-bearing → quaternion geometry             │
│ BeamDirectionEstimate(source_sequence=N, status, timing)       │
└──────────────────────────────┬──────────────────────────────────┘
                               │ /beam/direction_estimate
                               ▼
┌──────────────────────────── Unity 6 ────────────────────────────┐
│ sequence match → resume Time.timeScale → draw cyan ray         │
└─────────────────────────────────────────────────────────────────┘
```

검증 환경의 detection 단계는 WSL의 ROS node가 binary pipe로 Windows Conda persistent YOLO worker를 호출했다. worker는 weight를 한 번 적재하고 각 PNG를 반복 처리한다. [Adapter source](../runtime/windows_conda/)는 공개하되 개인 실행 경로와 weight는 공개하지 않는다. 공개 `center` backend는 transport와 geometry 배선을 확인하기 위한 smoke double이며 YOLO 검출 성능을 나타내지 않는다.

## 2. Runtime 경계

| 경계 | 연결 | 책임 |
|---|---|---|
| Unity C# → Endpoint | ROS-TCP-Connector, TCP | Unity message serialization과 socket 연결 |
| Endpoint → ROS node | ROS 2 Jazzy, DDS | topic discovery, QoS, message delivery |
| WSL ROS node → Windows worker | binary stdin/stdout pipe | sequence와 PNG 요청, detection 응답 |
| Detector → geometry | Python call boundary | bbox center와 confidence 전달 |
| ROS result → Unity | DDS → Endpoint → TCP | source sequence와 계산 결과 반환 |

WSL system Python은 `rclpy`와 ROS 2 package를 담당하고, Windows Conda Python은 기존 CUDA·PyTorch·Ultralytics 환경을 담당한다. 두 runtime을 process boundary로 분리해 Python ABI 충돌을 피하고, persistent worker로 model reload 비용을 매 frame 반복하지 않는다.

## 3. 메시지와 topic

| Topic | Message | Producer → Consumer | 실제 queue/QoS 경계 |
|---|---|---|---|
| `/unity/sensor_frame` | `uav_interfaces/SensorFrame` | Unity → beam estimator | Unity publisher queue 1; Python subscription은 RELIABLE, VOLATILE, KEEP_LAST(1) |
| `/beam/direction_estimate` | `uav_interfaces/BeamDirectionEstimate` | beam estimator → Unity | Python publisher는 RELIABLE, VOLATILE, KEEP_LAST(1); Unity subscription queue는 Endpoint가 관리 |

`SensorFrame`은 다음 값을 한 publish cycle에서 하나의 message로 조립한다.

- `header`: message-assembly wall-clock stamp와 `unity_world` frame ID
- `sequence`: uint32 frame 식별자
- `camera_image`: 같은 publish cycle에서 강제 렌더링한 compressed PNG
- `uav_orientation_noisy`: noise를 더한 UAV world orientation
- `camera_orientation_local`: Camera `Transform.localRotation`이며 검증 scene에서 UAV-relative로 해석
- `vertical_fov_deg`: pixel-to-bearing에 사용한 Camera FoV

PNG render/encode, UAV Transform sampling, timestamp 생성, Camera Transform sampling은 같은 함수 안에서 순차 실행된다. 이 계약은 파일 기반 사후 결합을 제거하지만 hardware-level simultaneous acquisition이나 별도 센서 clock 동기화를 의미하지 않는다.

`BeamDirectionEstimate`는 source header를 복사하고 `source_sequence`를 되돌려 보낸다. payload는 Unity-world 단위 방향벡터, azimuth, elevation, confidence, Python processing time, status다.

| Status | 의미 | Unity 동작 |
|---|---|---|
| `STATUS_OK` | 유효한 검출과 geometry 결과 | 일치 sequence이면 cyan ray 표시 |
| `STATUS_NO_DETECTION` | 검출 없음 | 결과를 기록하고 다음 frame으로 진행 |
| `STATUS_INVALID_INPUT` | FoV·quaternion·수치 입력 오류 | 결과를 기록하고 다음 frame으로 진행 |
| `STATUS_INFERENCE_ERROR` | detector 실행 실패 | 결과를 기록하고 다음 frame으로 진행 |

모든 matching-sequence status 응답은 lockstep을 해제한다. 응답 자체가 없을 때는 Unity timeout이 simulation을 복원한다.

## 4. 처리 파이프라인과 좌표 계약

Python은 검출 bbox 중심을 pinhole camera ray로 바꾼 뒤 두 quaternion을 합성해 Unity world frame으로 회전한다.

```text
bbox center (u, v)
  → normalized camera ray(vertical FoV + aspect ratio)
  → q_world_camera = q_world_uav × q_uav_camera
  → direction_world
  → azimuth = atan2(x, z)
  → elevation = asin(y)
```

입력 quaternion과 출력 방향벡터는 계산 전에 정규화한다. 0-norm과 NaN/Inf 등 유효하지 않은 입력은 정상 방향값으로 포장하지 않고 `STATUS_INVALID_INPUT`으로 반환한다.

현재 message frame은 `unity_world`이고 반환 방향도 Unity world 축 기준이다. 이 구현은 ROS ENU나 차량 좌표계로 변환했다고 주장하지 않는다. 다른 시스템과 연결할 때는 Unity world, Camera local, ROS/차량 frame 사이의 축 방향과 handedness를 별도 contract와 test로 고정해야 한다.

현재 C# bridge는 Camera의 `Transform.localRotation`을 직렬화한다. 검증 scene에서는 Camera의 direct parent와 basis가 UAV frame이라는 hierarchy를 전제하며, nested rig에 일반화하려면 `q_world_uav⁻¹ × q_world_camera`로 UAV-relative rotation을 명시적으로 계산해야 한다.

FNN·CNN-LSTM 보정은 기존 offline 연구 경로에만 있다. 현재 live callback은 YOLO detection과 geometry까지만 수행한다.

## 5. Lockstep과 freshness

```text
capture N
  → publish SensorFrame(N)
  → save unscaled send time
  → Time.timeScale = 0
  → wait
       ├─ estimate.source_sequence == N
       │    → record status/timing
       │    → restore Time.timeScale
       │    → STATUS_OK이면 cyan ray
       ├─ stale or unsolicited sequence
       │    → ignore and keep waiting
       └─ response timeout
            → restore Time.timeScale
            → continue without a result
```

Lockstep은 sensor frame과 결과의 대응 관계를 관찰하기 위한 application-level 동작이다. Python estimator의 `KEEP_LAST(1)`과 Unity publisher queue 1은 backlog보다 최신 입력을 우선하지만, Unity-bound subscription까지 동일한 depth를 강제하지는 않는다. 최종 freshness는 Unity의 `source_sequence` 비교로 판단한다. 이 구조는 OS scheduling, DDS, TCP, GPU inference에 deadline을 강제하지 않으며 hard real-time이나 완전한 결정론을 보장하지 않는다.

결과는 UAV dynamics에 적용되지 않는다. `Time.timeScale` 정지는 capture 사이 simulation 진행을 억제할 뿐 actuator-level control이 아니다.

## 6. 시간 계약

`SensorFrame.header.stamp`는 PNG render/encode와 UAV pose sampling 뒤, Camera local Transform을 읽기 직전에 message assembly 중 생성한 UTC wall-clock stamp다. Python은 결과 header에 이 값을 복사한다. Unity의 roundtrip 측정은 `Time.realtimeSinceStartupAsDouble`을 사용하므로 `Time.timeScale=0` 중에도 증가한다. Python의 `processing_ms`는 callback 진입부터 결과 publish 호출 직전까지이며, Conda 경로에서는 worker IPC·PNG 확인·inference·geometry·결과 구성을 포함한다. publish 이후 DDS/TCP 전달시간은 포함하지 않는다.

Sequence는 clock domain과 무관하게 원본과 결과의 identity를 연결한다. Timestamp는 message-assembly wall-clock context를 보존한다. 두 값을 같은 문제의 대체 수단으로 사용하지 않는다.

## 7. 실행 증거

검증은 서로 다른 범위를 가진 두 계층으로 기록한다.

| 계층 | 입력과 경로 | 확인 결과 | 한계 |
|---|---|---|---|
| ROS graph roundtrip | synthetic PNG header, `SensorFrame(42)` → ROS node → `BeamDirectionEstimate` | header·sequence·status·기본 방향 일치 | Unity, TCP, 실제 YOLO 미포함 |
| Unity integration smoke | Camera PNG → Endpoint → ROS 2 → persistent YOLO → geometry → Unity | [`STATUS_OK` 3회, 마지막 publish/accept sequence `4` 일치](evidence/sensor-frame-smoke.json) | 단 한 번의 실행 |

통합 smoke run의 마지막 관찰값은 roundtrip `170.7 ms`, Python processing `108.9 ms`였다. 이 값은 평균·p95·처리율·deadline을 나타내지 않으며 성능 비교에 사용하지 않는다.

## 8. 대표 경로 밖의 기능

- 빔 방향의 `Rigidbody` 또는 actuator 적용
- live FNN·CNN-LSTM 방향 보정
- Camera–IMU MSCKF와 ATE/RPE 평가
- 반복 실행 기반 latency·drop benchmark
- CAN, AUTOSAR, ASIL 또는 인증된 기능안전 메커니즘

상태–명령 P-controller/Rigidbody 아이디어와 구현은 삭제하지 않고 로컬 `further_work`에 별도 보존한다. 공개 저장소의 대표 SensorFrame 경로와 검증 evidence에는 포함하지 않는다.

## 9. 다음 evidence 기준

후속 작업은 기능 이름보다 이를 입증할 evidence를 먼저 정의한다.

- 여러 실행에서 response count, sequence mismatch, timeout 원인을 함께 기록한다.
- roundtrip과 Python processing을 분리해 표본 수와 함께 p50/p95를 계산한다.
- no detection, inference error, stale response, timeout을 Unity–TCP 통합 수준에서 주입한다.
- live FNN/CNN-LSTM을 추가하면 같은 source sequence와 offline regression을 함께 검증한다.
