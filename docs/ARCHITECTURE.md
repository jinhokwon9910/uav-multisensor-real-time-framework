# 시스템 아키텍처

[← 프로젝트 README로 돌아가기](../README.md)

이 문서는 Unity에서 생성한 `SensorFrame`이 Python의 `BeamDirectionEstimate`로 처리되어 Unity에 반환되는 경로와 interface contract를 정리합니다.

## 1. System Flow

```text
┌──────────────────────── Unity 6 ────────────────────────┐
│ Camera.Render → PNG                                     │
│ UAV orientation + Camera orientation + FoV             │
│ timestamp + sequence N                                  │
│                    SensorFrame                          │
└──────────────────────────┬──────────────────────────────┘
                           │ ROS-TCP Connector / TCP
                           ▼
┌────────────────── ROS-TCP Endpoint ─────────────────────┐
│             Unity transport ↔ ROS 2 graph               │
└──────────────────────────┬──────────────────────────────┘
                           │ ROS 2 Jazzy / DDS
                           ▼
┌──────────────────── Python Runtime ──────────────────────┐
│ YOLO detection → pixel-to-bearing → quaternion geometry │
│          BeamDirectionEstimate(source_sequence=N)       │
└──────────────────────────┬──────────────────────────────┘
                           │ DDS → Endpoint → TCP
                           ▼
┌──────────────────────── Unity 6 ────────────────────────┐
│ sequence match → simulation resume → cyan ray          │
└─────────────────────────────────────────────────────────┘
```

## 2. Runtime Boundaries

| Boundary | Connection | Responsibility |
|---|---|---|
| Unity C# → Endpoint | ROS-TCP-Connector, TCP | Message serialization and socket transport |
| Endpoint → ROS node | ROS 2 Jazzy, DDS | Topic discovery, QoS and delivery |
| WSL ROS node → Windows worker | Binary stdin/stdout pipe | PNG·sequence request and detection response |
| Detector → geometry | Python call | Bounding-box center and confidence transfer |
| ROS result → Unity | DDS → Endpoint → TCP | Source sequence, direction and status return |

ROS 2와 `rclpy`는 WSL system Python에서 실행합니다. 기존 CUDA·PyTorch·Ultralytics 환경은 Windows Conda worker로 유지하며, worker가 model을 한 번 적재한 뒤 반복 추론합니다.

## 3. Message and Topic Contract

| Topic | Message | Direction | ROS node QoS |
|---|---|---|---|
| `/unity/sensor_frame` | `uav_interfaces/SensorFrame` | Unity → Python | `RELIABLE`, `VOLATILE`, `KEEP_LAST(1)` |
| `/beam/direction_estimate` | `uav_interfaces/BeamDirectionEstimate` | Python → Unity | `RELIABLE`, `VOLATILE`, `KEEP_LAST(1)` |

`SensorFrame`은 한 publish cycle에서 다음 값을 하나의 message로 조립합니다.

- `header`: message timestamp와 `unity_world` frame ID
- `sequence`: uint32 frame identifier
- `camera_image`: compressed PNG
- `uav_orientation_noisy`: UAV world orientation
- `camera_orientation_local`: UAV-relative Camera orientation
- `vertical_fov_deg`: pixel-to-bearing 계산에 사용한 FoV

`BeamDirectionEstimate`는 source header와 `source_sequence`를 반환하고, Unity-world 방향벡터, azimuth, elevation, confidence, Python processing time과 status를 포함합니다.

| Status | Meaning | Unity behavior |
|---|---|---|
| `STATUS_OK` | Valid detection and geometry | Matching sequence이면 방향 시각화 |
| `STATUS_NO_DETECTION` | Detection 없음 | 결과 기록 후 simulation 재개 |
| `STATUS_INVALID_INPUT` | FoV·quaternion·수치 입력 오류 | 결과 기록 후 simulation 재개 |
| `STATUS_INFERENCE_ERROR` | Detector 실행 실패 | 결과 기록 후 simulation 재개 |

## 4. Coordinate Contract

Python은 bounding-box 중심을 pinhole camera ray로 바꾸고 Camera orientation과 UAV orientation을 합성해 Unity world 방향을 계산합니다.

```text
bbox center (u, v)
  → normalized camera ray (vertical FoV + aspect ratio)
  → q_world_camera = q_world_uav × q_uav_camera
  → direction_world
  → azimuth = atan2(x, z)
  → elevation = asin(y)
```

Quaternion과 방향벡터는 정규화하며 0-norm, NaN과 Inf는 `STATUS_INVALID_INPUT`으로 처리합니다. 현재 output frame은 `unity_world`입니다.

## 5. Frame Correlation and Failure Handling

```text
publish SensorFrame(N)
  → save unscaled send time
  → Time.timeScale = 0
  → wait for response
       ├─ source_sequence == N
       │    → record status and timing
       │    → restore Time.timeScale
       │    → STATUS_OK이면 cyan ray 표시
       ├─ stale or unsolicited sequence
       │    → ignore
       └─ timeout
            → restore Time.timeScale
```

Timestamp는 message 생성 시점을 기록하고, sequence는 원본 frame과 결과의 identity를 연결합니다. Unity는 현재 기다리는 sequence와 일치하는 결과만 수락합니다.

## 6. Timing and Validation

- **Python processing time:** ROS callback 진입부터 result publish 호출 직전까지
- **Roundtrip time:** Unity publish부터 matching response 수신까지의 unscaled wall-clock 시간

| Validation | Result |
|---|---|
| ROS graph roundtrip | Synthetic `SensorFrame(sequence=42)`에 대해 동일한 source sequence와 정상 방향 결과 수신 |
| Unity integration smoke | [`STATUS_OK` 3회](evidence/sensor-frame-smoke.json) |
| Frame correlation | 마지막 publish/accept sequence 모두 `4` |
| Observed timing | Roundtrip `170.7 ms`, Python processing `108.9 ms` |

시간 값은 한 번의 smoke run에서 관찰한 값이며 평균, p95 또는 hard real-time 보장을 의미하지 않습니다.

## 7. Public Scope

공개 저장소에는 integration code, custom message, contract test와 sanitized evidence를 포함합니다. Third-party Unity asset, dataset, trained weight와 개인 실행 경로는 제외했습니다. 반환된 빔 방향은 Unity에서 시각화하며 actuator control에는 사용하지 않습니다.

