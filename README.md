# Unity–ROS 2–Python UAV Sensor-Frame Framework

Unity에서 생성한 UAV Camera·state를 ROS 2로 전달하고, Python에서 계산한 빔 방향 결과를 원본 frame과 대응시켜 Unity에 반환하는 online framework입니다.

ICTC 2025 제1저자 논문 *A Multi-Modal Simulator for Aerial Communication with Applications to Beam Search*에서 구축한 Unity–Python 시뮬레이션을 ROS 2 기반 real-time feedback 구조로 확장했습니다.

## Data Flow

```text
Unity SensorFrame(sequence=N)
  → ROS-TCP Connector / TCP
  → ROS-TCP Endpoint / ROS 2 DDS
  → Python YOLO detection
  → pixel-to-bearing / quaternion geometry
  → BeamDirectionEstimate(source_sequence=N)
  → Unity feedback
```

검증 환경에서는 ROS 2 node를 WSL에서 실행하고, 기존 CUDA·PyTorch·Ultralytics 환경은 Windows Conda persistent worker로 분리했습니다.

## Implemented

- Camera image, UAV orientation, Camera orientation, FoV, timestamp와 sequence를 하나의 `SensorFrame`으로 전송
- YOLO bounding box 중심을 camera ray로 변환하고 Camera → UAV → Unity World 좌표계로 회전
- `source_sequence`가 현재 frame과 일치하는 결과만 수락하고 stale 결과는 거부
- `NO_DETECTION`, `INVALID_INPUT`, `INFERENCE_ERROR` 상태와 응답 timeout 처리
- matching response 또는 timeout에서 Unity simulation state 복원
- 빔 방향 결과를 Unity에 반환하고 cyan ray로 시각화

Message field, topic, QoS, 좌표계와 시간 기준은 [Architecture](docs/ARCHITECTURE.md)에 정리했습니다.

## Unity 6 Simulator Demo

[▶ Unity 6 기반 UAV–기지국 시뮬레이터 데모 영상 보기 (MP4, 5.5 MB)](docs/media/simulator-test-video.mp4)

## Validation

| Check | Result |
|---|---|
| ROS graph roundtrip | Synthetic `SensorFrame(sequence=42)`에 대해 동일한 `source_sequence`와 정상 방향 결과 수신 |
| Unity–ROS 2–YOLO–Unity smoke run | [`STATUS_OK` 3회](docs/evidence/sensor-frame-smoke.json) |
| Frame correlation | 마지막 publish/accept sequence 모두 `4` |
| Observed timing | Roundtrip `170.7 ms`, Python processing `108.9 ms` |

시간 값은 단일 smoke run의 관찰값이며 평균, p95 또는 hard real-time 성능을 의미하지 않습니다.

CI에서는 Python source compile, ROS–C# message contract와 Windows worker protocol을 검사합니다.

```bash
python tests/test_interface_contract.py
python runtime/windows_conda/test_protocol.py
```

## Repository Layout

```text
unity/Assets/UavBeamBridge/
  SensorFrameCoordinator.cs       # capture, publish, sequence, timeout, feedback
  RosMessages/                    # Unity C# custom messages

ros2_ws/src/uav_interfaces/       # SensorFrame, BeamDirectionEstimate
ros2_ws/src/beam_estimator/       # ROS node, detector adapter, beam geometry, tests

runtime/windows_conda/
  ros_beam_estimator_node.py      # WSL ROS node ↔ Windows worker adapter
  conda_yolo_worker.py            # persistent YOLO inference worker
  test_protocol.py

docs/ARCHITECTURE.md              # interface and runtime details
docs/evidence/                    # sanitized smoke-run evidence
```

## Public Scope

저장소에는 직접 작성한 integration code와 공개 가능한 message contract·검증 자료만 포함합니다. Third-party Unity asset, dataset, trained weight, 개인 실행 경로는 제외했습니다. 현재 공개 경로의 반환값은 Unity 시각화에 사용하며 UAV actuator에는 적용하지 않습니다.
