"""ROS 2 beam estimator backed by an existing Windows Conda YOLO environment."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import select
import struct
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from beam_estimator.beam_geometry import InvalidBeamInput, estimate_beam_direction
from beam_estimator.detectors import png_dimensions
from uav_interfaces.msg import BeamDirectionEstimate, SensorFrame


REQUEST = struct.Struct(">4sII")
RESPONSE = struct.Struct(">4sIBffffI")
REQUEST_MAGIC = b"YBR1"
RESPONSE_MAGIC = b"YBS1"
MAX_ERROR_BYTES = 16 * 1024


@dataclass(frozen=True)
class WorkerResult:
    status: int
    x_center: float
    y_center: float
    confidence: float
    inference_ms: float
    error: str


def _required_path(name: str, *, directory: bool = False) -> Path:
    raw_path = os.environ.get(name, "").strip()
    if not raw_path:
        raise RuntimeError(f"{name} is required")
    path = Path(raw_path).expanduser().resolve()
    if directory:
        path.mkdir(parents=True, exist_ok=True)
        if not path.is_dir():
            raise NotADirectoryError(f"{name} is not a directory: {path}")
    elif not path.is_file():
        raise FileNotFoundError(f"{name} does not exist: {path}")
    return path


def _windows_path(linux_path: Path) -> str:
    completed = subprocess.run(
        ["wslpath", "-w", str(linux_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def _read_exact_with_timeout(stream, size: int, timeout_seconds: float) -> bytes:
    deadline = time.monotonic() + timeout_seconds
    chunks = bytearray()
    while len(chunks) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("YOLO worker response timed out")
        readable, _, _ = select.select([stream], [], [], remaining)
        if not readable:
            raise TimeoutError("YOLO worker response timed out")
        chunk = os.read(stream.fileno(), size - len(chunks))
        if not chunk:
            raise RuntimeError("YOLO worker closed its output pipe")
        chunks.extend(chunk)
    return bytes(chunks)


class CondaYoloWorker:
    def __init__(self, logger) -> None:
        worker_script = Path(__file__).resolve().with_name("conda_yolo_worker.py")
        conda_python = _required_path("UAV_CONDA_PYTHON")
        model_path = _required_path("UAV_YOLO_MODEL")
        config_dir = _required_path("UAV_YOLO_CONFIG_DIR", directory=True)
        warmup_raw = os.environ.get("UAV_YOLO_WARMUP_IMAGE", "").strip()
        warmup_image = (
            _required_path("UAV_YOLO_WARMUP_IMAGE") if warmup_raw else None
        )

        command = [
            str(conda_python),
            _windows_path(worker_script),
            "--model",
            _windows_path(model_path),
            "--config-dir",
            _windows_path(config_dir),
            "--image-size",
            os.environ.get("UAV_IMAGE_SIZE", "960"),
            "--confidence",
            os.environ.get("UAV_CONFIDENCE", "0.01"),
            "--device",
            os.environ.get("UAV_YOLO_DEVICE", "auto"),
        ]
        if warmup_image is not None:
            command.extend(["--warmup-image", _windows_path(warmup_image)])

        self._logger = logger
        self._response_timeout = float(
            os.environ.get("UAV_WORKER_TIMEOUT_SECONDS", "10.0")
        )
        if self._response_timeout <= 0.0:
            raise ValueError("UAV_WORKER_TIMEOUT_SECONDS must be positive")

        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        if self._process.stdin is None or self._process.stdout is None:
            raise RuntimeError("Could not open YOLO worker pipes")
        self._stdin = self._process.stdin
        self._stdout = self._process.stdout
        self._wait_until_ready()
        self._stderr_thread = threading.Thread(target=self._drain_stderr, daemon=True)
        self._stderr_thread.start()

    def _wait_until_ready(self) -> None:
        assert self._process.stderr is not None
        while True:
            line = self._process.stderr.readline()
            if not line:
                raise RuntimeError(
                    f"YOLO worker exited during startup ({self._process.poll()})"
                )
            text = line.decode("utf-8", errors="replace").rstrip()
            self._logger.info(f"[Conda YOLO] {text}")
            if text.startswith("READY "):
                return

    def _drain_stderr(self) -> None:
        assert self._process.stderr is not None
        for line in self._process.stderr:
            text = line.decode("utf-8", errors="replace").rstrip()
            if text:
                self._logger.info(f"[Conda YOLO] {text}")

    def infer(self, sequence: int, png: bytes) -> WorkerResult:
        self._stdin.write(REQUEST.pack(REQUEST_MAGIC, sequence, len(png)))
        self._stdin.write(png)
        self._stdin.flush()

        header = _read_exact_with_timeout(
            self._stdout, RESPONSE.size, self._response_timeout
        )
        magic, returned_sequence, status, x, y, confidence, inference_ms, error_size = (
            RESPONSE.unpack(header)
        )
        if magic != RESPONSE_MAGIC:
            raise RuntimeError("YOLO worker returned invalid response magic")
        if returned_sequence != sequence:
            raise RuntimeError(
                f"YOLO worker sequence mismatch: sent {sequence}, got {returned_sequence}"
            )
        if error_size > MAX_ERROR_BYTES:
            raise RuntimeError(f"YOLO worker error is too large: {error_size} bytes")
        error_bytes = (
            _read_exact_with_timeout(self._stdout, error_size, self._response_timeout)
            if error_size
            else b""
        )
        return WorkerResult(
            status,
            x,
            y,
            confidence,
            inference_ms,
            error_bytes.decode("utf-8", errors="replace"),
        )

    def close(self) -> None:
        if self._process.poll() is not None:
            return
        try:
            self._stdin.close()
            self._process.wait(timeout=3.0)
        except Exception:
            self._process.terminate()


def _quaternion_tuple(value) -> tuple[float, float, float, float]:
    return value.x, value.y, value.z, value.w


class CondaBeamEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("beam_estimator")
        self._worker = CondaYoloWorker(self.get_logger())
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        estimate_topic = os.environ.get(
            "UAV_ESTIMATE_TOPIC", "/beam/direction_estimate"
        )
        sensor_topic = os.environ.get("UAV_SENSOR_TOPIC", "/unity/sensor_frame")
        self._publisher = self.create_publisher(
            BeamDirectionEstimate, estimate_topic, qos
        )
        self._subscription = self.create_subscription(
            SensorFrame, sensor_topic, self._on_sensor_frame, qos
        )
        self.get_logger().info(
            f"Ready: {sensor_topic} -> YOLO -> {estimate_topic}"
        )

    def _on_sensor_frame(self, frame: SensorFrame) -> None:
        started = time.perf_counter()
        estimate = BeamDirectionEstimate()
        estimate.header = frame.header
        estimate.source_sequence = frame.sequence
        try:
            png = bytes(frame.camera_image.data)
            width, height = png_dimensions(png)
            detection = self._worker.infer(frame.sequence, png)
            if detection.status == BeamDirectionEstimate.STATUS_NO_DETECTION:
                estimate.status = BeamDirectionEstimate.STATUS_NO_DETECTION
            elif detection.status != BeamDirectionEstimate.STATUS_OK:
                estimate.status = BeamDirectionEstimate.STATUS_INFERENCE_ERROR
                self.get_logger().error(
                    f"YOLO failed for sequence {frame.sequence}: {detection.error}"
                )
            else:
                geometry = estimate_beam_direction(
                    detection.x_center,
                    detection.y_center,
                    width,
                    height,
                    frame.vertical_fov_deg,
                    _quaternion_tuple(frame.uav_orientation_noisy),
                    _quaternion_tuple(frame.camera_orientation_local),
                )
                (
                    estimate.direction_world.x,
                    estimate.direction_world.y,
                    estimate.direction_world.z,
                ) = geometry.direction_world
                estimate.azimuth_deg = geometry.azimuth_deg
                estimate.elevation_deg = geometry.elevation_deg
                estimate.confidence = detection.confidence
                estimate.status = BeamDirectionEstimate.STATUS_OK
        except InvalidBeamInput as exc:
            estimate.status = BeamDirectionEstimate.STATUS_INVALID_INPUT
            self.get_logger().warning(f"Invalid frame {frame.sequence}: {exc}")
        except Exception as exc:
            estimate.status = BeamDirectionEstimate.STATUS_INFERENCE_ERROR
            self.get_logger().error(
                f"Pipeline failed for sequence {frame.sequence}: "
                f"{type(exc).__name__}: {exc}"
            )

        estimate.processing_ms = (time.perf_counter() - started) * 1000.0
        self._publisher.publish(estimate)
        self.get_logger().info(
            f"seq={frame.sequence} status={estimate.status} "
            f"confidence={estimate.confidence:.3f} "
            f"processing={estimate.processing_ms:.1f} ms"
        )

    def destroy_node(self):
        self._worker.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CondaBeamEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
