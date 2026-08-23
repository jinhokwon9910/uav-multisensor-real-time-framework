"""ROS 2 node that converts Unity SensorFrame messages to beam estimates."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from uav_interfaces.msg import BeamDirectionEstimate, SensorFrame

from .beam_geometry import InvalidBeamInput, estimate_beam_direction
from .detectors import create_detector


def _quaternion_tuple(quaternion) -> tuple[float, float, float, float]:
    return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)


class BeamEstimatorNode(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__(
            "beam_estimator",
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("sensor_topic", "/unity/sensor_frame")
        self.declare_parameter("estimate_topic", "/beam/direction_estimate")
        # The dependency-free center backend makes the transport path runnable
        # before a private model is configured. Model-backed runs select an
        # explicit YOLO backend instead of silently assuming a weight path.
        self.declare_parameter("detector_backend", "center")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_size", 960)
        self.declare_parameter("confidence_threshold", 0.01)
        self.declare_parameter("device", "auto")

        backend = self.get_parameter("detector_backend").value
        self._detector = create_detector(
            backend=backend,
            model_path=self.get_parameter("model_path").value,
            image_size=self.get_parameter("image_size").value,
            confidence_threshold=self.get_parameter("confidence_threshold").value,
            device=self.get_parameter("device").value,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            BeamDirectionEstimate,
            self.get_parameter("estimate_topic").value,
            qos,
        )
        self._subscription = self.create_subscription(
            SensorFrame,
            self.get_parameter("sensor_topic").value,
            self._on_sensor_frame,
            qos,
        )
        self.get_logger().info(f"Beam estimator ready (backend={backend})")

    def _on_sensor_frame(self, frame: SensorFrame) -> None:
        started = time.perf_counter()
        estimate = BeamDirectionEstimate()
        estimate.header = frame.header
        estimate.source_sequence = frame.sequence

        try:
            detection = self._detector.detect(bytes(frame.camera_image.data))
            if detection is None:
                estimate.status = BeamDirectionEstimate.STATUS_NO_DETECTION
            else:
                geometry = estimate_beam_direction(
                    detection.x_center,
                    detection.y_center,
                    detection.image_width,
                    detection.image_height,
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
            self.get_logger().warning(
                f"Rejected sequence {frame.sequence}: {exc}"
            )
        except Exception as exc:  # inference errors must still release lockstep
            estimate.status = BeamDirectionEstimate.STATUS_INFERENCE_ERROR
            self.get_logger().error(
                f"Inference failed for sequence {frame.sequence}: {exc}"
            )

        estimate.processing_ms = (time.perf_counter() - started) * 1000.0
        self._publisher.publish(estimate)
        self.get_logger().info(
            f"seq={frame.sequence} status={estimate.status} "
            f"confidence={estimate.confidence:.3f} "
            f"processing={estimate.processing_ms:.1f} ms"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BeamEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
