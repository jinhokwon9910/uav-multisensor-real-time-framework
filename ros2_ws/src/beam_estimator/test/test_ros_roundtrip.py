import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from beam_estimator.beam_estimator_node import BeamEstimatorNode
from uav_interfaces.msg import BeamDirectionEstimate, SensorFrame


def _png_header(width: int, height: int) -> bytes:
    return (
        b"\x89PNG\r\n\x1a\n"
        + b"\x00\x00\x00\rIHDR"
        + width.to_bytes(4, "big")
        + height.to_bytes(4, "big")
    )


def _wait_until(executor, predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_sensor_frame_to_beam_estimate_roundtrip():
    rclpy.init()
    estimator = BeamEstimatorNode(
        parameter_overrides=[Parameter("detector_backend", value="center")]
    )
    probe = Node("beam_estimator_roundtrip_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(estimator)
    executor.add_node(probe)

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    received = []
    subscription = probe.create_subscription(
        BeamDirectionEstimate,
        "/beam/direction_estimate",
        received.append,
        qos,
    )
    publisher = probe.create_publisher(
        SensorFrame,
        "/unity/sensor_frame",
        qos,
    )

    try:
        discovered = _wait_until(
            executor,
            lambda: publisher.get_subscription_count() == 1
            and probe.count_publishers("/beam/direction_estimate") == 1,
            timeout_sec=5.0,
        )
        assert discovered, "ROS graph did not discover the estimator endpoints"

        frame = SensorFrame()
        frame.header.frame_id = "unity_world"
        frame.sequence = 42
        frame.camera_image.header.frame_id = "camera_optical"
        frame.camera_image.format = "png"
        frame.camera_image.data = list(_png_header(1920, 1080))
        frame.uav_orientation_noisy.w = 1.0
        frame.camera_orientation_local.w = 1.0
        frame.vertical_fov_deg = 78.0
        publisher.publish(frame)

        assert _wait_until(executor, lambda: len(received) == 1, timeout_sec=5.0)
        estimate = received[0]
        assert estimate.source_sequence == 42
        assert estimate.header.frame_id == "unity_world"
        assert estimate.status == BeamDirectionEstimate.STATUS_OK
        assert (
            estimate.direction_world.x,
            estimate.direction_world.y,
            estimate.direction_world.z,
        ) == pytest.approx((0.0, 0.0, 1.0))
        assert estimate.azimuth_deg == pytest.approx(0.0)
        assert estimate.elevation_deg == pytest.approx(0.0)
        assert estimate.processing_ms >= 0.0
    finally:
        probe.destroy_subscription(subscription)
        probe.destroy_publisher(publisher)
        executor.remove_node(probe)
        executor.remove_node(estimator)
        probe.destroy_node()
        estimator.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
