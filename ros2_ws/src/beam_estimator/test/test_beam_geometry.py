import math

import pytest

from beam_estimator.beam_geometry import (
    InvalidBeamInput,
    estimate_beam_direction,
    pixel_to_camera_direction,
)
from beam_estimator.detectors import CenterDetector, png_dimensions


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def test_center_pixel_points_along_camera_forward():
    direction = pixel_to_camera_direction(960.0, 540.0, 1920, 1080, 78.0)
    assert direction == pytest.approx((0.0, 0.0, 1.0))


def test_identity_orientations_keep_center_beam_forward():
    result = estimate_beam_direction(
        960.0, 540.0, 1920, 1080, 78.0, IDENTITY, IDENTITY
    )
    assert result.direction_world == pytest.approx((0.0, 0.0, 1.0))
    assert result.azimuth_deg == pytest.approx(0.0)
    assert result.elevation_deg == pytest.approx(0.0)


def test_positive_90_degree_unity_yaw_points_toward_positive_x():
    half_angle = math.radians(90.0) / 2.0
    yaw = (0.0, math.sin(half_angle), 0.0, math.cos(half_angle))
    result = estimate_beam_direction(
        960.0, 540.0, 1920, 1080, 78.0, yaw, IDENTITY
    )
    assert result.direction_world == pytest.approx((1.0, 0.0, 0.0), abs=1e-7)
    assert result.azimuth_deg == pytest.approx(90.0)


@pytest.mark.parametrize(
    "fov",
    [0.0, 180.0, float("nan")],
)
def test_invalid_vertical_fov_is_rejected(fov):
    with pytest.raises(InvalidBeamInput):
        pixel_to_camera_direction(960.0, 540.0, 1920, 1080, fov)


def test_zero_norm_quaternion_is_rejected():
    with pytest.raises(InvalidBeamInput):
        estimate_beam_direction(
            960.0,
            540.0,
            1920,
            1080,
            78.0,
            (0.0, 0.0, 0.0, 0.0),
            IDENTITY,
        )


def test_png_dimensions_and_center_detector():
    header = b"\x89PNG\r\n\x1a\n" + b"\x00\x00\x00\rIHDR"
    image_data = header + (1920).to_bytes(4, "big") + (1080).to_bytes(4, "big")
    assert png_dimensions(image_data) == (1920, 1080)
    detection = CenterDetector().detect(image_data)
    assert detection.x_center == 960.0
    assert detection.y_center == 540.0
    assert detection.confidence == 1.0
