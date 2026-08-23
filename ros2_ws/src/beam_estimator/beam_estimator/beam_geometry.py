"""Pure geometry used by both offline evaluation and the ROS 2 node."""

from dataclasses import dataclass
import math
from typing import Sequence


class InvalidBeamInput(ValueError):
    """Raised when a sensor frame cannot produce a finite beam estimate."""


@dataclass(frozen=True)
class BeamGeometryResult:
    direction_world: tuple[float, float, float]
    azimuth_deg: float
    elevation_deg: float


def _require_finite(values: Sequence[float], label: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise InvalidBeamInput(f"{label} contains a non-finite value")


def _normalize_vector(vector: Sequence[float]) -> tuple[float, float, float]:
    _require_finite(vector, "vector")
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 1.0e-12:
        raise InvalidBeamInput("vector norm is zero")
    return tuple(value / norm for value in vector)


def normalize_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    if len(quaternion) != 4:
        raise InvalidBeamInput("quaternion must contain x, y, z, w")
    _require_finite(quaternion, "quaternion")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise InvalidBeamInput("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)


def multiply_quaternions(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    """Return Hamilton product ``left * right`` in x, y, z, w order."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def rotate_vector(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    """Rotate a 3-D vector by an x, y, z, w unit quaternion."""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    _require_finite((vx, vy, vz), "vector")

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def pixel_to_camera_direction(
    x_center: float,
    y_center: float,
    image_width: int,
    image_height: int,
    vertical_fov_deg: float,
) -> tuple[float, float, float]:
    """Reproduce the existing pixel-to-bearing pinhole calculation."""
    _require_finite((x_center, y_center, vertical_fov_deg), "camera input")
    if image_width <= 0 or image_height <= 0:
        raise InvalidBeamInput("image dimensions must be positive")
    if not 0.0 < vertical_fov_deg < 180.0:
        raise InvalidBeamInput("vertical FoV must be between 0 and 180 degrees")

    cx = image_width / 2.0
    cy = image_height / 2.0
    x_normalized = (x_center - cx) / cx
    y_normalized = (cy - y_center) / cy

    vertical_fov_rad = math.radians(vertical_fov_deg)
    aspect_ratio = image_width / image_height
    horizontal_fov_rad = 2.0 * math.atan(
        math.tan(vertical_fov_rad / 2.0) * aspect_ratio
    )

    return _normalize_vector(
        (
            x_normalized * math.tan(horizontal_fov_rad / 2.0),
            y_normalized * math.tan(vertical_fov_rad / 2.0),
            1.0,
        )
    )


def estimate_beam_direction(
    x_center: float,
    y_center: float,
    image_width: int,
    image_height: int,
    vertical_fov_deg: float,
    uav_orientation_noisy: Sequence[float],
    camera_orientation_local: Sequence[float],
) -> BeamGeometryResult:
    """Transform the detected camera bearing into the Unity world frame."""
    direction_camera = pixel_to_camera_direction(
        x_center,
        y_center,
        image_width,
        image_height,
        vertical_fov_deg,
    )
    world_from_camera = multiply_quaternions(
        uav_orientation_noisy, camera_orientation_local
    )
    direction_world = _normalize_vector(
        rotate_vector(world_from_camera, direction_camera)
    )

    vx, vy, vz = direction_world
    azimuth_deg = math.degrees(math.atan2(vx, vz))
    elevation_deg = math.degrees(math.asin(max(-1.0, min(1.0, vy))))
    return BeamGeometryResult(direction_world, azimuth_deg, elevation_deg)
