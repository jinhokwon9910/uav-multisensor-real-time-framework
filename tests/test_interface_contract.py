"""Pure tests for the ROS 2-to-Unity message contract and public evidence."""

from __future__ import annotations

import json
from pathlib import Path
import re
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
INTERFACE_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "uav_interfaces" / "msg"
CSHARP_ROOT = (
    REPOSITORY_ROOT
    / "unity"
    / "Assets"
    / "UavBeamBridge"
    / "RosMessages"
    / "UavInterfaces"
    / "msg"
)

ROS_TO_CSHARP = {
    "std_msgs/Header": "Std.HeaderMsg",
    "sensor_msgs/CompressedImage": "Sensor.CompressedImageMsg",
    "geometry_msgs/Quaternion": "Geometry.QuaternionMsg",
    "geometry_msgs/Vector3": "Geometry.Vector3Msg",
    "uint32": "uint",
    "uint8": "byte",
    "float32": "float",
}

CONTRACTS = (
    ("SensorFrame", "uav_interfaces/SensorFrame"),
    ("BeamDirectionEstimate", "uav_interfaces/BeamDirectionEstimate"),
)


def parse_ros_message(path: Path) -> tuple[list[tuple[str, str]], dict[str, int]]:
    fields: list[tuple[str, str]] = []
    constants: dict[str, int] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        ros_type, declaration = line.split(maxsplit=1)
        if "=" in declaration:
            name, value = declaration.split("=", 1)
            constants[name.strip()] = int(value.strip(), 0)
        else:
            fields.append((ros_type, declaration.strip()))
    return fields, constants


def parse_csharp_fields(source: str) -> list[tuple[str, str]]:
    return re.findall(
        r"^\s*public\s+(?!const\b)([A-Za-z_][\w.]*)\s+([a-z][A-Za-z0-9_]*)\s*;\s*$",
        source,
        flags=re.MULTILINE,
    )


def parse_csharp_constants(source: str) -> dict[str, int]:
    return {
        name: int(value, 0)
        for name, value in re.findall(
            r"^\s*public\s+const\s+byte\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*;\s*$",
            source,
            flags=re.MULTILINE,
        )
    }


def serialized_field_order(source: str) -> list[str]:
    marker = "public override void SerializeTo"
    method_start = source.index(marker)
    body_start = source.index("{", method_start)
    depth = 0
    for index in range(body_start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                body = source[body_start + 1 : index]
                return re.findall(r"serializer\.Write\(([a-z][A-Za-z0-9_]*)\);", body)
    raise AssertionError("SerializeTo method has no closing brace")


class InterfaceContractTests(unittest.TestCase):
    def test_ros_fields_match_generated_csharp_types_and_order(self) -> None:
        for message_name, ros_message_name in CONTRACTS:
            with self.subTest(message=message_name):
                ros_fields, ros_constants = parse_ros_message(
                    INTERFACE_ROOT / f"{message_name}.msg"
                )
                csharp_source = (CSHARP_ROOT / f"{message_name}Msg.cs").read_text(
                    encoding="utf-8"
                )
                expected_csharp_fields = [
                    (ROS_TO_CSHARP[ros_type], field_name)
                    for ros_type, field_name in ros_fields
                ]

                self.assertEqual(parse_csharp_fields(csharp_source), expected_csharp_fields)
                self.assertEqual(
                    serialized_field_order(csharp_source),
                    [field_name for _, field_name in ros_fields],
                )
                self.assertEqual(parse_csharp_constants(csharp_source), ros_constants)
                self.assertIn(
                    f'k_RosMessageName = "{ros_message_name}";', csharp_source
                )


class PublicEvidenceTests(unittest.TestCase):
    def test_sensor_frame_smoke_evidence_is_sanitized(self) -> None:
        evidence_path = (
            REPOSITORY_ROOT / "docs" / "evidence" / "sensor-frame-smoke.json"
        )
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        serialized = json.dumps(evidence, ensure_ascii=False)

        forbidden = (
            r"[A-Za-z]:[\\/]",
            r"/mnt/[a-z]/",
            r"/home/[^/]+/",
            r"\\\\wsl(?:\.localhost)?[\\/]",
        )
        for pattern in forbidden:
            with self.subTest(pattern=pattern):
                self.assertIsNone(re.search(pattern, serialized, flags=re.IGNORECASE))

        self.assertEqual(evidence["scene"], "private_demo_scene")
        self.assertEqual(evidence["expectedResponses"], 3)
        self.assertEqual(evidence["successfulResponses"], 3)
        self.assertEqual(evidence["lastPublishedSequence"], 4)
        self.assertEqual(evidence["lastAcceptedSequence"], 4)
        self.assertTrue(evidence["single_smoke_run"])
        self.assertFalse(evidence["modelArtifact"]["includedInRepository"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
