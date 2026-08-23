"""Executable wire contract for the persistent Windows Conda worker.

This test uses only the Python standard library. The ROS 2 adapter and Windows
worker must match these golden, big-endian frames.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
import struct
import unittest

from conda_yolo_worker import (
    MAX_ERROR_BYTES,
    MAX_IMAGE_BYTES,
    REQUEST,
    REQUEST_MAGIC,
    RESPONSE,
    RESPONSE_MAGIC,
    STATUS_INFERENCE_ERROR,
    STATUS_NO_DETECTION,
    STATUS_OK,
)


def integer_literal(expression: ast.expr) -> int:
    if isinstance(expression, ast.Constant) and isinstance(expression.value, int):
        return expression.value
    if isinstance(expression, ast.BinOp):
        left = integer_literal(expression.left)
        right = integer_literal(expression.right)
        if isinstance(expression.op, ast.Mult):
            return left * right
        if isinstance(expression.op, ast.Add):
            return left + right
        if isinstance(expression.op, ast.Sub):
            return left - right
    raise ValueError("wire limit must be a literal integer expression")


def adapter_wire_contract() -> dict[str, object]:
    """Read wire constants without importing the ROS-dependent adapter."""
    source_path = Path(__file__).with_name("ros_beam_estimator_node.py")
    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    contract: dict[str, object] = {}
    literal_names = {"REQUEST_MAGIC", "RESPONSE_MAGIC", "MAX_ERROR_BYTES"}
    struct_names = {"REQUEST", "RESPONSE"}

    for statement in tree.body:
        if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
            continue
        target = statement.targets[0]
        if not isinstance(target, ast.Name):
            continue
        if target.id in literal_names:
            if target.id == "MAX_ERROR_BYTES":
                contract[target.id] = integer_literal(statement.value)
            else:
                contract[target.id] = ast.literal_eval(statement.value)
        elif target.id in struct_names:
            call = statement.value
            if (
                isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and call.func.attr == "Struct"
                and len(call.args) == 1
            ):
                contract[target.id] = ast.literal_eval(call.args[0])

    expected_names = literal_names | struct_names
    missing = expected_names - contract.keys()
    if missing:
        raise AssertionError(f"adapter wire constants missing: {sorted(missing)}")
    return contract


@dataclass(frozen=True)
class WorkerResponse:
    sequence: int
    status: int
    x_center: float
    y_center: float
    confidence: float
    inference_ms: float
    error: str


def read_exact(stream: BytesIO, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            raise EOFError(f"expected {size} bytes, received {len(chunks)}")
        chunks.extend(chunk)
    return bytes(chunks)


def encode_request(sequence: int, png: bytes) -> bytes:
    if len(png) > MAX_IMAGE_BYTES:
        raise ValueError("image exceeds protocol limit")
    return REQUEST.pack(REQUEST_MAGIC, sequence, len(png)) + png


def decode_request(stream: BytesIO) -> tuple[int, bytes]:
    magic, sequence, image_size = REQUEST.unpack(read_exact(stream, REQUEST.size))
    if magic != REQUEST_MAGIC:
        raise ValueError("invalid request magic")
    if image_size > MAX_IMAGE_BYTES:
        raise ValueError("image exceeds protocol limit")
    return sequence, read_exact(stream, image_size)


def encode_response(response: WorkerResponse) -> bytes:
    error = response.error.encode("utf-8", errors="replace")[:MAX_ERROR_BYTES]
    header = RESPONSE.pack(
        RESPONSE_MAGIC,
        response.sequence,
        response.status,
        response.x_center,
        response.y_center,
        response.confidence,
        response.inference_ms,
        len(error),
    )
    return header + error


def decode_response(stream: BytesIO) -> WorkerResponse:
    values = RESPONSE.unpack(read_exact(stream, RESPONSE.size))
    magic, sequence, status, x, y, confidence, inference_ms, error_size = values
    if magic != RESPONSE_MAGIC:
        raise ValueError("invalid response magic")
    if error_size > MAX_ERROR_BYTES:
        raise ValueError("error exceeds protocol limit")
    error = read_exact(stream, error_size).decode("utf-8", errors="replace")
    return WorkerResponse(sequence, status, x, y, confidence, inference_ms, error)


class PersistentWorkerProtocolTests(unittest.TestCase):
    def test_ros_adapter_and_worker_use_the_same_wire_contract(self) -> None:
        adapter = adapter_wire_contract()
        self.assertEqual(adapter["REQUEST_MAGIC"], REQUEST_MAGIC)
        self.assertEqual(adapter["RESPONSE_MAGIC"], RESPONSE_MAGIC)
        self.assertEqual(adapter["REQUEST"], REQUEST.format)
        self.assertEqual(adapter["RESPONSE"], RESPONSE.format)
        self.assertEqual(adapter["MAX_ERROR_BYTES"], MAX_ERROR_BYTES)

    def test_wire_layout_is_stable_and_big_endian(self) -> None:
        self.assertEqual(REQUEST.size, 12)
        self.assertEqual(RESPONSE.size, 29)
        self.assertEqual(
            REQUEST.pack(REQUEST_MAGIC, 0x01020304, 3),
            b"YBR1\x01\x02\x03\x04\x00\x00\x00\x03",
        )

    def test_multiple_requests_share_one_persistent_stream(self) -> None:
        stream = BytesIO(
            encode_request(101, b"\x89PNG-one")
            + encode_request(102, b"\x89PNG-two")
        )
        self.assertEqual(decode_request(stream), (101, b"\x89PNG-one"))
        self.assertEqual(decode_request(stream), (102, b"\x89PNG-two"))
        self.assertEqual(stream.read(), b"")

    def test_response_round_trip_preserves_sequence_status_and_error(self) -> None:
        expected = WorkerResponse(
            sequence=4242,
            status=STATUS_INFERENCE_ERROR,
            x_center=321.25,
            y_center=123.5,
            confidence=0.875,
            inference_ms=108.9378,
            error="synthetic failure",
        )
        actual = decode_response(BytesIO(encode_response(expected)))
        self.assertEqual(actual.sequence, expected.sequence)
        self.assertEqual(actual.status, expected.status)
        self.assertAlmostEqual(actual.x_center, expected.x_center, places=5)
        self.assertAlmostEqual(actual.y_center, expected.y_center, places=5)
        self.assertAlmostEqual(actual.confidence, expected.confidence, places=5)
        self.assertAlmostEqual(actual.inference_ms, expected.inference_ms, places=3)
        self.assertEqual(actual.error, expected.error)

    def test_truncated_payload_is_rejected(self) -> None:
        truncated = REQUEST.pack(REQUEST_MAGIC, 7, 10) + b"short"
        with self.assertRaises(EOFError):
            decode_request(BytesIO(truncated))

    def test_invalid_magic_is_rejected(self) -> None:
        request = REQUEST.pack(b"BAD!", 7, 0)
        with self.assertRaisesRegex(ValueError, "request magic"):
            decode_request(BytesIO(request))


if __name__ == "__main__":
    unittest.main(verbosity=2)
