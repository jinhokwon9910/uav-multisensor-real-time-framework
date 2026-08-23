"""Persistent YOLO worker executed by a Windows Conda Python interpreter.

The ROS 2 node runs in WSL and exchanges length-prefixed binary records with
this worker over stdin/stdout. Human-readable diagnostics always use stderr so
they cannot corrupt the protocol stream.
"""

from __future__ import annotations

import argparse
from io import BytesIO
import os
from pathlib import Path
import struct
import sys
import time


REQUEST = struct.Struct(">4sII")
RESPONSE = struct.Struct(">4sIBffffI")
REQUEST_MAGIC = b"YBR1"
RESPONSE_MAGIC = b"YBS1"
MAX_IMAGE_BYTES = 64 * 1024 * 1024
MAX_ERROR_BYTES = 16 * 1024

STATUS_OK = 0
STATUS_NO_DETECTION = 1
STATUS_INFERENCE_ERROR = 3


def _read_exact(stream, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            return None if not chunks else bytes(chunks)
        chunks.extend(chunk)
    return bytes(chunks)


def _send_result(
    stream,
    sequence: int,
    status: int,
    x_center: float = 0.0,
    y_center: float = 0.0,
    confidence: float = 0.0,
    inference_ms: float = 0.0,
    error: str = "",
) -> None:
    error_bytes = error.encode("utf-8", errors="replace")[:MAX_ERROR_BYTES]
    stream.write(
        RESPONSE.pack(
            RESPONSE_MAGIC,
            sequence,
            status,
            x_center,
            y_center,
            confidence,
            inference_ms,
            len(error_bytes),
        )
    )
    stream.write(error_bytes)
    stream.flush()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Persistent Ultralytics worker for the WSL ROS 2 bridge."
    )
    parser.add_argument("--model", required=True)
    parser.add_argument("--config-dir", required=True)
    parser.add_argument("--image-size", type=int, default=960)
    parser.add_argument("--confidence", type=float, default=0.01)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--warmup-image", default="")
    args = parser.parse_args()
    if args.image_size <= 0:
        parser.error("--image-size must be positive")
    if not 0.0 <= args.confidence <= 1.0:
        parser.error("--confidence must be between 0 and 1")
    return args


def main() -> int:
    protocol_input = sys.stdin.buffer
    protocol_output = sys.stdout.buffer
    sys.stdout = sys.stderr

    args = _parse_args()
    model_path = Path(args.model).expanduser()
    config_dir = Path(args.config_dir).expanduser()
    warmup_image = Path(args.warmup_image).expanduser() if args.warmup_image else None

    if not model_path.is_file():
        raise FileNotFoundError(f"YOLO model does not exist: {model_path}")
    if warmup_image is not None and not warmup_image.is_file():
        raise FileNotFoundError(f"Warm-up image does not exist: {warmup_image}")
    config_dir.mkdir(parents=True, exist_ok=True)
    os.environ["YOLO_CONFIG_DIR"] = str(config_dir)

    from PIL import Image
    import torch
    from ultralytics import YOLO

    print("Loading YOLO model...", flush=True)
    model = YOLO(str(model_path))
    requested_device = args.device.strip()
    if requested_device.lower() == "auto":
        selected_device = "0" if torch.cuda.is_available() else "cpu"
    elif requested_device.lower() == "cpu":
        selected_device = "cpu"
    elif not torch.cuda.is_available():
        raise RuntimeError(
            f"CUDA device {requested_device!r} was requested, but CUDA is unavailable"
        )
    else:
        selected_device = requested_device

    if warmup_image is not None:
        model.predict(
            source=str(warmup_image),
            imgsz=args.image_size,
            conf=args.confidence,
            device=selected_device,
            save=False,
            verbose=False,
        )

    print(f"READY device={selected_device}", flush=True)

    while True:
        header = _read_exact(protocol_input, REQUEST.size)
        if header is None:
            return 0
        if len(header) != REQUEST.size:
            print("Incomplete request header; stopping worker.", file=sys.stderr)
            return 2

        magic, sequence, image_size_bytes = REQUEST.unpack(header)
        if magic != REQUEST_MAGIC or image_size_bytes > MAX_IMAGE_BYTES:
            print("Invalid request header; stopping worker.", file=sys.stderr)
            return 2

        image_bytes = _read_exact(protocol_input, image_size_bytes)
        if image_bytes is None or len(image_bytes) != image_size_bytes:
            print("Incomplete image payload; stopping worker.", file=sys.stderr)
            return 2

        started = time.perf_counter()
        try:
            with Image.open(BytesIO(image_bytes)) as source:
                image = source.convert("RGB")
                prediction = model.predict(
                    source=image,
                    imgsz=args.image_size,
                    conf=args.confidence,
                    device=selected_device,
                    save=False,
                    verbose=False,
                )[0]

            inference_ms = (time.perf_counter() - started) * 1000.0
            boxes = prediction.boxes
            if boxes is None or len(boxes) == 0:
                _send_result(
                    protocol_output,
                    sequence,
                    STATUS_NO_DETECTION,
                    inference_ms=inference_ms,
                )
                continue

            confidences = boxes.conf.detach().cpu().numpy()
            centers = boxes.xywh.detach().cpu().numpy()
            best_index = int(confidences.argmax())
            x_center, y_center, _, _ = centers[best_index].tolist()
            _send_result(
                protocol_output,
                sequence,
                STATUS_OK,
                float(x_center),
                float(y_center),
                float(confidences[best_index]),
                inference_ms,
            )
        except Exception as exc:
            inference_ms = (time.perf_counter() - started) * 1000.0
            _send_result(
                protocol_output,
                sequence,
                STATUS_INFERENCE_ERROR,
                inference_ms=inference_ms,
                error=f"{type(exc).__name__}: {exc}",
            )


if __name__ == "__main__":
    raise SystemExit(main())
