"""Detection backends for transport testing and the existing YOLO weight."""

from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
import struct

from .beam_geometry import InvalidBeamInput


@dataclass(frozen=True)
class Detection:
    x_center: float
    y_center: float
    confidence: float
    image_width: int
    image_height: int


def png_dimensions(image_data: bytes) -> tuple[int, int]:
    """Read PNG dimensions without decoding pixels."""
    if len(image_data) < 24 or image_data[:8] != b"\x89PNG\r\n\x1a\n":
        raise InvalidBeamInput("camera_image is not a PNG byte stream")
    if image_data[12:16] != b"IHDR":
        raise InvalidBeamInput("camera_image has no PNG IHDR chunk")
    width, height = struct.unpack(">II", image_data[16:24])
    if width <= 0 or height <= 0:
        raise InvalidBeamInput("PNG dimensions must be positive")
    return width, height


class CenterDetector:
    """Transport-only backend that reports the image center."""

    def detect(self, image_data: bytes) -> Detection:
        width, height = png_dimensions(image_data)
        return Detection(width / 2.0, height / 2.0, 1.0, width, height)


class UltralyticsDetector:
    """Adapter around the existing Ultralytics YOLO inference behavior."""

    def __init__(
        self,
        model_path: str,
        image_size: int,
        confidence_threshold: float,
        device: str | None,
    ) -> None:
        resolved_model = Path(model_path).expanduser()
        if not resolved_model.is_file():
            raise FileNotFoundError(f"YOLO weight not found: {resolved_model}")

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics is not installed in the ROS Python environment"
            ) from exc

        self._model = YOLO(str(resolved_model))
        self._image_size = image_size
        self._confidence_threshold = confidence_threshold
        self._device = device

    def detect(self, image_data: bytes) -> Detection | None:
        try:
            from PIL import Image

            with Image.open(BytesIO(image_data)) as source:
                image = source.convert("RGB")
                width, height = image.size
        except InvalidBeamInput:
            raise
        except Exception as exc:
            raise InvalidBeamInput(f"camera_image decode failed: {exc}") from exc

        try:
            prediction = self._model.predict(
                source=image,
                imgsz=self._image_size,
                conf=self._confidence_threshold,
                device=self._device,
                save=False,
                verbose=False,
            )[0]
        except Exception as exc:
            raise RuntimeError(f"detector inference failed: {exc}") from exc

        boxes = prediction.boxes
        if boxes is None or len(boxes) == 0:
            return None

        confidence = boxes.conf.cpu().numpy()
        xywh = boxes.xywh.cpu().numpy()
        best_index = int(confidence.argmax())
        x_center, y_center, _, _ = xywh[best_index].tolist()
        return Detection(
            float(x_center),
            float(y_center),
            float(confidence[best_index]),
            width,
            height,
        )


def create_detector(
    backend: str,
    model_path: str,
    image_size: int,
    confidence_threshold: float,
    device: str,
):
    normalized_backend = backend.strip().lower()
    if normalized_backend == "center":
        return CenterDetector()
    if normalized_backend == "ultralytics":
        selected_device = None if device.strip().lower() == "auto" else device
        return UltralyticsDetector(
            model_path,
            image_size,
            confidence_threshold,
            selected_device,
        )
    raise ValueError(f"unsupported detector_backend: {backend}")
