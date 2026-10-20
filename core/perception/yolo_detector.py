"""YOLO Person Detector for NIDAR AirMouse.

Runs object detection on the camera frame only.
Does not access simulator ground-truth coordinates.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


@dataclass
class DetectionBox:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_name: str

    @property
    def bottom_center(self) -> tuple[float, float]:
        """Ground contact point of the detected person in image pixel coordinates."""
        return ((self.x1 + self.x2) / 2.0, self.y2)


class YoloDetector:
    def __init__(
        self,
        weights_path: str = "yolo11n.pt",
        conf_threshold: float = 0.25,
        target_class: str = "person",
    ):
        self.weights_path = weights_path
        self.conf_thresh = conf_threshold
        self.target_class = target_class

        self.model = None
        self.status = "YOLO uninitialized"
        self._load_model()

    def _load_model(self):
        if YOLO is None:
            self.status = "ultralytics not installed"
            return
        try:
            self.model = YOLO(self.weights_path)
            self.status = f"YOLO ready ({Path(self.weights_path).name})"
        except Exception as e:
            self.status = f"YOLO load error: {str(e)[:40]}"

    def detect(self, image_bgr: np.ndarray) -> List[DetectionBox]:
        """Run person detection on a single camera frame."""
        if self.model is None:
            return []

        detections = []
        try:
            results = self.model.predict(
                image_bgr,
                imgsz=320,
                conf=self.conf_thresh,
                verbose=False,
            )
            for r in results:
                names = r.names
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    name = names.get(cls_id, str(cls_id))
                    if name == self.target_class:
                        conf = float(box.conf[0])
                        b = box.xyxy[0].cpu().numpy()
                        detections.append(DetectionBox(
                            x1=float(b[0]),
                            y1=float(b[1]),
                            x2=float(b[2]),
                            y2=float(b[3]),
                            confidence=conf,
                            class_name=name,
                        ))
        except Exception as e:
            self.status = f"Inference error: {str(e)[:40]}"

        return detections

