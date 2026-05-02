"""
DepthAI pipeline that runs YOLO on the OAK-D Lite and returns spatial detections.

The camera itself performs inference on its Myriad X VPU and fuses the output
with stereo depth, so each detection arrives with real-world (x, y, z) in
meters relative to the camera center.

All 80 COCO classes are exposed so the operator can pick any of them as a
follow target from the web UI; downstream nodes are responsible for any
class-specific behavior.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List, Optional, Tuple

import depthai as dai
import numpy as np

# Full COCO label map (YOLOv8n class IDs). Indexed by class_id.
TARGET_CLASSES: dict[int, str] = {
    0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 4: "airplane",
    5: "bus", 6: "train", 7: "truck", 8: "boat", 9: "traffic light",
    10: "fire hydrant", 11: "stop sign", 12: "parking meter", 13: "bench",
    14: "bird", 15: "cat", 16: "dog", 17: "horse", 18: "sheep", 19: "cow",
    20: "elephant", 21: "bear", 22: "zebra", 23: "giraffe", 24: "backpack",
    25: "umbrella", 26: "handbag", 27: "tie", 28: "suitcase", 29: "frisbee",
    30: "skis", 31: "snowboard", 32: "sports ball", 33: "kite",
    34: "baseball bat", 35: "baseball glove", 36: "skateboard", 37: "surfboard",
    38: "tennis racket", 39: "bottle", 40: "wine glass", 41: "cup", 42: "fork",
    43: "knife", 44: "spoon", 45: "bowl", 46: "banana", 47: "apple",
    48: "sandwich", 49: "orange", 50: "broccoli", 51: "carrot", 52: "hot dog",
    53: "pizza", 54: "donut", 55: "cake", 56: "chair", 57: "couch",
    58: "potted plant", 59: "bed", 60: "dining table", 61: "toilet", 62: "tv",
    63: "laptop", 64: "mouse", 65: "remote", 66: "keyboard", 67: "cell phone",
    68: "microwave", 69: "oven", 70: "toaster", 71: "sink", 72: "refrigerator",
    73: "book", 74: "clock", 75: "vase", 76: "scissors", 77: "teddy bear",
    78: "hair drier", 79: "toothbrush",
}

# YOLOv8n metadata. Must match the downloaded blob.
INPUT_SIZE = (640, 352)
NUM_CLASSES = 80
COORD_SIZE = 4
ANCHORS: list[float] = []  # YOLOv8 is anchor-free
ANCHOR_MASKS: dict[str, list[int]] = {}
IOU_THRESHOLD = 0.5
# 0.35 catches mid-range objects that hovered around the previous 0.5 cutoff
# (e.g. a chair at ~2 m on the small YOLOv8n input).
CONFIDENCE_THRESHOLD = 0.35


@dataclass
class Detection:
    """A single object detection with 3D position in camera frame.

    Camera frame convention (DepthAI): +x right, +y down, +z forward (meters).
    Bounding-box fields are normalized to [0, 1] in the preview image frame.
    """

    class_id: int
    class_name: str
    confidence: float
    x: float  # meters, right of camera center
    y: float  # meters, below camera center
    z: float  # meters, in front of camera
    bbox_xmin: float
    bbox_ymin: float
    bbox_xmax: float
    bbox_ymax: float

    @property
    def distance(self) -> float:
        """Straight-line distance from camera to object."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    @property
    def bearing(self) -> float:
        """Horizontal angle to object in radians. Positive = object is to the right."""
        return math.atan2(self.x, self.z)


class YoloSpatialDetector:
    """Context-managed DepthAI pipeline producing spatial YOLO detections."""

    def __init__(self, blob_path: Path, *, flipped: bool = False):
        """flipped=True if the OAK-D is mounted upside-down. Rotates all three
        cameras 180° and swaps the stereo correspondence so depth still works."""
        if not blob_path.exists():
            raise FileNotFoundError(
                f"Model blob not found at {blob_path}. Run download_model.py first."
            )
        self._blob_path = blob_path
        self._flipped = flipped
        self._device: Optional[dai.Device] = None
        self._queue: Optional[dai.DataOutputQueue] = None
        self._rgb_queue: Optional[dai.DataOutputQueue] = None
        self._depth_queue: Optional[dai.DataOutputQueue] = None

    def __enter__(self) -> "YoloSpatialDetector":
        pipeline = self._build_pipeline()
        self._device = dai.Device(pipeline)
        self._queue = self._device.getOutputQueue(
            name="detections", maxSize=4, blocking=False
        )
        self._rgb_queue = self._device.getOutputQueue(
            name="rgb", maxSize=4, blocking=False
        )
        self._depth_queue = self._device.getOutputQueue(
            name="depth", maxSize=4, blocking=False
        )
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._device is not None:
            self._device.close()
            self._device = None

    def _build_pipeline(self) -> dai.Pipeline:
        pipeline = dai.Pipeline()

        rotate_180 = dai.CameraImageOrientation.ROTATE_180_DEG

        # Color camera (for YOLO input)
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(*INPUT_SIZE)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(30)
        if self._flipped:
            cam.setImageOrientation(rotate_180)

        # Stereo depth from mono cameras
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera("left")
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setCamera("right")
        if self._flipped:
            mono_left.setImageOrientation(rotate_180)
            mono_right.setImageOrientation(rotate_180)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # align depth to color
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        # When flipped, the physically-left lens is rotated to spatially-right
        # (and vice versa). Swap the stereo inputs so triangulation still has
        # the actual left view feeding stereo.left.
        if self._flipped:
            mono_right.out.link(stereo.left)
            mono_left.out.link(stereo.right)
        else:
            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

        # YOLO network running on-device with spatial output
        yolo = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        yolo.setBlobPath(str(self._blob_path))
        yolo.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
        yolo.setNumClasses(NUM_CLASSES)
        yolo.setCoordinateSize(COORD_SIZE)
        yolo.setAnchors(ANCHORS)
        yolo.setAnchorMasks(ANCHOR_MASKS)
        yolo.setIouThreshold(IOU_THRESHOLD)
        yolo.setBoundingBoxScaleFactor(0.5)
        yolo.setDepthLowerThreshold(100)    # mm; ignore returns closer than 10 cm
        yolo.setDepthUpperThreshold(10_000) # mm; ignore returns farther than 10 m
        cam.preview.link(yolo.input)
        stereo.depth.link(yolo.inputDepth)

        # Output queues
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("detections")
        yolo.out.link(xout.input)

        # Frame that was actually fed to YOLO — keeps bbox coords consistent.
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        yolo.passthrough.link(xout_rgb.input)

        # Full aligned depth frame for generic path-obstacle checking.
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline

    def detections(
        self,
    ) -> Iterator[Tuple[Optional[np.ndarray], Optional[np.ndarray], List[Detection]]]:
        """Yield (frame, depth_frame, detections) tuples per YOLO inference.

        The frame is the BGR image that was fed into YOLO (so bbox coordinates
        line up). It may be None on the very first iteration if the RGB queue
        hasn't produced a packet yet. The depth frame is uint16 millimeters,
        aligned to the RGB camera, and may likewise be None on startup.
        """
        if self._queue is None or self._rgb_queue is None or self._depth_queue is None:
            raise RuntimeError("Detector used outside of `with` block.")
        while True:
            packet = self._queue.get()  # blocks until a new detection frame arrives
            rgb_packet = self._rgb_queue.tryGet()
            frame = rgb_packet.getCvFrame() if rgb_packet is not None else None
            depth_packet = self._depth_queue.tryGet()
            depth_frame = depth_packet.getFrame() if depth_packet is not None else None
            yield frame, depth_frame, self._filter(packet.detections)

    @staticmethod
    def _filter(raw_detections) -> List[Detection]:
        out: List[Detection] = []
        for d in raw_detections:
            if d.label not in TARGET_CLASSES:
                continue
            # spatialCoordinates are in mm; convert to meters
            out.append(
                Detection(
                    class_id=d.label,
                    class_name=TARGET_CLASSES[d.label],
                    confidence=float(d.confidence),
                    x=d.spatialCoordinates.x / 1000.0,
                    y=d.spatialCoordinates.y / 1000.0,
                    z=d.spatialCoordinates.z / 1000.0,
                    bbox_xmin=float(d.xmin),
                    bbox_ymin=float(d.ymin),
                    bbox_xmax=float(d.xmax),
                    bbox_ymax=float(d.ymax),
                )
            )
        return out
