#!/usr/bin/env python3
"""
Fetch a YOLOv6n COCO model compiled for the OAK-D Lite's Myriad X VPU.

YOLO models are pretrained on all 80 COCO classes — we cannot download just
"chair" and "sports ball." We download the full model and filter detections
at runtime inside detector.py.

The .blob file is a Myriad X VPU binary produced from the ONNX model via
OpenVINO's Model Compiler. Luxonis hosts prebuilt blobs for common models.
"""

from pathlib import Path

import blobconverter

MODEL_NAME = "yolov8n_coco_640x352"
SHAVE_COUNT = 6  # Number of SHAVE cores allocated — 6 is standard for OAK-D Lite
OUTPUT_DIR = Path(__file__).parent / "models"


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Downloading {MODEL_NAME} ({SHAVE_COUNT} SHAVES) from Luxonis zoo...")
    blob_path = blobconverter.from_zoo(
        name=MODEL_NAME,
        shaves=SHAVE_COUNT,
        zoo_type="depthai",
        use_cache=True,
    )

    dest = OUTPUT_DIR / f"{MODEL_NAME}.blob"
    Path(blob_path).replace(dest)
    print(f"Saved: {dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
