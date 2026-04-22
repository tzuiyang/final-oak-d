#!/usr/bin/env python3
"""
Deployment entry point on the Pupper.

Brings up the OAK-D follower + mission controller via `ros2 launch`. The
Pupper's default walking controller (the pretrained policy that ships with
the Pupper) must already be running on the robot — our nodes only publish
to /person_following_cmd_vel and do not drive the legs directly. See
reference/pupperv3-monorepo for the upstream launch.

Downloads the YOLO blob if missing.
"""

import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
BLOB_PATH = SCRIPT_DIR / "models" / "yolov8n_coco_640x352.blob"
LAUNCH_FILE = SCRIPT_DIR / "oakd.launch.py"


def ensure_model() -> bool:
    if BLOB_PATH.exists():
        return True
    print(f"Model blob missing at {BLOB_PATH}. Downloading...")
    result = subprocess.run(
        [sys.executable, str(SCRIPT_DIR / "download_model.py")],
        cwd=str(SCRIPT_DIR),
    )
    return result.returncode == 0 and BLOB_PATH.exists()


def run_launch() -> int:
    result = subprocess.run(["ros2", "launch", str(LAUNCH_FILE)])
    return result.returncode


def main() -> int:
    if not ensure_model():
        print("Model download failed. Aborting.")
        return 1
    return run_launch()


if __name__ == "__main__":
    sys.exit(main())
