#!/usr/bin/env python3
"""
Deployment entry point on the Pupper.

Starts the object-follower node, which publishes Twist messages on /cmd_vel.
The Pupper's default walking controller (the pretrained policy that ships
with the Pupper) is expected to already be running on the robot, and will
consume those Twist messages automatically.

If the YOLO blob hasn't been downloaded yet, it downloads it first.
"""

import os
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
BLOB_PATH = SCRIPT_DIR / "models" / "yolov8n_coco_640x352.blob"


def ensure_model() -> bool:
    if BLOB_PATH.exists():
        return True
    print(f"Model blob missing at {BLOB_PATH}. Downloading...")
    result = subprocess.run(
        [sys.executable, str(SCRIPT_DIR / "download_model.py")],
        cwd=str(SCRIPT_DIR),
    )
    return result.returncode == 0 and BLOB_PATH.exists()


def run_node() -> int:
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{SCRIPT_DIR}:{env.get('PYTHONPATH', '')}"
    result = subprocess.run(
        [sys.executable, str(SCRIPT_DIR / "object_follower_node.py")],
        env=env,
    )
    return result.returncode


def main() -> int:
    if not ensure_model():
        print("Model download failed. Aborting.")
        return 1
    return run_node()


if __name__ == "__main__":
    sys.exit(main())
