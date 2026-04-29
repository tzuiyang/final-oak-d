#!/usr/bin/env python3
"""
Standalone test of the perception + follower pipeline on macOS.

Plug in the OAK-D Lite, run this script, and hold up a chair or ball in front
of the camera. The script prints the detection, the corresponding velocity
command the robot would receive, and exits on Ctrl-C.

No ROS 2 required. No robot required.
"""

from pathlib import Path

from detector import YoloSpatialDetector
from follower import FollowerConfig, ObjectFollower

BLOB_PATH = Path(__file__).parent / "models" / "yolov8n_coco_640x352.blob"


def main() -> int:
    cfg = FollowerConfig()
    follower = ObjectFollower(cfg)

    with YoloSpatialDetector(BLOB_PATH) as detector:
        print("Camera running. Ctrl-C to quit.")
        print(f"{'class':>12}  {'conf':>5}  {'x':>6}  {'y':>6}  {'z':>6}  "
              f"{'dist':>5}  {'bearing':>7}  |  x_vel   y_vel   ang_vel")
        for _frame, _depth_frame, detections in detector.detections():
            cmd = follower.step(detections)
            if detections:
                d = min(detections, key=lambda x: x.distance)
                print(
                    f"{d.class_name:>12}  {d.confidence:5.2f}  "
                    f"{d.x:6.2f}  {d.y:6.2f}  {d.z:6.2f}  "
                    f"{d.distance:5.2f}  {d.bearing:7.3f}  |  "
                    f"{cmd.x_vel:5.2f}  {cmd.y_vel:5.2f}  {cmd.ang_vel:6.2f}"
                )
            else:
                print(f"{'(none)':>12}  {'---':>5}  {'---':>6}  {'---':>6}  "
                      f"{'---':>6}  {'---':>5}  {'---':>7}  |  "
                      f"{cmd.x_vel:5.2f}  {cmd.y_vel:5.2f}  {cmd.ang_vel:6.2f}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
