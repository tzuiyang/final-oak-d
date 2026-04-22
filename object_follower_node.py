#!/usr/bin/env python3
"""
ROS 2 node that runs the OAK-D + YOLO + follower pipeline on the Pupper and
publishes velocity commands on the same topic the Lab 5 gamepad currently uses.

Topic published:  /cmd_vel   (geometry_msgs/Twist)
Rate:             ~15 Hz (bounded by YOLO inference on OAK-D)

Parameters (see config.yaml):
    model_blob_path   str    Path to the compiled YOLO blob
    target_distance   float  Stop when target is within this many meters
    k_forward         float  Forward-velocity gain
    k_yaw             float  Yaw-velocity gain
    max_forward_vel   float  Clamp on linear.x
    max_yaw_vel       float  Clamp on angular.z
    preferred_class   str    "chair", "sports ball", or "" for closest-anything
    timeout_frames    int    Zero-command this many dropped frames
"""

from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from detector import YoloSpatialDetector
from follower import FollowerConfig, ObjectFollower


class ObjectFollowerNode(Node):
    def __init__(self):
        super().__init__("object_follower")

        self.declare_parameter("model_blob_path", "models/yolov8n_coco_640x352.blob")
        self.declare_parameter("target_distance", 0.4)
        self.declare_parameter("k_forward", 0.8)
        self.declare_parameter("k_yaw", 1.2)
        self.declare_parameter("max_forward_vel", 0.5)
        self.declare_parameter("max_yaw_vel", 1.5)
        self.declare_parameter("preferred_class", "")
        self.declare_parameter("timeout_frames", 10)

        cfg = FollowerConfig(
            target_distance=self.get_parameter("target_distance").value,
            k_forward=self.get_parameter("k_forward").value,
            k_yaw=self.get_parameter("k_yaw").value,
            max_forward_vel=self.get_parameter("max_forward_vel").value,
            max_yaw_vel=self.get_parameter("max_yaw_vel").value,
            preferred_class=(self.get_parameter("preferred_class").value or None),
            timeout_frames=self.get_parameter("timeout_frames").value,
        )
        self._follower = ObjectFollower(cfg)

        blob_path = Path(self.get_parameter("model_blob_path").value)
        if not blob_path.is_absolute():
            blob_path = Path(__file__).parent / blob_path

        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._detector_ctx = YoloSpatialDetector(blob_path)
        self._detector = self._detector_ctx.__enter__()
        self.get_logger().info(f"OAK-D pipeline started with model {blob_path}")

        # Drive the loop off a timer so ROS shutdown is clean.
        self._timer = self.create_timer(0.01, self._tick)
        self._detection_stream = self._detector.detections()

    def _tick(self):
        try:
            detections = next(self._detection_stream)
        except StopIteration:
            return

        cmd = self._follower.step(detections)

        msg = Twist()
        msg.linear.x = cmd.x_vel
        msg.linear.y = cmd.y_vel
        msg.angular.z = cmd.ang_vel
        self._publisher.publish(msg)

    def destroy_node(self):
        try:
            self._detector_ctx.__exit__(None, None, None)
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = ObjectFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
