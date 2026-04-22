#!/usr/bin/env python3
"""
Mission state machine for the OAK-D follow demo.

Flow:
  1. Pupper stands still. This node subscribes to /oakd/detections and
     periodically prints the visible objects so the operator can choose one.
  2. Operator selects a class by publishing on /oakd/select_target, e.g.:
         ros2 topic pub -1 /oakd/select_target std_msgs/String "{data: 'chair'}"
     Publishing an empty string disengages and returns to "stand still".
  3. Before engaging, this node checks whether the forward corridor toward
     the target has any other detection closer than the target itself.
     Limitation: only YOLO-visible obstacles are considered — there is no
     full depth map check. Walls and floor edges would not be detected.
  4. If clear → publish target on /oakd/target (picked up by the follower,
     which starts driving). If blocked → log and publish on /oakd/error,
     follower stays disengaged.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__("mission_controller")

        self.declare_parameter("corridor_half_width_rad", 0.25)
        self.declare_parameter("obstacle_clearance_m", 0.2)
        self.declare_parameter("print_period_s", 2.0)
        self.declare_parameter("evaluate_period_s", 0.2)

        self._latest_detections: list[dict] = []
        self._requested_target = ""
        self._engaged_target = ""

        self.create_subscription(String, "/oakd/detections", self._on_detections, 10)
        self.create_subscription(String, "/oakd/select_target", self._on_select, 10)

        self._target_pub = self.create_publisher(String, "/oakd/target", 10)
        self._error_pub = self.create_publisher(String, "/oakd/error", 10)

        self.create_timer(
            self.get_parameter("print_period_s").value, self._print_standing_menu
        )
        self.create_timer(
            self.get_parameter("evaluate_period_s").value, self._evaluate
        )

        self.get_logger().info(
            "Mission controller ready. Select a target with: "
            "ros2 topic pub -1 /oakd/select_target std_msgs/String "
            "\"{data: 'chair'}\""
        )

    def _on_detections(self, msg: String):
        payload = json.loads(msg.data)
        self._latest_detections = payload.get("detections", [])

    def _on_select(self, msg: String):
        new = (msg.data or "").strip()
        self._requested_target = new
        if new:
            self.get_logger().info(f"User selected: {new}")
        else:
            self.get_logger().info("User disengaged")
            self._publish_target("")

    def _print_standing_menu(self):
        if self._requested_target or self._engaged_target:
            return
        if not self._latest_detections:
            self.get_logger().info("Standing still — no objects detected.")
            return
        summary = ", ".join(
            f"{d['class_name']} @ {d['distance']:.2f}m"
            for d in self._latest_detections
        )
        self.get_logger().info(f"Standing still — visible: {summary}")

    def _evaluate(self):
        if not self._requested_target:
            return

        target_class = self._requested_target
        snapshot = list(self._latest_detections)
        matching = [d for d in snapshot if d["class_name"] == target_class]

        if not matching:
            self._raise_error(f"No {target_class} visible")
            self._requested_target = ""
            return

        target = min(matching, key=lambda d: d["distance"])

        corridor = self.get_parameter("corridor_half_width_rad").value
        clearance = self.get_parameter("obstacle_clearance_m").value

        blockers = [
            d
            for d in snapshot
            if d is not target
            and abs(d["bearing"] - target["bearing"]) <= corridor
            and d["z"] < target["z"] - clearance
        ]

        if blockers:
            names = ", ".join(sorted({b["class_name"] for b in blockers}))
            self._raise_error(f"Path to {target_class} blocked by: {names}")
            return

        if self._engaged_target != target_class:
            self.get_logger().info(
                f"Path clear. Engaging follower toward {target_class}."
            )
        self._publish_target(target_class)

    def _publish_target(self, class_name: str):
        self._engaged_target = class_name
        msg = String()
        msg.data = class_name
        self._target_pub.publish(msg)

    def _raise_error(self, text: str):
        self.get_logger().warn(text)
        msg = String()
        msg.data = text
        self._error_pub.publish(msg)
        if self._engaged_target:
            self._publish_target("")


def main():
    rclpy.init()
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
