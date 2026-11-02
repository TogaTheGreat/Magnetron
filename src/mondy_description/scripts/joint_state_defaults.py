#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
from typing import Dict, List, Tuple

import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import JointState

UNITS_PER_REV = 169.1972

CORNER_NAME_MAP = {
    "corner_right_front": "front_wheel_joint_R",
    "corner_left_front":  "front_wheel_joint_L",
    "corner_right_back":  "rear_wheel_joint_R",
    "corner_left_back":   "rear_wheel_joint_L",

    "drive_left_front": "front_wheel_joint_left",
    "drive_right_front": "front_wheel_joint_right",
    "drive_left_middle": "middle_wheel_joint_left",
    "drive_right_middle": "middle_wheel_joint_right",
    "drive_left_back": "rear_wheel_joint_left",
    "drive_right_back": "rear_wheel_joint_right",

    "pan": "camera_pan",
    "tilt": "camera_tilt",
}


DEFAULT_JOINTS = {
    "rocker_bogie_joint_L_2": 0.0,
    "rocker_bogie_joint_R_2": 0.0,

    "front_wheel_joint_R": 0.0,
    "front_wheel_joint_L": 0.0,
    "rear_wheel_joint_R": 0.0,
    "rear_wheel_joint_L": 0.0,

    "front_wheel_joint_left": 0.0,
    "front_wheel_joint_right": 0.0,
    "middle_wheel_joint_left": 0.0,
    "middle_wheel_joint_right": 0.0,
    "rear_wheel_joint_left": 0.0,
    "rear_wheel_joint_right": 0.0,

    "camera_pan": 0.0,
    "camera_tilt": 0.0,

    # RoArm joints. Real encoders override these when alive.
    "roarm_link_to_link1": 0.0,
    "link1_to_link2": 0.0,
    "link2_to_link3": 0.0,
    "link3_to_link4": 0.0,
    "link4_to_link5": 0.0,
    "link5_to_gripper_link": 0.0,
    "link5_to_hand_tcp": 0.0,
}


def stamp_key(msg: JointState) -> Tuple[int, int]:
    return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))


class JointStateDebugFallback(Node):
    def __init__(self):
        super().__init__("joint_state_debug_fallback")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("stale_timeout_sec", 0.5)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("corner_state_topic", "/corner_state")
        self.declare_parameter("drive_state_topic", "/drive_state")
        self.declare_parameter("own_stamp_cache_size", 200)

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        joint_topic = str(self.get_parameter("joint_states_topic").value)
        corner_topic = str(self.get_parameter("corner_state_topic").value)
        drive_topic = str(self.get_parameter("drive_state_topic").value)
        cache_size = int(self.get_parameter("own_stamp_cache_size").value)

        self.default_pos: Dict[str, float] = dict(DEFAULT_JOINTS)
        self.default_vel: Dict[str, float] = {name: 0.0 for name in self.default_pos}
        self.default_eff: Dict[str, float] = {name: 0.0 for name in self.default_pos}

        self.real_pos: Dict[str, float] = {}
        self.real_vel: Dict[str, float] = {}
        self.real_eff: Dict[str, float] = {}
        self.last_real_time: Dict[str, rclpy.time.Time] = {}

        self.joint_order: List[str] = list(DEFAULT_JOINTS.keys())

        # Used to ignore our own /joint_states messages when subscribing to the same topic.
        self.own_stamps = deque(maxlen=max(cache_size, 10))
        self.own_stamp_set = set()

        self.pub = self.create_publisher(JointState, joint_topic, 20)

        self.js_sub = self.create_subscription(
            JointState,
            joint_topic,
            self.joint_states_cb,
            50,
        )

        self.corner_sub = self.create_subscription(
            JointState,
            corner_topic,
            self.corner_cb,
            20,
        )

        self.drive_sub = self.create_subscription(
            JointState,
            drive_topic,
            self.drive_cb,
            20
        )

        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self.publish_joint_states)

        self.get_logger().warn(
            f"DEBUG fallback is both subscribing and publishing on {joint_topic}. "
            f"It ignores its own messages by timestamp. "
            f"stale_timeout={self.stale_timeout:.3f}s"
        )

    def _remember_own_stamp(self, key: Tuple[int, int]):
        if key in self.own_stamp_set:
            return
        if len(self.own_stamps) == self.own_stamps.maxlen:
            old = self.own_stamps.popleft()
            self.own_stamp_set.discard(old)
        self.own_stamps.append(key)
        self.own_stamp_set.add(key)

    def _is_own_message(self, msg: JointState) -> bool:
        return stamp_key(msg) in self.own_stamp_set

    def _mark_real_joint(self, name: str, pos=None, vel=None, eff=None):
        now = self.get_clock().now()
        # self.get_logger().info(f"MARK REAL {name}: pos={pos} vel={vel}")

        if name not in self.joint_order:
            self.joint_order.append(name)
            self.default_pos.setdefault(name, 0.0)
            self.default_vel.setdefault(name, 0.0)
            self.default_eff.setdefault(name, 0.0)
            self.get_logger().info(f"Discovered joint not in defaults: {name}")

        if pos is not None:
            self.real_pos[name] = float(pos)
        if vel is not None:
            self.real_vel[name] = float(vel)
        if eff is not None:
            self.real_eff[name] = float(eff)

        self.last_real_time[name] = now

    def remap_names(self, msg:JointState):
        for i, in_name in enumerate(msg.name):
            out_name = CORNER_NAME_MAP.get(in_name)
            if out_name is None:
                continue

            pos = msg.position[i] if i < len(msg.position) else None
            vel = msg.velocity[i] if i < len(msg.velocity) else None
            eff = msg.effort[i] if i < len(msg.effort) else None
            self._mark_real_joint(out_name, pos, vel, eff)

    def joint_states_cb(self, msg: JointState):
        # Ignore our own combined fallback messages.
        if self._is_own_message(msg):
            return

        # Anything else on /joint_states is treated as a real source.
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else None
            vel = msg.velocity[i] if i < len(msg.velocity) else None
            eff = msg.effort[i] if i < len(msg.effort) else None
            self._mark_real_joint(name, pos, vel, eff)

    def corner_cb(self, msg: JointState):
        self.remap_names(msg)
    def drive_cb(self, msg: JointState):
        # self.get_logger().info("Drive CB! msg: {msg}")
        out = JointState()
        out.header = msg.header

        for i, name in enumerate(msg.name):
            if name is None:
                continue

            out.name.append(name)

            if i < len(msg.position):
                out.position.append(self.raw_to_rad(float(msg.position[i])))

            if i < len(msg.velocity):
                out.velocity.append(self.raw_to_rad(float(msg.velocity[i])))

            if i < len(msg.effort):
                out.effort.append(msg.effort[i])
        self.remap_names(out)

    def _is_fresh(self, name: str, now: rclpy.time.Time) -> bool:
        last = self.last_real_time.get(name)
        if last is None:
            return False
        age = (now - last).nanoseconds * 1e-9
        return age <= self.stale_timeout

    def raw_to_rad(self, raw):
        return raw * (2.0 * math.pi / UNITS_PER_REV)

    def publish_joint_states(self):
        now = self.get_clock().now()

        msg = JointState()
        msg.header.stamp = now.to_msg()

        for name in self.joint_order:
            fresh = self._is_fresh(name, now)

            msg.name.append(name)

            if fresh:
                msg.position.append(self.real_pos.get(name, self.default_pos.get(name, 0.0)))
                msg.velocity.append(self.real_vel.get(name, 0.0))
                msg.effort.append(self.real_eff.get(name, 0.0))
            else:
                msg.position.append(self.default_pos.get(name, 0.0))
                msg.velocity.append(self.default_vel.get(name, 0.0))
                msg.effort.append(self.default_eff.get(name, 0.0))

        self._remember_own_stamp(stamp_key(msg))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateDebugFallback()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
