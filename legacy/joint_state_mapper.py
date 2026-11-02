#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


CORNER_NAME_MAP = {
    "corner_right_front": "front_wheel_joint_R",
    "corner_left_front":  "front_wheel_joint_L",
    "corner_right_back":  "rear_wheel_joint_R",
    "corner_left_back":   "rear_wheel_joint_L",
    "pan": "camera_pan",
    "tilt": "camera_tilt",
}

# These joints exist in the OSR URDF but may not have real sensors yet.
DEFAULT_JOINTS = {
    # Passive rocker-bogie hinge joints. Zero is a good starting pose.
    "rocker_bogie_joint_L_2": 0.0,
    "rocker_bogie_joint_R_2": 0.0,

    # Steering/corner joints. Overwritten by /corner_state when available.
    "front_wheel_joint_R": 0.0,
    "front_wheel_joint_L": 0.0,
    "rear_wheel_joint_R": 0.0,
    "rear_wheel_joint_L": 0.0,

    # Wheel spin joints. Keep zero until encoder animation is added.
    "front_wheel_joint_left": 0.0,
    "front_wheel_joint_right": 0.0,
    "middle_wheel_joint_left": 0.0,
    "middle_wheel_joint_right": 0.0,
    "rear_wheel_joint_left": 0.0,
    "rear_wheel_joint_right": 0.0,

    "camera_pan": 0.0,
    "camera_tilt": 0.0,

    # yeesh id wana rename them but these are the arm joints fr fr
    # NOTE: Below is commented out because it is incompatible with actual
    # encoders from Mon-D. If not using encoders, uncomment to set initial pose.
    #"roarm_link_to_link1": 0.0,
    #"link1_to_link2": 0.0,
    #"link2_to_link3": 0.0,
    #"link3_to_link4": 0.0,
    #"link4_to_link5": 0.0,
    #"link5_to_gripper_link": 0.0,
    #"link5_to_hand_tcp": 0.0,

}


class OsrJointStateMapper(Node):
    def __init__(self):
        super().__init__("osr_joint_state_mapper")

        self.joints = dict(DEFAULT_JOINTS)
        self.velocities = {name: 0.0 for name in self.joints}
        self.efforts = {name: 0.0 for name in self.joints}

        self.pub = self.create_publisher(JointState, "/joint_states", 20)
        self.js_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.js_cb,
            20,
        )

        self.corner_sub = self.create_subscription(
            JointState,
            "/corner_state",
            self.corner_cb,
            20,
        )

        # Publish continuously so robot_state_publisher always has a full joint set.
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)

    def corner_cb(self, msg: JointState):
        for i, in_name in enumerate(msg.name):
            out_name = CORNER_NAME_MAP.get(in_name)
            if out_name is None:
                continue

            if i < len(msg.position):
                self.joints[out_name] = msg.position[i]

            if i < len(msg.velocity):
                self.velocities[out_name] = msg.velocity[i]

            if i < len(msg.effort):
                self.efforts[out_name] = msg.effort[i]

    def js_cb(self, msg: JointState):
        # TODO: make a flexible system that publishes default states for
        # topics not publishing joints themselves
        return

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Stable order makes debugging easier.
        names = list(DEFAULT_JOINTS.keys())

        msg.name = names
        msg.position = [self.joints.get(name, 0.0) for name in names]
        msg.velocity = [self.velocities.get(name, 0.0) for name in names]
        msg.effort = [self.efforts.get(name, 0.0) for name in names]

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OsrJointStateMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
