# Copyright (c) 2024, ETRI(Electronics and Telecommunications Research Institute). All rights reserved.
#
# ETRI and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure, or
# distribution of this software and related documentation without an express
# license agreement from ETRI is strictly prohibited.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


def is_valid_joint_command(input_joint_pos):
    # Check length of input_joint_pos is 49, which means the DOF of the robot
    if len(input_joint_pos) != 49:
        print('[Error] Input joint position is not valid! Check length of input_joint_pos!')
        return False

    # Check if all elements in the list are either floats or ints
    for item in input_joint_pos:
        if not isinstance(item, (float, int)):
            print('[Error] Input joint position is not valid! Check if all elements in the list are either floats or ints!')
            return False

    return True

def input_49dof_joint_position(
    liftkit_extension: float,
    head_pan_joint: float,
    head_tilt_joint: float,
    left_arm_joint_1: float, left_arm_joint_2: float, left_arm_joint_3: float,
    left_arm_joint_4: float, left_arm_joint_5: float, left_arm_joint_6: float, left_arm_joint_7: float,
    right_arm_joint_1: float, right_arm_joint_2: float, right_arm_joint_3: float,
    right_arm_joint_4: float, right_arm_joint_5: float, right_arm_joint_6: float, right_arm_joint_7: float,
    left_hand_joint_0: float, left_hand_joint_1: float, left_hand_joint_2: float,
    left_hand_joint_3: float, left_hand_joint_4: float, left_hand_joint_5: float, left_hand_joint_6: float,
    left_hand_joint_7: float, left_hand_joint_8: float, left_hand_joint_9: float, left_hand_joint_10: float,
    left_hand_joint_11: float, left_hand_joint_12: float, left_hand_joint_13: float, left_hand_joint_14: float,
    left_hand_joint_15: float,
    right_hand_joint_0: float, right_hand_joint_1: float, right_hand_joint_2: float,
    right_hand_joint_3: float, right_hand_joint_4: float, right_hand_joint_5: float, right_hand_joint_6: float,
    right_hand_joint_7: float, right_hand_joint_8: float, right_hand_joint_9: float, right_hand_joint_10: float,
    right_hand_joint_11: float, right_hand_joint_12: float, right_hand_joint_13: float, right_hand_joint_14: float,
    right_hand_joint_15: float
) -> list:
    return [
        liftkit_extension,
        head_pan_joint,
        head_tilt_joint,
        left_arm_joint_1, left_arm_joint_2, left_arm_joint_3,
        left_arm_joint_4, left_arm_joint_5, left_arm_joint_6, left_arm_joint_7,
        right_arm_joint_1, right_arm_joint_2, right_arm_joint_3,
        right_arm_joint_4, right_arm_joint_5, right_arm_joint_6, right_arm_joint_7,
        left_hand_joint_0, left_hand_joint_1, left_hand_joint_2,
        left_hand_joint_3, left_hand_joint_4, left_hand_joint_5, left_hand_joint_6,
        left_hand_joint_7, left_hand_joint_8, left_hand_joint_9, left_hand_joint_10,
        left_hand_joint_11, left_hand_joint_12, left_hand_joint_13, left_hand_joint_14,
        left_hand_joint_15,
        right_hand_joint_0, right_hand_joint_1, right_hand_joint_2,
        right_hand_joint_3, right_hand_joint_4, right_hand_joint_5, right_hand_joint_6,
        right_hand_joint_7, right_hand_joint_8, right_hand_joint_9, right_hand_joint_10,
        right_hand_joint_11, right_hand_joint_12, right_hand_joint_13, right_hand_joint_14,
        right_hand_joint_15
    ]


class JointCommandPublisherContinuous(Node):
    def __init__(self):

        super().__init__("joint_command_publisher_continuous")

        # Create the publisher. This publisher will publish a JointState message to the /joint_command topic.
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Create a JointState message
        self.joint_state = JointState()

        self.joint_state.name = [
            "liftkit_extension",

            "head_pan_joint",
            "head_tilt_joint",

            "left_arm_joint_1",
            "left_arm_joint_2",
            "left_arm_joint_3",
            "left_arm_joint_4",
            "left_arm_joint_5",
            "left_arm_joint_6",
            "left_arm_joint_7",

            "right_arm_joint_1",
            "right_arm_joint_2",
            "right_arm_joint_3",
            "right_arm_joint_4",
            "right_arm_joint_5",
            "right_arm_joint_6",
            "right_arm_joint_7",

            "left_hand_joint_0",
            "left_hand_joint_1",
            "left_hand_joint_2",
            "left_hand_joint_3",
            "left_hand_joint_4",
            "left_hand_joint_5",
            "left_hand_joint_6",
            "left_hand_joint_7",
            "left_hand_joint_8",
            "left_hand_joint_9",
            "left_hand_joint_10",
            "left_hand_joint_11",
            "left_hand_joint_12",
            "left_hand_joint_13",
            "left_hand_joint_14",
            "left_hand_joint_15",

            "right_hand_joint_0",
            "right_hand_joint_1",
            "right_hand_joint_2",
            "right_hand_joint_3",
            "right_hand_joint_4",
            "right_hand_joint_5",
            "right_hand_joint_6",
            "right_hand_joint_7",
            "right_hand_joint_8",
            "right_hand_joint_9",
            "right_hand_joint_10",
            "right_hand_joint_11",
            "right_hand_joint_12",
            "right_hand_joint_13",
            "right_hand_joint_14",
            "right_hand_joint_15",
        ]

        self.default_joints = input_49dof_joint_position(0.4,
                                                         0, 1.0559,
                                                         1.5708, -1.5708, 1.5708, -1.5708, 0, 0, 0,
                                                         1.5708, 1.5708, -1.5708, 1.5708, 0, 0, 0,
                                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

        # limiting the movements to a smaller range (this is not the range of the robot, just the range of the sinusoidal motion)
        self.max_joints = np.array(self.default_joints) + 0.3
        self.min_joints = np.array(self.default_joints) - 0.3

        # position control the robot to wiggle around each joint
        self.time_start = time.time()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Set the time stamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        ##################################
        #         Your code here         #
        ##################################
        #   => You can write code here that determines the desired joint positions, which will be published at a 20Hz frequency
        #   => This example generates sinusoidal motion for joints
        joint_position = (
            np.sin(time.time() - self.time_start) * (self.max_joints - self.min_joints) * 0.5 + self.default_joints
        )
        self.joint_state.position = joint_position.tolist()

        # Publish the message to the /joint_command topic
        if is_valid_joint_command(self.joint_state.position):
            self.publisher_.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)

    joint_command_publisher_continuous = JointCommandPublisherContinuous()

    rclpy.spin(joint_command_publisher_continuous)

    # Destroy the node explicitly
    joint_command_publisher_continuous.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
