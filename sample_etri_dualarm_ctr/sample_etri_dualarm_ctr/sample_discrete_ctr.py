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


class JointCommandPublisherDiscrete(Node):
    def __init__(self):

      super().__init__("joint_command_publisher_discrete")

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

    def send_joint_command(self, joint_pos):

       if is_valid_joint_command(joint_pos):
          self.input_joint_pos = joint_pos

          # Set the time stamp
          self.joint_state.header.stamp = self.get_clock().now().to_msg()

          # Ensure all elements in default_joints are floats
          self.joint_state.position = [float(x) for x in self.input_joint_pos]

          # Publish the message to the /joint_command topic
          self.publisher_.publish(self.joint_state)


def main(args=None):

    rclpy.init(args=args)

    joint_command_publisher_discrete = JointCommandPublisherDiscrete()

    ##################################
    #         Your code here         #
    ##################################
    #   => Write your code here to sequentially move the robot to a specific position
    #   => This example sequentially moves the robot through a total of five demo positions

    demo_pose_1 = input_49dof_joint_position(0.4,
                                             0, 1.0559,
                                             1.5708, -1.5708, 1.5708, -1.5708, 0, 0, 0,
                                             1.5708, 1.5708, -1.5708, 1.5708, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    demo_pose_2 = input_49dof_joint_position(0.402,
                                             -0.0001, 1.0558,
                                             2.362, -1.8103, 1.2467, -0.1479, 0.0178, -0.5707, 0.0936,
                                             1.5708, 1.5708, -1.5708, 1.5708, 0.0001, -0.0001, 0.0001,
                                             0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0)

    demo_pose_3 = input_49dof_joint_position(0.402,
                                             -0.0001, 1.0558,
                                             2.3628, -1.8105, 1.245, -0.1489, 0.0092, -0.6718, 0.0981,
                                             1.5708, 1.5708, -1.5708, 1.5708, 0.0001, -0.0001, 0.0001,
                                             -0.2076, 1.3496, 1.018, 1.2326, 0.0105, 1.3469, 0.9243, 1.5173, -0.0003, 1.108, 1.1257, 1.2467, 0.5705, 1.2626, 0.8681, 1.0617,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.363, 0.0, 0.0002, 0.0)

    demo_pose_4 = input_49dof_joint_position(0.4,
                                             0.0, 1.0558,
                                             1.9894, -0.8858, 0.8647, -1.6571, 0.749, 0.0576, -0.0009,
                                             1.5708, 1.5708, -1.5708, 1.5708, 0.0001, -0.0001, 0.0001,
                                             -0.2076, 1.3496, 1.018, 1.2326, 0.0105, 1.3469, 0.9243, 1.5173, -0.0003, 1.108, 1.1257, 1.2467, 0.5705, 1.2626, 0.8681, 1.0617,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.363, 0.0, 0.0002, 0.0)

    demo_pose_5 = input_49dof_joint_position(0.4,
                                             0.0, 1.0558,
                                             1.9894, -0.8858, 0.8647, -1.6571, 0.749, 0.0576, -0.0009,
                                             1.5708, 1.5708, -1.5708, 1.5708, 0.0001, -0.0001, 0.0001,
                                             0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.363, 0.0, 0.0002, 0.0)

    joint_command_publisher_discrete.send_joint_command(demo_pose_1)
    time.sleep(6.0)  # seconds

    joint_command_publisher_discrete.send_joint_command(demo_pose_2)
    time.sleep(4.0)  # seconds

    joint_command_publisher_discrete.send_joint_command(demo_pose_3)
    time.sleep(4.0)  # seconds

    joint_command_publisher_discrete.send_joint_command(demo_pose_4)
    time.sleep(4.0)  # seconds

    joint_command_publisher_discrete.send_joint_command(demo_pose_5)

    # Destroy the node explicitly
    joint_command_publisher_discrete.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


