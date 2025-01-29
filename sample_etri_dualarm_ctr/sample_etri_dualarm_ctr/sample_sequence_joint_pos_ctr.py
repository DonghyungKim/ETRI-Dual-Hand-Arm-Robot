import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

demo_poses = [
    [0.4,                                               # liftkit_extension
     0, 1.0559,                                         # head_pan_joint, head_tilt_joint
     1.5708, -1.5708, 1.5708, -1.5708, 0, 0, 0,         # left arm joints
     1.5708, 1.5708, -1.5708, 1.5708, 0, 0, 0,          # right arm joints
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    # finger joints of left hand
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],   # finger joints of right hand

    [0.402,
     -0.0001, 1.0558,
     2.362, -1.8103, 1.2467, -0.1479, 0.0178, -0.5707, 0.0936,
     1.5708, 1.5708, -1.5708, 1.5708, 0.0001, -0.0001, 0.0001,
     0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0],

    [0.402,
     -0.0001, 1.0558,
     2.3628, -1.8105, 1.245, -0.1489, 0.0092, -0.6718, 0.0981,
     1.5708, 1.5708, -1.5708, 1.5708, 0.0001, -0.0001, 0.0001,
     -0.2076, 1.3496, 1.018, 1.2326, 0.0105, 1.3469, 0.9243, 1.5173, -0.0003, 1.108, 1.1257, 1.2467, 0.5705, 1.2626, 0.8681, 1.0617,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.363, 0.0, 0.0002, 0.0],

    [0.4,
     0.0, 1.0558,
     1.9894, -0.8858, 0.8647, -1.6571, 0.749, 0.0576, -0.0009,
     1.5708, 1.5708, -1.5708, 1.5708, 0.0001, -0.0001, 0.0001,
     -0.2076, 1.3496, 1.018, 1.2326, 0.0105, 1.3469, 0.9243, 1.5173, -0.0003, 1.108, 1.1257, 1.2467, 0.5705, 1.2626, 0.8681, 1.0617,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.363, 0.0, 0.0002, 0.0],

    [0.4,
     0.0, 1.0558,
     1.9894, -0.8858, 0.8647, -1.6571, 0.749, 0.0576, -0.0009,
     1.5708, 1.5708, -1.5708, 1.5708, 0.0001, -0.0001, 0.0001,
     0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.363, 0.0, 0.0002, 0.0]
]


class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__("joint_command_publisher")

        # ROS 2 publisher setup
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Initialize joint state message
        self.joint_command = JointState()
        self.joint_command.name = self._get_joint_names()

    def _get_joint_names(self):
        """Return the list of joint names."""
        return [
            "liftkit_extension",
            "head_pan_joint", "head_tilt_joint",
            *(f"left_arm_joint_{i}" for i in range(1, 8)),
            *(f"right_arm_joint_{i}" for i in range(1, 8)),
            *(f"left_hand_joint_{i}" for i in range(16)),
            *(f"right_hand_joint_{i}" for i in range(16)),
        ]

    def _is_valid_joint_command(self, joint_command):
        """Validate joint positions."""
        if len(joint_command.name) != 49:
            self.get_logger().error("Invalid joint command: Incorrect number of names for /joint_command.")
            return False
        if (len(joint_command.position) != 49) and (len(joint_command.velocity) != 49):
            self.get_logger().error("Invalid joint command: Incorrect number of position or velocity for /joint_command.")
            return False
        if not all(isinstance(pos, (float, int)) for pos in joint_command.position):
            self.get_logger().error("Invalid joint command: All positions must be floats or ints.")
            return False
        if not all(isinstance(vel, (float, int)) for vel in joint_command.velocity):
            self.get_logger().error("Invalid joint command: All velocities must be floats or ints.")
            return False
        return True

    def send_demo_joint_pos(self, joint_pos):
        self.joint_command.header.stamp = self.get_clock().now().to_msg()
        self.joint_command.position = [float(x) for x in joint_pos]

        if self._is_valid_joint_command(self.joint_command):
            self.publisher_.publish(self.joint_command)

        # Print joint positions for debugging
        # self.get_logger().info(f"Publishing /joint_command: {self.joint_command}")

    def execute_robot_motions(self):
        self.get_logger().info("Move to 1st demo pose")
        self.send_demo_joint_pos(demo_poses[0])
        time.sleep(4.0)

        self.get_logger().info("Move to 2nd demo pose")
        self.send_demo_joint_pos(demo_poses[1])
        time.sleep(5.0)

        self.get_logger().info("Move to 3rd demo pose")
        self.send_demo_joint_pos(demo_poses[2])
        time.sleep(3.0)

        self.get_logger().info("Move to 4th demo pose")
        self.send_demo_joint_pos(demo_poses[3])
        time.sleep(5.0)

        self.get_logger().info("Move to 5th demo pose")
        self.send_demo_joint_pos(demo_poses[4])

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    node.execute_robot_motions()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
