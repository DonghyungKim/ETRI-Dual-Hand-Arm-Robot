import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

# Constants for generating sinusoidal joint motion
SINE_FREQUENCY = 0.5  # Hz
PUBLISH_FREQUENCY = 30.0  # Hz
ITERATIONS = 2  # Number of sine wave cycles

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__("joint_command_publisher")

        # ROS 2 publisher setup
        self.publisher_ = self.create_publisher(JointState, "joint_command", 10)

        # Initialize joint state message
        self.joint_command = JointState()
        self.joint_command.name = self._get_joint_names()

        # Set default positions for all joints
        self.default_position = self._get_default_joint_position()

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

    def _get_default_joint_position(self):
        """Return the default joint position"""
        return np.array([
            0.4,  # liftkit_extension
            0, 1.0559,  # head_pan_joint, head_tilt_joint
            1.5708, -1.5708, 1.5708, -1.5708, 0, 0, 0,  # left arm joints
            1.5708, 1.5708, -1.5708, 1.5708, 0, 0, 0,  # right arm joints
            *(0 for _ in range(16)), # finger joints of left hand
            *(0 for _ in range(16))  # finger joints of right hand
        ])

    def publish_sinusoidal_joint_position(self, joint_names, amplitude):
        """Publish sine wave positions for the specified joints."""
        total_duration = ITERATIONS / SINE_FREQUENCY
        start_time = time.time()

        # Extract default positions for the given joints
        joint_indices = [self.joint_command.name.index(name) for name in joint_names]
        default_position = self.default_position[joint_indices]

        while time.time() - start_time < total_duration:
            elapsed_time = time.time() - start_time

            # Calculate sine wave positions
            sine_positions = default_position + amplitude * np.sin(2 * np.pi * SINE_FREQUENCY * elapsed_time)

            # Update joint positions
            for idx, joint_idx in enumerate(joint_indices):
                self.default_position[joint_idx] = sine_positions[idx]

            # Update and publish joint state message if valid
            self.joint_command.header.stamp = self.get_clock().now().to_msg()
            self.joint_command.position = self.default_position.tolist()

            if self._is_valid_joint_command(self.joint_command):
                self.publisher_.publish(self.joint_command)

            # Print joint positions for debugging
            # self.get_logger().info(f"Publishing joint positions: {self.joint_command}")

            time.sleep(1 / PUBLISH_FREQUENCY)

    def execute_robot_motions(self):
        self.get_logger().info("Move the lifting column")
        self.publish_sinusoidal_joint_position(["liftkit_extension"], 0.1)  # Unit: m

        self.get_logger().info("Move the head pan")
        self.publish_sinusoidal_joint_position(["head_pan_joint"], 0.3)     # Unit: rad

        self.get_logger().info("Move the head tilt")
        self.publish_sinusoidal_joint_position(["head_tilt_joint"], 0.3)    # Unit: rad

        self.get_logger().info("Move the left arm")
        self.publish_sinusoidal_joint_position([f"left_arm_joint_{i}" for i in range(1, 8)], 0.2)   # Unit: rad

        self.get_logger().info("Move the right arm")
        self.publish_sinusoidal_joint_position([f"right_arm_joint_{i}" for i in range(1, 8)], 0.2)  # Unit: rad

        self.get_logger().info("Move the left hand")
        self.publish_sinusoidal_joint_position([f"left_hand_joint_{i}" for i in range(16)], 0.6)    # Unit: rad

        self.get_logger().info("Move the right hand")
        self.publish_sinusoidal_joint_position([f"right_hand_joint_{i}" for i in range(16)], 0.6)   # Unit: rad

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    node.execute_robot_motions()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
