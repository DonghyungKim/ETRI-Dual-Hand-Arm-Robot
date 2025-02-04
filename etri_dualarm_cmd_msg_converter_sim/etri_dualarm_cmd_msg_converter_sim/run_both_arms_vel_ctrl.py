import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

class CommandMsgConverterSim(Node):
    def __init__(self, dt):
        super().__init__('cmd_msg_converter_sim')

        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Subscriber for the /joint_command topic
        self.subscription_cmd = self.create_subscription(
            JointState,
            '/joint_command',
            self.joint_command_callback,
            qos_profile
        )

        # Subscriber for the /joint_states_isaac topic
        self.subscription_states = self.create_subscription(
            JointState,
            '/joint_states_isaac',
            self.joint_states_callback,
            qos_profile
        )

        # Publisher for the /joint_command_isaac topic
        self.publisher = self.create_publisher(JointState, '/joint_command_isaac', qos_profile)

        self.joint_states = None  # To store the latest /joint_states_isaac message
        self.new_positions = {}  # Store calculated positions for integration
        self.dt = dt  # Set dt from argument
        self.last_command_time = time.time()
        self.command_timeout = 0.5  # Reset if no joint_command is received for a certain period

        self.create_timer(0.1, self.check_command_timeout)  # 0.1초마다 확인

        self.get_logger().info('Command Message Converter Node has been started.')

    def joint_states_callback(self, msg):
        """
        Callback function to process incoming joint_states messages from Isaac Sim.
        """
        self.joint_states = msg  # Store the latest /joint_states_isaac message

    def joint_command_callback(self, msg):
        """
        Callback function to process incoming joint_command messages and
        publish converted messages to /joint_command_isaac.
        """
        self.last_command_time = time.time()  # Update the last received time

        if self.joint_states is None:
            #self.get_logger().warning('No /joint_states_isaac message received yet. Skipping processing.')
            return

        # Define the mapping between input and output joint names
        joint_mapping = {
            'liftkit_extension': ['ewellix_lift_top_joint', 'ewellix_lift_top2_joint'],
            'head_pan_joint': ['pan_joint'],
            'head_tilt_joint': ['tilt_joint'],
            'left_arm_joint_1': ['left_joint_1'],
            'right_arm_joint_1': ['right_joint_1'],
            'left_arm_joint_2': ['left_joint_2'],
            'right_arm_joint_2': ['right_joint_2'],
            'left_arm_joint_3': ['left_joint_3'],
            'right_arm_joint_3': ['right_joint_3'],
            'left_arm_joint_4': ['left_joint_4'],
            'right_arm_joint_4': ['right_joint_4'],
            'left_arm_joint_5': ['left_joint_5'],
            'right_arm_joint_5': ['right_joint_5'],
            'left_arm_joint_6': ['left_joint_6'],
            'right_arm_joint_6': ['right_joint_6'],
            'left_arm_joint_7': ['left_joint_7'],
            'right_arm_joint_7': ['right_joint_7'],
            'left_hand_joint_0': ['left_hand_joint_0'],
            'left_hand_joint_1': ['left_hand_joint_1'],
            'left_hand_joint_2': ['left_hand_joint_2'],
            'left_hand_joint_3': ['left_hand_joint_3'],
            'left_hand_joint_4': ['left_hand_joint_4'],
            'left_hand_joint_5': ['left_hand_joint_5'],
            'left_hand_joint_6': ['left_hand_joint_6'],
            'left_hand_joint_7': ['left_hand_joint_7'],
            'left_hand_joint_8': ['left_hand_joint_8'],
            'left_hand_joint_9': ['left_hand_joint_9'],
            'left_hand_joint_10': ['left_hand_joint_10'],
            'left_hand_joint_11': ['left_hand_joint_11'],
            'left_hand_joint_12': ['left_hand_joint_12'],
            'left_hand_joint_13': ['left_hand_joint_13'],
            'left_hand_joint_14': ['left_hand_joint_14'],
            'left_hand_joint_15': ['left_hand_joint_15'],
            'right_hand_joint_0': ['right_hand_joint_0'],
            'right_hand_joint_1': ['right_hand_joint_1'],
            'right_hand_joint_2': ['right_hand_joint_2'],
            'right_hand_joint_3': ['right_hand_joint_3'],
            'right_hand_joint_4': ['right_hand_joint_4'],
            'right_hand_joint_5': ['right_hand_joint_5'],
            'right_hand_joint_6': ['right_hand_joint_6'],
            'right_hand_joint_7': ['right_hand_joint_7'],
            'right_hand_joint_8': ['right_hand_joint_8'],
            'right_hand_joint_9': ['right_hand_joint_9'],
            'right_hand_joint_10': ['right_hand_joint_10'],
            'right_hand_joint_11': ['right_hand_joint_11'],
            'right_hand_joint_12': ['right_hand_joint_12'],
            'right_hand_joint_13': ['right_hand_joint_13'],
            'right_hand_joint_14': ['right_hand_joint_14'],
            'right_hand_joint_15': ['right_hand_joint_15']
        }

        # Initialize the converted message
        converted_msg = JointState()
        converted_msg.header = msg.header

        # Process each input joint name and map to output joint names
        for i, name in enumerate(msg.name):
            if name in joint_mapping:
                mapped_names = joint_mapping[name]
                for j, mapped_name in enumerate(mapped_names):
                    converted_msg.name.append(mapped_name)
                    if name == 'liftkit_extension' and len(mapped_names) == 2:
                        # Input stroke 'liftkit_extension' is evenly distributed to the vertical displacements of 'ewellix_lift_top_joint' and 'ewellix_lift_top2_joint'
                        converted_msg.position.append(msg.position[i] / 2)
                    elif name.startswith('left_arm_joint') or name.startswith('right_arm_joint'):
                        # Update position based on /joint_states_isaac and velocity
                        try:
                            state_index = self.joint_states.name.index(mapped_name)
                            if mapped_name not in self.new_positions:
                                # First iteration: initialize new_position
                                new_position = self.joint_states.position[state_index] + msg.velocity[i] * self.dt
                            else:
                                # Subsequent iterations: integrate velocity
                                new_position = self.new_positions[mapped_name] + msg.velocity[i] * self.dt
                            self.new_positions[mapped_name] = new_position
                            converted_msg.position.append(new_position)
                        except ValueError:
                            self.get_logger().warning(f'Joint {mapped_name} not found in /joint_states.')
                            converted_msg.position.append(msg.position[i])
                    else:
                        converted_msg.position.append(msg.position[i])

                    # Copy velocity and effort if available
                    if msg.velocity:
                        converted_msg.velocity.append(msg.velocity[i])
                    if msg.effort:
                        converted_msg.effort.append(msg.effort[i])

        # Publish the converted message
        self.publisher.publish(converted_msg)
        # self.get_logger().info(f'Published converted message to /joint_command_isaac: {converted_msg}')

    def check_command_timeout(self):
        if time.time() - self.last_command_time > self.command_timeout:
            self.joint_states = None
            self.new_positions = {}
            # self.get_logger().warning('/joint_command timeout! Resetting joint_states and new_positions.')

def main(args=None):
    rclpy.init(args=args)

    # Parse the hz argument from the command line
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    if len(args_without_ros) <= 1 or ':=' not in args_without_ros[1]:
        raise ValueError("You must provide a valid hz value as an argument, e.g., hz:=30")

    hz_str = args_without_ros[1].split(':=')[-1]
    try:
        hz = float(hz_str)
        if hz <= 1.0:
            raise ValueError("The hz value must be greater than 1.")
    except ValueError as e:
        raise ValueError(f"Invalid hz value: {hz_str}. Please provide a valid number greater than 1.") from e

    dt = 1.0 / hz

    # Create and run the node
    node = CommandMsgConverterSim(dt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
