import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

class CommandMsgConverterSim(Node):
    def __init__(self):
        super().__init__('cmd_msg_converter_sim')

        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Subscriber for the /joint_command topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_command',
            self.joint_command_callback,
            qos_profile

        )

        # Publisher for the /joint_command_isaac topic
        self.publisher = self.create_publisher(JointState, '/joint_command_isaac', 10)

        self.get_logger().info('Command Message Converter Node has been started.')

    def joint_command_callback(self, msg):
        """
        Callback function to process incoming joint_command messages and
        publish converted messages to /joint_command_isaac.
        """
        # self.get_logger().info(f'Received message on /joint_command: {msg}')

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
                    else:
                        converted_msg.position.append(msg.position[i])
                    if msg.velocity:
                        converted_msg.velocity.append(msg.velocity[i])
                    if msg.effort:
                        converted_msg.effort.append(msg.effort[i])

        # Publish the converted message
        self.publisher.publish(converted_msg)
        # self.get_logger().info(f'Published converted message to /joint_command_isaac: {converted_msg}')

def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    node = CommandMsgConverterSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
