# ETRI's Dual Hand-Arm Robot in Issac Sim
Here we provide USD files and ROS 2 package for ETRI's dual hand-arm robot in Isaac Sim, designed for manipulation skill learning. ETRI's dual hand-arm is a 49 DOF robot consisting of two sets of Kinova Gen3 arms and Allegro Hands. For more details on the robot system, refer to [this documentation](https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/README_details_etri_dual_hand_arm_system.md)

<img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/etri_dualarm_prototype_isaacsim_test_grasp_success.gif" width="485" height="350"/>
<img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/etri_dualarm_prototype_isaacsim_test_grasp_with_head_cam.gif" width="1020" height="350"/>


## Prerequisites

Intermediate or higher-level skills in both ROS 2 and Isaac Sim are required. And here is the version of ROS 2 and Isaac Sim that we used:

#### NVIDIA Omniverse / Isaac Sim
- Omniverse Launcher 1.9.11
- Isaac Sim 2023.1.1
#### ROS
- ROS 2 Humble Hawksbill (Ubuntu 22.04.4 LTS)

*NOTE:  It is recommended to perform ['ROS and ROS 2 Installation for Isaac Sim'](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html) in advance to ensure Isaac Sim's compatibility with ROS 2.

## Installation
Clone this git repository using the command line or download zip.
```
git clone https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot.git
```
Copy __usd_files__ folder to your IsaacSim workspace and __sample_etri_dualarm_ctr__ folder to the __src__ directory of your ROS 2 workspace. And build your ROS 2 workspace.

- __usd_files__ can be placed in any folder chosen by the user
- Let's say your ROS 2 workspace is ~/robot_ws, then the path to __sample_etri_dualarm_ctr__ should be ~/robot_ws/src/sample_etri_dualarm_ctr. You need to run colcon build, e.g.
```
cd ~/robot_ws/src
colcon build --symlink-install
```

## File Description

#### usd_files (USD file folder)
- `etri_dualarm_ros2_ctr.usd`: main USD file for Isaac Sim simulation. This not only publishes camera images(one head camera and two wrist cameras) and joint states as ROS 2 topic messages but also allows the user to send desired joint position commands via topic message.
- `etri_dualarm_prototype.usd`: USD file of ETRI's dual hand-arm robot automatically generated by Isaac Sim's URDF Importer.
- `rsd455.usd`: USD file of Intel RealSense Camera D455.

#### sample_etri_dualarm_ctr (Sample code folder)
- `/sample_etri_dualarm_ctr/sample_cont_ctr.py`: Sample code for continuously sending the robot's joint position at a 20Hz cycle.
- `/sample_etri_dualarm_ctr/sample_discrete_ctr.py`: Sample code for sending sequential joint position commands to the robot.


## How to Run

#### Run etri_dualarm_ros2_ctr.usd in Isaac Sim
Launch the Isaac Sim and open etri_dualarm_ros2_ctr.usd(File -> Open). Then ETRI's dual hand-arm robot will appear in the Viewport. Start the simulation by pressing play button ( :arrow_forward: ). If the USD file loads correctly without any issues, you can verify the list of topic messages in the terminal by using the `ros2 topic list` command.
| Topic name | Description |
|---|:---:|
| `/joint_states` | Current state of the robot. The robot has totally 49 DOF, so the state consists of 49 values. For more details, refer to 'description of ETRI's dual hand-arm robot'.|
| `/joint_command` | Joint position command. Once user sends this joint position command to the Isaac Sim simulator, the controller within the simulator moves the robot's joints to the desired positions. |
| `/head_camera_rgb`, `/head_camera_depth` | RGB/Depth image of robot's head camera (Intel RealSense D455 from the Isaac Sim assets) |
| `/wrist_camera_left_rgb`, `/wrist_camera_left_depth`, `/wrist_camera_right_rgb`, `/wrist_camera_right_depth` | RGB/Depth image of robot's wrist camera (Intel RealSense D455 from the Isaac Sim assets) |
| `/tf_left_arm_tool`, `/tf_left_hand_root`, `/tf_right_arm_tool`, `/tf_right_hand_root` | Position and orientation of robot's tool frame/hand root frame with respect to base frame |

Referring to the Stage panel in Isaac Sim, key features here are as follows:
- Please note that the robot in __etri_dualarm_ros2_ctr.usd__ is different from the robot in __etri_dualarm_prototype.usd__, as we have tuned various parameter values such as Joint Drive Gains through trial and error.
- As seen in the ActionGraph, __etri_dualarm_ros2_ctr.usd__ not only utilizes Omnigraph nodes for ROS 2 message communication but also controls the robot's joint positions using the Articulation Controller.

#### Run Sample code (ROS 2 package)
While Isaac Sim is running with __etri_dualarm_ros2_ctr.usd__, enter the following command in the terminal. This will execute the sample_cont_ctr.py code, which sends the robot's joint positions at a 20Hz cycle.
```
ros2 run sample_etri_dualarm_ctr sample_cont_ctr
```
<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/etri_dualarm_sample_cont.gif" width="407" height="321"/></center>

Let's run another sample code that send sequential joint position commands to the robot(sample_discrete_ctr.py):
```
ros2 run sample_etri_dualarm_ctr sample_discrete_ctr
```
<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/etri_dualarm_sample_discrete.gif" width="407" height="321"/></center>

We will cover the details of the sample code in the next section.

## How to Use

This chapter describes how to use ETRI's dual hand-arm robot to your research.

#### Preperation
For your Let's make a copy of __etri_dualarm_ros2_ctr.usd__ and rename it. Since we are going to use new environment called 'simple room' with the robot, the name of the new file is:
```
etri_dualarm_simple_room_example.usd
```

#### Changing the environment and adding the objects

Let's use the simple room from the [Environment Assets](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/assets/usd_assets_environments.html). In version 2023.1.1, the USD file path for the simple room is as follows. Delete the default environment and add the simple room. Then, change the position and orientation of the robot and the room as you want.
```
omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Room/simple_room.usd
```
Let's add objects like [YCB objects](https://www.ycbbenchmarks.com/object-set/) to the simple room. The following is the path to the USD file.
```
omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/YCB/
```
Here are examples of using various environments from Isaac Sim's assets, including the simple room.
<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/example_environments.jpg" width="990" height="176"/></center>

We suggest you start by using the assets in Isaac Sim. Feel free to make your own custom environment.

#### Setting initial joint positions

The default joint positions of the robot needed to be changed depending on the robot's task and environments. You can modify the initial joint positions using the Physics Inspector, such as the joint angles of the robot arm, the pan/tilt angles of the camera head, and the stroke length of the lifting column. Refer to the details on the Physics Inspector [here](https://docs.omniverse.nvidia.com/extensions/latest/ext_physics/support-ui.html#physics-inspector).

<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/setting_init_joint_pos.png" width="500" height="386"/></center>

#### Using sample code to control the robot

Try applying the sample code from the previous section to your own robot application.

1. `/sample_etri_dualarm_ctr/sample_cont_ctr.py`

Within the JointCommandPublisherContinuous class, self.default_joint represents the default joint position for the robot. The input_49dof_joint_position function takes the joint positions as arguments and returns them as a list. Since there are 49 joint position values to input, this method is used to prevent confusion when entering them manually.

```python
self.default_joints = input_49dof_joint_position(0.4,
                                                  0, 1.0559,
                                                  1.5708, -1.5708, 1.5708, -1.5708, 0, 0, 0,
                                                  1.5708, 1.5708, -1.5708, 1.5708, 0, 0, 0,
                                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
```

The maximum and minimum displacement values for the sinusoidal motion were set to 0.3 radians to generate a sinusoidal motion within the range of ±0.3 radians from the default joint position. Additionally, the `timer_period` for the `timer_callback` was set to 0.5 seconds to match the 20Hz control frequency of Isaac Sim's Articulation Controller

```python
# limiting the movements to a smaller range (this is not the range of the robot, just the range of the sinusoidal motion)
self.max_joints = np.array(self.default_joints) + 0.3
self.min_joints = np.array(self.default_joints) - 0.3

# position control the robot to wiggle around each joint
self.time_start = time.time()

timer_period = 0.05  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)

```

In the following `timer_callback` function, the joint position generated using the sine function was assigned to `self.joint_state.position`. You can delete this part and write your own code.

```python
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
```

2. `/sample_etri_dualarm_ctr/sample_discrete_ctr.py`

In the main function, five demo joint positions, from `demo_pose_1` to `demo_pose_5`, are defined. The `send_joint_command` function is used to publish topic messages, and the `sleep` function is used to wait while the robot moves to the commanded joint positions. You can delete or modify this part to repeatedly publish the desired joint position and wait for the robot to move accordingly.

```python
joint_command_publisher_discrete.send_joint_command(demo_pose_1)
time.sleep(6.0)  # seconds
```

Here, we have arbitrarily set the waiting time, but the user can modify the code so that the next target joint position is automatically published when the current joint position reaches the target joint position by using the `timer_callback` function.



( :construction: Under Construction! :construction: )


## Acknowledgements
This work was supported by Electronics and Telecommunications Research Institute (ETRI) grant funded by the Korean government foundation. [24ZB1200, Research of Human-centered autonomous intelligence system original technology]

