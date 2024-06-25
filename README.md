# ETRI's Dual Hand-Arm Robot in Issac Sim
Here we provide USD files and ROS 2 package for ETRI's dual hand-arm robot in Isaac Sim, designed for manipulation skill learning.

<img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/etri_dualarm_prototype_isaacsim_test_grasp_success.gif" width="485" height="350"/>
<img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/etri_dualarm_prototype_isaacsim_test_grasp_with_head_cam.gif" width="1020" height="350"/>


## Prerequisites
#### NVIDIA Omniverse / Isaac Sim
- Omniverse Launcher 1.9.11
- Isaac Sim 2023.1.1
#### ROS
- ROS 2 Humble Hawksbill (Ubuntu 22.04.4 LTS)

*NOTE: It is recommended to perform [1] in advance to ensure Isaac Sim's compatibility with ROS 2. Also intermediate or higher ROS 2 skills are required.

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

#### usd_files
- `etri_dualarm_ros2_ctr.usd`: main USD file for Isaac Sim simulation. This not only publishes camera images(one head camera and two wrist cameras) and joint states as ROS 2 topic messages but also allows the user to send desired joint position commands via topic message.
- `etri_dualarm_prototype.usd`: USD file of ETRI's dual hand-arm robot automatically generated by Isaac Sim's URDF Importer.
- `rsd455.usd`: USD file of Intel RealSense Camera D455.

#### sample_etri_dualarm_ctr
- `/sample_etri_dualarm_ctr/sample_cont_ctr.py`: Sample code for continuously sensing the robot's joint position at a 20Hz cycle.
- `/sample_etri_dualarm_ctr/sample_discrete_ctr.py`: Sample code for sending sequential joint position commands to the robot.


## How to Run

#### Run etri_dialarm_ros2_ctr.usd in Isaac Sim
Launch the Isaac Sim and open etri_dialarm_ros2_ctr.usd(File -> Open). Then ETRI's dual hand-arm robot will appear. Start the simulation by pressing play button ( :arrow_forward: ). If the USD file loads correctly without any issues, you can verify the list of topic messages in the terminal by using the `ros2 topic list` command.
| Topic name | Description |
|---|:---:|
| `/joint_states` | Current state of the robot. The robot has totally 49 DOF, so the state consists of 49 values. For more details, refer to [2].|
| `/joint_command` | Joint position command. Once user sends this joint position command to the Isaac Sim simulator, the controller within the simulator moves the robot's joints to the desired positions. |
| `/head_camera_rgb`, `/head_camera_depth` | RGB/Depth image of robot's head camera (Intel RealSense D455 from the Isaac Sim assets) |
| `/wrist_camera_left_rgb`, `/wrist_camera_left_depth`, `/wrist_camera_right_rgb`, `/wrist_camera_right_depth` | RGB/Depth image of robot's wrist camera (Intel RealSense D455 from the Isaac Sim assets) |
| `/tf_left_arm_tool`, `/tf_left_hand_root`, `/tf_right_arm_tool`, `/tf_right_hand_root` | Position and orientation of robot's tool frame/hand root frame with respect to base frame |

#### Run Sample code (ROS 2 package)
While Isaac Sim is running with __etri_dialarm_ros2_ctr.usd__, enter the following command in the terminal. This will execute the sample_cont_ctr.py code, which sends the robot's joint positions at a 20Hz cycle.
```
ros2 run sample_etri_dualarm_ctr sample_cont_ctr
```
<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/etri_dualarm_sample_cont.gif" width="407" height="321"/></center>

Let's run another sample code that send sequential joint position commands to the robot(sample_discrete_ctr.py):
```
ros2 run sample_etri_dualarm_ctr sample_discrete_ctr
```
<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/etri_dualarm_sample_discrete.gif" width="407" height="321"/></center>




