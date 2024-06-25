# ETRI's Dual Hand-Arm Robot in Issac Sim
Here we provide USD files and ROS 2 package for ETRI's dual hand-arm robot in Isaac Sim, designed for manipulation skill learning.

<img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/etri_dualarm_prototype_isaacsim_test_grasp_success.gif" width="485" height="350"/>
<img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/etri_dualarm_prototype_isaacsim_test_grasp_with_head_cam.gif" width="1020" height="350"/>


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
- '/sample_etri_dualarm_ctr/sample_cont_ctr.py':
- '/sample_etri_dualarm_ctr/sample_discrete_ctr.py': 

