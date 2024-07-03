# Details on the ETRI's Dual Hand-Arm Robot System

## Hardware Components

The ETRI's Dual Hand-Arm Robot has a total of 49 DOF, which includes two 7 DOF arms, two 16 DOF hands, a 1 DOF lifting column, and a 2 DOF pan and tilt system for the head camera. Here, the robot is built with the following off-the-shelf components:
- Two arms: Kinova Gen 3 (7 DOF) with wrist camera
- Two hands: Wonik Robotics's Allegro Hand
- Lifting column: Ewellix LIFTKIT
- Head camera: Intel RealSense D455. The D455 is able to look in two axes using the pan and tilt system powered by ROBOTIS DYNAMIXEL

## Kinematic Structure

The structure of the robot is as follows: A lifting column is installed on the base of the robot. The torso, placed on top of the lifting column, has the left and right arms fixed to it. Additionally, a head pan and tilt system is mounted on the upper part of the torso. First, the important frames for describing the robot's kinematics are as follows.

| Name of the frame  | Description |
|---|:---:|
| `base` | The robot's base frame. It is located at the geometric center on top of the base |
| `left_arm_base_link`, `right_arm_base_link` | The base frame of the arm |
| `left_arm_tool_frame`, `right_arm_tool_frame` | The tool frame of the arm |
| `left_hand_root`, `right_hand_root` | The root frame of the hand |

<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/robot_frames.jpg" width="784" height="722"/></center>

With 49 degrees of freedom (DOF), the robot has many links and joints, making its kinematic chain very complex. You don't need to know everything, but for reference, the kinematic chain of the robot can be visualized in graph format from its URDF using `urdf_to_graphiz`. You can zoom in on the image by clicking

<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/kinematic_chain_etri_dual_hand_arm.jpg" width="655" height="932"/></center>

## Cameras

The robot has a total of three cameras: one on the head and one on each wrist. All three cameras are modeled as Intel RealSense D455 in Isaac Sim, so all specifications are identical to those of the D455. Currently, the cameras provide RGB and Depth images as output. The following is an example of publishing images from the three cameras as individual ROS 2 Topic messages and visualizing them in RViz.

<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/example_camera_rgb_img.png" width="600" height="422"/></center>
<center><img src="https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/example_camera_depth_img.png" width="600" height="422"/></center>

