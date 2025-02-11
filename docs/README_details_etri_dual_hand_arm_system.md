# Details on the ETRI's Dual Hand-Arm Robot System

## Hardware Components

The ETRI's Dual Hand-Arm Robot has a total of 49 DOF, which includes two 7 DOF arms, two 16 DOF hands, a 1 DOF lifting column, and a 2 DOF pan and tilt system for the head camera. Here, the robot is built with the following off-the-shelf components:
- Two arms: Kinova Gen 3 (7 DOF) with wrist camera
- Two hands: Wonik Robotics's Allegro Hand
- Lifting column: Ewellix LIFTKIT
- Head camera: Intel RealSense D455. The D455 is able to look in two axes using the pan and tilt system powered by ROBOTIS DYNAMIXEL

The structure of the robot is as follows: A lifting column is installed on the base of the robot. The torso, placed on top of the lifting column, has the left and right arms fixed to it. Additionally, a head pan and tilt system is mounted on the upper part of the torso.

## TF information

The transforms of important links of the robot, such as the tool frame, the root frame of the hand, and the camera frame, are published as `/tf` topic messages. To see where each frame is attached, click on the corresponding Xform in the stage panel of IssacSim.

| Frame ID  |  Xform in the stage panel | the Stage Tre Description |
|---|---|---|
| `lift_base_link` | etri_dualarm_robot &rarr; lift_base_link | The robot's base frame. It is located at the geometric center on bottom of the lifting column |
| `left_tool_frame`, `right_tool_frame` | etri_dualarm_robot &rarr; left_tool_frame and etri_dualarm_robot &rarr; right_tool_frame | The tool frame for each arm relative to `lift_base_link` |
| `left_hand_root`, `right_hand_root` | etri_dualarm_robot &rarr; left_hand_root and etri_dualarm_robot &rarr; right_hand_root | The root frame for each hand relative to `lift_base_link` |
| `head_camera:color`, `head_camera:depth` | etri_dualarm_robot &rarr; rsd455 &rarr; RSD455 &rarr; Visual &rarr; Camera_OmniVision_OV9782_Color and etri_dualarm_robot &rarr; rsd455 &rarr; RSD455 &rarr; Camera_Pseudo_Depth | The head camera's frame for RGB sensor and depth sensor relative to `lift_base_link` |
| `left_camera_color_frame`, `left_camera_depth_frame`, `right_camera_color_frame`, `right_camera_depth_frame` | etri_dualarm_robot &rarr; left_camera_color_frame, etri_dualarm_robot &rarr; left_camera_depth_frame, etri_dualarm_robot &rarr; right_camera_color_frame, etri_dualarm_robot &rarr; right_camera_depth_frame | The wrist camera's frame for RGB sensor and depth sensor relative to `lift_base_link` |

![Frames](https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/robot_frames.jpg)

## Cameras

The robot has a total of three cameras: one on the head and one on each wrist. All cameras of the robot provide RGB and Depth images, and specifically, only the head camera additionally provides a point cloud, just like the actual D455. The following is an example of RGB image and point cloud from the cameras in the Rviz2.

![Example of RGB image and point cloud from the cameras in the Rviz2](https://github.com/DonghyungKim/ETRI-Dual-Hand-Arm-Robot/blob/main/docs/example_rgb_img_and_point_cloud_in_rviz2.jpg)
