## Topic Message Details 

### Name: /joint_command

### Type: [sensor_msgs/msg/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)
- For 49 DOF position control, only the joint positions in the control command are valid for all joints.
- For 7X2 DOF velocity control(two arms) + 35 DOF position control, only the joint velocities in the control command are valid for the arm joints, while the joint positions remain valid for for all other joints.
- Effort is not used.

### Raw Message Definition

| Number | Data type | Name | Description |
|---|---|---|---|
| 1 | float | liftkit_extension | Desired vertical position of a lifting column  |
| 2 | float | head_pan_joint | Desired position of pan joint |
| 3 | float | head_tilt_joint |  Desired position of tilt joint  |
| 4 | float | left_arm_joint_1 | Desired position or velocity of the left arm joint 1 |
| 5 | float | left_arm_joint_2 | Desired position or velocity of the left arm joint 2 |
| 6 | float | left_arm_joint_3 | Desired position or velocity of the left arm joint 3 |
| 7 | float | left_arm_joint_4 | Desired position or velocity of the left arm joint 4 |
| 8 | float | left_arm_joint_5 | Desired position or velocity of the left arm joint 5 |
| 9 | float | left_arm_joint_6 | Desired position or velocity of the left arm joint 6 |
| 10 | float | left_arm_joint_7 | Desired position or velocity of the left arm joint 7 |
| 11 | float | right_arm_joint_1 | Desired position or velocity of the right arm joint 1 |
| 12 | float | right_arm_joint_2 | Desired position or velocity of the right arm joint 2 |
| 13 | float | right_arm_joint_3 | Desired position or velocity of the right arm joint 3 |
| 14 | float | right_arm_joint_4 | Desired position or velocity of the right arm joint 4 |
| 15 | float | right_arm_joint_5 | Desired position or velocity of the right arm joint 5 |
| 16 | float | right_arm_joint_6 | Desired position or velocity of the right arm joint 6 |
| 17 | float | right_arm_joint_7 | Desired position or velocity of the right arm joint 7 |
| 18 | float | left_hand_joint_0 | Desired position of the left hand joint 0 |
| 19 | float | left_hand_joint_1 | Desired position of the left hand joint 1 |
| 20 | float | left_hand_joint_2 | Desired position of the left hand joint 2 |
| 21 | float | left_hand_joint_3 | Desired position of the left hand joint 3 |
| 22 | float | left_hand_joint_4 | Desired position of the left hand joint 4 |
| 23 | float | left_hand_joint_5 | Desired position of the left hand joint 5 |
| 24 | float | left_hand_joint_6 | Desired position of the left hand joint 6 |
| 25 | float | left_hand_joint_7 | Desired position of the left hand joint 7 |
| 26 | float | left_hand_joint_8 | Desired position of the left hand joint 8 |
| 27 | float | left_hand_joint_9 | Desired position of the left hand joint 9 |
| 28 | float | left_hand_joint_10 | Desired position of the left hand joint 10 |
| 29 | float | left_hand_joint_11 | Desired position of the left hand joint 11 |
| 30 | float | left_hand_joint_12 | Desired position of the left hand joint 12 |
| 31 | float | left_hand_joint_13 | Desired position of the left hand joint 13 |
| 32 | float | left_hand_joint_14 | Desired position of the left hand joint 14 |
| 33 | float | left_hand_joint_15 | Desired position of the left hand joint 15|
| 34 | float | right_hand_joint_0 | Desired position of the right hand joint 0 |
| 35 | float | right_hand_joint_1 | Desired position of the right hand joint 1 |
| 36 | float | right_hand_joint_2 | Desired position of the right hand joint 2 |
| 37 | float | right_hand_joint_3 | Desired position of the right hand joint 3 |
| 38 | float | right_hand_joint_4 | Desired position of the right hand joint 4 |
| 39 | float | right_hand_joint_5 | Desired position of the right hand joint 5 |
| 40 | float | right_hand_joint_6 | Desired position of the right hand joint 6 |
| 41 | float | right_hand_joint_7 | Desired position of the right hand joint 7 |
| 42 | float | right_hand_joint_8 | Desired position of the right hand joint 8 |
| 43 | float | right_hand_joint_9 | Desired position of the right hand joint 9|
| 44 | float | right_hand_joint_10 | Desired position of the right hand joint 10 |
| 45 | float | right_hand_joint_11 | Desired position of the right hand joint 11 |
| 46 | float | right_hand_joint_12 | Desired position of the right hand joint 12|
| 47 | float | right_hand_joint_13 | Desired position of the right hand joint 13 |
| 48 | float | right_hand_joint_14 | Desired position of the right hand joint 14 |
| 49 | float | right_hand_joint_15 | Desired position of the right hand joint 15 |
