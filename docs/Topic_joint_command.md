## Topic Message Details 

### Name: /joint_command

### Type: [sensor_msgs/msg/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)
- For 49 DOF position control, only the joint positions in the control command are valid for all joints.
- For 7X2 DOF velocity control(two arms) + 35 DOF position control, only the joint velocities in the control command are valid for the arm joints, while the joint positions remain valid for for all other joints.
- Effort is not used.

### Raw Message Definition

| Number | Name | Hardware component | Description |
|---|---|---|---|
| 1 | liftkit_extension | Lifting column | Desired vertical position of a lifting column  |
| 2 | head_pan_joint | Pan and tilt system | Desired position of pan joint |
| 3 | head_tilt_joint | Pan and tilt system | Desired position of tilt joint  |
| 4 | left_arm_joint_1 | Left arm | Desired position or velocity of the left arm joint 1 |
| 5 | left_arm_joint_2 | Left arm | Desired position or velocity of the left arm joint 2 |
| 6 | left_arm_joint_3 | Left arm | Desired position or velocity of the left arm joint 3 |
| 7 | left_arm_joint_4 | Left arm | Desired position or velocity of the left arm joint 4 |
| 8 | left_arm_joint_5 | Left arm | Desired position or velocity of the left arm joint 5 |
| 9 | left_arm_joint_6 | Left arm | Desired position or velocity of the left arm joint 6 |
| 10 | left_arm_joint_7 | Left arm | Desired position or velocity of the left arm joint 7 |
| 11 | right_arm_joint_1 | Right arm | Desired position or velocity of the right arm joint 1 |
| 12 | right_arm_joint_2 | Right arm | Desired position or velocity of the right arm joint 2 |
| 13 | right_arm_joint_3 | Right arm | Desired position or velocity of the right arm joint 3 |
| 14 | right_arm_joint_4 | Right arm | Desired position or velocity of the right arm joint 4 |
| 15 | right_arm_joint_5 | Right arm | Desired position or velocity of the right arm joint 5 |
| 16 | right_arm_joint_6 | Right arm | Desired position or velocity of the right arm joint 6 |
| 17 | right_arm_joint_7 | Right arm | Desired position or velocity of the right arm joint 7 |
| 18 | left_hand_joint_0 | Left hand | Desired position of the left hand joint 0 (or index finger joint 0) |
| 19 | left_hand_joint_1 | Left hand | Desired position of the left hand joint 1 (or index finger joint 1) |
| 20 | left_hand_joint_2 | Left hand | Desired position of the left hand joint 2 (or index finger joint 2) |
| 21 | left_hand_joint_3 | Left hand | Desired position of the left hand joint 3 (or index finger joint 3) |
| 22 | left_hand_joint_4 | Left hand | Desired position of the left hand joint 4 (or middle finger joint 0) |
| 23 | left_hand_joint_5 | Left hand | Desired position of the left hand joint 5 (or middle finger joint 1) |
| 24 | left_hand_joint_6 | Left hand | Desired position of the left hand joint 6 (or middle finger joint 2) |
| 25 | left_hand_joint_7 | Left hand | Desired position of the left hand joint 7 (or middle finger joint 3) |
| 26 | left_hand_joint_8 | Left hand | Desired position of the left hand joint 8 (or pinky joint 0) |
| 27 | left_hand_joint_9 | Left hand | Desired position of the left hand joint 9 (or pinky joint 1) |
| 28 | left_hand_joint_10 | Left hand | Desired position of the left hand joint 10 (or pinky joint 2) |
| 29 | left_hand_joint_11 | Left hand | Desired position of the left hand joint 11 (or pinky joint 3) |
| 30 | left_hand_joint_12 | Left hand | Desired position of the left hand joint 12 (or thumb joint 0) |
| 31 | left_hand_joint_13 | Left hand | Desired position of the left hand joint 13 (or thumb joint 1) |
| 32 | left_hand_joint_14 | Left hand | Desired position of the left hand joint 14 (or thumb joint 2) |
| 33 | left_hand_joint_15 | Left hand | Desired position of the left hand joint 15 (or thumb joint 3) |
| 34 | right_hand_joint_0 | Right hand | Desired position of the right hand joint 0 (or index finger joint 0) |
| 35 | right_hand_joint_1 | Right hand | Desired position of the right hand joint 1 (or index finger joint 1) |
| 36 | right_hand_joint_2 | Right hand | Desired position of the right hand joint 2 (or index finger joint 2) |
| 37 | right_hand_joint_3 | Right hand | Desired position of the right hand joint 3 (or index finger joint 3) |
| 38 | right_hand_joint_4 | Right hand | Desired position of the right hand joint 4 (or middle finger joint 0) |
| 39 | right_hand_joint_5 | Right hand | Desired position of the right hand joint 5 (or middle finger joint 1) |
| 40 | right_hand_joint_6 | Right hand | Desired position of the right hand joint 6 (or middle finger joint 2) |
| 41 | right_hand_joint_7 | Right hand | Desired position of the right hand joint 7 (or middle finger joint 3) |
| 42 | right_hand_joint_8 | Right hand | Desired position of the right hand joint 8 (or pinky joint 0) |
| 43 | right_hand_joint_9 | Right hand | Desired position of the right hand joint 9 (or pinky joint 1) |
| 44 | right_hand_joint_10 | Right hand | Desired position of the right hand joint 10 (or pinky joint 2) |
| 45 | right_hand_joint_11 | Right hand | Desired position of the right hand joint 11 (or pinky joint 3) |
| 46 | right_hand_joint_12 | Right hand | Desired position of the right hand joint 12 (or thumb joint 0) |
| 47 | right_hand_joint_13 | Right hand | Desired position of the right hand joint 13 (or thumb joint 1) |
| 48 | right_hand_joint_14 | Right hand | Desired position of the right hand joint 14 (or thumb joint 2) |
| 49 | right_hand_joint_15 | Right hand | Desired position of the right hand joint 15 (or thumb joint 3) |
