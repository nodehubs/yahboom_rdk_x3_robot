English| [简体中文](./README_cn.md)

# Feature Introduction

Yahboom Intelligent RKD X3 ROBOT is an intelligent robot development kit for robot developers and educational ecosystems. This kit is powered by the Sunrise X3 chip as the core computing unit and the TogetherROS.Bot as the core robot operating system, integrating functions such as mapping, navigation, human tracking, and gesture recognition. This project is the minimum functional system for the Yahboom Intelligent RKD X3 ROBOT, which can achieve basic motion control and pose feedback for the RKD X3 ROBOT. The project includes three function packages:

- yahboom_sunrise_robot_lib: Responsible for basic communication with the MCU and command conversion
- yahboomcar_base_node: Responsible for updating the robot's Odom and tf information
- yahboomcar_description: Responsible for describing the robot's mechanical structure information
- yahboomcar_bringup: Responsible for robot initialization, subscribing to motion control topics, publishing robot status description topics, and IMU information

# Bill of Materials

| Robot Name           | Manufacturer | Reference Link                                                |
| :------------------ | :-------- | :------------------------------------------------------------ |
| Yahboom Intelligent RKD X3 ROBOT | Yahboom | [Purchase Link](https://detail.tmall.com/item.htm?abbucket=8&id=726857243156&rn=bf4bcf345d154daf6f1572f015c838f4&scene=taobao_shop&spm=a1z10.1-b-s.w5003-25077533329.15.324244e1UOLGab)                       |
| RDK X3             | Various Manufacturers | [Purchase Link](https://developer.horizon.ai/sunrise) |

# Instructions for Use

## Preparation

Refer to the Yahboom Intelligent RKD X3 ROBOT assembly video and documentation to complete the hardware assembly of the robot.

## Installation

1. Connect to the robot through MobaXterm or other terminal software

2. Click on the [NodeHub OriginBot project](http://it-dev.horizon.ai/nodehubDetail/170117036053371431) in the top right corner for quick deployment. Copy and run the following commands in the RDK terminal to install the Yahboom Intelligent RKD X3 ROBOT minimum system.

```bash
sudo apt update
sudo apt install -y tros-yahboom-sunrise-robot-lib tros-yahboomcar-bringup tros-yahboomcar-description tros-yahboomcar-base-node
```

## Running

### Start the Robot

Enter the following command in the terminal:

```bash
source /opt/tros/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_nodehub_bringup_launch.py
```

Upon successful execution, you will see the following prompt:

```shell
root@ubuntu:~# ros2 launch yahboomcar_bringup yahboomcar_nodehub_bringup_launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-10-07-15-48-46-154585-ubuntu-4064
[INFO] [launch]: Default logging verbosity is set to INFO
```[INFO] [Mcnamu_driver-1]: process started with pid [4625]
[INFO] [base_node-2]: process started with pid [4627]
[INFO] [imu_filter_madgwick_node-3]: process started with pid [4629]
[INFO] [ekf_node-4]: process started with pid [4631]
[INFO] [joint_state_publisher-5]: process started with pid [4633]
[INFO] [robot_state_publisher-6]: process started with pid [4635]
[INFO] [static_transform_publisher-7]: process started with pid [4637]
[imu_filter_madgwick_node-3] [INFO] [1696664928.206501590] [imu_filter_madgwick]: Starting ImuFilter
[imu_filter_madgwick_node-3] [INFO] [1696664928.222764213] [imu_filter_madgwick]: Using dt computed from messags
[imu_filter_madgwick_node-3] [INFO] [1696664928.222977621] [imu_filter_madgwick]: The gravity vector is kept in.
[imu_filter_madgwick_node-3] [INFO] [1696664928.251886613] [imu_filter_madgwick]: Imu filter gain set to 0.10000
[imu_filter_madgwick_node-3] [INFO] [1696664928.252147777] [imu_filter_madgwick]: Gyro drift bias set to 0.00000
[imu_filter_madgwick_node-3] [INFO] [1696664928.252259252] [imu_filter_madgwick]: Magnetometer bias values: 0.00
[static_transform_publisher-7] [INFO] [1696664928.286630944] [static_transform_publisher_lRrC5ZXgWzegDUvw]: Spi'
[robot_state_publisher-6] Parsing robot urdf xml string.
[robot_state_publisher-6] The root link base_link has an inertia specified in the URDF, but KDL does not suppor.
[robot_state_publisher-6] Link camera2_link had 0 children
[robot_state_publisher-6] Link camera_link had 0 children
[robot_state_publisher-6] Link imu_link had 0 children
[robot_state_publisher-6] Link left_back_wheel had 0 children
[robot_state_publisher-6] Link left_front_wheel had 0 children
[robot_state_publisher-6] Link lidar_link had 0 children
[robot_state_publisher-6] Link right_back_wheel had 0 children
[robot_state_publisher-6] Link right_front_wheel had 0 children
[robot_state_publisher-6] [INFO] [1696664928.460429490] [robot_state_publisher]: got segment base_link
[robot_state_publisher-6] [INFO] [1696664928.461156350] [robot_state_publisher]: got segment camera2_link
[robot_state_publisher-6] [INFO] [1696664928.461358672] [robot_state_publisher]: got segment camera_link
[robot_state_publisher-6] [INFO] [1696664928.461447143] [robot_state_publisher]: got segment imu_link
[robot_state_publisher-6] [INFO] [1696664928.461507486] [robot_state_publisher]: got segment left_back_wheel
[robot_state_publisher-6] [INFO] [1696664928.461565078] [robot_state_publisher]: got segment left_front_wheel
[robot_state_publisher-6] [INFO] [1696664928.461622045] [robot_state_publisher]: got segment lidar_link
[robot_state_publisher-6] [INFO] [1696664928.461675970] [robot_state_publisher]: got segment right_back_wheel
[robot_state_publisher-6] [INFO] [1696664928.461731520] [robot_state_publisher]: got segment right_front_wheel
[ekf_node-4] X acceleration is being measured from IMU; X velocity control input is disabled
[joint_state_publisher-5] [INFO] [1696664930.805215712] [joint_state_publisher]: Waiting for robot_description .
[imu_filter_madgwick_node-3] [INFO] [1696664930.874444424] [imu_filter_madgwick]: First IMU message received.
```

### Control the robot using the keyboard.

Run the following command in another terminal to enable keyboard control functionality:

```bash
sudo apt install tros-teleop-twist-keyboard
source /opt/tros/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

After successful execution, the following prompt appears.

```bash
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:      speed 0.5       turn 1.0
```

Control the robot movement using the corresponding keys on the keyboard.

# Interface Description
## Topics

### Subscribed Topics

| Name        | Message Type                | Description                     |
| ----------- | --------------------------- | ------------------------------- |
| /cmd_vel    | geometry_msgs/msg/Twist     | Publishes velocity commands to control robot movement |


### Published Topics

| Name          | Message Type                | Description                            |
| ------------- | --------------------------  | ------------------------------------- |
| /imu/data     | sensor_msgs/msg/Imu         | IMU data                              |
| /imu/data_raw | sensor_msgs/msg/Imu         | Raw IMU data                          |
| /imu/mag      | sensor_msgs/msg/MagneticField | Magnetometer information             |
| /tf_static    | tf2_msgs/msg/TFMessage      | Publishes static coordinate system information related to the robot |
```| /tf                    |  tf2_msgs/msg/TFMessage          | Publish robot related dynamic coordinate system information                           |
| /odom_raw              |  nav_msgs/msg/Odometry           | Robot odometer raw information                           |
| /odometry/filtered     |  nav_msgs/msg/Odometry           | Robot odometer information                           |

## Parameters
| Parameter Name         | Type        | Description              |
| ---------------------  | ----------- | ---------------------------------- | 
| angular_limit          | float       | Angular speed limit in rad/s                 |
| xlinear_limit          | float       | Limit of linear velocity in the x direction in m/s                 |
| ylinear_limit          | float       | Limit of linear velocity in the y direction in m/s                |


# FAQ
