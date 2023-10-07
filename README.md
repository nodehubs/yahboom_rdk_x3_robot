# 功能介绍

亚博智能RKD X3 ROBOT是一款面向机器人开发者和教育生态的智能机器人开发套件。该套件以旭日X3派为核心运算单元，以TogetherROS.Bot为核心机器人操作系统，集成成了建图、导航、人体跟随、手势识别等功能。本项目为亚博智能RKD X3 ROBOT机器人最小功能系统，该系统能够实现RKD X3 ROBOT的基本运动控制和位姿反馈。该项目包含以下三个功能包：

- yahboom_sunrise_robot_lib：负责与MCU的基础通信以及命令转换
- yahboomcar_base_node：负责更新机器人的Odom和tf信息
- yahboomcar_description：负责描述机器人机械机构信息
- yahboomcar_bringup：负责机器人的初始化，订阅运动控制话题、发布机器人状态描述话题和IMU信息

# 物料清单

| 机器人名称          | 生产厂家 | 参考链接                                                     |
| :------------------ | -------- | ------------------------------------------------------------ |
| 亚博智能RKD X3 ROBOT | 亚博智能   | [购买链接](https://detail.tmall.com/item.htm?abbucket=8&id=726857243156&rn=bf4bcf345d154daf6f1572f015c838f4&scene=taobao_shop&spm=a1z10.1-b-s.w5003-25077533329.15.324244e1UOLGab)                       |
| RDK X3             | 多厂家 | [购买链接](https://developer.horizon.ai/sunrise) |

# 使用方法

## 准备工作

参考亚博智能RKD X3 ROBOT组装视频和文档，完成机器人的硬件组装。

## 安装 

1. 通过MobaXterm或者其他终端软件连接机器人

2. 点击[NodeHub OriginBot项目](http://it-dev.horizon.ai/nodehubDetail/170117036053371431)右上角快速部署，复制如下命令在RDK的终端中运行，完成人亚博智能RKD X3 ROBOT机器人最小系统安装。

```bash
sudo apt update
sudo apt install -y tros-yahboom-sunrise-robot-lib tros-yahboomcar-bringup tros-yahboomcar-description tros-yahboomcar-base-node
```

## 运行

### 启动机器人

在终端中输入：

```bash
source /opt/tros/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_nodehub_bringup_launch.py
```

运行成功后可看到如下提示

```shell
root@ubuntu:~# ros2 launch yahboomcar_bringup yahboomcar_nodehub_bringup_launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-10-07-15-48-46-154585-ubuntu-4064
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [Mcnamu_driver-1]: process started with pid [4625]
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

### 键盘控制机器人

在另一个终端中运行如下指令打开键盘控制功能：

```bash
source /opt/tros/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

运行成功后出现以下提示

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

根据提示使用键盘对应按键控制机器人运动。

# 接口说明
## 话题

### 订阅话题

| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /cmd_vel                      | geometry_msgs/msg/Twist                                      | 发布控制机器人移动的速度指令                           |


### 发布话题

| 名称                    |           消息类型                | 说明                                                   |
| ---------------------- | ------rigin--------------------- | ------------------------------------------------------ |
| /imu/data              |  sensor_msgs/msg/Imu             | IMU数据                          |
| /imu/data_raw          |  sensor_msgs/msg/Imu             | IMU原始数据                          |
| /imu/mag               |  sensor_msgs/msg/MagneticField   | 磁力计信息                           |
| /tf_static             |  tf2_msgs/msg/TFMessage          | 发布机器人相关静态坐标系信息                           |
| /tf                    |  tf2_msgs/msg/TFMessage          | 发布机器人相关动态坐标系信息                           |
| /odom_raw              |  nav_msgs/msg/Odometry           | 机器人里程计原始信息                           |
| /odometry/filtered     |  nav_msgs/msg/Odometry           | 机器人里程计信息                           |

## 参数
| 参数名                | 类型        | 说明              |
| --------------------- | ----------- | ---------------------------------- | 
| angular_limit      | float    |     角速度限制单位rad/s                 |
| xlinear_limit     | float    |     x方向线速度限制单位m/s                 |
| ylinear_limit      | float    |     y方向线速度限制单位m/s                |


# 常见问题


