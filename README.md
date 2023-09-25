# 功能介绍

亚博智能RKD X3 ROBOT是一款面向机器人开发者和教育生态的智能机器人开发套件。该套件以旭日X3派为核心运算单元，以TogetherROS.Bot为核心机器人操作系统，集成成了建图、导航、人体跟随、手势识别等功能。本项目为亚博智能RKD X3 ROBOT机器人最小功能系统，该系统能够实现RKD X3 ROBOT的基本运动控制和位姿反馈。该项目包含以下三个功能包：

- SunriseRobotLib：负责与MCU的基础通信以及命令转换
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
sudo apt install -y tros-originbot-base tros-serial tros-originbot-msgs
```

## 运行

### 启动机器人

在终端中输入：

```bash
source /opt/tros/setup.bash
ros2 launch originbot_base robot.launch.py 
```

运行成功后可看到如下提示

```shell
root@ubuntu:/userdata# ros2 launch originbot_base robot.launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-09-16-49-58-754723-ubuntu-6891
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [originbot_base-1]: process started with pid [6893]
[INFO] [static_transform_publisher-2]: process started with pid [6895]
[INFO] [static_transform_publisher-3]: process started with pid [6897]
[INFO] [static_transform_publisher-4]: process started with pid [6899]
[originbot_base-1] Loading parameters:
[originbot_base-1]              - port name: ttyS3
[originbot_base-1]              - correct factor vx: 0.8980
[originbot_base-1]              - correct factor vth: 0.8740
[originbot_base-1]              - auto stop on: 0
[originbot_base-1]              - use imu: 0
[static_transform_publisher-4] [INFO] [1688892599.398482288] [static_transform_publisher_wrXGY3d4cJPrfMMt]: Spinning until killed publishing transform from '/base_link' to '/imu_link'
[static_transform_publisher-2] [INFO] [1688892599.404346159] [static_transform_publisher_f8bfUI3IdvTPSv5L]: Spinning until killed publishing transform from '/base_footprint' to '/base_link'
[originbot_base-1] [INFO] [1688892599.417785811] [originbot_base]: originbot serial port opened
[originbot_base-1] [INFO] [1688892599.919219715] [originbot_base]: OriginBot Start, enjoy it.

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

| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /originbot_status             |  originbot_msgs/msg/OriginbotStatus                          | 发布OriginBot机器人状态                           |
| /odom                         |  nav_msgs/msg/Odometry                                       | 发布OriginBot里程计信息                           |
| /tf_static                    |  tf2_msgs/msg/TFMessage                                      | 发布OriginBot相关坐标系信息                           |

## 参数
| 参数名                | 类型        | 解释                                 | 是否必须 | 默认值                                               |
| --------------------- | ----------- | ---------------------------------- | -------- | --------------------------- |
| auto_stop_on_arg      | bool    |     是否使能自动停止功能                 | 否       | false |
| use_imu_arg      | bool    |     是否使能IMU                 | 否       | false |
| pub_odom_arg      | bool    |     是否使能发布Odom话题                | 否       | true |
| correct_factor_vx_arg      | float    |  线速度校正参数                | 否       | 0.898 |
| correct_factor_vth_arg      | float    |  角速度校正参数                | 否       | 0.874 |


# 常见问题


