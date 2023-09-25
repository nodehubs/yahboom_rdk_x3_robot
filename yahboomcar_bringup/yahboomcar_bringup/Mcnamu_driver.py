#!/usr/bin/env python
# encoding: utf-8

#public lib
from math import pi
from SunriseRobotLib import SunriseRobot
import smbus

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
from sensor_msgs.msg import Imu, MagneticField

bus = smbus.SMBus(0)

class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		self.RA2DE = 180 / pi
		self.car = SunriseRobot()
		self.car.set_car_type(6)
		self.car.create_receive_threading()
		#get parameter
		self.declare_parameter('imu_link', 'imu_link')
		self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
		print (self.imu_link)
		self.declare_parameter('Prefix', "")
		self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
		print (self.Prefix)
		self.declare_parameter('xlinear_limit', 1.0)
		self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
		print (self.xlinear_limit)
		self.declare_parameter('ylinear_limit', 1.0)
		self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
		print (self.ylinear_limit)
		self.declare_parameter('angular_limit', 1.0)
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)

		#create subcriber 创建订阅者
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,1)
		self.sub_Buzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)

		#create publisher 创建发布者
		self.EdiPublisher = self.create_publisher(Float32,"edition",100)
		self.volPublisher = self.create_publisher(Float32,"voltage",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"/imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",100)

		self.RGBLight_ctrl = True
		
		#create timer
		self.timer = self.create_timer(0.1, self.pub_data)

	#callback function
	def cmd_vel_callback(self,msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
		vx = msg.linear.x
		vy = msg.linear.y
		angular = msg.angular.z
		self.car.set_car_motion(vx, vy, angular)

	def RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		if msg.data == 5:
			if self.RGBLight_ctrl == True: bus.write_byte_data(0x0d, 0x07, 0)
			else: bus.write_byte_data(0x0d, 0x07, 1)
			self.RGBLight_ctrl = not self.RGBLight_ctrl
		else: bus.write_byte_data(0x0d, 0x04, msg.data)

	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	#pub data
	def pub_data(self):
		time_stamp = Clock().now()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		mag = MagneticField()
		
		edition.data = self.car.get_version() * 1.0
		battery.data = self.car.get_battery_voltage() * 1.0

		ax, ay, az = self.car.get_accelerometer_data()
		ax_copy, ay_copy, az_copy = ax, ay, az
		ax, ay, az = -ax_copy, az_copy, ay_copy

		gx, gy, gz = self.car.get_gyroscope_data()
		gx_copy, gy_copy, gz_copy = gx, gy, gz
		gx, gy, gz = -gx_copy, gz_copy, gy_copy
		
		mx, my, mz = self.car.get_magnetometer_data()
		# mx_copy, my_copy, mz_copy = mx, my, mz
		# mx, my, mz = -mx_copy, -mz_copy, -my_copy

		vx, vy, angular = self.car.get_motion_data()


		# self.get_logger().info("ax={}, ay={}, az={},gx={}, gy={}, gz={}".format(ax, ay, az, gx, gy, gz))
		# self.get_logger().info("vx={}, vy={}, angular={}".format(vx, vy, angular))

		# 发布陀螺仪的数据
		# Publish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = self.imu_link
		imu.linear_acceleration.x = ax * 1.0
		imu.linear_acceleration.y = ay * 1.0
		imu.linear_acceleration.z = az * 1.0
		imu.angular_velocity.x = gx * 1.0
		imu.angular_velocity.y = gy * 1.0
		imu.angular_velocity.z = gz * 1.0

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx * 1.0
		mag.magnetic_field.y = my * 1.0
		mag.magnetic_field.z = mz * 1.0
		
		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx * 1.0
		twist.linear.y = vy * 1.0
		twist.angular.z = angular * 1.0

		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# print("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))

		self.EdiPublisher.publish(edition)
		self.volPublisher.publish(battery)
		self.imuPublisher.publish(imu)
		self.magPublisher.publish(mag)
		self.velPublisher.publish(twist)
		
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	rclpy.spin(driver)
		