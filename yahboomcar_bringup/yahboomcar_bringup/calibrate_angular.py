import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, radians
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import PyKDL
from math import pi

class Calibrateangular(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a spublisher
        self.cmd_vel = self.create_publisher(Twist,"/cmd_vel",5)

        #declare parameters
        self.declare_parameter('start_test',False)        
        self.declare_parameter('direction',1.0)
        self.declare_parameter('test_angle',360.0)
        self.declare_parameter('speed',2.0)
        self.declare_parameter('tolerance',1.5)
        self.declare_parameter('odom_angular_scale_correction',1.0)      
        self.declare_parameter('base_frame','base_footprint')
        self.declare_parameter('odom_frame','odom')
        self.get_param()
        
        #init the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        self.first_angle = 0
        self.reverse = 1
        self.turn_angle = 0

        print ("finish init work")         
        #create timer 
        self.timer = self.create_timer(0.05, self.on_timer)
    
    def get_param(self):
        self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.direction = self.get_parameter('direction').get_parameter_value().double_value
        self.test_angle = self.get_parameter('test_angle').get_parameter_value().double_value
        self.test_angle = radians(self.test_angle)  # angle to radians 角度转成弧度
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

    def on_timer(self):
        self.get_param()
        move_cmd = Twist()
        self.test_angle *= self.reverse
        self.error = self.test_angle - self.turn_angle
        if self.start_test:
            self.error = self.test_angle - self.turn_angle
            if self.start_test and (abs(self.error) > self.tolerance or self.error==0) :
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = copysign(self.speed, self.error)
                #print("angular: ",move_cmd.angular.z)
                self.cmd_vel.publish(move_cmd)
                self.odom_angle = self.get_odom_angle() 
                self.delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - self.first_angle)
                #print("delta_angle: ",self.delta_angle)
                self.turn_angle += self.delta_angle
                print("turn_angle: ",self.turn_angle)
                self.error = self.test_angle - self.turn_angle
                print("error: ",self.error)
                self.first_angle = self.odom_angle
                #print("first_angle: ",self.first_angle)
            else:
                self.turn_angle = 0.0
                self.cmd_vel.publish(Twist())
                self.start_test  = rclpy.parameter.Parameter('start_test',rclpy.Parameter.Type.BOOL,False)
                all_new_parameters = [self.start_test]
                self.set_parameters(all_new_parameters)
                self.reverse = -self.reverse
                self.first_angle = 0
                
        else:
            self.cmd_vel.publish(Twist())
            self.start_test  = rclpy.parameter.Parameter('start_test',rclpy.Parameter.Type.BOOL,False)           
            all_new_parameters = [self.start_test]
            self.set_parameters(all_new_parameters)
            
    def get_odom_angle(self):
         try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)   
            print("orig_rot: ",rot.transform.rotation) 
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            #print("cacl_rot: ",cacl_rot)
            angle_rot = cacl_rot.GetRPY()[2]
            #print("angle_rot: ",angle_rot)
            return angle_rot
            
         except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            raise
            return
    
    def normalize_angle(self,angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
             
         
def main():
    rclpy.init()
    class_calibrateangular = Calibrateangular("calibrate_angular")
    rclpy.spin(class_calibrateangular)
