#for patrol
#math
from math import radians, copysign, sqrt, pow
from math import pi
import numpy as np
#rclpy
import rclpy
from rclpy.node import Node
#tf
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
#msg
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
#others
import PyKDL
from time import sleep

print("import finish")

class YahboomCarPatrol(Node):
    def __init__(self,name):
        super().__init__(name)
        self.moving = True
        self.Joy_active = False
        self.warning = 1
        self.Angle = 360.0
        self.LineScaling = 1.0
        self.LineTolerance = 0.1

        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        self.error = 0.0
        self.distance = 0.0 
        self.last_angle = 0.0 
        self.delta_angle = 0.0
        self.turn_angle = 0.0
        self.odom_frame = "odom"
        self.base_frame = "base_footprint"
        #create publisher
        self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",5)
        #create subscriber
        self.sub_scan = self.create_subscription(LaserScan,"/scan",self.LaserScanCallback,1)
        self.sub_joy = self.create_subscription(Bool,"/JoyState",self.JoyStateCallback,1)
        #create TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        #declare param
        self.declare_parameter('Switch',False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.declare_parameter('Set_loop',False)
        self.Set_loop = self.get_parameter('Set_loop').get_parameter_value().bool_value
        self.declare_parameter('Command',"Square")
        self.Command = self.get_parameter('Command').get_parameter_value().string_value
        self.command_src = self.Command
        self.declare_parameter('Length',1.0)
        self.Length = self.get_parameter('Length').get_parameter_value().double_value
        self.declare_parameter('Linear',0.2)
        self.Linear = self.get_parameter('Linear').get_parameter_value().double_value
        self.declare_parameter('Angular',2.0)
        self.Angular = self.get_parameter('Angular').get_parameter_value().double_value
        self.declare_parameter('ResponseDist',0.6)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter('LaserAngle',20.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter('RotationScaling',1.25)
        self.RotationScaling = self.get_parameter('RotationScaling').get_parameter_value().double_value
        self.declare_parameter('RotationTolerance',0.3)
        self.RotationTolerance = self.get_parameter('RotationTolerance').get_parameter_value().double_value
        #create timer
        self.timer = self.create_timer(0.01,self.on_timer)
        self.index = 0
        
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.Set_loop = self.get_parameter('Set_loop').get_parameter_value().bool_value
        self.Command = self.get_parameter('Command').get_parameter_value().string_value
        self.Linear = self.get_parameter('Linear').get_parameter_value().double_value
        self.Angular = self.get_parameter('Angular').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        
        if self.Switch == True:
            # print("Switch True")
            if self.Command == "LengthTest":
                self.command_src = self.Command
                advancing = self.advancing(self.Length)
                if advancing == True:
                    print("LengthTest done")
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,"finish")
                    all_new_parameters = [self.Switch, self.Command]
                    self.set_parameters(all_new_parameters)
            
            elif self.Command == "Circle":
                self.command_src = self.Command
                spin = self.Spin(360)
                if spin == True:
                    print("Circle done")
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,"finish")
                    all_new_parameters = [self.Switch, self.Command]
                    self.set_parameters(all_new_parameters)

            elif self.Command == "Square":
                self.command_src = self.Command
                square = self.Square()
                if square == True:
                    print("Square done")
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,"finish")
                    all_new_parameters = [self.Switch, self.Command]
                    self.set_parameters(all_new_parameters)
            
            elif self.Command == "Triangle":
                self.command_src = self.Command
                triangle = self.Triangle()
                if triangle == True:
                    print("Triangle done")
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,False)
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,"finish")
                    all_new_parameters = [self.Switch, self.Command]
                    self.set_parameters(all_new_parameters)

        else: 
            if self.Command == "finish":
                if self.Set_loop == True:
                    print("Continute")
                    self.Switch  = rclpy.parameter.Parameter('Switch',rclpy.Parameter.Type.BOOL,True)
                    self.Command  = rclpy.parameter.Parameter('Command',rclpy.Parameter.Type.STRING,self.command_src)
                    all_new_parameters = [self.Switch, self.Command]
                    self.set_parameters(all_new_parameters)
                else:
                    print("Not loop")
                    self.pub_cmdVel.publish(Twist())
            else:
                # print("Switch False")
                self.pub_cmdVel.publish(Twist())
        
                    
    def advancing(self,target_distance):
        # print("Length")
        self.position.x = self.get_position().transform.translation.x
        self.position.y = self.get_position().transform.translation.y
        move_cmd = Twist()
        self.distance = sqrt(pow((self.position.x - self.x_start), 2) +
                            pow((self.position.y - self.y_start), 2))
        self.distance *= self.LineScaling
        print("distance: ",self.distance)
        self.error = self.distance - target_distance
        move_cmd.linear.x = self.Linear
        if abs(self.error) < self.LineTolerance : 
            print("stop")
            self.distance = 0.0
            self.pub_cmdVel.publish(Twist())
            self.x_start = self.position.x
            self.y_start = self.position.y
            return True
        else:
            if self.Joy_active == True or self.warning > 10:
                if self.moving == True:
                    self.pub_cmdVel.publish(Twist())
                    self.moving = False
                    print("obstacles")
            else:
                self.pub_cmdVel.publish(move_cmd)
                self.moving = True
            return False

    def Spin(self,angle):
        # print("Spin")
        self.target_angle = radians(angle)
        self.odom_angle = self.get_odom_angle()
        self.delta_angle = self.RotationScaling * self.normalize_angle(self.odom_angle - self.last_angle)
        self.turn_angle += self.delta_angle
        print("turn_angle: ",self.turn_angle)
        self.error = self.target_angle - self.turn_angle
        print("error: ",self.error)
        self.last_angle = self.odom_angle
        move_cmd = Twist()
        if abs(self.error) < self.RotationTolerance:
            self.pub_cmdVel.publish(Twist())
            self.turn_angle = 0.0
            return True
        if self.Joy_active == True or self.warning > 10:
            if self.moving == True:
                self.pub_cmdVel.publish(Twist())
                self.moving = False
                print("obstacles")
        else:
            move_cmd.linear.x = self.Linear
            move_cmd.angular.z = copysign(self.Angular, self.error)
            self.pub_cmdVel.publish(move_cmd)
            self.moving = True
        
        
    def Square(self):
        if self.index < 8:
            if self.index % 2 == 0:
                step_length = self.advancing(self.Length)
                if step_length == True:
                    self.index += 1
            else:
                step_spin = self.Spin(90)
                if step_spin == True:
                    self.index += 1
        else:
            self.index = 0
            return True

            
    def Triangle(self):
        if self.index < 6:
            if self.index % 2 == 0:
                step_length = self.advancing(self.Length)
                if step_length == True:
                    self.index += 1
            else:
                step_spin = self.Spin(120)
                if step_spin == True:
                    self.index += 1
        else:
            self.index = 0
            return True


    def get_odom_angle(self):
         try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)   
            #print("oring_rot: ",rot.transform.rotation) 
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            #print("cacl_rot: ",cacl_rot)
            angle_rot = cacl_rot.GetRPY()[2]
           
         except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return
        
         return angle_rot      
        
        
    def get_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            raise
            return
        
    def normalize_angle(self,angle):
        res = angle
        #print("res: ",res)
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
    
    def LaserScanCallback(self,scan_data):
        if self.ResponseDist == 0: return
        ranges = np.array(scan_data.ranges)
        self.warning = 1
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * 180 / pi
            if angle > 180: angle = angle - 360
            if ranges[i] < self.ResponseDist and abs(angle) < self.LaserAngle: 
                self.warning += 1
    
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
 
    
def main():
    rclpy.init()
    class_patrol = YahboomCarPatrol("YahboomCarPatrol")
    print("create done")
    rclpy.spin(class_patrol)
    
