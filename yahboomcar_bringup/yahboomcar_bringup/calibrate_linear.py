import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class CalibrateLinear(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a publisher
        self.cmd_vel = self.create_publisher(Twist,"/cmd_vel",5)

        #declare parameters
        self.declare_parameter('start_test',False)
        self.declare_parameter('direction',1.0)        
        self.declare_parameter('test_distance',1.0)
        self.declare_parameter('speed',0.3)
        self.declare_parameter('tolerance',0.03)
        self.declare_parameter('odom_linear_scale_correction',1.0)        
        self.declare_parameter('base_frame','base_footprint')
        self.declare_parameter('odom_frame','odom')
        self.get_param()

        #init the tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        
        print ("finish init work")
        #create timer 
        self.timer = self.create_timer(0.05, self.on_timer)

    def get_param(self):
        self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.direction = self.get_parameter('direction').get_parameter_value().double_value
        self.test_distance = self.get_parameter('test_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').get_parameter_value().double_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        
    def on_timer(self):
        self.get_param()
        move_cmd = Twist()
        if self.start_test:
            self.position.x = self.get_position().transform.translation.x
            self.position.y = self.get_position().transform.translation.y
            print("self.position.x: ",self.position.x)
            print("self.position.y: ",self.position.y)
            distance = sqrt(pow((self.position.x - self.x_start), 2) +
                                pow((self.position.y - self.y_start), 2))
            distance *= self.odom_linear_scale_correction
            print("distance: ",distance)
            error = distance - self.test_distance
            print("error: ",error)

            if not self.start_test or abs(error) < self.tolerance:
                self.start_test  = rclpy.parameter.Parameter('start_test',rclpy.Parameter.Type.BOOL,False)
                all_new_parameters = [self.start_test]
                self.set_parameters(all_new_parameters)   
                print("done")
            else:
                if self.direction:
                    print("x")
                    move_cmd.linear.x = copysign(self.speed, -1 * error)
                else:
                    move_cmd.linear.y = copysign(self.speed, -1 * error)
                    print("y")
            self.cmd_vel.publish(move_cmd)
        else:
            self.x_start = self.get_position().transform.translation.x
            self.y_start = self.get_position().transform.translation.y
            print("self.x_start: ",self.x_start)
            print("self.y_start:",self.y_start)
            self.cmd_vel.publish(Twist()) 
     
    def get_position(self):
         try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.odom_frame,self.base_frame,now)   
            return trans       
         except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            raise
            return
         
def main():
    rclpy.init()
    class_calibratelinear = CalibrateLinear("calibrate_linear")
    rclpy.spin(class_calibratelinear)
