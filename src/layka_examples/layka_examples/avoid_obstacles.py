#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        
        self.declare_parameter("K_att",1.0)         ## attractive coef 
        self.declare_parameter("K_rep",1.0)         ## repulsive  coef
        self.declare_parameter("rep_field",1.0)     ## repulsive force affection field
        self.declare_parameter("v_linear_max",0.5)  ## m/s
        self.declare_parameter("v_angular_max",1.0) ## rad/s
        
        self.k_att = self.get_parameter("K_att")
        self.k_rep = self.get_parameter("K_rep")
        self.rep_field = self.get_parameter("rep_field")
        self.v_linear_max_ms = self.get_parameter("v_linear_max")
        self.v_angular_max_rads = self.get_parameter("v_angular_max")
        
        self.publisher = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)
        
        self.timer_period = 0.1 ## 10 hertz for control loop
        self.counter = 0
        self.lastest_scan:LaserScan = None
        self.current_twist = TwistStamped()
        
        self.get_logger().info('Object Avoidance Node Started')



    def lidar_callback(self, msg):
        
        self.lastest_scan = msg

    def control_loop(self):

        if self.lastest_scan == None:
            return
        
        scan = self.lastest_scan
        
        # TODO: Implement avoidance algorithm uses APF (Artificial potential fileds )
    
    def _compute_att_force(self):
        pass
    
    def _compute_rep_force(self):
        pass            
    
    def conv_force_to_twist(self):
        pass




def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()