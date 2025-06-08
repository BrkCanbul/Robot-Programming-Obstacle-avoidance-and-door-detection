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
        self.counter = 0
        self.publisher = self.create_publisher(TwistStamped, '/layka_controller/cmd_vel', 10)
        self.get_logger().info('Object Avoidance Node Started')

    def lidar_callback(self, msg):
        ranges = msg.ranges
        min_distance = min(ranges)

        if self.counter % 100 == 0: 
            print('Min lidar distance: ', min_distance)  
            #self.get_logger().info(f"Publishing to: {self.publisher.topic}") 
            
            twist_msg = TwistStamped()
            twist_msg.header = Header()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            
            # No obstacle detected, move forward
            twist_msg.twist.linear.x = -0.3
            twist_msg.twist.angular.z = 0.0

            self.publisher.publish(twist_msg)

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