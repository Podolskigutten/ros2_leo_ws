#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Odom_monitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')  # Use lowercase for node name
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.callback_function,
            10)
        
        self.get_logger().info('Odom monitor started')

    def callback_function(self, msg):
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        
        yaw = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        
        yaw_degrees = math.degrees(yaw)
        
        print(f"Position: x={x_pos:.3f}, y={y_pos:.3f}, yaw={yaw_degrees:.1f}°")

def main(args=None):
    rclpy.init(args=args)
    node = Odom_monitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()