#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BasicParamNode(Node):
    def __init__(self):
        super().__init__('basic_param_node')
        
        # Déclarer les paramètres avec valeurs par défaut
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('update_rate', 1.0)
        self.declare_parameter('enable_safety', True)
        
        # Récupérer les valeurs
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.enable_safety = self.get_parameter('enable_safety').value
        
        # Afficher la config
        self.get_logger().info(f'Robot Name: {self.robot_name}')
        self.get_logger().info(f'Max Speed: {self.max_speed} m/s')
        
        # Publisher + Timer
        self.publisher_ = self.create_publisher(String, 'status', 10)
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        self.counter = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'{self.robot_name}: Count={self.counter}, Speed={self.max_speed}'
        self.publisher_.publish(msg)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = BasicParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    