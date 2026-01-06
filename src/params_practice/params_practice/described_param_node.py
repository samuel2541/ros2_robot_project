#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from std_msgs.msg import String


class DescribedParamNode(Node):
    """
    Node avec paramètres documentés via descriptors.
    """
    
    def __init__(self):
        super().__init__('described_param_node')
        
        # max_speed avec descriptor
        max_speed_descriptor = ParameterDescriptor(
            description='Vitesse maximale du robot en mètres par seconde',
            additional_constraints='Doit être entre 0.1 et 10.0 m/s',
            read_only=False,
            floating_point_range=[FloatingPointRange(
                from_value=0.1,
                to_value=10.0,
                step=0.1
            )]
        )
        self.declare_parameter('max_speed', 2.0, max_speed_descriptor)
        
        # update_rate avec descriptor
        rate_descriptor = ParameterDescriptor(
            description='Fréquence de mise à jour du node en Hertz',
            additional_constraints='Doit être entre 1 et 100 Hz',
            read_only=False,
            integer_range=[IntegerRange(
                from_value=1,
                to_value=100,
                step=1
            )]
        )
        self.declare_parameter('update_rate', 10, rate_descriptor)
        
        # robot_name avec descriptor
        name_descriptor = ParameterDescriptor(
            description='Identifiant unique du robot',
            additional_constraints='Ne peut pas être vide',
            read_only=False
        )
        self.declare_parameter('robot_name', 'documented_robot', name_descriptor)
        
        # enable_safety avec descriptor
        safety_descriptor = ParameterDescriptor(
            description='Active les fonctions de sécurité (arrêt urgence, évitement collision)',
            additional_constraints='Devrait toujours être true en production',
            read_only=False
        )
        self.declare_parameter('enable_safety', True, safety_descriptor)
        
        # Récupérer les valeurs
        self.max_speed = self.get_parameter('max_speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.robot_name = self.get_parameter('robot_name').value
        self.enable_safety = self.get_parameter('enable_safety').value
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'status', 10)
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('=== Node avec Descriptors démarré ===')
        self.log_parameters()
    
    def log_parameters(self):
        """Affiche tous les paramètres."""
        self.get_logger().info(f'robot_name: {self.robot_name}')
        self.get_logger().info(f'max_speed: {self.max_speed} m/s')
        self.get_logger().info(f'update_rate: {self.update_rate} Hz')
        self.get_logger().info(f'enable_safety: {self.enable_safety}')
    
    def timer_callback(self):
        """Publie le statut."""
        msg = String()
        msg.data = f'{self.robot_name}: {self.max_speed}m/s (Safety: {self.enable_safety})'
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DescribedParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()