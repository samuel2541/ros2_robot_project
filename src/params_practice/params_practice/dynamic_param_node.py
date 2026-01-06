#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


class DynamicParamNode(Node):
    """
    Node qui réagit aux changements de paramètres en temps réel.
    """
    
    def __init__(self):
        super().__init__('dynamic_param_node')
        
        # Déclarer les paramètres
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.1)
        self.declare_parameter('robot_name', 'dynamic_robot')
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('enable_warnings', True)
        
        # Récupérer les valeurs initiales
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.robot_name = self.get_parameter('robot_name').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.enable_warnings = self.get_parameter('enable_warnings').value
        
        # Ajouter le callback pour les changements de paramètres
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.log_configuration()
    
    def parameter_callback(self, params):
        """
        Appelé quand un paramètre est modifié.
        Valide et applique les nouvelles valeurs.
        """
        self.get_logger().info('Changement de paramètre demandé...')
        
        for param in params:
            self.get_logger().info(f'  {param.name} = {param.value}')
            
            # Valider max_speed
            if param.name == 'max_speed':
                if param.value < 0.0:
                    self.get_logger().error('max_speed ne peut pas être négatif!')
                    return SetParametersResult(successful=False)
                if param.value > 10.0:
                    self.get_logger().error('max_speed trop élevé! Maximum: 10.0 m/s')
                    return SetParametersResult(successful=False)
                if param.value < self.min_speed:
                    self.get_logger().error(
                        f'max_speed ({param.value}) doit être >= min_speed ({self.min_speed})'
                    )
                    return SetParametersResult(successful=False)
                
                self.max_speed = param.value
                self.get_logger().info(f'✅ max_speed mis à jour: {param.value}')
            
            # Valider min_speed
            elif param.name == 'min_speed':
                if param.value < 0.0:
                    self.get_logger().error('min_speed ne peut pas être négatif!')
                    return SetParametersResult(successful=False)
                if param.value > self.max_speed:
                    self.get_logger().error(
                        f'min_speed ({param.value}) doit être <= max_speed ({self.max_speed})'
                    )
                    return SetParametersResult(successful=False)
                
                self.min_speed = param.value
                self.get_logger().info(f'✅ min_speed mis à jour: {param.value}')
            
            # Valider safety_distance
            elif param.name == 'safety_distance':
                if param.value < 0.1:
                    self.get_logger().error('safety_distance doit être >= 0.1 mètres')
                    return SetParametersResult(successful=False)
                if param.value > 5.0:
                    self.get_logger().warn(f'safety_distance {param.value}m est très grand!')
                
                self.safety_distance = param.value
                self.get_logger().info(f'✅ safety_distance mis à jour: {param.value}')
            
            # Valider robot_name
            elif param.name == 'robot_name':
                if len(param.value) == 0:
                    self.get_logger().error('robot_name ne peut pas être vide!')
                    return SetParametersResult(successful=False)
                
                self.robot_name = param.value
                self.get_logger().info(f'✅ robot_name mis à jour: {param.value}')
            
            # enable_warnings (pas de validation)
            elif param.name == 'enable_warnings':
                self.enable_warnings = param.value
                status = "activés" if param.value else "désactivés"
                self.get_logger().info(f'✅ Warnings {status}')
        
        self.log_configuration()
        return SetParametersResult(successful=True)
    
    def log_configuration(self):
        """Affiche la configuration actuelle."""
        self.get_logger().info('=== Configuration Actuelle ===')
        self.get_logger().info(f'Robot: {self.robot_name}')
        self.get_logger().info(f'Vitesse: {self.min_speed} - {self.max_speed} m/s')
        self.get_logger().info(f'Distance sécurité: {self.safety_distance} m')
        self.get_logger().info(f'Warnings: {"Oui" if self.enable_warnings else "Non"}')
    
    def publish_status(self):
        """Publie le statut du robot."""
        msg = String()
        msg.data = f'{self.robot_name}: speed={self.max_speed:.1f}m/s, safety={self.safety_distance:.2f}m'
        self.publisher_.publish(msg)
        
        if self.enable_warnings and self.max_speed > 5.0:
            self.get_logger().warn('⚠️  Vitesse élevée!')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()