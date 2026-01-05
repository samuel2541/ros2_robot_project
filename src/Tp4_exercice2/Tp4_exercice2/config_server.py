#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fleet_interfaces.srv import ConfigureRobot


class ConfigServer(Node):
    def __init__(self):
        super().__init__('config_server')
        
        # Stockage des configurations des robots
        self.robot_configs = {}
        
        # Créer le service
        self.service = self.create_service(
            ConfigureRobot,
            'configure_robot',
            self.configure_callback
        )
        
        self.get_logger().info('Robot Configuration Server is ready!')

    def configure_callback(self, request, response):
        robot_name = request.robot_name
        
        # Validation des paramètres
        if request.max_velocity <= 0:
            response.success = False
            response.message = f"Erreur: max_velocity doit etre > 0"
            return response
            
        if request.acceleration_limit <= 0:
            response.success = False
            response.message = f"Erreur: acceleration_limit doit etre > 0"
            return response
            
        if request.safety_distance < 0:
            response.success = False
            response.message = f"Erreur: safety_distance doit etre >= 0"
            return response
        
        # Sauvegarder la configuration
        self.robot_configs[robot_name] = {
            'max_velocity': request.max_velocity,
            'acceleration_limit': request.acceleration_limit,
            'safety_distance': request.safety_distance,
            'obstacle_avoidance': request.enable_obstacle_avoidance,
            'logging': request.enable_logging
        }
        
        # Préparer la réponse
        response.success = True
        response.message = f"Robot '{robot_name}' configure avec succes!"
        response.max_velocity_set = request.max_velocity
        response.acceleration_limit_set = request.acceleration_limit
        response.safety_distance_set = request.safety_distance
        
        self.get_logger().info(f"Configuration de '{robot_name}':")
        self.get_logger().info(f"  - Max velocity: {request.max_velocity} m/s")
        self.get_logger().info(f"  - Acceleration: {request.acceleration_limit} m/s2")
        self.get_logger().info(f"  - Safety distance: {request.safety_distance} m")
        self.get_logger().info(f"  - Obstacle avoidance: {request.enable_obstacle_avoidance}")
        self.get_logger().info(f"  - Logging: {request.enable_logging}")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ConfigServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
