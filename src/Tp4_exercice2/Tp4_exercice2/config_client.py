#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fleet_interfaces.srv import ConfigureRobot


class ConfigClient(Node):
    def __init__(self):
        super().__init__('config_client')
        
        # Créer le client
        self.client = self.create_client(ConfigureRobot, 'configure_robot')
        
        # Attendre que le service soit disponible
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for configure_robot service...')
        
        self.get_logger().info('Service available! Sending configuration...')
        
        # Envoyer une configuration de test
        self.send_config()

    def send_config(self):
        request = ConfigureRobot.Request()
        request.robot_name = 'robot_1'
        request.max_velocity = 2.5
        request.acceleration_limit = 1.0
        request.safety_distance = 0.5
        request.enable_obstacle_avoidance = True
        request.enable_logging = True
        
        self.get_logger().info('Sending configuration request...')
        
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'SUCCESS: {response.message}')
                self.get_logger().info(f'  Max velocity set: {response.max_velocity_set} m/s')
                self.get_logger().info(f'  Acceleration set: {response.acceleration_limit_set} m/s2')
                self.get_logger().info(f'  Safety distance set: {response.safety_distance_set} m')
            else:
                self.get_logger().error(f'FAILED: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        
        # Quitter après la réponse
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ConfigClient()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
