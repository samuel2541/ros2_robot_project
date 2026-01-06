#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import BatteryStatus
import random


class BatteryPublisher(Node):
    """
    Publie des données de batterie simulées.
    """
    
    def __init__(self):
        super().__init__('battery_publisher')
        
        self.publisher_ = self.create_publisher(BatteryStatus, 'battery/status', 10)
        self.timer = self.create_timer(1.0, self.publish_battery)
        
        # Valeurs initiales
        self.percentage = 100.0
        self.voltage = 12.6
        self.temperature = 25.0
        self.is_charging = False
        
        self.get_logger().info('Battery Publisher démarré!')
    
    def publish_battery(self):
        """Publie le statut de batterie."""
        msg = BatteryStatus()
        
        # Simuler décharge de batterie
        if not self.is_charging:
            self.percentage -= random.uniform(0.5, 2.0)
            self.percentage = max(0.0, self.percentage)
        else:
            self.percentage += random.uniform(1.0, 3.0)
            self.percentage = min(100.0, self.percentage)
        
        # Voltage proportionnel au pourcentage
        self.voltage = 10.5 + (self.percentage / 100.0) * 2.1
        
        # Température varie un peu
        self.temperature += random.uniform(-0.5, 0.5)
        self.temperature = max(20.0, min(60.0, self.temperature))
        
        # Déterminer l'état de santé
        if self.percentage > 30:
            health = "good"
        elif self.percentage > 15:
            health = "degraded"
        else:
            health = "critical"
        
        # Remplir le message
        msg.percentage = self.percentage
        msg.voltage = self.voltage
        msg.temperature = self.temperature
        msg.health_status = health
        msg.is_charging = self.is_charging
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Batterie: {self.percentage:.1f}% | {self.voltage:.2f}V | '
            f'{self.temperature:.1f}°C | {health}'
        )
        
        # Recharger si vide
        if self.percentage <= 0:
            self.is_charging = True
            self.get_logger().info('🔌 Début de la charge...')
        elif self.percentage >= 100:
            self.is_charging = False
            self.get_logger().info('🔋 Charge complète!')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
