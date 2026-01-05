#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fleet_interfaces.msg import FleetCommand


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')
        self.publisher = self.create_publisher(FleetCommand, 'fleet_command', 10)
        
        self.timer = self.create_timer(0.5, self.send_emergency_stop)
        
    def send_emergency_stop(self):
        cmd = FleetCommand()
        cmd.command_id = 'EMERGENCY_STOP'
        cmd.command_type = 'stop'
        cmd.priority = 10
        cmd.target_robots = []
        cmd.parameters = []
        
        self.publisher.publish(cmd)
        self.get_logger().info('EMERGENCY STOP SENT TO ALL ROBOTS')
        
        self.timer.cancel()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EmergencyStop()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
