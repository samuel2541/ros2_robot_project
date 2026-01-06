#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


class SensorManager(Node):
    def __init__(self):
        super().__init__('sensor_manager')
        
        self.declare_parameter('sensor_names', ['lidar', 'camera', 'imu', 'gps', 'ultrasonic'])
        self.declare_parameter('sensor_rates', [10.0, 30.0, 100.0, 1.0, 20.0])
        self.declare_parameter('sensor_enabled', [True, True, True, True, False])
        
        self.sensor_names = self.get_parameter('sensor_names').value
        self.sensor_rates = self.get_parameter('sensor_rates').value
        self.sensor_enabled = self.get_parameter('sensor_enabled').value
        
        if not self.validate_arrays():
            self.get_logger().error('Configuration initiale invalide!')
            return
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.publisher_ = self.create_publisher(String, 'sensor_status', 10)
        self.timer = self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info('Sensor Manager demarre')
        self.log_configuration()
    
    def validate_arrays(self):
        len_names = len(self.sensor_names)
        len_rates = len(self.sensor_rates)
        len_enabled = len(self.sensor_enabled)
        
        if len_names != len_rates or len_names != len_enabled:
            self.get_logger().error('Arrays pas meme taille!')
            return False
        
        for i, rate in enumerate(self.sensor_rates):
            if rate <= 0:
                self.get_logger().error('Rate invalide')
                return False
        
        return True
    
    def parameter_callback(self, params):
        new_names = self.sensor_names.copy()
        new_rates = self.sensor_rates.copy()
        new_enabled = self.sensor_enabled.copy()
        
        for param in params:
            self.get_logger().info('Changement: ' + str(param.name))
            
            if param.name == 'sensor_names':
                new_names = param.value
            elif param.name == 'sensor_rates':
                new_rates = param.value
            elif param.name == 'sensor_enabled':
                new_enabled = param.value
        
        if len(new_names) != len(new_rates) or len(new_names) != len(new_enabled):
            self.get_logger().error('Arrays pas meme taille!')
            return SetParametersResult(successful=False)
        
        for rate in new_rates:
            if rate <= 0:
                self.get_logger().error('Rate invalide')
                return SetParametersResult(successful=False)
        
        self.sensor_names = new_names
        self.sensor_rates = new_rates
        self.sensor_enabled = new_enabled
        
        self.get_logger().info('Config mise a jour!')
        self.log_configuration()
        
        return SetParametersResult(successful=True)
    
    def log_configuration(self):
        self.get_logger().info('--- Configuration Capteurs ---')
        
        for i in range(len(self.sensor_names)):
            if self.sensor_enabled[i]:
                status = 'ON'
            else:
                status = 'OFF'
            msg = '[' + str(i) + '] ' + self.sensor_names[i] + ': ' + str(self.sensor_rates[i]) + ' Hz - ' + status
            self.get_logger().info(msg)
        
        active = sum(self.sensor_enabled)
        self.get_logger().info('Actifs: ' + str(active) + '/' + str(len(self.sensor_names)))
    
    def publish_status(self):
        active_sensors = []
        for i in range(len(self.sensor_names)):
            if self.sensor_enabled[i]:
                active_sensors.append(self.sensor_names[i])
        
        msg = String()
        msg.data = 'Actifs: ' + ', '.join(active_sensors)
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SensorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
