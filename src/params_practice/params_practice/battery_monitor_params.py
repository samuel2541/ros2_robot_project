#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from my_robot_interfaces.msg import BatteryStatus


class BatteryMonitorParams(Node):
    """
    Moniteur de batterie avec seuils configurables.
    """
    
    def __init__(self):
        super().__init__('battery_monitor_params')
        
        # Paramètre seuil batterie faible
        low_descriptor = ParameterDescriptor(
            description='Seuil de pourcentage pour alerte batterie faible',
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=100.0,
                step=1.0
            )]
        )
        self.declare_parameter('low_battery_threshold', 30.0, low_descriptor)
        
        # Paramètre seuil batterie critique
        critical_descriptor = ParameterDescriptor(
            description='Seuil de pourcentage pour alerte batterie critique',
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=100.0,
                step=1.0
            )]
        )
        self.declare_parameter('critical_battery_threshold', 15.0, critical_descriptor)
        
        # Paramètre température max
        temp_descriptor = ParameterDescriptor(
            description='Température maximale sûre en Celsius',
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=100.0,
                step=1.0
            )]
        )
        self.declare_parameter('max_temperature', 45.0, temp_descriptor)
        
        # Paramètre voltage min
        voltage_descriptor = ParameterDescriptor(
            description='Voltage minimum sûr en Volts',
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=20.0,
                step=0.1
            )]
        )
        self.declare_parameter('min_voltage', 11.0, voltage_descriptor)
        
        # Paramètres de comportement
        self.declare_parameter('enable_audio_alerts', True)
        self.declare_parameter('enable_visual_alerts', True)
        self.declare_parameter('alert_frequency', 5.0)
        
        # Récupérer les valeurs initiales
        self.update_parameters()
        
        # Ajouter callback pour changements
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # S'abonner au topic battery/status
        self.subscription = self.create_subscription(
            BatteryStatus,
            'battery/status',
            self.battery_callback,
            10
        )
        
        # Tracking des alertes
        self.last_low_alert = self.get_clock().now()
        self.last_critical_alert = self.get_clock().now()
        
        self.get_logger().info('Battery Monitor avec Paramètres démarré!')
        self.log_configuration()
    
    def update_parameters(self):
        """Met à jour les valeurs internes des paramètres."""
        self.low_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_threshold = self.get_parameter('critical_battery_threshold').value
        self.max_temp = self.get_parameter('max_temperature').value
        self.min_voltage = self.get_parameter('min_voltage').value
        self.enable_audio = self.get_parameter('enable_audio_alerts').value
        self.enable_visual = self.get_parameter('enable_visual_alerts').value
        self.alert_freq = self.get_parameter('alert_frequency').value
    
    def parameter_callback(self, params):
        """Valide et applique les changements de paramètres."""
        for param in params:
            if param.name == 'low_battery_threshold':
                if param.value <= self.critical_threshold:
                    self.get_logger().error(
                        f'low_battery_threshold ({param.value}) doit être > '
                        f'critical_battery_threshold ({self.critical_threshold})'
                    )
                    return SetParametersResult(successful=False)
            
            elif param.name == 'critical_battery_threshold':
                if param.value >= self.low_threshold:
                    self.get_logger().error(
                        f'critical_battery_threshold ({param.value}) doit être < '
                        f'low_battery_threshold ({self.low_threshold})'
                    )
                    return SetParametersResult(successful=False)
            
            elif param.name == 'max_temperature':
                if param.value < 20.0:
                    self.get_logger().warn(f'max_temperature {param.value}°C est très bas!')
            
            elif param.name == 'alert_frequency':
                if param.value < 1.0:
                    self.get_logger().error('alert_frequency doit être >= 1.0 secondes')
                    return SetParametersResult(successful=False)
        
        self.update_parameters()
        self.log_configuration()
        
        return SetParametersResult(successful=True)
    
    def log_configuration(self):
        """Affiche la configuration actuelle."""
        self.get_logger().info('=== Configuration Battery Monitor ===')
        self.get_logger().info(f'Seuil batterie faible: {self.low_threshold}%')
        self.get_logger().info(f'Seuil batterie critique: {self.critical_threshold}%')
        self.get_logger().info(f'Température max: {self.max_temp}°C')
        self.get_logger().info(f'Voltage min: {self.min_voltage}V')
        self.get_logger().info(f'Alertes audio: {self.enable_audio}')
        self.get_logger().info(f'Alertes visuelles: {self.enable_visual}')
        self.get_logger().info(f'Fréquence alertes: {self.alert_freq}s')
    
    def battery_callback(self, msg):
        """Traite le statut de batterie avec seuils configurables."""
        current_time = self.get_clock().now()
        
        if msg.percentage <= self.critical_threshold:
            time_since_alert = (current_time - self.last_critical_alert).nanoseconds / 1e9
            if time_since_alert >= self.alert_freq:
                self.trigger_alert('CRITIQUE', 
                    f'Batterie critique: {msg.percentage}%', 
                    severity='error')
                self.last_critical_alert = current_time
        
        elif msg.percentage <= self.low_threshold:
            time_since_alert = (current_time - self.last_low_alert).nanoseconds / 1e9
            if time_since_alert >= self.alert_freq:
                self.trigger_alert('ATTENTION', 
                    f'Batterie faible: {msg.percentage}%', 
                    severity='warn')
                self.last_low_alert = current_time
        
        if msg.voltage < self.min_voltage:
            self.trigger_alert('ATTENTION',
                f'Voltage bas: {msg.voltage:.2f}V (min: {self.min_voltage}V)',
                severity='warn')
        
        if msg.temperature > self.max_temp:
            self.trigger_alert('ATTENTION',
                f'Température haute: {msg.temperature:.1f}°C (max: {self.max_temp}°C)',
                severity='warn')
    
    def trigger_alert(self, level, message, severity='info'):
        """Déclenche une alerte selon la configuration."""
        if severity == 'error':
            log_func = self.get_logger().error
        elif severity == 'warn':
            log_func = self.get_logger().warn
        else:
            log_func = self.get_logger().info
        
        if self.enable_audio:
            message = f'🔊 {message}'
        
        if self.enable_visual:
            message = f'⚠️  [{level}] {message}'
        
        log_func(message)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
