#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill


class MultiTurtleManager(Node):
    def __init__(self):
        super().__init__('multi_turtle_manager')
        
        # Liste pour tracker les tortues actives
        self.active_turtles = ['turtle1']  # turtle1 existe par défaut
        
        # Clients pour les services turtlesim
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        
        # Attendre que les services soient disponibles
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')
        
        self.get_logger().info('Multi-Turtle Manager is ready!')
        
        # Lancer la démonstration
        self.demo()

    def spawn_turtle(self, name, x, y, theta=0.0):
        """Spawn une tortue à une position donnée"""
        # Bonus: Vérifier les doublons
        if name in self.active_turtles:
            self.get_logger().warn(f"La tortue '{name}' existe deja!")
            return False
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            result = future.result()
            self.active_turtles.append(name)
            self.get_logger().info(f"Tortue '{name}' creee a ({x}, {y})")
            return True
        except Exception as e:
            self.get_logger().error(f"Erreur: {e}")
            return False

    def delete_turtle(self, name):
        """Supprimer une tortue spécifique"""
        if name not in self.active_turtles:
            self.get_logger().warn(f"La tortue '{name}' n'existe pas!")
            return False
        
        request = Kill.Request()
        request.name = name
        
        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            future.result()
            self.active_turtles.remove(name)
            self.get_logger().info(f"Tortue '{name}' supprimee!")
            return True
        except Exception as e:
            self.get_logger().error(f"Erreur: {e}")
            return False

    def list_turtles(self):
        """Lister toutes les tortues actives"""
        self.get_logger().info(f"Tortues actives ({len(self.active_turtles)}): {self.active_turtles}")
        return self.active_turtles

    def demo(self):
        """Démonstration des fonctionnalités"""
        self.get_logger().info("=== DEMONSTRATION ===")
        
        # Lister les tortues initiales
        self.list_turtles()
        
        # Spawn de nouvelles tortues
        self.spawn_turtle('bob', 2.0, 2.0)
        self.spawn_turtle('alice', 8.0, 8.0)
        self.spawn_turtle('charlie', 5.0, 2.0, 1.57)
        
        # Tester le doublon (bonus)
        self.get_logger().info("=== Test doublon ===")
        self.spawn_turtle('bob', 3.0, 3.0)  # Devrait echouer
        
        # Lister toutes les tortues
        self.list_turtles()
        
        # Supprimer une tortue
        self.get_logger().info("=== Suppression alice ===")
        self.delete_turtle('alice')
        
        # Lister après suppression
        self.list_turtles()
        
        self.get_logger().info("=== FIN DEMO ===")


def main(args=None):
    rclpy.init(args=args)
    node = MultiTurtleManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()