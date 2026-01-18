#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time

class TestSuite(Node):
    def __init__(self):
        super().__init__('test_suite_node')
        
        # Publicadores para "forzar" el inicio y la meta
        self.start_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # DEFINIR TUS CASOS DE PRUEBA AQUÍ (x_start, y_start, x_goal, y_goal)
        self.test_cases = [
            (0.019, -0.11, 6.6, -10),   # Caso 1
            (6, -10, 12, -6.04),     # Caso 2
            (12,-6.04, 2.08, -9),     # Caso 3
            (2.08, -9.74, 10.98, -6.84),     # Caso 4
        ]

    def run_tests(self):
        self.get_logger().info(f"Iniciando {len(self.test_cases)} pruebas...")
        time.sleep(2) # Esperar conexiones

        for i, (sx, sy, gx, gy) in enumerate(self.test_cases):
            self.get_logger().info(f"--- TEST {i+1}/{len(self.test_cases)} ---")
            
            # 1. FORZAR PUNTO DE INICIO (Simular que el robot está ahí)
            start_msg = PoseWithCovarianceStamped()
            start_msg.header.frame_id = 'map'
            start_msg.header.stamp = self.get_clock().now().to_msg()
            start_msg.pose.pose.position.x = float(sx)
            start_msg.pose.pose.position.y = float(sy)
            start_msg.pose.pose.orientation.w = 1.0
            
            self.start_pub.publish(start_msg)
            self.get_logger().info(f"Set Start: ({sx}, {sy})")
            time.sleep(0.5) # Dar tiempo al planificador para recibir el 'amcl_pose'

            # 2. ENVIAR GOAL
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = float(gx)
            goal_msg.pose.position.y = float(gy)
            goal_msg.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal_msg)
            self.get_logger().info(f"Set Goal: ({gx}, {gy})")
            
            # 3. ESPERAR A QUE SE PLANIFIQUE
            # Ajusta este tiempo según lo que tarde tu PC
            time.sleep(3.0) 

        self.get_logger().info("Todos los tests completados.")

def main(args=None):
    rclpy.init(args=args)
    node = TestSuite()
    try:
        node.run_tests()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()