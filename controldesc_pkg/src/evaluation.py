#!/usr/bin/env python3
import math
import os
import sys
from datetime import datetime
import numpy as np

# Intentamos importar OpenCV
try:
    import cv2
except ImportError:
    print("ERROR: Necesitas instalar opencv-python")
    print("Ejecuta: pip install opencv-python")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid

class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger_node')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/astar_path')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('file_path', 'planning_metrics.txt')

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value

        self.start_time = None
        self.goal_received = False
        
        # Datos del Mapa
        self.distance_map = None 
        self.map_res = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_origin = None

        # Suscripciones
        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        self.create_subscription(Path, path_topic, self.path_cb, 10)
        
        qos_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, map_topic, self.map_cb, qos_map)

        self.init_file()
        self.get_logger().info(f'Metrics Logger iniciado. Guardando en: {self.file_path}')

    def init_file(self):
        if not os.path.exists(self.file_path):
            with open(self.file_path, 'w') as f:
                f.write("TIMESTAMP_LOG; TIEMPO_PLANIFICACION(s); LONGITUD_CAMINO(m); NUM_PUNTOS; DIST_MEDIA_OBST(m)\n")

    def map_cb(self, msg: OccupancyGrid):
        """
        CORREGIDO: Tratamiento estricto de 100 = Obstaculo, 0 = Libre.
        """
        self.get_logger().info("Procesando mapa...")
        
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        
        # 1. Convertir la lista plana de datos en una matriz numpy
        # Usamos int8 porque los datos vienen como -1, 0...100
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 2. Crear imagen binaria para OpenCV
        # Inicializamos todo en BLANCO (255) -> Asumimos LIBRE por defecto
        bin_img = np.full((height, width), 255, dtype=np.uint8)
        
        # 3. Marcar OBSTÁCULOS estrictos
        # OpenCV necesita que el obstáculo sea NEGRO (0) para medir distancia "hacia él"
        # Aquí decimos: Donde el mapa sea exactamente 100, pon un 0 en la imagen.
        bin_img[data == 100] = 0 
        
        # (Opcional) Si quieres considerar los desconocidos (-1) tambien como obstaculos:
        # bin_img[data == -1] = 0 
        
        # 4. Calcular Distance Transform
        # Calcula la distancia euclidea de cada pixel blanco al pixel negro (0) más cercano
        dist_px = cv2.distanceTransform(bin_img, cv2.DIST_L2, 5)
        
        # 5. Convertir distancia de pixeles a metros
        self.distance_map = dist_px * res
        
        self.map_res = res
        self.map_width = width
        self.map_height = height
        self.map_origin = msg.info.origin
        
        self.get_logger().info("Mapa de distancias actualizado.")

    def goal_cb(self, msg: PoseStamped):
        self.start_time = self.get_clock().now()
        self.goal_received = True
        self.get_logger().info("Goal recibido.")

    def path_cb(self, msg: Path):
        if not self.goal_received or self.start_time is None:
            return

        end_time = self.get_clock().now()
        duration_sec = (end_time - self.start_time).nanoseconds / 1e9

        path_length = 0.0
        total_obs_dist = 0.0
        valid_points = 0
        
        poses = msg.poses
        num_points = len(poses)
        has_map = (self.distance_map is not None)

        for i in range(num_points):
            # Calcular longitud
            if i < num_points - 1:
                p1 = poses[i].pose.position
                p2 = poses[i+1].pose.position
                path_length += math.hypot(p2.x - p1.x, p2.y - p1.y)
            
            # Calcular distancia a obstaculos
            if has_map:
                px = poses[i].pose.position.x
                py = poses[i].pose.position.y
                
                # Convertir coordenadas Mundo -> Mapa
                mx = int((px - self.map_origin.position.x) / self.map_res)
                my = int((py - self.map_origin.position.y) / self.map_res)
                
                # Verificar límites
                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    dist = self.distance_map[my, mx]
                    total_obs_dist += dist
                    valid_points += 1

        avg_obs_dist = 0.0
        if valid_points > 0:
            avg_obs_dist = total_obs_dist / valid_points
        elif not has_map:
            avg_obs_dist = -1.0 

        self.save_metrics(duration_sec, path_length, num_points, avg_obs_dist)

        self.goal_received = False
        self.start_time = None
        self.get_logger().info(f"Métricas: T={duration_sec:.3f}s, L={path_length:.2f}m, DistObst={avg_obs_dist:.2f}m")

    def save_metrics(self, time_sec, length, points, obs_dist):
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            line = f"{timestamp}; {time_sec:.6f}; {length:.4f}; {points}; {obs_dist:.4f}\n"
            with open(self.file_path, 'a') as f:
                f.write(line)
        except Exception as e:
            self.get_logger().error(f"Error escribiendo fichero: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()