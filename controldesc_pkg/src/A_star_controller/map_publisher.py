#!/usr/bin/env python3
import os
import math
import time
import yaml
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def read_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic not in (b'P2', b'P5'):
            raise RuntimeError('Unsupported PGM format')

        def tokens():
            while True:
                line = f.readline()
                if not line:
                    return
                line = line.strip()
                if line.startswith(b'#') or not line:
                    continue
                for t in line.split():
                    yield t

        it = tokens()
        width = int(next(it))
        height = int(next(it))
        maxval = int(next(it))

        if magic == b'P2':
            data = [int(t) for t in f.read().split()]
        else:
            raw = f.read(width * height)
            data = list(raw)

        return width, height, maxval, data


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher_fixed')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('publish_tf', True)

        yaml_path = self.get_parameter('map_yaml').value
        publish_tf = self.get_parameter('publish_tf').value

        if not os.path.exists(yaml_path):
            raise RuntimeError(f'Map YAML not found: {yaml_path}')

        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)

        image_path = cfg['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        resolution = float(cfg['resolution'])
        origin = cfg.get('origin', [0.0, 0.0, 0.0])
        yaw = float(origin[2]) if len(origin) > 2 else 0.0
        negate = int(cfg.get('negate', 0))
        occ_thresh = float(cfg.get('occupied_thresh', 0.65))
        free_thresh = float(cfg.get('free_thresh', 0.196))

        w, h, maxval, pixels = read_pgm(image_path)

        # ---- Convertir PGM → OccupancyGrid ----
        grid = []
        for p in pixels:
            norm = p / maxval
            if negate:
                norm = 1.0 - norm

            if norm <= (1.0 - occ_thresh):
                grid.append(100)
            elif norm >= (1.0 - free_thresh):
                grid.append(0)
            else:
                grid.append(-1)

        # ---- FLIP EN Y (CLAVE) ----
        grid_2d = [
            grid[(h - 1 - y) * w:(h - y) * w]
            for y in range(h)
        ]
        
        # ---- CIERRE MORFOLÓGICO PARA TAPAR HUECOS EN PAREDES ----
        # Convertir a numpy array para procesamiento con OpenCV
        grid_np = np.array(grid_2d, dtype=np.int8)
        
        # Crear máscara binaria: 1 para obstáculos (100), 0 para el resto
        obstacles_mask = (grid_np == 100).astype(np.uint8)
        
        # Aplicar cierre morfológico con kernel pequeño para cerrar huecos pequeños
        kernel_size = 3  # Tamaño del kernel (3x3 pixels)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        closed_mask = cv2.morphologyEx(obstacles_mask, cv2.MORPH_CLOSE, kernel)
        
        # Aplicar el resultado: donde closed_mask tiene 1 y grid_np tenía 0 o -1, poner 100
        for y in range(h):
            for x in range(w):
                if closed_mask[y, x] == 1 and grid_2d[y][x] != 100:
                    grid_2d[y][x] = 100
        
        self.get_logger().info(f'Morfología: cerrados huecos pequeños con kernel {kernel_size}x{kernel_size}')
        
        # ---- DILATACIÓN DE OBSTÁCULOS (coste 50) ----
        inflation_radius = 6  # celdas de dilatación
        inflated_cost = 98
        inflated_cost2 = 70
        inflated_cost3 = 35
        inflated_cost4 = 20
        inflated_grid = [row[:] for row in grid_2d]  # copia
        
        for y in range(h):
            for x in range(w):
                if grid_2d[y][x] == 100:  # si es obstáculo
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < w and 0 <= ny < h:
                                if grid_2d[ny][nx] == 0:  # solo celdas libres
                                    dist = math.sqrt(dx*dx + dy*dy)
                                    new_cost = 0
                                    if dist > 0 and dist <= inflation_radius/3:
                                        new_cost = inflated_cost
                                    elif dist > 0 and dist <= inflation_radius/2:
                                        new_cost = inflated_cost2  # 75
                                    elif dist <= 3*inflation_radius/4:
                                        new_cost = inflated_cost3  # 50
                                    elif dist <= inflation_radius:
                                        new_cost = inflated_cost4  # 25
                                    
                                    # Mantener el coste más alto (más peligroso)
                                    if new_cost > inflated_grid[ny][nx]:
                                        inflated_grid[ny][nx] = new_cost
        
        grid = [cell for row in inflated_grid for cell in row]

        # ---- Crear mensaje ----
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        meta = MapMetaData()
        meta.resolution = resolution
        meta.width = w
        meta.height = h
        meta.origin.position.x = float(origin[0])
        meta.origin.position.y = float(origin[1])
        meta.origin.position.z = 0.0

        # ---- YAW CORRECTO ----
        meta.origin.orientation.z = math.sin(yaw / 2.0)
        meta.origin.orientation.w = math.cos(yaw / 2.0)

        msg.info = meta
        msg.data = grid

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.pub.publish(msg)
        self.get_logger().info('Map published correctly')

        

def main():
    rclpy.init()
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
