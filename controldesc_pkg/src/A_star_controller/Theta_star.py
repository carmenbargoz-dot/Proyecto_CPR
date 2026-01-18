#!/usr/bin/env python3
"""
Theta* (Any-Angle A*) planner node for ROS 2 Humble.
Permite trayectorias en cualquier angulo si existe Linea de Vision (Line of Sight).

Subscriptions:
 - /map (nav_msgs/OccupancyGrid)
 - /amcl_pose (nav_msgs/PoseWithCovarianceStamped) for robot position as start
 - configurable goal topic (default: /goal_pose) (geometry_msgs/PoseStamped)

Publishes:
 - /astar_path (nav_msgs/Path)
"""
import math
import heapq
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header


class AStarNode(Node):
    def __init__(self):
        super().__init__('astar_planner_theta')

        self.declare_parameter('goal_topic', '/goal_pose') 
        self.declare_parameter('downsample_factor', 4)  # Factor de reducción
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value 
        self.downsample_factor = self.get_parameter('downsample_factor').get_parameter_value().integer_value

        # Map (original)
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_width = 0
        self.map_height = 0
        self.map_res = 0.0
        self.map_origin = None
        self.map_data: List[int] = [] 
        
        # Map (downsampled para A*)
        self.ds_width = 0
        self.ds_height = 0
        self.ds_res = 0.0
        self.ds_data: List[int] = []

        # Poses
        self.start_pose: Optional[PoseStamped] = None 
        self.goal_pose: Optional[PoseStamped] = None 

        # Subscriptions 
        qos_map = QoSProfile(depth=10)
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_map)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_cb, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.get_logger().info('Theta* (Any-Angle) planner node started')

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_data = list(msg.data)
        
        self.downsample_map()
        
        self.get_logger().info(f'Received map: {self.map_width}x{self.map_height} res={self.map_res}')
        self.get_logger().info(f'Downsampled to: {self.ds_width}x{self.ds_height} res={self.ds_res:.4f}')

    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.start_pose = ps

    def goal_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        self.get_logger().info(f'Goal received on {self.goal_topic}, planning...')
        self.plan_and_publish()

    def downsample_map(self):
        """Reduce el mapa tomando bloques y calculando el máximo coste"""
        if self.downsample_factor <= 1:
            self.ds_width = self.map_width
            self.ds_height = self.map_height
            self.ds_res = self.map_res
            self.ds_data = self.map_data
            return
        
        factor = self.downsample_factor
        self.ds_width = self.map_width // factor
        self.ds_height = self.map_height // factor
        self.ds_res = self.map_res * factor
        self.ds_data = []
        
        for ds_y in range(self.ds_height):
            for ds_x in range(self.ds_width):
                max_cost = 0
                for by in range(factor):
                    for bx in range(factor):
                        orig_x = ds_x * factor + bx
                        orig_y = ds_y * factor + by
                        if orig_x < self.map_width and orig_y < self.map_height:
                            idx = orig_y * self.map_width + orig_x
                            cost = self.map_data[idx]
                            if cost < 0:
                                cost = 100
                            max_cost = max(max_cost, cost)
                self.ds_data.append(max_cost)
    
    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if not self.map_msg:
            return None
        ox = self.map_origin.position.x 
        oy = self.map_origin.position.y
        mx = int((x - ox) / self.ds_res)
        my = int((y - oy) / self.ds_res)
        if mx < 0 or my < 0 or mx >= self.ds_width or my >= self.ds_height:
            return None
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        ox = self.map_origin.position.x
        oy = self.map_origin.position.y
        x = ox + (mx + 0.5) * self.ds_res
        y = oy + (my + 0.5) * self.ds_res
        return x, y

    def get_cell_cost(self, mx: int, my: int) -> float:
        idx = my * self.ds_width + mx
        val = self.ds_data[idx]
        if val < 0:
            return 100.0  
        return float(val)

    def is_traversable(self, mx: int, my: int) -> bool:
        """Verifica limites y si es un obstaculo"""
        if mx < 0 or mx >= self.ds_width or my < 0 or my >= self.ds_height:
            return False
        idx = my * self.ds_width + mx 
        val = self.ds_data[idx]
        # Consideramos traversable si coste < 100
        return val >= 0 and val < 100

    def has_line_of_sight(self, start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        """
        Algoritmo de Bresenham para verificar si hay linea libre entre dos celdas.
        Devuelve True si todas las celdas en la linea son traversables.
        """
        x0, y0 = start
        x1, y1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        while True:
            if not self.is_traversable(x, y):
                return False
            
            if x == x1 and y == y1:
                return True
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def get_neighbors(self, mx: int, my: int):
        """Devuelve vecinos 8-conectados"""
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in dirs:
            nx = mx + dx
            ny = my + dy
            if self.is_traversable(nx, ny):
                # Calculo coste normal para Path 1
                distance = math.hypot(dx, dy)
                cell_cost = self.get_cell_cost(nx, ny)
                # Coste: distancia + pequeña penalizacion por terreno
                total_cost = distance + (cell_cost * 0.1)
                yield (nx, ny), total_cost

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        (x1, y1) = a
        (x2, y2) = b
        return math.hypot(x2 - x1, y2 - y1)

    def reconstruct_path(self, current: Tuple[int, int], parent: dict) -> List[Tuple[int, int]]:
        path = []
        while current is not None:
            path.insert(0, current)
            current = parent.get(current, None)
        return path

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Theta* pathfinding algorithm.
        Intenta conectar el padre del nodo actual (abuelo) con el vecino si hay vision directa (Line of Sight).
        """
        self.get_logger().info(f'Theta* searching from {start} to {goal}')
        
        open_list = [] 
        closed_list = set()
        open_set = set() # Para búsqueda rápida O(1)
        
        # Estructuras de datos
        g_score = {}  
        h_score = {}  
        f_score = {}  
        parent = {}   
        
        # Setup inicial
        g_score[start] = 0.0
        h_score[start] = self.heuristic(start, goal)
        f_score[start] = g_score[start] + h_score[start]
        parent[start] = None 
        
        heapq.heappush(open_list, (f_score[start], start)) 
        open_set.add(start)
        
        while open_list and rclpy.ok():
            current_f, current = heapq.heappop(open_list)
            
            if current in open_set:
                open_set.remove(current)
            
            if current == goal:
                return self.reconstruct_path(current, parent)
            
            closed_list.add(current)
            
            # Recuperamos al padre (abuelo del vecino potencial)
            par_curr = parent.get(current)
            
            # Iteramos sobre vecinos
            for neighbor, distance_cost in self.get_neighbors(*current):
                #no procesar si ya está en closed_list
                if neighbor in closed_list:
                    continue
                
                # --- LOGICA THETA* (A* All Angles) ---
                
                # Opcion 2 (Path 2): Linea directa desde el Abuelo -> Vecino
                # Verificamos si existe abuelo y si tiene linea de visión directa con el vecino
                if par_curr is not None and self.has_line_of_sight(par_curr, neighbor):
                    # Coste es g(abuelo) + distancia directa(abuelo, vecino) en este caso no miramos coste terreno intermedio, 
                    dist_los = self.heuristic(par_curr, neighbor)
                    tentative_g = g_score[par_curr] + dist_los
                    best_parent = par_curr
                
                # Opcion 1 (Path 1): Camino normal A* (Padre -> Vecino)
                else:
                    tentative_g = g_score[current] + distance_cost
                    best_parent = current

                # --- Actualización de los valores de coste de los vecinos ---
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    parent[neighbor] = best_parent # Actualizamos el padre al mejor encontrado
                    g_score[neighbor] = tentative_g 
                    h_score[neighbor] = self.heuristic(neighbor, goal)
                    f_score[neighbor] = g_score[neighbor] + h_score[neighbor]
                    
                    if neighbor not in open_set:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        open_set.add(neighbor)
                    else:
                        # Si ya está en open_list pero encontramos mejor camino, 
                        # hacemos push de nuevo (lazy update)
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        return None

    def plan_and_publish(self):
        if not self.map_msg or not self.goal_pose:
            return

        if self.start_pose:
            sx = self.start_pose.pose.position.x 
            sy = self.start_pose.pose.position.y
        else:
            sx = self.map_origin.position.x + 0.5 * self.map_res
            sy = self.map_origin.position.y + 0.5 * self.map_res

        gx = self.goal_pose.pose.position.x 
        gy = self.goal_pose.pose.position.y

        s_idx = self.world_to_map(sx, sy) 
        g_idx = self.world_to_map(gx, gy)

        if s_idx is None or g_idx is None:
            self.get_logger().error('Start or goal out of map bounds')
            return

        # Comprobación de start/goal no obstáculos (lógica original simplificada)
        if not self.is_traversable(*s_idx):
             self.get_logger().warning('Start in obstacle')
             return # Podrías añadir la logica de búsqueda cercana aquí si quieres mantenerla

        if not self.is_traversable(*g_idx):
            self.get_logger().warning('Goal in obstacle')
            # Búsqueda simple de vecino libre para el goal
            found = False
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    nx, ny = g_idx[0]+dx, g_idx[1]+dy
                    if self.is_traversable(nx, ny):
                        g_idx = (nx, ny)
                        found = True
                        break
                if found: break
            if not found: return

        path_cells = self.a_star(s_idx, g_idx)
        
        if path_cells is None:
            self.get_logger().error('Theta* failed to find a path') 
            return

        path_msg = Path()
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.map_msg.header.frame_id 
        path_msg.header = hdr

        for mx, my in path_cells:
            x, y = self.map_to_world(mx, my)
            pose = PoseStamped()
            pose.header = hdr
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published Theta* path with {len(path_msg.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()