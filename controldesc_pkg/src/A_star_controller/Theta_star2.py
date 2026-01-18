#!/usr/bin/env python3
"""
Theta* (Any-Angle A*) planner node for ROS 2 Humble.
Versión Final: Comparación explícita de costes + Penalización por cercanía a obstáculos.

Subscriptions:
 - /map (nav_msgs/OccupancyGrid)
 - /amcl_pose (nav_msgs/PoseWithCovarianceStamped)
 - /goal_pose (geometry_msgs/PoseStamped)

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
        self.declare_parameter('downsample_factor', 4)
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value 
        self.downsample_factor = self.get_parameter('downsample_factor').get_parameter_value().integer_value

        # Mapas
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_width = 0
        self.map_height = 0
        self.map_res = 0.0
        self.map_origin = None
        
        self.ds_width = 0
        self.ds_height = 0
        self.ds_res = 0.0
        self.ds_data: List[int] = []

        # Poses
        self.start_pose: Optional[PoseStamped] = None 
        self.goal_pose: Optional[PoseStamped] = None 

        # QoS
        qos_map = QoSProfile(depth=10)
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        
        # Subs/Pubs
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_map)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_cb, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.get_logger().info('Theta* Planner (Explicit Cost + Wall Avoidance) started')

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_data = list(msg.data)
        self.downsample_map()
        self.get_logger().info(f'Map processed: {self.ds_width}x{self.ds_height}')

    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.start_pose = ps

    def goal_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        self.get_logger().info('Goal received, planning...')
        self.plan_and_publish()

    def downsample_map(self):
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
                            if cost < 0: cost = 100
                            max_cost = max(max_cost, cost)
                self.ds_data.append(max_cost)
    
    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if not self.map_msg: return None
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
        if mx < 0 or mx >= self.ds_width or my < 0 or my >= self.ds_height:
            return 100.0
        idx = my * self.ds_width + mx
        val = self.ds_data[idx]
        return 100.0 if val < 0 else float(val)

    def is_traversable(self, mx: int, my: int) -> bool:
        c = self.get_cell_cost(mx, my)
        return c < 100.0

    def get_line_data(self, start: Tuple[int, int], end: Tuple[int, int]) -> Tuple[bool, float]:
        """
        Bresenham modificado.
        Retorna: (visible: bool, costo_promedio: float)
        """
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        total_cost = 0.0
        count = 0
        
        while True:
            cost = self.get_cell_cost(x, y)
            if cost >= 100.0: # Obstáculo
                return False, 0.0
            
            total_cost += cost
            count += 1
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        avg_cost = total_cost / max(1, count)
        return True, avg_cost

    def get_neighbors(self, mx: int, my: int):
        """Vecinos 8-conectados"""
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in dirs:
            nx = mx + dx
            ny = my + dy
            if self.is_traversable(nx, ny):
                dist = math.hypot(dx, dy)
                cost = self.get_cell_cost(nx, ny)
                # Coste base para A*: distancia + 10% del coste de celda
                yield (nx, ny), dist + (cost * 0.1)

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def reconstruct_path(self, current: Tuple[int, int], parent: dict) -> List[Tuple[int, int]]:
        path = []
        while current is not None:
            path.insert(0, current)
            current = parent.get(current, None)
        return path

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Theta* con comparacion explícita y penalización de paredes.
        """
        self.get_logger().info(f'Planning from {start} to {goal}')
        
        open_list = [] 
        closed_list = set()
        open_set = set()
        
        g_score = {}  #coste acumulado desde inicio hasta el nodo que estamos evaluando
        h_score = {}  #coste heurístico estimado desde el nodo actual hasta el objetivo
        f_score = {}  #suma de g + h
        parent = {}   #padre de cada nodo en el camino, en caso de que se use el atajo, el padre puede ser el abuelo
        
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
            par_curr = parent.get(current)
            
            for neighbor, distance_cost in self.get_neighbors(*current):
                if neighbor in closed_list:
                    continue
                
                # --- OPCIÓN A: Camino normal (padre -> vecino) ---
                # distance_cost ya trae (dist + cell_cost*0.1)
                g_via_current = g_score[current] + distance_cost
                
                best_g = g_via_current
                best_parent = current

                # --- OPCIÓN B: Theta* Shortcut (abuelo -> vecino) ---
                if par_curr is not None:
                    # Usamos get_line_data para saber si es visible (no hay obstaculos entre medio) 
                    #Y el coste del terreno entre el abuelo y el vecino.
                    visible, line_avg_cost = self.get_line_data(par_curr, neighbor)
                    
                    if visible:
                        #si existe línea de visión directa calculamos el coste heurístico directo entre abuelo y vecino
                        dist_direct = self.heuristic(par_curr, neighbor)
                        
                        # PENALIZACIÓN: Si la línea pasa cerca de paredes (line_avg_cost alto),
                        # aumentamos artificialmente el coste para que prefiera otro camino.
                        # Factor 0.05 ajustable: 
                        #   Bajo (0.01) -> Corta esquinas agresivamente.
                        #   Alto (0.10) -> Evita paredes, hace curvas más amplias.
                        penalty = line_avg_cost * 0.05 * dist_direct
                        
                        g_via_grandparent = g_score[par_curr] + dist_direct + penalty 
                        
                        # COMPARACIÓN EXPLÍCITA
                        # Si el atajo (con su penalización) sigue siendo más barato, lo tomamos.
                        if g_via_grandparent < g_via_current:
                            best_g = g_via_grandparent
                            best_parent = par_curr

                # --- Actualización ---
                if neighbor not in g_score or best_g < g_score[neighbor]:
                    parent[neighbor] = best_parent
                    g_score[neighbor] = best_g 
                    h_score[neighbor] = self.heuristic(neighbor, goal)
                    f_score[neighbor] = g_score[neighbor] + h_score[neighbor]
                    
                    if neighbor not in open_set:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        open_set.add(neighbor)
                    else:
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
            self.get_logger().error('Bounds error')
            return

        if not self.is_traversable(*s_idx):
             self.get_logger().warning('Start in obstacle')
             return 

        if not self.is_traversable(*g_idx):
            self.get_logger().warning('Goal in obstacle - searching neighbor')
            found = False
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    nx, ny = g_idx[0]+dx, g_idx[1]+dy
                    if self.is_traversable(nx, ny):
                        g_idx = (nx, ny)
                        found = True
                        break
                if found: break
            if not found: return

        path_cells = self.a_star(s_idx, g_idx)
        
        if path_cells is None:
            self.get_logger().error('Pathfinder failed') 
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
        self.get_logger().info(f'Path published ({len(path_msg.poses)} points)')

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