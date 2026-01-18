#!/usr/bin/env python3
"""
A* planner node for ROS 2 Humble - Version with explicit open/closed lists.

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

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header


class AStarNode(Node):
    def __init__(self):
        super().__init__('astar_planner_mejor')

        self.declare_parameter('goal_topic', '/goal_pose') 
        self.declare_parameter('downsample_factor', 3)  # Factor de reducción (1=sin reducción, 4=4x4 bloques)
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value 
        self.downsample_factor = self.get_parameter('downsample_factor').get_parameter_value().integer_value

        # Map (original)
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_width = 0
        self.map_height = 0
        self.map_res = 0.0
        self.map_origin = None
        self.map_data: List[int] = [] #donde guardamos el mapa de coste en una lista
        
        # Map (downsampled para A*)
        self.ds_width = 0
        self.ds_height = 0
        self.ds_res = 0.0
        self.ds_data: List[int] = []

        # Poses
        self.start_pose: Optional[PoseStamped] = None #para guardar el punto de partida
        self.goal_pose: Optional[PoseStamped] = None #para guardar el punto objetivo

        # Subscriptions 
        qos_map = QoSProfile(depth=10)
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_map) #nos suscribimos al mapa
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_cb, 10) #nos suscribimos a nuestra pose
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/astar_path', 10) #creamos topic para publicar el path

        self.get_logger().info('A* planner node (mejor version) started')

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_data = list(msg.data)
        
        # Crear versión downsampled del mapa: en vez de ir pixel a pixel tomo bloques de 4x4
        self.downsample_map()
        
        self.get_logger().info(f'Received map: {self.map_width}x{self.map_height} res={self.map_res}')
        self.get_logger().info(f'Downsampled to: {self.ds_width}x{self.ds_height} res={self.ds_res:.4f} (factor={self.downsample_factor})')

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
        """Reduce el mapa tomando bloques de downsample_factor x downsample_factor y calculando el máximo"""
        if self.downsample_factor <= 1:
            # Sin downsampling, usar mapa original
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
        
        # Para cada bloque del mapa reducido
        for ds_y in range(self.ds_height):
            for ds_x in range(self.ds_width):
                # Calcular el máximo coste en el bloque factor x factor y lo asignamos a la celda reducida
                max_cost = 0
                for by in range(factor):
                    for bx in range(factor):
                        orig_x = ds_x * factor + bx
                        orig_y = ds_y * factor + by
                        if orig_x < self.map_width and orig_y < self.map_height:
                            idx = orig_y * self.map_width + orig_x
                            cost = self.map_data[idx]
                            if cost < 0:  # desconocido tratarlo como obstáculo
                                cost = 100
                            max_cost = max(max_cost, cost)
                
                self.ds_data.append(max_cost)
    
    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """Convierte coordenadas mundo a coordenadas del mapa downsampled"""
        if not self.map_msg:
            return None
        ox = self.map_origin.position.x 
        oy = self.map_origin.position.y
        mx = int((x - ox) / self.ds_res)  #Usar resolución del mapa segmentado
        my = int((y - oy) / self.ds_res)
        if mx < 0 or my < 0 or mx >= self.ds_width or my >= self.ds_height:
            return None
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convierte coordenadas del mapa downsampled a coordenadas mundo"""
        ox = self.map_origin.position.x
        oy = self.map_origin.position.y
        x = ox + (mx + 0.5) * self.ds_res  # Usar resolución del mapa segmentado
        y = oy + (my + 0.5) * self.ds_res
        return x, y

    def get_cell_cost(self, mx: int, my: int) -> float:
        """Get the cost of a cell (0-100 scale) del mapa downsampled"""
        idx = my * self.ds_width + mx  # index del mapa downsampled
        val = self.ds_data[idx]  # toma el coste de esa posición
        if val < 0:  # valor de coste no válido suponemos que es obstaculo
            return 100.0  
        return float(val)

    def is_traversable(self, mx: int, my: int) -> bool:
        """Mira que no es un obstáculo (tiene un coste menor que 100 por lo que lo puede atravesar)"""
        idx = my * self.ds_width + mx 
        val = self.ds_data[idx]
        return val >= 0 and val < 100

    def get_neighbors(self, mx: int, my: int):
        """Vemos a que vecinos puede ir (los 8 de alrededor cual es atravesable y calcula sus costes)"""
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in dirs:
            nx = mx + dx
            ny = my + dy
            if 0 <= nx < self.ds_width and 0 <= ny < self.ds_height:  # Usar dimensiones del mapa segmentado
                if self.is_traversable(nx, ny): 
                    #se puede atravesar, calculo su coste:
                    # Simple cost: distance + small penalty for cell cost
                    distance = math.hypot(dx, dy)
                    cell_cost = self.get_cell_cost(nx, ny) 
                    # Add cell cost as a small additional factor
                    total_cost = distance + (cell_cost * 0.1) # El coste de la celda se añade con un peso pequeño para no romper la heurística
                    yield (nx, ny), total_cost

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Estimate cost from a to b (Euclidean distance)"""
        (x1, y1) = a
        (x2, y2) = b
        return math.hypot(x2 - x1, y2 - y1)

    def reconstruct_path(self, current: Tuple[int, int], parent: dict) -> List[Tuple[int, int]]:
        """Reconstruct path from start to current by following parent pointers"""
        path = []
        while current is not None:
            path.insert(0, current)  # voy añadiendo puntos al path y lo devuelvo en un array
            current = parent.get(current, None)
        return path  # de start a goal

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        A* pathfinding algorithm.
        """
        self.get_logger().info(f'A* searching from {start} to {goal}')
        
        # creamos una lista de de puntos por evaluar y ya evaluados
        open_list = [] 
        closed_list = set()  
        
        # Track which nodes are in open_list for fast lookup
        open_set = set()
        
        # Node properties
        g_score = {}  # Costo desde start al nodo que se evalua
        h_score = {}  # costo estimado del nodo al goal
        f_score = {}  # coste total es la suma de los anteriores
        parent = {}   # ponemos de donde viene el nodo para luego poder reconstruir el camino
        
        # Inicialmente en el comienzo
        g_score[start] = 0.0
        h_score[start] = self.heuristic(start, goal)
        f_score[start] = g_score[start] + h_score[start]
        parent[start] = None #no tiene padre porque es el primero
        
        heapq.heappush(open_list, (f_score[start], start)) ## Insertamos el nodo inicial en la open list priorizado por f_score
        open_set.add(start)
        
        iterations = 0
        
        while open_list and rclpy.ok():
            # Sacamos de la lista el de menor coste y el coste del mismo 
            current_f, current = heapq.heappop(open_list)
            open_set.discard(current)
            
            # vemos si el punto a evaluar ya es el goal, si es asi creamos el path y salimos
            if current == goal:
                return self.reconstruct_path(current, parent)
            
            #Movemos el punto que se esta evaluando a la lista de cerrados
            closed_list.add(current)
            
            # miramos todos los vecinos y sus costes
            for neighbor, distance_cost in self.get_neighbors(*current):
                # no miramos los vecinos que ya se han visitado
                if neighbor in closed_list:
                    continue
                
                # calculamos el coste de start al nodo: coste del nodo evaluado mas la distancia al nuevo
                tentative_g = g_score[current] + distance_cost
                
                # si el vecino no estaba en la lista abierta lo anadimos y le ponemos su coste
                if neighbor not in open_set:
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h_score[neighbor] = self.heuristic(neighbor, goal)
                    f_score[neighbor] = g_score[neighbor] + h_score[neighbor]
                    heapq.heappush(open_list, (f_score[neighbor], neighbor)) 
                    open_set.add(neighbor)
                elif tentative_g < g_score[neighbor]: 
                    # Si el coste del vecino es menor al que se calculo anteriormente significa que este path es menos costoso
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + h_score[neighbor]
                    #Lo metemos con el nuevo costo
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        # No encuentra path
        return None

    def plan_and_publish(self):
        if not self.map_msg:
            self.get_logger().warning('No map received yet')
            return

        if not self.goal_pose:
            self.get_logger().warning('No goal pose')
            return

        if self.start_pose:
            sx = self.start_pose.pose.position.x #tomamos pose inicial la que tiene el robot en ese momento /amcl_pose
            sy = self.start_pose.pose.position.y
        else:
            sx = self.map_origin.position.x + 0.5 * self.map_res
            sy = self.map_origin.position.y + 0.5 * self.map_res

        gx = self.goal_pose.pose.position.x #tomo el goal
        gy = self.goal_pose.pose.position.y

        s_idx = self.world_to_map(sx, sy) #vemos que estemos dentro del mapa
        g_idx = self.world_to_map(gx, gy)

        if s_idx is None or g_idx is None:
            self.get_logger().error('Start or goal out of map bounds')
            return

        if not self.is_traversable(*s_idx):
            self.get_logger().warning('Start in obstacle, searching nearby traversable cell') #si estamos es un obstaculo busca en sus vecinos uno que no sea y lo pone ahí el inicio (no pasara)
            found = False
            for r in range(1, 6):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx = s_idx[0] + dx
                        ny = s_idx[1] + dy
                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height and self.is_traversable(nx, ny):
                            s_idx = (nx, ny)
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                self.get_logger().error('No traversable start found nearby')
                return

        if not self.is_traversable(*g_idx): #si goal es un obstaculo busca entre sus vecinos una celda libre para poner como goal
            self.get_logger().warning('Goal cell is occupied, searching nearby traversable cell')
            found = False
            for r in range(1, 10):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx = g_idx[0] + dx
                        ny = g_idx[1] + dy
                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height and self.is_traversable(nx, ny):
                            g_idx = (nx, ny)
                            self.get_logger().info(f'Adjusted goal to nearby traversable cell')
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                self.get_logger().error('No traversable goal found nearby')
                return

        path_cells = self.a_star(s_idx, g_idx) #hacemos a*
        if path_cells is None:
            self.get_logger().error('A* failed to find a path') 
            return

        path_msg = Path() #creamos el mensaje de envio del path y enviamos en cada punto el valor en el world de los goals
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.map_msg.header.frame_id if self.map_msg and self.map_msg.header.frame_id else 'map'
        path_msg.header = hdr

        for mx, my in path_cells:
            x, y = self.map_to_world(mx, my)
            pose = PoseStamped()
            pose.header = hdr
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published A* path with {len(path_msg.poses)} poses')


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
