#!/usr/bin/env python3
"""
A* planner node for ROS 2 Humble.

Subscriptions:
 - /map (nav_msgs/OccupancyGrid)
 - /amcl_pose (geometry_msgs/PoseWithCovarianceStamped) optional start
 - configurable goal topic (default: /goal_pose) (geometry_msgs/PoseStamped)

Publishes:
 - /astar_path (nav_msgs/Path)

Use the launch file in `launch/astar_launch.py` or run directly.
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
        super().__init__('astar_planner')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value

        # Map
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_width = 0
        self.map_height = 0
        self.map_res = 0.0
        self.map_origin = None
        self.map_data: List[int] = []

        # Poses
        self.start_pose: Optional[PoseStamped] = None
        self.goal_pose: Optional[PoseStamped] = None

        # Subscriptions (map uses transient_local to receive published map)
        qos_map = QoSProfile(depth=10)
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_map)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.get_logger().info('A* planner node (ROS 2 Humble) started')

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_data = list(msg.data)
        self.get_logger().info(f'Received map: {self.map_width}x{self.map_height} res={self.map_res}')

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.start_pose = ps

    def goal_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        self.get_logger().info(f'Goal received on {self.goal_topic}, planning...')
        self.plan_and_publish()

    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if not self.map_msg:
            return None
        ox = self.map_origin.position.x
        oy = self.map_origin.position.y
        mx = int((x - ox) / self.map_res)
        my = int((y - oy) / self.map_res)
        if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
            return None
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        ox = self.map_origin.position.x
        oy = self.map_origin.position.y
        x = ox + (mx + 0.5) * self.map_res
        y = oy + (my + 0.5) * self.map_res
        return x, y

    def is_free(self, mx: int, my: int) -> bool:
        idx = my * self.map_width + mx
        val = self.map_data[idx]
        return val >= 0 and val < 50

    def neighbors(self, mx: int, my: int):
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in dirs:
            nx = mx + dx
            ny = my + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                if self.is_free(nx, ny):
                    cost = math.hypot(dx, dy)
                    yield nx, ny, cost

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        (x1, y1) = a
        (x2, y2) = b
        return math.hypot(x2 - x1, y2 - y1)

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        gscore = {start: 0.0}

        while open_set and rclpy.ok():
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()
                return path

            for nx, ny, cost in self.neighbors(*current):
                neighbor = (nx, ny)
                tentative_g = gscore[current] + cost
                if neighbor not in gscore or tentative_g < gscore[neighbor]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None

    def plan_and_publish(self):
        if not self.map_msg:
            self.get_logger().warning('No map received yet')
            return

        if not self.goal_pose:
            self.get_logger().warning('No goal pose')
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

        if not self.is_free(*s_idx):
            self.get_logger().warning('Start in obstacle, searching nearby free cell')
            found = False
            for r in range(1, 6):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx = s_idx[0] + dx
                        ny = s_idx[1] + dy
                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height and self.is_free(nx, ny):
                            s_idx = (nx, ny)
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                self.get_logger().error('No free start found nearby')
                return

        if not self.is_free(*g_idx):
            self.get_logger().warning('Goal cell is occupied (may fail)')

        path_cells = self.a_star(s_idx, g_idx)
        if path_cells is None:
            self.get_logger().error('A* failed to find a path')
            return

        path_msg = Path()
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
