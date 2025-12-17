#!/usr/bin/env python3
"""
Simple ROS2 node to publish a map from a PGM+YAML file as nav_msgs/OccupancyGrid.
Useful to test RViz2 and the A* node without installing Nav2.
"""
import os
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

import yaml
import time
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def read_pgm(path):
    # Support both P2 (ASCII) and P5 (binary) PGM formats
    with open(path, 'rb') as f:
        # read magic
        magic = f.readline().strip()
        if magic not in (b'P2', b'P5'):
            raise RuntimeError('Unsupported PGM format: %s' % magic)

        # read header tokens, skipping comments
        def _read_token():
            while True:
                line = f.readline()
                if not line:
                    return b''
                line = line.strip()
                if line.startswith(b'#') or len(line) == 0:
                    continue
                for tok in line.split():
                    yield tok

        tokens = _read_token()
        try:
            width = int(next(tokens))
            height = int(next(tokens))
        except StopIteration:
            # fallback: maybe width/height were split across lines
            rest = b''
            while len(rest.split()) < 2:
                line = f.readline()
                if not line:
                    break
                rest += line
            parts = rest.split()
            if len(parts) < 2:
                raise RuntimeError('Invalid PGM header')
            width = int(parts[0])
            height = int(parts[1])

        # read maxval
        # maxval may be on the same token stream
        try:
            maxval = int(next(tokens))
        except StopIteration:
            # try reading next non-comment line
            while True:
                line = f.readline()
                if not line:
                    raise RuntimeError('Invalid PGM header (missing maxval)')
                line = line.strip()
                if line.startswith(b'#') or len(line) == 0:
                    continue
                maxval = int(line.split()[0])
                break

        # Now read pixel data
        if magic == b'P2':
            # ASCII: read remaining text and split
            rest = f.read().decode('ascii', errors='ignore')
            data = [int(tok) for tok in rest.split()]
            if len(data) < width * height:
                raise RuntimeError('PGM data truncated')
        else:
            # Binary P5. If maxval < 256 -> one byte per pixel, else two bytes
            if maxval < 256:
                raw = f.read(width * height)
                if len(raw) < width * height:
                    raise RuntimeError('PGM data truncated')
                data = [b for b in raw]
            else:
                raw = f.read(2 * width * height)
                if len(raw) < 2 * width * height:
                    raise RuntimeError('PGM data truncated')
                data = []
                for i in range(0, len(raw), 2):
                    # big-endian per PGM spec
                    val = (raw[i] << 8) + raw[i + 1]
                    data.append(val)

        return width, height, maxval, data


class MapPublisher(Node):
    def __init__(self):
        super().__init__('test_map_publisher')

        self.declare_parameter('map_yaml', 'maps/map.yaml')
        yaml_rel = self.get_parameter('map_yaml').get_parameter_value().string_value

        # If the path is relative and we're inside the package workspace, try to resolve
        if not os.path.isabs(yaml_rel):
            # try relative to package folder
            base = os.path.dirname(os.path.realpath(__file__))
            yaml_path = os.path.join(base, yaml_rel)
        else:
            yaml_path = yaml_rel

        if not os.path.exists(yaml_path):
            self.get_logger().error('Map YAML not found: %s' % yaml_path)
            raise SystemExit(1)

        with open(yaml_path, 'r') as yf:
            cfg = yaml.safe_load(yf)

        image_path = cfg.get('image')
        if not os.path.isabs(image_path):
            yaml_dir = os.path.dirname(yaml_path)
            candidate = os.path.join(yaml_dir, image_path)
            if os.path.exists(candidate):
                image_path = candidate
            else:
                # try one level up (package root) in case yaml lives in maps/ and image path already includes 'maps/'
                candidate2 = os.path.join(os.path.dirname(yaml_dir), image_path)
                if os.path.exists(candidate2):
                    image_path = candidate2
                else:
                    # fallback to candidate (even if missing) so error is explicit
                    image_path = candidate

        resolution = float(cfg.get('resolution', 0.1))
        origin = cfg.get('origin', [0.0, 0.0, 0.0])
        negate = int(cfg.get('negate', 0))
        occ_thresh = float(cfg.get('occupied_thresh', 0.65))
        free_thresh = float(cfg.get('free_thresh', 0.196))

        try:
            w, h, maxval, pixels = read_pgm(image_path)
        except Exception as e:
            self.get_logger().warn('Failed to read PGM %s: %s -- falling back to generated test map' % (image_path, str(e)))
            # create default 40x40 free map with a 10x10 obstacle in center
            w, h, maxval = 40, 40, 255
            pixels = []
            for y in range(h):
                for x in range(w):
                    if 15 <= x < 25 and 15 <= y < 25:
                        pixels.append(0)
                    else:
                        pixels.append(255)

        # Convert PGM pixel values to occupancy: 0..100, -1 unknown
        grid = []
        for p in pixels:
            # normalize to 0..1 (0 black, 1 white for PGM)
            norm = float(p) / float(maxval)
            if negate:
                norm = 1.0 - norm
            if norm <= (1.0 - occ_thresh):
                # obstacle
                grid.append(100)
            elif norm >= (1.0 - free_thresh):
                grid.append(0)
            else:
                grid.append(-1)

        # Create OccupancyGrid
        ag = OccupancyGrid()
        hdr = Header()
        hdr.frame_id = 'map'
        hdr.stamp = self.get_clock().now().to_msg()
        ag.header = hdr

        meta = MapMetaData()
        meta.resolution = resolution
        meta.width = w
        meta.height = h
        meta.origin.position.x = float(origin[0])
        meta.origin.position.y = float(origin[1])
        meta.origin.position.z = float(origin[2]) if len(origin) > 2 else 0.0
        meta.origin.orientation.w = 1.0
        ag.info = meta
        ag.data = grid

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)

        # Optionally wait for RViz or another subscriber before publishing
        wait_for_rviz = bool(cfg.get('wait_for_rviz', True))
        wait_timeout = float(cfg.get('wait_timeout', 15.0))
        if wait_for_rviz:
            self.get_logger().info(f'Waiting up to {wait_timeout}s for a /map subscriber (e.g. rviz2) before publishing map...')
            waited = 0.0
            interval = 0.25
            while rclpy.ok() and self.pub.get_subscription_count() == 0 and waited < wait_timeout:
                time.sleep(interval)
                waited += interval
            if self.pub.get_subscription_count() == 0:
                self.get_logger().warning('No /map subscribers detected after timeout; publishing anyway.')
            else:
                self.get_logger().info('Subscriber detected, publishing map now.')

        # publish once (transient local keeps it available for late subscribers)
        self.pub.publish(ag)
        self.get_logger().info('Published map %s (%dx%d) on /map' % (image_path, w, h))

        # Publish a static transform map -> odom so RViz has a TF to use for tests
        try:
            self._static_broadcaster = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self._static_broadcaster.sendTransform([t])
            self.get_logger().info('Published static transform map -> odom')
        except Exception:
            self.get_logger().warning('tf2_ros not available; skipping static transform publish')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        # keep alive so the transient_local publication remains available
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
