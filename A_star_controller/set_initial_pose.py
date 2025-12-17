#!/usr/bin/env python3
"""Publish an initial robot pose to /initialpose and /amcl_pose (ROS 2 Humble).

Usage:
  python3 set_initial_pose.py --x 0.5 --y 0.0 --yaw 0.0

This helps set the robot start for AMCL and the A* node.
"""
import math
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


def quaternion_from_yaw(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)


class InitialPosePublisher(Node):
    def __init__(self, x, y, yaw, frame_id='map'):
        super().__init__('set_initial_pose')
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(float(yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # a reasonable small covariance (x,y,yaw variances)
        cov = [0.0] * 36
        cov[0] = 0.05 * 0.05  # x variance
        cov[7] = 0.05 * 0.05  # y variance
        cov[35] = (math.radians(10.0))**2  # yaw variance
        msg.pose.covariance = cov

        # publish a few times to ensure delivery
        for _ in range(6):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_init.publish(msg)
            self.pub_amcl.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser(description='Set initial robot pose')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw in radians')
    parser.add_argument('--frame', type=str, default='map')
    args = parser.parse_args()

    rclpy.init()
    node = InitialPosePublisher(args.x, args.y, args.yaw, frame_id=args.frame)
    node.get_logger().info(f'Published initial pose x={args.x} y={args.y} yaw={args.yaw} frame={args.frame}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
