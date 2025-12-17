#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path


class TurtlebotController(Node):

    def __init__(self, rate):
        super().__init__('TurtlebotController')

        # Read parameters
        self.goal_tol = 0.15
        self.rate = rate  # Hz

        # Initialize internal data
        self.goal = PoseStamped()
        self.goal_received = False

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers / Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goalCallback,
            10
        )

        self.create_subscription(
            LaserScan,
            'scan',
            self.scanCallback,
            10
        )

        self.create_subscription(
            Path,
            'path',
            self.pathCallback,
            10
        )

        self.get_logger().info('TurtlebotController started')

        # nuestras variables
        self.transform_1 = 0
        self.bandera = 0
        self.theta = 0.0
        self.scan_enfrente = []
        self.scan_objetivo = []
        self.scan_derecha = []
        self.scan_izquierda = []
        self.estado = 0
        self.path_received = False
        self.k = 0
        self.sentido_giro = 1

    def shutdown(self):
        self.get_logger().info('Stop TurtleBot')
        self.cmd_vel_pub.publish(Twist())

    def goalCallback(self, goal):
        self.get_logger().info(
            'Goal received! x: %.2f, y: %.2f' %
            (goal.pose.position.x, goal.pose.position.y)
        )
        self.goal = goal
        self.goal_received = True

    def pathCallback(self, path):
        self.get_logger().info('Path received!')
        self.path = path
        self.path_received = True

    def scanCallback(self, scan):
        #self.get_logger().info('Scan received!')
        self.scan = scan

        self.i = int(self.theta * len(self.scan.ranges) / (2 * math.pi))

        self.scan_enfrente = (
            self.scan.ranges[len(self.scan.ranges) - 1 - 20:len(self.scan.ranges) - 1] +
            self.scan.ranges[0:20]
        )

        self.scan_derecha = self.scan.ranges[
            int(3 * len(self.scan.ranges) / 4) - 15:
            int(3 * len(self.scan.ranges) / 4) + 20
        ]

        self.scan_izquierda = self.scan.ranges[
            int(len(self.scan.ranges) / 4) - 15:
            int(len(self.scan.ranges) / 4) + 20
        ]

        if self.i < 20:
            self.scan_objetivo = (
                self.scan.ranges[0:self.i + 20] +
                self.scan.ranges[len(self.scan.ranges) - 1 + (self.i - 20):len(self.scan.ranges) - 1]
            )
        elif self.i > len(self.scan.ranges) - 20:
            self.scan_objetivo = (
                self.scan.ranges[0:self.i - len(self.scan.ranges) - 1 + 20] +
                self.scan.ranges[self.i - 20:len(self.scan.ranges) - 1]
            )
        else:
            self.scan_objetivo = self.scan.ranges[self.i - 20:self.i + 20]

    def command(self):

        if self.path_received:
            self.goal = self.path.poses[self.k]
            self.goal_received = True

        if not self.goal_received:
            #self.get_logger().info('Goal not received. Waiting...')
            return

        if self.goalReached():
            self.get_logger().info('GOAL REACHED!!! Stopping!')
            self.goal_received = False
            self.path_received = False
            self.publish(0.0, 0.0)
            self.k += 1
            return

        self.get_logger().info(
            'Goal received! x: %.2f, y: %.2f' %
            (self.goal.pose.position.x, self.goal.pose.position.y)
        )

        try: 
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                self.goal.header.frame_id,
                Time()
            )
            pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                self.goal, transform
            )
        except Exception as e:
            self.get_logger().error(f'TF ERROR: {e}')
            return


        self.theta = math.atan2(
            pose_transformed.pose.position.y,
            pose_transformed.pose.position.x
        )
        if self.theta < 0:
            self.theta += 2 * math.pi

        dist_objetivo = math.sqrt(
            pose_transformed.pose.position.x ** 2 +
            pose_transformed.pose.position.y ** 2
        )

        error = 0.50 if dist_objetivo > 1.0 else 0.15

        if self.bandera == 1:
            if self.estado == 0:
                angular = self.sentido_giro * 0.3
                linear = 0.0
                self.estado = 1
                for v in self.scan_enfrente:
                    if v < 0.55:
                        self.estado = 0
                        break

            elif self.estado == 1:
                angular = 0.0
                linear = 0.2
                self.estado = 0
                self.bandera = 0

                for v in self.scan_objetivo:
                    if v < 0.55:
                        self.estado = 1
                        self.bandera = 1
                        break

                for v in self.scan_enfrente:
                    if v < 0.55:
                        self.estado = 0
                        self.bandera = 1
                        break

            elif self.estado == 3:
                media_enf_der = sum(self.scan_enfrente[0:20]) / 20
                media_enf_izq = sum(self.scan_enfrente[21:-1]) / 20

                if media_enf_der < 0.55 and media_enf_izq < 0.55:
                    self.sentido_giro = -1 if min(self.scan_derecha) > min(self.scan_izquierda) else 1
                elif media_enf_der > media_enf_izq:
                    self.sentido_giro = -1
                else:
                    self.sentido_giro = 1

                self.estado = 0
                linear = 0.0
                angular = self.sentido_giro * 0.3

        elif (abs(pose_transformed.pose.position.y) > error or
              pose_transformed.pose.position.x < 0) and self.bandera == 0:
            linear = 0.0
            angular = pose_transformed.pose.position.y

        else:
            for v in self.scan_enfrente:
                if v < 0.55 and dist_objetivo > 0.56:
                    self.bandera = 1
                    self.estado = 3
                    break

            if abs(pose_transformed.pose.position.y - self.transform_1) < 1:
                if min(self.scan_enfrente) < 0.55 and dist_objetivo < 0.56:
                    linear = 0.05
                else:
                    linear = 0.4 - (0.3 / error) * abs(pose_transformed.pose.position.y)
                angular = pose_transformed.pose.position.y * 2
            else:
                linear = 0.0
                angular = 0.0

        self.transform_1 = pose_transformed.pose.position.y
        self.publish(linear, angular)

    def goalReached(self):
        if self.goal_received:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    self.goal.header.frame_id,
                    Time()
                )
                pose_transformed = tf2_geometry_msgs.do_transform_pose(
                    self.goal, transform
                )
            except Exception:
                return False

            goal_distance = math.sqrt(
                pose_transformed.pose.position.x ** 2 +
                pose_transformed.pose.position.y ** 2
            )

            if goal_distance < self.goal_tol:
                return True

        return False

    def publish(self, lin_vel, ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        self.cmd_vel_pub.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)

    rate = 10
    robot = TurtlebotController(rate)

    timer = robot.create_timer(
        1.0 / rate,
        robot.command
    )

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass

    robot.shutdown()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

