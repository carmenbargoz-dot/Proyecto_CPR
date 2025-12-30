#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist, PoseStamped

class RosbotController(Node):

    def __init__(self, rate):
        super().__init__('RosbotController')
        self.rate = rate
        
        # AJUSTES
        self.goal_tol = 0.15
        self.robot_frame = 'base_link' 
        
        # Parámetros del control descentrado
        self.d = 0.35  # Distancia 'd' al punto adelantado (en metros)
        self.k_p = 0.8 # Ganancia proporcional del controlador

        self.goal = PoseStamped()
        self.goal_received = False
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalCallback, 10)

        self.get_logger().info('CONTROLADOR DESCENTRADO LISTO.')

    def goalCallback(self, goal):
        self.get_logger().info(f'Meta Recibida en: {goal.header.frame_id}')
        if goal.header.frame_id == self.robot_frame:
            self.get_logger().error('¡ERROR! Meta referenciada a base_link.')
            return
        self.goal = goal
        self.goal_received = True

    def command(self):
        if not self.goal_received:
            return
        
        try:
            if not self.tf_buffer.can_transform(self.robot_frame, self.goal.header.frame_id, Time()):
                return

            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.goal.header.frame_id,
                Time()
            )
            
            pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(self.goal, transform)
            
            #CÁLCULOS DEL ERROR RESPECTO AL PUNTO DESCENTRADO, en este caso el punto está adelantado una distancia 'd'
            # El centro del robot es (0,0) en robot_frame. 
            # El punto P está en (self.d, 0) en robot_frame.
            # El error (ex, ey) es la posición de la meta respecto a ese punto P.
            ex = pose_transformed.pose.position.x - self.d
            ey = pose_transformed.pose.position.y
            
            dist_p = math.sqrt(ex**2 + ey**2) # Distancia del punto P a la meta

            self.get_logger().info(f'Error en P: [{ex:.2f}, {ey:.2f}] | Dist: {dist_p:.2f}')

            #LEY DE CONTROL DESCENTRADA
            linear = 0.0
            angular = 0.0

            if dist_p > self.goal_tol:
                # MODIFICACIÓN: Cálculo de velocidades cinemáticas
                # u_x = kp * ex, u_y = kp * ey
                # v = u_x
                # w = u_y / d
                linear = self.k_p * ex
                angular = (self.k_p * ey) / self.d
            else:
                self.get_logger().info('¡PUNTO P LLEGÓ A LA META!')
                self.goal_received = False

            # Saturación
            linear = max(min(linear, 0.5), -0.5)
            angular = max(min(angular, 0.8), -0.8) # El angular suele ser mayor en este control

            self.publish(linear, angular)

        except Exception as e:
            self.get_logger().error(f'Error TF: {e}')

    def publish(self, lin_vel, ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = float(lin_vel)
        move_cmd.angular.z = float(ang_vel)
        self.cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    robot = RosbotController(10)
    timer = robot.create_timer(0.1, robot.command)
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
