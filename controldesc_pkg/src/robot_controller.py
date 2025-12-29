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

        self.goal = PoseStamped()
        self.goal_received = False
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Topic cmd_vel (según tu lista de topics, este es el correcto)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goalCallback, 10)

        self.get_logger().info('CONTROLADOR LISTO. Envía una meta (2D Nav Goal)...')

    def goalCallback(self, goal):
        self.get_logger().info(f'Meta Recibida en el marco de referencia: {goal.header.frame_id}')
        
        # ALERTA DE SEGURIDAD
        if goal.header.frame_id == self.robot_frame:
            self.get_logger().error('¡ERROR! Has enviado la meta respecto al propio robot (base_link).')
            self.get_logger().error('Así el robot nunca llegará. Envía la meta en "odom" o "map".')
            return

        self.goal = goal
        self.goal_received = True

    def command(self):
        if not self.goal_received:
            return
        
        # (Se ha eliminado la parte de la copia de la meta)

        try:
            # Buscamos transformación (Latest available) usando self.goal directamente
            if not self.tf_buffer.can_transform(self.robot_frame, self.goal.header.frame_id, Time()):
                self.get_logger().warn('Esperando enlace TF...')
                return

            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.goal.header.frame_id,
                Time() # Dame la última conocida
            )
            
            # Aplicamos la transformación directamente a self.goal
            pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(self.goal, transform)
            
            # 2. CÁLCULOS
            x = pose_transformed.pose.position.x
            y = pose_transformed.pose.position.y
            dist = math.sqrt(x**2 + y**2)

            # Log para verificar que BAJA la distancia
            self.get_logger().info(f'Distancia: {dist:.2f} m | X_rel: {x:.2f} | Y_rel: {y:.2f}')

            # 3. CONTROL
            linear = 0.0
            angular = 0.0

            if dist > self.goal_tol:
                # Giro Proporcional
                angular = y * 1.5
                
                # Avance (solo si el error angular es pequeño)
                if abs(y) < 0.3:
                    linear = 0.4
                else:
                    linear = 0.0
            else:
                self.get_logger().info('¡META ALCANZADA!')
                self.goal_received = False

            # Saturación (Límites de velocidad)
            linear = max(min(linear, 0.5), -0.5)
            angular = max(min(angular, 0.5), -0.5)

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
    # robot.shutdown() # Comentado porque shutdown no es un metodo de Node standard, se maneja abajo
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()