#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

class RosbotController(Node):

    def __init__(self, rate):
        super().__init__('RosbotController')
        self.rate = rate
        
        # AJUSTES
        self.goal_tol = 0.15
        self.robot_frame = 'base_link' 
        
        # Parámetros del control descentrado
        self.d = 0.35  
        self.k_p = 0.8 
        
        # PARAMETROS PARA EL DISENO CARROT
        self.carrot_dist = 0.6 
        self.path_tol = 0.2    

        # TRAYECTORIA
        self.path = []         
        self.current_idx = 0   
        self.path_received = False
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Aquí hemos cambiado /path por /astar_path
        self.create_subscription(Path, '/astar_path', self.path_callback, 10)

        self.create_timer(0.1, self.command)
        self.get_logger().info('CONTROLADOR DESCENTRADO LISTO.')
        
    def path_callback(self, msg):
        self.path = msg.poses 
        self.current_idx = 0 
        self.path_received = True
        self.get_logger().info(f'Trayectoria recibida con {len(self.path)} puntos')

    def get_carrot(self):
        if self.current_idx >= len(self.path):
            return None

        dist_acc = 0.0
        carrot_idx = self.current_idx

        while carrot_idx + 1 < len(self.path) and dist_acc < self.carrot_dist: 
            p1 = self.path[carrot_idx].pose.position
            p2 = self.path[carrot_idx + 1].pose.position
            dist_acc += math.hypot(p2.x - p1.x, p2.y - p1.y)
            carrot_idx += 1

        return self.path[carrot_idx]

    def command(self):
        if not self.path_received:
            return
        
        carrot = self.get_carrot()
        if carrot is None:
            self.publish(0.0, 0.0)
            self.get_logger().info('Trayectoria completada')
            return
        
        try:
            # Intentar obtener la transformación
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                carrot.header.frame_id,
                Time()
            )

            carrot_tf = tf2_geometry_msgs.do_transform_pose_stamped(carrot, transform)
            
            ex = carrot_tf.pose.position.x - self.d
            ey = carrot_tf.pose.position.y
            dist_p = math.sqrt(ex**2 + ey**2) 

            # LEY DE CONTROL
            linear = 0.0
            angular = 0.0

            if dist_p > self.path_tol:
                linear = self.k_p * ex
                angular = (self.k_p * ey) / self.d
            else:
                self.current_idx += 1  

            # Saturación
            linear = max(min(linear, 0.5), -0.5)
            angular = max(min(angular, 0.8), -0.8)

            self.publish(linear, angular)

        except Exception as e:
            # Si AMCL no está listo o falta el Pose Estimate, saldrá este error
            self.get_logger().warn(f'Esperando transformacion (TF): {e}', once=True)

    def publish(self, lin_vel, ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = float(lin_vel)
        move_cmd.angular.z = float(ang_vel)
        self.cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    robot = RosbotController(10)
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
