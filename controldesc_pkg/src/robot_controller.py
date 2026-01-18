#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rclpy
import os  # Para gestionar archivos ---

from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

# --- IMPORTS PARA SUAVIZADO ---
import numpy as np
from scipy.interpolate import splprep, splev
# ------------------------------

# --- PARA OBSTACULOS ---
from sensor_msgs.msg import LaserScan
# ------------------------------

# --- Para calcular x_d en archivo de datos) ---
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)
# ------------------------------------------------------------

class RosbotController(Node):
    def __init__(self, rate):
        super().__init__('RosbotController')
        self.rate = rate
        # AJUSTES
        self.goal_tol = 0.02 
        self.robot_frame = 'base_link'
        
        # Parámetros del control descentrado
        self.d = 0.1
        self.k_p = 0.8
        
        # PARAMETROS PARA EL DISENO CARROT
        self.carrot_dist = 0.4 
        self.path_tol = 0.5
        
        self.min_cruise_vel = 0.18 # Velocidad mínima de crucero

        # TRAYECTORIA
        self.path = []
        self.current_idx = 0
        self.path_received = False
        # PARA OBSTACULOS
        self.bandera = 0  # bandera de estado de movimiento: 0=normal, 1=evasion obstaculo
        self.theta = 0.0  # angulo hacia objetivo
        self.scan_enfrente = []  # scan frontal
        self.scan_objetivo = []  # scan hacia objetivo
        self.scan_derecha = []  # scan derecha
        self.scan_izquierda = []  # scan izquierda
        self.estado = 0  # estado de evasion: 0=girar, 1=avanzar, 3=eleccion sentido giro
        self.path_received = False  # path recibido
        self.sentido_giro = 1  # sentido de giro en evasion
        
        # Subscripcion al Laser
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- VARIABLES DE CONFIGURACIÓN OBSTACULOS ---
        
        # Configuración de índices del Escáner (Ancho de los conos de visión) --> LIDAR DE 3000 muestras
        self.idx_frente = 325      
        self.idx_burbuja = 750      
        self.idx_lat_ancho = 210   
        self.idx_obj_width = 325 
        
        # Distancias de seguridad para obstáculos
        self.dist_evasion = 0.30     
        self.dist_evasion_hys = 0.31
        
        # Velocidades y Límites
        self.vel_evasion_ang = 0.3     
        self.vel_evasion_lin = 0.15   
        self.max_lin_vel = 0.5        
        self.max_ang_vel = 0.8         
        self.ang_align_tol = 0.35     

        # Para Rviz
        self.create_subscription(Path, '/astar_path', self.path_callback, 10)
        self.smoothed_path_pub = self.create_publisher(Path, '/smoothed_path', 10)

        # --- PREPARAR ARCHIVOS DE DATOS (INICIALIZACION COMPLETA) ---
        # Borramos archivos anteriores para empezar limpio cada vez que se inicia el nodo
        archivos = ["trayectoria_real.txt", "trayectoria_original.txt", "trayectoria_suavizada.txt"]
        for a in archivos:
            if os.path.exists(a):
                os.remove(a)
        
        # Creamos los archivos y escribimos las cabeceras
        with open("trayectoria_real.txt", "w") as f:
            f.write("x_robot,y_robot,x_desc,y_desc\n")
            
        with open("trayectoria_original.txt", "w") as f:
            f.write("x,y\n")
            
        with open("trayectoria_suavizada.txt", "w") as f:
            f.write("x,y\n")
        # -----------------------------------------------------------

        self.create_timer(0.1, self.command)
        self.get_logger().info('CONTROLADOR DESCENTRADO LISTO (MODO ACUMULATIVO).')

    # --- FUNCIÓN DE SUAVIZADO (Rectas densas + Splines) ---
    def smooth_path(self, poses):
        if len(poses) < 3:
            return poses
        try:
            # 1. Rellenar huecos en rectas largas
            x_dense = []
            y_dense = []
            resolution = 0.5 # Meter punto si distancia > 0.5m
            
            for i in range(len(poses) - 1):
                p1 = poses[i].pose.position
                p2 = poses[i+1].pose.position
                x_dense.append(p1.x)
                y_dense.append(p1.y)
                
                dist = math.hypot(p2.x - p1.x, p2.y - p1.y)
                if dist > resolution:
                    num_points = int(dist / resolution)
                    for j in range(1, num_points):
                        x_dense.append(p1.x + (p2.x - p1.x) * (j/num_points))
                        y_dense.append(p1.y + (p2.y - p1.y) * (j/num_points))
            
            # Añadir el último punto
            x_dense.append(poses[-1].pose.position.x)
            y_dense.append(poses[-1].pose.position.y)

            # 2. Interpolación de Spline
            # s=0.0 asegura que pase por los puntos 
            tck, u = splprep([x_dense, y_dense], s=0.0, k=2)
            
            # Generamos los puntos finales (densidad alta para movimiento fluido)
            u_new = np.linspace(0, 1, num=len(x_dense) * 2) 
            x_new, y_new = splev(u_new, tck)
            
            new_path = []
            frame_id = poses[0].header.frame_id
            for i in range(len(x_new)):
                p = PoseStamped()
                p.header.frame_id = frame_id
                p.pose.position.x = x_new[i]
                p.pose.position.y = y_new[i]
                p.pose.orientation.w = 1.0
                new_path.append(p)
                
            return new_path
        except Exception as e:
            self.get_logger().warn(f'Fallo al suavizar: {e}')
            return poses
    # -----------------------------------------------------

    # CALLBACK LASER
    def scan_callback(self, msg):
        self.scan = msg
        self.scan.ranges = [10.0 if i < 0.1 else i for i in self.scan.ranges]
        # Procesar scan, obtener conos de interes
        total_len = len(self.scan.ranges)
        
        self.i = int(self.theta * total_len / (2 * math.pi))
        
        # Scan enfrente
        self.scan_enfrente = self.scan.ranges[total_len - self.idx_frente : total_len] + self.scan.ranges[0 : self.idx_frente]
        
        # Scan derecha e izquierda 
        base_der = int(3 * total_len / 4)
        base_izq = int(total_len / 4)
        
        self.scan_derecha = self.scan.ranges[base_der - self.idx_lat_ancho : base_der + self.idx_lat_ancho]
        self.scan_izquierda = self.scan.ranges[base_izq - self.idx_lat_ancho : base_izq + self.idx_lat_ancho]
        
        # Scan Objetivo
        w_obj = self.idx_obj_width
        if self.i < w_obj:
            self.scan_objetivo = self.scan.ranges[0 : self.i + w_obj] + self.scan.ranges[total_len + (self.i - w_obj) : total_len]
        elif self.i > total_len - w_obj:
            self.scan_objetivo = self.scan.ranges[0 : self.i - total_len + w_obj] + self.scan.ranges[self.i - w_obj : total_len]
        else:
            self.scan_objetivo = self.scan.ranges[self.i - w_obj : self.i + w_obj]
    # -----------------------------------------------------

    # CALLBACK DEL PATH PLANIFICADO --> SUAVIZADO
    def path_callback(self, msg):
        # --- Añadir puntos del path al archivo de datos ---
        try:
            with open("trayectoria_original.txt", "a") as f:
                for p in msg.poses:
                    f.write(f"{p.pose.position.x},{p.pose.position.y}\n")
        except Exception: pass
        # ----------------------------------------------------

        # 1. Suavizar
        self.path = self.smooth_path(msg.poses)
        
        # --- Añadir puntos del spline al archivo de datos ---
        try:
            with open("trayectoria_suavizada.txt", "a") as f:
                for p in self.path:
                    f.write(f"{p.pose.position.x},{p.pose.position.y}\n")
        except Exception: pass
        # ----------------------------------------------------

        # 2. Publicar en Rviz
        viz_msg = Path()
        viz_msg.header = msg.header 
        viz_msg.poses = self.path
        self.smoothed_path_pub.publish(viz_msg)

        self.current_idx = 0
        self.path_received = True
        self.get_logger().info(f'Trayectoria recibida con {len(self.path)} puntos')
    # -----------------------------------------------------

    # CARROT --> Tener en cuenta puntos futuros
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
    # -----------------------------------------------------

    # CLOSEST POINT 
    def update_index_to_closest(self):
        # Busca el punto de la trayectoria más cercano al robot. Permite que si evitamos
        # un obstaculo y avanzamos, el robot sepa que ha avanzado y no intente volver atrás

        if not self.path:
            return

        # Evita que salte prematuramente al ultimo punto
        if self.current_idx > len(self.path) - 10:
            return

        try:
            # 1. Obtenemos la posicion global del robot 
            path_frame = self.path[0].header.frame_id
            
            # Buscamos la transformación
            trans = self.tf_buffer.lookup_transform( path_frame, self.robot_frame, Time())
            
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y

            # 2. Busca punto más cercano, solo los de delante
            min_dist = 10000.0
            best_idx = self.current_idx
            
            search_limit = min(self.current_idx + 200, len(self.path))
            
            for i in range(self.current_idx, search_limit):
                px = self.path[i].pose.position.x
                py = self.path[i].pose.position.y
                
                # Distancia euclidea
                d = math.hypot(px - rx, py - ry)
                
                if d < min_dist:
                    min_dist = d
                    best_idx = i
            
            # Solo actualizamos si hemos encontrado un punto más avanzado que esté más cerca
            if best_idx > self.current_idx:
                self.current_idx = best_idx

        except Exception as e:
            # Si falla la TF (pasa al inicio), no hacemos nada y usamos la logica normal
            pass
    # -----------------------------------------------------

    def command(self):
        if not self.path_received:
            return

        # --- GUARDRA DATOS TRAYECTORIA REAL ---
        try:
            # Obtenemos posición real para guardar en txt
            target_frame = self.path[0].header.frame_id
            t = self.tf_buffer.lookup_transform(target_frame, self.robot_frame, Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            
            # Obtenemos Yaw para calcular punto descentrado
            q = t.transform.rotation
            yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            # Punto descentrado
            dx = rx + self.d * math.cos(yaw)
            dy = ry + self.d * math.sin(yaw)
            
            with open("trayectoria_real.txt", "a") as f:
                f.write(f"{rx},{ry},{dx},{dy}\n")
        except Exception: 
            pass
        # ---------------------------------------

        self.update_index_to_closest()  # Actualizar índice al más cercano

        carrot = self.get_carrot()

        # Comprobar si se ha completado la trayectoria
        if carrot is None:
            self.publish(0.0, 0.0)
            self.get_logger().info(f'Trayectoria completada', once=True)
            return

        while carrot is not None:
            try:
                # Intentar obtener la transformación
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    carrot.header.frame_id,
                    Time()
                )
                carrot_tf = tf2_geometry_msgs.do_transform_pose_stamped(carrot, transform)
                dist_objetivo = math.sqrt(carrot_tf.pose.position.x**2 + carrot_tf.pose.position.y**2)

                es_ultimo = (carrot == self.path[-1])
                
                tolerancia_actual = 0.02 if es_ultimo else self.goal_tol    # Tolerancia dinámica --> Menor si es el último punto

                if dist_objetivo < tolerancia_actual: # siguiente objetivo si se llega
                    self.current_idx += 1
                    carrot = self.get_carrot()
                else:
                    break
            except Exception as e:
                self.get_logger().warn(f'Esperando transformacion (TF): {e}', once=True)
                return

        ang_objetivo = math.atan2(carrot_tf.pose.position.y, carrot_tf.pose.position.x)
        
        # LEY DE CONTROL
        linear = 0.0
        angular = 0.0
        
        # Hallar angulo objetivo
        self.theta = math.atan2(carrot_tf.pose.position.y, carrot_tf.pose.position.x)
        if self.theta < 0:
            self.theta = self.theta + 2*math.pi

        # Evasion de obstaculos
        if self.bandera == 1:
            if self.estado == 0:  # Girando
                angular = self.sentido_giro * self.vel_evasion_ang
                linear = 0.0
                self.estado = 1  # Si no hay obstaculo enfrente pasamos a estado 1
                if min(self.scan_enfrente) < self.dist_evasion:
                    self.estado = 0
            elif self.estado == 1:  # Avanzando
                angular = 0.0
                linear = self.vel_evasion_lin 
                self.estado = 0
                self.bandera = 0
                # Si hay obstaculo hacia objetivo seguimos en este estado
                if min(self.scan_objetivo) < self.dist_evasion:
                    self.estado = 1
                    self.bandera = 1
                    
                # Si hay obstaculo enfrente pasamos al estado 0
                if min(self.scan_enfrente) < self.dist_evasion: 
                    self.estado = 0
                    self.bandera = 1
                # Si no hay obstaculo en los conos anteriores pasamos al estado 0 y salimos de evasion de obstaculos (bandera --> 0)
            elif self.estado == 3:  # Eleccion de sentido de giro inicial
                # Calculamos la media de la mitad izq y der del cono de enfrente
                mid_point = self.idx_frente
                # Verificar para evitar errores si scan vacio
                if len(self.scan_enfrente) > 0:
                    media_enf_der = sum(self.scan_enfrente[0:mid_point]) / len(self.scan_enfrente[0:mid_point])
                    media_enf_izq = sum(self.scan_enfrente[mid_point:len(self.scan_enfrente)]) / len(self.scan_enfrente[mid_point:len(self.scan_enfrente)])
                    
                    if media_enf_der < self.dist_evasion and media_enf_izq < self.dist_evasion: # Antes 0.55
                        if min(self.scan_derecha) > min(self.scan_izquierda):
                            self.sentido_giro = -1
                        else:
                            self.sentido_giro = 1
                    elif media_enf_der > media_enf_izq:  # si no, se gira en el sentido en que haya menos obstaculos enfrente
                        self.sentido_giro = -1
                    else:
                        self.sentido_giro = 1
                
                self.estado = 0  # una vez decidido el sentido de giro se va al estado 0
                linear = 0.0
                angular = self.sentido_giro * self.vel_evasion_ang

        # Actuar normal si no hay obstaculo
        else:
            # Si el objetivo no esta centrado, girar sobre si mismo            
            if abs(ang_objetivo) > self.ang_align_tol: 
                linear = 0.0
                angular = ang_objetivo
            else:
                # Revisar si hay obstaculo enfrente, mas cercano que el propio objetivo
                if min(self.scan_enfrente) < self.dist_evasion and dist_objetivo > self.dist_evasion_hys:
                    self.bandera = 1  # Si hay un obstaculo pasamos a evitacion de obstaculos
                    self.estado = 3
                ex = carrot_tf.pose.position.x - self.d
                ey = carrot_tf.pose.position.y
                dist_p = math.sqrt(ex**2 + ey**2)
                if dist_p > self.path_tol:
                    linear = self.k_p * ex
                    angular = (self.k_p * ey) / self.d
                else:
                    if self.get_carrot() != self.path[-1]:
                        self.current_idx += 1
                    else:
                        # Si es el ultimo, seguimos calculando
                        linear = self.k_p * ex
                        angular = (self.k_p * ey) / self.d

                # --- CORRECCION DE VELOCIDAD ---
                # Si la velocidad calculada es muy baja y NO estamos en la evasion, forzamos una velocidad minima
                # Si estamos llegando al ultimo punto que frene              
                es_ultimo_punto = (self.get_carrot() == self.path[-1])
                dist_meta = dist_objetivo if es_ultimo_punto else 999.0
                
                if abs(linear) < self.min_cruise_vel:
                    if not (es_ultimo_punto and dist_meta < 0.15):
                        if linear >= 0: 
                            linear = self.min_cruise_vel
                        else: 
                            linear = -self.min_cruise_vel
                # ----------------------------------------

        # Saturación (Usando max_lin_vel y max_ang_vel)
        linear = max(min(linear, self.max_lin_vel), -self.max_lin_vel)
        angular = max(min(angular, self.max_ang_vel), -self.max_ang_vel)

        self.publish(linear, angular)

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