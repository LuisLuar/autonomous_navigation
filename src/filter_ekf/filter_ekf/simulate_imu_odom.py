#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import time
import numpy as np

class SimulateImuOdom(Node):
    def __init__(self):
        super().__init__('simulate_imu_odom_gps')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom/unfiltered', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/unfiltered', 50)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber para cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # Timers
        self.odom_timer = self.create_timer(0.1, self.publish_odom_tf)   # 10 Hz
        self.imu_timer = self.create_timer(0.02, self.publish_imu)       # 50 Hz
        self.gps_timer = self.create_timer(1.0, self.publish_gps)        # 1 Hz para GPS
        
        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.start_time = time.time()
        self.last_odom_time = self.get_clock().now()
        
        # Variables para comando de velocidad
        self.linear_vel_x = 0.0
        self.angular_vel_z = 0.0
        
        # Punto de origen GPS fijo
        self.origin_latitude = -0.31700598389186346
        self.origin_longitude = -78.44395726919176
        self.latitude = self.origin_latitude
        self.longitude = self.origin_longitude
        self.altitude = 2850.0
        
        # Constantes para conversión de metros a grados (aproximado)
        self.meters_to_degrees = 1.0 / 111320.0  # 1 grado ≈ 111.32 km en Ecuador
        
        self.get_logger().info('Nodo de simulación listo. Esperando cmd_vel...')
        
    def cmd_vel_callback(self, msg):
        """Callback para recibir comandos de velocidad"""
        self.linear_vel_x = msg.linear.x
        self.angular_vel_z = msg.angular.z
        
        self.get_logger().debug(
            f'Cmd_vel recibido: vx={self.linear_vel_x:.2f} m/s, wz={self.angular_vel_z:.2f} rad/s',
            throttle_duration_sec=1.0
        )
        
    def update_position(self, dt):
        """Actualizar posición basada en el modelo cinemático diferencial"""
        if abs(self.angular_vel_z) < 0.001:  # Movimiento rectilíneo
            self.x += self.linear_vel_x * math.cos(self.yaw) * dt
            self.y += self.linear_vel_x * math.sin(self.yaw) * dt
        else:  # Movimiento curvilíneo
            # Radio de curvatura
            radius = self.linear_vel_x / self.angular_vel_z if self.angular_vel_z != 0 else float('inf')
            
            # Actualizar orientación
            self.yaw += self.angular_vel_z * dt
            
            # Normalizar yaw entre -pi y pi
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            
            # Actualizar posición (modelo de odometría diferencial)
            if abs(radius) < 1000:  # Evitar problemas con radios muy grandes
                self.x += radius * (math.sin(self.yaw) - math.sin(self.yaw - self.angular_vel_z * dt))
                self.y += radius * (math.cos(self.yaw - self.angular_vel_z * dt) - math.cos(self.yaw))
            else:
                self.x += self.linear_vel_x * math.cos(self.yaw) * dt
                self.y += self.linear_vel_x * math.sin(self.yaw) * dt
        
        # Actualizar posición GPS (simplificado)
        # En realidad necesitarías convertir metros a grados usando proyección UTM
        self.latitude = self.origin_latitude + (self.y * self.meters_to_degrees)
        self.longitude = self.origin_longitude + (self.x * self.meters_to_degrees / 
                                                  math.cos(math.radians(self.origin_latitude)))
        
    def publish_odom_tf(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time
        
        # Actualizar posición basada en cmd_vel
        self.update_position(dt)
        
        # 1. PUBLICAR TRANSFORMADA TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convertir yaw a cuaternión
        q = quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Enviar transformada
        self.tf_broadcaster.sendTransform(t)
        
        # 2. PUBLICAR MENSAJE DE ODOMETRÍA
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # Covarianza (simulando incertidumbre)
        odom_msg.pose.covariance[0] = 0.01  # x
        odom_msg.pose.covariance[7] = 0.01  # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        
        odom_msg.twist.twist.linear.x = self.linear_vel_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel_z
        
        self.odom_pub.publish(odom_msg)
        
        # Log cada 2 segundos
        self.get_logger().info(
            f'Odometría: x={self.x:.2f}m, y={self.y:.2f}m, yaw={math.degrees(self.yaw):.1f}°, '
            f'vx={self.linear_vel_x:.2f}m/s, wz={self.angular_vel_z:.2f}rad/s',
            throttle_duration_sec=2.0
        )
        
    def publish_imu(self):
        current_time = self.get_clock().now()
        
        # 3. PUBLICAR MENSAJE DE IMU
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Velocidad angular (coherente con cmd_vel)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = self.angular_vel_z
        
        # Aceleración lineal (simulando aceleración/deceleración)
        # Añadir algo de ruido para realismo
        noise = np.random.normal(0, 0.01, 3)
        
        if abs(self.linear_vel_x) > 0.01:
            # Simular aceleración cuando hay movimiento
            imu_msg.linear_acceleration.x = self.linear_vel_x * 0.5 + noise[0]
            imu_msg.linear_acceleration.y = noise[1]
        else:
            imu_msg.linear_acceleration.x = noise[0]
            imu_msg.linear_acceleration.y = noise[1]
        
        # Gravedad + algo de ruido
        imu_msg.linear_acceleration.z = 9.81 + noise[2]
        
        # Orientación del IMU (coherente con odometría)
        q = quaternion_from_euler(0, 0, self.yaw)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        
        # Covarianzas
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01
        imu_msg.linear_acceleration_covariance[0] = 0.01
        
        self.imu_pub.publish(imu_msg)
        
    def publish_gps(self):
        """Publicar datos GPS que se actualizan con el movimiento"""
        current_time = self.get_clock().now()
        
        gps_msg = NavSatFix()
        gps_msg.header.stamp = current_time.to_msg()
        gps_msg.header.frame_id = "gnss_frame"
        
        gps_msg.status.status = 0  # STATUS_FIX
        gps_msg.status.service = 1  # SERVICE_GPS
        
        # Coordenadas actualizadas según movimiento
        gps_msg.latitude = self.latitude
        gps_msg.longitude = self.longitude
        gps_msg.altitude = self.altitude
        
        # Matriz de covarianza (mejor precisión cuando está quieto)
        if abs(self.linear_vel_x) < 0.1 and abs(self.angular_vel_z) < 0.1:
            # Alta precisión cuando está quieto
            gps_msg.position_covariance[0] = 0.000001
            gps_msg.position_covariance[4] = 0.000001
        else:
            # Menor precisión en movimiento
            gps_msg.position_covariance[0] = 0.0001
            gps_msg.position_covariance[4] = 0.0001
            
        gps_msg.position_covariance[8] = 1.0
        gps_msg.position_covariance_type = 2
        
        self.gps_pub.publish(gps_msg)
        
        self.get_logger().info(
            f'GPS: lat={self.latitude:.9f}, lon={self.longitude:.9f}, '
            f'desplazamiento={math.sqrt(self.x**2 + self.y**2):.1f}m',
            throttle_duration_sec=5.0
        )

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimulateImuOdom()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('👋 Nodo OSM Planner detenido')
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()