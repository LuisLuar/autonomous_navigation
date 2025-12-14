#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math
import time

class SimulateImuOdom(Node):
    def __init__(self):
        super().__init__('simulate_imu_odom')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom/unfiltered', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/unfiltered', 50)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)  # NUEVO: Publisher GPS
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.odom_timer = self.create_timer(0.1, self.publish_odom_tf)   # 10 Hz
        self.imu_timer = self.create_timer(0.02, self.publish_imu)       # 50 Hz
        self.gps_timer = self.create_timer(1.0, self.publish_gps)        # NUEVO: 1 Hz para GPS
        
        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.start_time = time.time()
        self.last_odom_time = self.get_clock().now()
        
        # Coordenadas GPS fijas (las que me diste)
        self.latitude = -0.31700598389186346
        self.longitude = -78.44395726919176
        self.altitude = 2850.0  # Altitud aproximada de Quito en metros
        
        self.get_logger().info(f'GPS simulado: lat={self.latitude}, lon={self.longitude}')
        
    def publish_gps(self):
        """Publica datos GPS fijos en el topic /gps/fix"""
        current_time = self.get_clock().now()
        
        gps_msg = NavSatFix()
        gps_msg.header.stamp = current_time.to_msg()
        gps_msg.header.frame_id = "gnss_frame"
        
        # Estatus del GPS
        gps_msg.status.status = 0  # STATUS_NO_FIX = -1, STATUS_FIX = 0, STATUS_SBAS_FIX = 1, STATUS_GBAS_FIX = 2
        gps_msg.status.service = 1  # SERVICE_GPS = 1
        
        # Coordenadas fijas
        gps_msg.latitude = self.latitude
        gps_msg.longitude = self.longitude
        gps_msg.altitude = self.altitude
        
        # Matriz de covarianza (simulando precisión moderada)
        # Diagonal: [lat_var, lon_var, alt_var]
        gps_msg.position_covariance[0] = 0.000001  # ~0.11 m en latitud
        gps_msg.position_covariance[4] = 0.000001  # ~0.11 m en longitud
        gps_msg.position_covariance[8] = 1.0       # 1 m en altitud
        gps_msg.position_covariance_type = 2  # COVARIANCE_TYPE_APPROXIMATED
        
        # Publicar mensaje
        self.gps_pub.publish(gps_msg)
        self.get_logger().info(f'GPS publicado: lat={self.latitude:.9f}, lon={self.longitude:.9f}', 
                               throttle_duration_sec=5.0)
        
    def publish_odom_tf(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time
        
        # Simular movimiento (0.1 m/s en X)
        self.x = 0.1 #* dt
        
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
        
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        odom_msg.twist.twist.linear.x = 0.1
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f'Odometría: x={self.x:.2f}m', throttle_duration_sec=2.0)
        
    def publish_imu(self):
        current_time = self.get_clock().now()
        t = time.time() - self.start_time
        
        # 3. PUBLICAR MENSAJE DE IMU
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Simular lecturas de IMU con pequeñas oscilaciones
        imu_msg.angular_velocity.x = 0.01 * math.sin(2.0 * t)
        imu_msg.angular_velocity.y = 0.01 * math.cos(1.5 * t)
        imu_msg.angular_velocity.z = 0.01
        
        imu_msg.linear_acceleration.x = 0.02 * math.sin(1.0 * t)
        imu_msg.linear_acceleration.y = 0.02 * math.cos(1.0 * t)
        imu_msg.linear_acceleration.z = 9.81  # Gravedad
        
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulateImuOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()