#!/usr/bin/env python3
"""
camera_reconnect.py - Nodo ROS2 robusto para reconexión de cámara ASUS / OpenNI2

Mejoras:
 - Evita reinicios encadenados
 - Timeouts realistas
 - No bloquea por stdout/stderr
 - Manejo más seguro de procesos
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
import time
import signal
import psutil
import os


class CameraReconnectNode(Node):

    def __init__(self):
        super().__init__('camera_reconnect_node')

        # ---------------- PARÁMETROS ----------------
        self.declare_parameter('timeout_seconds', 6.0)
        self.declare_parameter('restart_cooldown', 10.0)
        self.declare_parameter('monitor_topic', '/camera/rgb/image_raw')

        self.timeout = self.get_parameter('timeout_seconds').value
        self.restart_cooldown = self.get_parameter('restart_cooldown').value
        self.monitor_topic = self.get_parameter('monitor_topic').value

        # ---------------- ESTADO ----------------
        self.last_message_time = time.time()
        self.last_restart_time = 0.0
        self.camera_process = None
        self.camera_starting = False

        # ---------------- SUBSCRIPCIÓN ----------------
        self.subscription = self.create_subscription(
            Image,
            self.monitor_topic,
            self.image_callback,
            10
        )

        # ---------------- TIMER ----------------
        self.timer = self.create_timer(1.0, self.check_camera_status)

        # ---------------- START ----------------
        #self.get_logger().info('Iniciando cámara por primera vez...')
        self.start_camera()

    # ------------------------------------------------
    def image_callback(self, msg):
        self.last_message_time = time.time()

    # ------------------------------------------------
    def check_camera_status(self):
        if self.camera_starting:
            return

        elapsed = time.time() - self.last_message_time

        if elapsed > self.timeout:
            now = time.time()

            if now - self.last_restart_time < self.restart_cooldown:
                return

            #self.get_logger().warn(f'Sin imágenes por {elapsed:.1f}s → reiniciando cámara')

            self.last_restart_time = now
            self.restart_camera()

    # ------------------------------------------------
    def start_camera(self):
        if self.camera_process is not None:
            return

        self.camera_starting = True

        try:
            cmd = [
                'ros2', 'launch',
                'openni2_camera',
                'camera_with_cloud.launch.py'
            ]

            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

            self.last_message_time = time.time()
            #self.get_logger().info('Proceso de cámara lanzado')

        except Exception as e:
            #self.get_logger().error(f'Error al iniciar cámara: {e}')
            self.camera_process = None

        finally:
            # Tiempo para que DDS + driver arranquen
            time.sleep(3.0)
            self.camera_starting = False

    # ------------------------------------------------
    def restart_camera(self):
        self.stop_camera()

        # Tiempo realista para liberar USB / OpenNI
        time.sleep(4.0)

        self.start_camera()

    # ------------------------------------------------
    def stop_camera(self):
        if self.camera_process is None:
            return

        try:
            parent = psutil.Process(self.camera_process.pid)
            children = parent.children(recursive=True)

            for child in children:
                try:
                    child.terminate()
                except psutil.NoSuchProcess:
                    pass

            parent.terminate()

            gone, alive = psutil.wait_procs(
                children + [parent],
                timeout=5
            )

            for p in alive:
                try:
                    p.kill()
                except psutil.NoSuchProcess:
                    pass

        except Exception:
            try:
                os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
            except Exception:
                pass

        self.camera_process = None
        #self.get_logger().info('Proceso de cámara detenido')

    # ------------------------------------------------
    def destroy_node(self):
        self.stop_camera()
        super().destroy_node()


# ==================================================
def main(args=None):
    rclpy.init(args=args)
    node = CameraReconnectNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass 



if __name__ == '__main__':
    main()
