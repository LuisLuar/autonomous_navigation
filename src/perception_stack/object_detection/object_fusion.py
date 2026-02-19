#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
import json
import os
from collections import deque, defaultdict

# Mensajes ROS
from custom_interfaces.msg import DetectionArray, ObjectInfoArray, ObjectInfo
from nav_msgs.msg import Odometry

class ObjectFusionNode(Node):
    def __init__(self):
        super().__init__('object_fusion')
        
        # ============ CONFIGURACIÓN DE REDES ============
        self.AIRBORNE_CLASSES = [0, 3] # Señales en el aire (signage)
        self.SIGNAGE_ID_OFFSET = 10000 
        
        # ============ CARGAR CALIBRACIÓN DESDE JSON ============
        # Ruta idéntica a tu nodo C++
        calib_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/camera_calibration.json"
        self.load_calibration(calib_path)
        
        # ============ VARIABLES DE ESTADO ============
        self.robot_speed = 0.0
        self.tracks_history = defaultdict(lambda: {
            'positions': deque(maxlen=20),
            'timestamps': deque(maxlen=20),
            'velocities': deque(maxlen=10),
            'last_seen': time.time(),
            'class_name': '',
            'class_id': 0,
            'is_airborne': False,
            'bbox': [0, 0, 0, 0, 0, 0] # x1, y1, x2, y2, cx, cy
        })

        # ============ ROS I/O ============
        self.create_subscription(DetectionArray, '/detection/results', self.cb_objects, 10)
        self.create_subscription(DetectionArray, '/detection/results_senaletica', self.cb_senaletica, 10)
        self.create_subscription(Odometry, '/odometry/local', self.cb_odom, 10)
        
        self.pub_fused = self.create_publisher(ObjectInfoArray, '/objects/fused_info', 10)
        
        self.create_timer(0.1, self.publish_fused_data)
        self.get_logger().info(f"Fusion Node listo. Pitch: {math.degrees(self.pitch_):.2f}°, H: {self.cam_h_}m")

    def load_calibration(self, path):
        try:
            with open(path, 'r') as f:
                calib = json.load(f)
            
            # Parámetros físicos (Cámara -> Robot)
            self.cam_h_ = float(calib['camera_height'])
            self.cam_x_ = float(calib['camera_x'])
            self.cam_y_ = float(calib['camera_y'])
            
            # Intrínsecos y Orientación
            self.pitch_ = float(calib['camera_pitch'])
            self.fx_ = float(calib['intrinsics']['fx'])
            self.fy_ = float(calib['intrinsics']['fy'])
            self.cx_ = float(calib['intrinsics']['cx'])
            self.cy_ = float(calib['intrinsics']['cy'])

            # Pre-calcular matriz de rotación de pitch (Igual que en C++)
            cp = math.cos(self.pitch_)
            sp = math.sin(self.pitch_)
            # Rotación en X: inclinación hacia abajo
            self.rotation_matrix = np.array([
                [1,   0,    0],
                [0,  cp,   sp],
                [0, -sp,   cp]
            ])
            
        except Exception as e:
            self.get_logger().error(f"Fallo al cargar JSON en {path}: {e}")
            # Valores de emergencia por si el archivo no existe
            self.cam_h_, self.cam_x_, self.cam_y_ = 1.2, -0.216, 0.0
            self.pitch_, self.fx_, self.fy = -0.386, 439.0, 439.0
            self.cx_, self.cy_ = 320.0, 180.0

    def cb_objects(self, msg): self.process_detections(msg, False)
    def cb_senaletica(self, msg): self.process_detections(msg, True)

    def process_detections(self, msg, is_senaletica):
        for det in msg.detections:
            if det.track_id == 0: continue
            
            t_id = det.track_id + (self.SIGNAGE_ID_OFFSET if is_senaletica else 0)
            is_air = is_senaletica and (det.class_id in self.AIRBORNE_CLASSES)
            
            track = self.tracks_history[t_id]
            track.update({
                'last_seen': time.time(),
                'class_name': det.class_name,
                'class_id': det.class_id,
                'is_airborne': is_air,
                'confidence': det.confidence,
                'bbox': [det.x1, det.y1, det.x2, det.y2, det.center_x, det.center_y]
            })

            # --- IPM (Cálculo idéntico al C++) ---
            if not is_air:
                # 1. Proyectar pixel a rayo
                x_ray = (det.center_x - self.cx_) / self.fx_
                y_ray = -(det.y2 - self.cy_) / self.fy_ # Inversión de signo clave
                ray_cam = np.array([x_ray, y_ray, 1.0])

                # 2. Rotar el rayo
                ray_robot = self.rotation_matrix @ ray_cam

                # 3. Intersección con suelo
                if ray_robot[1] >= 0: continue 

                t = self.cam_h_ / -ray_robot[1]
                Xc = ray_robot[0] * t
                Zc = ray_robot[2] * t

                # 4. Transformar a base_footprint
                final_x = Zc + self.cam_x_
                final_y = -Xc + self.cam_y_ 

                track['positions'].append((final_x, final_y))
                track['timestamps'].append(time.time())
                self.update_velocity(t_id)

    def update_velocity(self, t_id):
        t = self.tracks_history[t_id]
        if len(t['positions']) < 2: return
        
        dt = t['timestamps'][-1] - t['timestamps'][-2]
        if dt <= 0: return
        
        # Velocidad relativa en X (eje de avance del robot)
        rel_v = (t['positions'][-1][0] - t['positions'][-2][0]) / dt
        t['velocities'].append(rel_v)

    def publish_fused_data(self):
        now = time.time()
        msg = ObjectInfoArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        
        # Limpiar tracks (2 segundos de timeout)
        to_del = [tid for tid, info in self.tracks_history.items() if now - info['last_seen'] > 2.0]
        for tid in to_del: del self.tracks_history[tid]

        for tid, info in self.tracks_history.items():
            obj = ObjectInfo()
            obj.track_id = int(tid)
            obj.class_name = str(info['class_name'])
            obj.confidence = float(info['confidence'])
            
            bbox = info['bbox']
            obj.x1, obj.y1, obj.x2, obj.y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
            obj.center_x, obj.center_y = float(bbox[4]), float(bbox[5])

            if info['positions'] and not info['is_airborne']:
                dist_x, lateral_y = info['positions'][-1]
                obj.distance = float(dist_x)
                obj.lateral_offset = float(lateral_y)
                obj.distance_valid = True
                
                if info['velocities']:
                    avg_rel_v = sum(info['velocities']) / len(info['velocities'])
                    obj.relative_speed = float(avg_rel_v)
                    obj.is_moving_toward = bool(avg_rel_v < -0.1)
                    
                    if obj.is_moving_toward and abs(avg_rel_v) > 0.01:
                        obj.time_to_collision = float(dist_x / abs(avg_rel_v))
                    else:
                        obj.time_to_collision = 999.0
            else:
                obj.distance_valid = False
                obj.distance, obj.lateral_offset = 0.0, 0.0

            msg.objects.append(obj)
            
        self.pub_fused.publish(msg)

    def cb_odom(self, msg):
        self.robot_speed = msg.twist.twist.linear.x

def main():
    rclpy.init()
    node = ObjectFusionNode()
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