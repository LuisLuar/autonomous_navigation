#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import torch
import sys
import os
import json
from collections import OrderedDict
from rclpy.qos import qos_profile_sensor_data

# Configuración de Paths
SMOKE_ROOT = '/home/raynel/UFDV2/SMOKE'
sys.path.append(SMOKE_ROOT)

from smoke.modeling.detector import build_detection_model
from smoke.config import cfg

class ParamsList:
    def __init__(self, image_size):
        self.size = image_size
        self.extra_fields = {}

    def add_field(self, field, field_data):
        self.extra_fields[field] = field_data

    def get_field(self, field):
        return self.extra_fields[field]

    def to(self, device):
        for k, v in self.extra_fields.items():
            if hasattr(v, "to"):
                self.extra_fields[k] = v.to(device)
        return self

class SmokeNode(Node):
    def __init__(self):
        super().__init__('smoke_node')
        
        # 1. Cargar Calibración ORIGINAL (Sin escalar manualmente)
        calib_path = "/home/raynel/autonomous_navigation/src/params/camera_calibration.json"
        with open(calib_path, 'r') as f:
            calib_data = json.load(f)
        
        intr = calib_data['intrinsics']
        self.K = np.array([
            [intr['fx'], 0, intr['cx']],
            [0, intr['fy'], intr['cy']],
            [0, 0, 1]
        ], dtype=np.float32)

        # 2. Configurar SMOKE
        cfg.merge_from_file(os.path.join(SMOKE_ROOT, "configs/smoke_gn_vector.yaml"))
        cfg.TEST.DETECTIONS_THRESHOLD = 0.15 # Umbral balanceado
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = build_detection_model(cfg)
        self.model.to(self.device)

        # Cargar Pesos
        ckpt_path = os.path.join(SMOKE_ROOT, "model_final.pth")
        checkpoint = torch.load(ckpt_path, map_location=self.device)
        state_dict = checkpoint["model"]
        new_state_dict = OrderedDict()
        for k, v in state_dict.items():
            name = k[7:] if k.startswith('module.') else k
            new_state_dict[name] = v
        self.model.load_state_dict(new_state_dict)
        self.model.eval()

        # 3. ROS Infra
        self.subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.callback, qos_profile_sensor_data)
        
        self.get_logger().info("🚀 SMOKE 3D Nodo Activo - Tesis Raynel")

    def prepare_input(self, frame):
        # SMOKE espera 1280x384
        img = cv2.resize(frame, (1280, 384))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # IMPORTANTE: RGB
        
        # Normalización Estándar PyTorch (ImageNet/KITTI)
        img = img.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        img = (img - mean) / std
        
        img = img.transpose(2, 0, 1)
        return torch.from_numpy(img).to(self.device).float().unsqueeze(0)

    def draw_box_3d(self, image, dims, loc, roty):
        h, w, l = dims
        x, y, z = loc
        
        # Filtro de seguridad para evitar colapsos en el origen
        if z < 1.0: return image 

        R = np.array([[np.cos(roty), 0, np.sin(roty)],
                      [0, 1, 0],
                      [-np.sin(roty), 0, np.cos(roty)]])

        # 8 esquinas del cubo (KITTI format: y es hacia abajo, x derecha, z frente)
        x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
        y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
        z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
        
        corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
        corners_3d[0, :] += x
        corners_3d[1, :] += y
        corners_3d[2, :] += z

        # PROYECCIÓN USANDO K ORIGINAL
        pts_2d = np.dot(self.K, corners_3d)
        pts_2d = (pts_2d[:2, :] / pts_2d[2, :]).T
        pts_2d = pts_2d.astype(int)

        # Dibujar aristas
        for k in range(4):
            cv2.line(image, tuple(pts_2d[k]), tuple(pts_2d[(k+1)%4]), (0, 255, 0), 2)
            cv2.line(image, tuple(pts_2d[k+4]), tuple(pts_2d[((k+1)%4)+4]), (0, 255, 0), 2)
            cv2.line(image, tuple(pts_2d[k]), tuple(pts_2d[k+4]), (0, 255, 0), 2)
        return image

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        # 1. Preparar Input
        input_tensor = self.prepare_input(frame)
        
        # 2. Preparar trans_mat (Relación de aspecto Original -> 1280x384)
        # Esto le dice a SMOKE cómo mapear sus detecciones internas a tu imagen
        trans_mat = np.array([
            [1280 / frame.shape[1], 0, 0],
            [0, 384 / frame.shape[0], 0],
            [0, 0, 1]
        ], dtype=np.float32)

        target = ParamsList(image_size=(1280, 384))
        target.add_field("K", torch.as_tensor(self.K).to(self.device).float())
        target.add_field("trans_mat", torch.as_tensor(trans_mat).to(self.device).float())
        
        try:
            with torch.no_grad():
                output = self.model(input_tensor, targets=[target])
            
            vis_img = frame.copy()
            if output.shape[0] > 0:
                dets = output.cpu().numpy()
                for d in dets:
                    score = d[13]
                    if score < cfg.TEST.DETECTIONS_THRESHOLD: continue
                    
                    dims, loc, roty = d[6:9], d[9:12], d[12]
                    # Log para debug de tesis: si loc[2] (Z) es coherente
                    self.get_logger().info(f"Distancia detectada: {loc[2]:.2f}m")
                    vis_img = self.draw_box_3d(vis_img, dims, loc, roty)

            cv2.imshow("SMOKE 3D - Tesis Raynel", vis_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    node = SmokeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()