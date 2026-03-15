#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import torch
import sys
import os
from rclpy.qos import qos_profile_sensor_data

# 1. Configuración de Rutas - Ajusta esto a tu carpeta
TWINLITE_PATH = '/home/raynel/UFDV2/TwinLiteNet'
sys.path.append(TWINLITE_PATH)

# Importamos la clase según lo que vimos en el archivo que pasaste
from model.TwinLite import TwinLiteNet

class TwinLiteNode(Node):
    def __init__(self):
        super().__init__('twinlite_node')
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # 2. Inicializar Modelo
        self.model = TwinLiteNet()
        
        # Ruta al archivo .pth que vimos en tu 'ls'
        weights_path = os.path.join(TWINLITE_PATH, 'pretrained/best.pth')
        
        try:
            checkpoint = torch.load(weights_path, map_location=self.device)
            
            # Si el checkpoint es un diccionario que contiene 'model' o 'state_dict'
            if isinstance(checkpoint, dict) and 'model' in checkpoint:
                state_dict = checkpoint['model']
            elif isinstance(checkpoint, dict) and 'state_dict' in checkpoint:
                state_dict = checkpoint['state_dict']
            else:
                state_dict = checkpoint

            # Eliminar prefijos 'module.' si fue entrenado con DataParallel
            new_state_dict = {}
            for k, v in state_dict.items():
                name = k[7:] if k.startswith('module.') else k
                new_state_dict[name] = v
            
            self.model.load_state_dict(new_state_dict)
            self.model.to(self.device).eval()
            self.get_logger().info(f"✅ TwinLiteNet cargado exitosamente en {self.device}")
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando pesos: {e}")
            # Imprime las llaves para ver qué hay dentro si falla
            if 'state_dict' in locals():
                self.get_logger().info(f"Llaves disponibles: {list(state_dict.keys())[:5]}")

        # 3. ROS Sub/Pub
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.callback,
            qos_profile_sensor_data
        )

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        h_orig, w_orig = frame.shape[:2]

        # 1. Redimensionar
        img = cv2.resize(frame, (640, 360))
        
        # 2. CAMBIO VITAL: Convertir BGR a RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # 3. CAMBIO VITAL: Solo normalizar a [0, 1] (Sin ImageNet)
        img = img.astype(np.float32) / 255.0
        
        img = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).to(self.device).float()

        with torch.no_grad():
            da_seg, ll_seg = self.model(img)
            
            # Si sigue sin verse nada, vamos a imprimir el máximo valor de probabilidad
            # para ver si el modelo está "vivo"
            # self.get_logger().info(f"Max Prob LL: {torch.softmax(ll_seg, dim=1)[0,1].max()}")

            da_probs = torch.softmax(da_seg, dim=1).squeeze(0)
            ll_probs = torch.softmax(ll_seg, dim=1).squeeze(0)
            
            # Umbrales estándar
            da_mask = (da_probs[1] > 0.5).cpu().numpy().astype(np.uint8)
            ll_mask = (ll_probs[1] > 0.5).cpu().numpy().astype(np.uint8)

        # --- POST-PROCESAMIENTO Y VISUALIZACIÓN ---
        # Redimensionar máscaras al tamaño de la cámara original
        da_mask = cv2.resize(da_mask, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)
        ll_mask = cv2.resize(ll_mask, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)

        # Crear overlay
        vis_img = frame.copy()
        
        # Color Verde para Drivable Area (da_mask == 1)
        vis_img[da_mask == 1] = [0, 255, 0] 
        
        # Color Rojo para Lane Lines (ll_mask == 1)
        vis_img[ll_mask == 1] = [0, 0, 255]

        # Mezclar con la original (transparencia)
        result = cv2.addWeighted(vis_img, 0.4, frame, 0.6, 0)

        cv2.imshow("TwinLiteNet - Tesis Raynel", result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TwinLiteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()