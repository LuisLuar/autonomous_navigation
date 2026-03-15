#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import torch
import sys
import os
import time  # <--- Faltaba esta importación arriba
from rclpy.qos import qos_profile_sensor_data
from torchvision import transforms

# 1. Configuración de Rutas
TWINLITE_PATH = '/home/raynel/UFDV2/HybridNets'
if TWINLITE_PATH not in sys.path:
    sys.path.append(TWINLITE_PATH)

# Cambiar el directorio de trabajo para que encuentre 'projects/bdd100k.yml'
os.chdir(TWINLITE_PATH)

# Imports específicos de HybridNets
from backbone import HybridNetsBackbone
from utils.utils import letterbox, Params
from utils.constants import *

class HybridNetsNode(Node):
    def __init__(self):
        super().__init__('hybridnets_node')
        
        self.compound_coef = 3
        self.project_name = 'bdd100k'
        # Ahora cargará correctamente desde la ruta relativa
        self.params = Params(f'projects/{self.project_name}.yml')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.get_logger().info(f"Cargando HybridNets en {self.device}...")
        
        self.model = HybridNetsBackbone(
            compound_coef=self.compound_coef, 
            num_classes=len(self.params.obj_list), 
            ratios=eval(self.params.anchors_ratios),
            scales=eval(self.params.anchors_scales), 
            seg_classes=len(self.params.seg_list),
            seg_mode=MULTICLASS_MODE
        )
        
        # 1. Mover modelo a GPU primero
        self.model.to(self.device)
        
        # 2. Cargar pesos
        weight_path = 'weights/hybridnets.pth'
        checkpoint = torch.load(weight_path, map_location=self.device)
        self.model.load_state_dict(checkpoint)

        # 3. Convertir a FP16 si hay CUDA disponible
        self.use_fp16 = torch.cuda.is_available()
        if self.use_fp16:
            self.model = self.model.half()
            self.get_logger().info("Inferencia en modo FP16 (Media Precisión) activa.")
            
        torch.backends.cudnn.benchmark = True
        self.model.eval()
        
        self.img_size = self.params.model['image_size']
        if isinstance(self.img_size, list): self.img_size = max(self.img_size)
        
        self.normalize = transforms.Normalize(mean=self.params.mean, std=self.params.std)
        self.transform = transforms.Compose([transforms.ToTensor(), self.normalize])
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("✅ HybridNets listo.")

    def callback(self, msg):
        start_time = time.time()
        np_arr = np.frombuffer(msg.data, np.uint8)
        ori_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if ori_img is None: return

        h0, w0 = ori_img.shape[:2]
        
        # 1. PRE-PROCESO FIEL AL ORIGINAL
        # Redimensionar manteniendo aspecto antes del letterbox
        r = self.img_size / max(h0, w0)
        input_img = cv2.resize(ori_img, (int(w0 * r), int(h0 * r)), interpolation=cv2.INTER_AREA)
        
        # Letterbox para llegar al tamaño cuadrado del modelo
        (input_img, _), ratio, pad = letterbox((input_img, None), self.img_size, auto=True, scaleup=False)
        
        # Transform y envío a GPU en FP16 si es posible
        input_tensor = self.transform(input_img).unsqueeze(0).to(self.device)
        if self.use_fp16:
            input_tensor = input_tensor.half()
        
        with torch.no_grad():
            features, regression, classification, anchors, seg = self.model(input_tensor)
            
            # 2. PROCESAR SEGMENTACIÓN CON CORRECCIÓN DE PADDING
            seg = torch.softmax(seg, dim=1)
            _, seg_mask = torch.max(seg, 1)
            seg_mask = seg_mask[0].cpu().numpy().astype(np.uint8)
            
            # Recuperar solo la zona útil (quitar las barras negras del letterbox)
            pad_w = int(pad[0])
            pad_h = int(pad[1])
            # Cortamos el padding: [y_inicio : y_fin, x_inicio : x_fin]
            seg_mask_no_pad = seg_mask[pad_h:seg_mask.shape[0]-pad_h, pad_w:seg_mask.shape[1]-pad_w]
            
            # Redimensionar la zona útil al tamaño original de la cámara
            seg_mask_resized = cv2.resize(seg_mask_no_pad, (w0, h0), interpolation=cv2.INTER_NEAREST)

        # 3. VISUALIZACIÓN (Optimización de color)
        # BDD100K: 1=Área (Azul), 2=Carril (Verde)
        seg_color_img = np.zeros((h0, w0, 3), dtype=np.uint8)
        seg_color_img[seg_mask_resized == 1] = [100, 0, 0] # Área manejable
        seg_color_img[seg_mask_resized == 2] = [0, 255, 0] # Carril

        combined_img = cv2.addWeighted(ori_img, 0.7, seg_color_img, 0.3, 0)
        
        # Mostrar FPS
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(combined_img, f"FPS: {fps:.2f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.imshow("HybridNets Preciso", combined_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HybridNetsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()