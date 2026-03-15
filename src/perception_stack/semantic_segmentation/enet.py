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

# 1. Configuración de Rutas
ENET_PATH = '/home/raynel/UFDV2/Lane_Detection_Using_ENet'
sys.path.append(ENET_PATH)
from enet_model import ENet 

class ENetNode(Node):
    def __init__(self):
        super().__init__('enet_node')
        # El repositorio original usa CPU en el ejemplo, pero usaremos CUDA si está disponible
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Arquitectura según el código original: ENet(2, 4)
        self.model = ENet(2, 4) 
        weights_path = os.path.join(ENET_PATH, 'ENET.pth')
        
        try:
            # Carga de pesos idéntica al repo
            self.model.load_state_dict(torch.load(weights_path, map_location=self.device))
            self.model.to(self.device).eval()
            self.get_logger().info(f"✅ ENet cargado con éxito en {self.device}")
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando pesos: {e}")

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

        # --- PRE-PROCESAMIENTO ORIGINAL ---
        # 1. Resize a (512, 256)
        input_image = cv2.resize(frame, (512, 256))
        
        # 2. Convertir a Gris
        input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        
        # 3. Añadir dimensión de canal [H, W, 1] e igualar tipo float (SIN DIVIDIR POR 255)
        input_tensor_np = input_image[..., None]
        
        # 4. Convertir a Tensor y permutar a [C, H, W]
        input_tensor = torch.from_numpy(input_tensor_np).float().permute(2, 0, 1)
        
        # 5. Añadir dimensión de Batch [1, C, H, W] y enviar a dispositivo
        input_batch = input_tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            # Inferencia: el modelo devuelve binary e instance logits
            binary_logits, _ = self.model(input_batch)
            
            # Post-proceso original: argmax en la dimensión de canales (dim=1)
            binary_seg = torch.argmax(binary_logits, dim=1).squeeze().cpu().numpy()

        # --- VISUALIZACIÓN ORIGINAL ---
        # Crear máscara de escala de grises (0 o 255)
        binary_seg_grayscale = np.zeros_like(input_image)
        binary_seg_grayscale[binary_seg == 1] = 255

        # Overlay siguiendo el peso del repositorio (0.7 original, 0.3 máscara)
        output_image = cv2.addWeighted(input_image, 0.7, binary_seg_grayscale, 0.3, 0)

        # Mostrar resultados
        cv2.imshow("Original Input Image (ENet)", input_image)
        cv2.imshow("Lane Overlay (Tesis Raynel)", output_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ENetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()