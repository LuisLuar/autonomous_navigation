#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import time
from custom_interfaces.msg import SegmentationData

# ---------------------------------------------------------
# Clase ligera para manejar engine TensorRT (buffers reutilizables)
# ---------------------------------------------------------
class TensorRTInference:
    def __init__(self, engine_path: str):
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.engine = self._load_engine(engine_path)
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        # Preparar inputs/outputs (usando nombres/shape/dtypes desde engine)
        self.inputs = []
        self.outputs = []
        self.bindings = []

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))  # e.g. (1,3,512,512)
            dtype = self.engine.get_tensor_dtype(name)         # trt.DataType.FLOAT
            nptype = trt.nptype(dtype)
            size = int(trt.volume(shape))

            # reservar memoria dispositivo para cada tensor
            dev_mem = cuda.mem_alloc(size * np.dtype(nptype).itemsize)
            self.bindings.append(int(dev_mem))

            # distinguir input / output por su modo
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                # no crear host copy para input; pedimos que el usuario nos pase ndarray contiguo
                self.inputs.append({
                    'name': name, 'shape': shape, 'dtype': dtype, 'nptype': nptype, 'device': dev_mem, 'size': size
                })
            else:
                # host page-locked para salida (rápido copia D2H)
                host_mem = cuda.pagelocked_empty(size, nptype)
                self.outputs.append({
                    'name': name, 'shape': shape, 'dtype': dtype, 'nptype': nptype, 'device': dev_mem, 'host': host_mem, 'size': size
                })

        # fijar direcciones a context
        if len(self.inputs) > 0:
            self.context.set_input_shape(self.inputs[0]['name'], self.inputs[0]['shape'])
            self.context.set_tensor_address(self.inputs[0]['name'], int(self.inputs[0]['device']))
        for out in self.outputs:
            self.context.set_tensor_address(out['name'], int(out['device']))

    def _load_engine(self, path):
        with open(path, 'rb') as f, trt.Runtime(self.logger) as runtime:
            return runtime.deserialize_cuda_engine(f.read())

    def infer(self, input_array: np.ndarray):
        # input_array debe estar en formato (1, C, H, W) y dtype correcto (np.float32) y contiguous
        arr = np.ascontiguousarray(input_array)
        # copiar a device async
        cuda.memcpy_htod_async(self.inputs[0]['device'], arr.ravel(), self.stream)
        # Ejecutar (async)
        self.context.execute_async_v3(stream_handle=self.stream.handle)
        # copiar salida a host async (usamos el primer output)
        out = self.outputs[0]
        cuda.memcpy_dtoh_async(out['host'], out['device'], self.stream)
        # sincronizar stream (esperar solo la inferencia actual)
        self.stream.synchronize()
        # devolver vista numpy del host pinned memory (ya es numpy)
        return out['host']

# ---------------------------------------------------------
# Nodo ROS2 que publica SÓLO /segmentation/overlay (BGR)
# ---------------------------------------------------------
class LaneSegmenterNode(Node):
    def __init__(self):
        super().__init__('segmenter')

        self.bridge = CvBridge()

        # Tamaño de entrada (debe coincidir con el engine)
        self.input_size = (512, 512)   # (w,h)
        self.model_classes = 19        # ajustar si tu modelo tiene distinto nclass

        # Cargar engine TensorRT (ruta relativa u absoluta)
        engine_path = os.getenv('SEG_ENGINE', '/home/raynel/autonomous_navigation/src/perception_stack/semantic_segmentation/pretrained/bisenetv1.engine')
        try:
            self.trt = TensorRTInference(engine_path)
            #self.get_logger().info("TensorRT engine cargado correctamente")
        except Exception as e:
            #self.get_logger().error(f"Fallo cargando engine TensorRT: {e}")
            raise

        # Suscripción: cola pequeña para evitar acumulación (1)
        self.sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.callback_image, 1)

        # Publicador: datos de segmentacióñ
        self.pub_seg_data = self.create_publisher(SegmentationData, '/segmentation/data', 1)

        # Pre-allocaciones para evitar nuevas asignaciones:
        # tensor de entrada (1,3,H,W) float32 contiguous
        ch = 3
        h, w = self.input_size[1], self.input_size[0]
        self.input_tensor = np.empty((1, ch, h, w), dtype=np.float32)
        # buffer para color mask (H,W,3) se crea por reshape posterior sin recrear demasiado
        self.frame_count = 0
        self.last_time = self.get_clock().now()
        self.last_frame_time = self.get_clock().now()

    def preprocess(self, img_bgr):
        # Resize -> RGB -> normalize in-place into self.input_tensor
        # cv2.resize devuelve nuevo array; mínimo coste aquí
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(img_rgb, self.input_size, interpolation=cv2.INTER_LINEAR)
        # colocar en formato CHW y normalizar (0..1)
        self.input_tensor[0] = resized.astype(np.float32).transpose(2,0,1) / 255.0
        return self.input_tensor

    def numpy_softmax(self, x):
        # x shape (N, C, H, W) or (C, H, W)
        # aplicar softmax por canal
        # estabilidad numérica
        x = x - np.max(x, axis=1, keepdims=True)
        exp = np.exp(x)
        return exp / np.sum(exp, axis=1, keepdims=True)
    
    def publish_segmentation_data(self, mask, header):
        #self.get_logger().info(f'PUBLICANDO')
        try:
            msg = SegmentationData()
            msg.header = header
            msg.height = mask.shape[0]
            msg.width = mask.shape[1]
            msg.mask_data = mask.flatten().tobytes()  # o comprimir
            #msg.probabilities = probabilities.flatten()  # si quieres enviar
            self.pub_seg_data.publish(msg)
        except Exception as e:
            #self.get_logger().error(f'Error publicando SegmentationData: {e}')
            pass


    def callback_image(self, msg: Image):
        try:
            # 1) ROS -> BGR OpenCV (evitar conversion adicional si se publica BGR)
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            orig_h, orig_w = frame_bgr.shape[:2]

            # 2) Preprocess (resize + CHW + normalize)
            inp = self.preprocess(frame_bgr)  # (1,3,512,512) float32 contiguous

            # 3) Inference (TensorRT)
            t_inf_s = time.time()
            out_host = self.trt.infer(inp)    # devuelve pagelocked numpy 1D
            t_inf_e = time.time()
            inf_ms = (t_inf_e - t_inf_s) * 1000.0

            # 4) Postprocess: forma (C,H,W) y softmax + argmax (TODO: ajusta C si tu modelo distinto)
            # Usamos shapes desde el engine si es necesario; asumimos salida: (1, C, H, W)
            out_shape = self.trt.outputs[0]['shape']  # ejemplo (1,19,512,512)
            C = int(out_shape[1])
            H = int(out_shape[2])
            W = int(out_shape[3])
            out_arr = out_host.reshape((1, C, H, W))

            # Softmax + argmax (vectorizado, en numpy)
            probs = self.numpy_softmax(out_arr)
            mask = np.argmax(probs[0], axis=0).astype(np.uint8)   # (H,W)

            # 5) Resize mask al tamaño original
            mask_big = cv2.resize(mask, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)  # mono
            
            # 6) Publicar datos de segmentación
            self.publish_segmentation_data(mask_big, msg.header)

        except Exception as e:
            #self.get_logger().error(f'Error en callback segmentation: {e}')
            pass
    
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneSegmenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('Segmenter detenido por usuario')
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()


"""import rclpy
from rclpy.node import Node

import torch
import torch.nn.functional as F

import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# IMPORTA TU MODELO
from semantic_segmentation.models.bisenetv1 import BiSeNetV1
from semantic_segmentation.models.bisenetv2 import BiSeNetV2
from semantic_segmentation.models.segformer import SegFormer


class LaneSegmenterNode(Node):
    def __init__(self):
        super().__init__("lane_segmenter")

        self.bridge = CvBridge()

        # -----------------------------
        # CONFIGURACIONES
        # -----------------------------
        self.model_type = "bisenetv1"     # Cambia: "bisenetv2" "segformer"
        self.num_classes = 19              # background, drivable, sidewalk
        self.input_size = (512, 512)      # resolución recomendada
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Configuración de transparencia para el overlay
        self.alpha = 0.6  # Transparencia de la máscara (0.0 = totalmente transparente, 1.0 = totalmente opaco)

        self.get_logger().info(f"Using device: {self.device}")

        # -----------------------------
        # CARGAR EL MODELO
        # -----------------------------
        if self.model_type == "bisenetv1":
            self.model = BiSeNetV1(self.num_classes)
            weight_path = "src/perception_stack/semantic_segmentation/pretrained/model_final_v1_city_new.pth"
            state = torch.load(weight_path, map_location=self.device)
            self.model.load_state_dict(state, strict=False)

        elif self.model_type == "bisenetv2":
            self.model = BiSeNetV2(self.num_classes)
            weight_path = "src/perception_stack/semantic_segmentation/pretrained/model_final_v2_city.pth"
            state = torch.load(weight_path, map_location=self.device)
            self.model.load_state_dict(state, strict=False)

        elif self.model_type == "segformer":
            checkpoint = "nvidia/segformer-b0-finetuned-cityscapes-1024-1024"
            self.model = SegFormer(self.num_classes, checkpoint)
        else:
            raise ValueError("Unknown model type")

        self.model.to(self.device)
        self.model.eval()

        self.get_logger().info(f"{self.model_type} loaded successfully.")

        # -----------------------------
        # SUBSCRIBER
        # -----------------------------
        self.sub_rgb = self.create_subscription(
            Image,
            "/camera/rgb/image_raw",
            self.callback_image,
            10
        )

        # -----------------------------
        # PUBLISHERS
        # -----------------------------
        self.pub_mask = self.create_publisher(Image, "/segmentation/mask", 10)
        self.pub_color = self.create_publisher(Image, "/segmentation/colored", 10)
        # NUEVO PUBLISHER PARA EL OVERLAY
        self.pub_overlay = self.create_publisher(Image, "/segmentation/overlay", 10)

    def safe_mask_processing(self, mask):

        Asegura que la máscara solo contenga valores entre 0-18
        y maneja valores fuera de rango

        # Forzar valores al rango 0-18
        mask = np.clip(mask, 0, 18)
        return mask.astype(np.uint8)
    
    def create_overlay_image(self, original_image, mask, color_mask):

        Crea una imagen de overlay donde solo las clases detectadas 
        se superponen semitransparentemente sobre la imagen original
        
        Args:
            original_image: Imagen original BGR
            mask: Máscara de segmentación (H, W)
            color_mask: Máscara de colores RGB
        
        Returns:
            Imagen con overlay en formato BGR

        # Convertir máscara de colores a BGR (para OpenCV)
        color_mask_bgr = cv2.cvtColor(color_mask, cv2.COLOR_RGB2BGR)
        
        # Crear máscara booleana de píxeles que NO son fondo (clase 0)
        non_background_mask = mask != 0
        
        # Si no hay ninguna clase detectada, retornar imagen original
        if not np.any(non_background_mask):
            return original_image
        
        # Crear una copia de la imagen original
        overlay_image = original_image.copy()
        
        # Aplicar la máscara de colores solo donde hay detecciones (no fondo)
        # Usamos alpha blending para la transparencia
        overlay_image[non_background_mask] = (
            self.alpha * color_mask_bgr[non_background_mask] + 
            (1 - self.alpha) * original_image[non_background_mask]
        ).astype(np.uint8)
        
        return overlay_image
    
    # ----------------------------------------------------------------------
    # CALLBACK
    # ----------------------------------------------------------------------
    def callback_image(self, msg):
        # Convert ROS -> CV2
        cv_image_original = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image_rgb = cv2.cvtColor(cv_image_original, cv2.COLOR_BGR2RGB)

        # Preprocess
        img_resized = cv2.resize(cv_image_rgb, self.input_size, interpolation=cv2.INTER_LINEAR)
        img_tensor = torch.from_numpy(img_resized).float().permute(2, 0, 1) / 255.0
        if self.model_type == "segformer":
            img_tensor = (img_tensor - 0.5) / 0.5

        img_tensor = img_tensor.unsqueeze(0).to(self.device)

        # Inferencia
        with torch.no_grad():
            out = self.model(img_tensor)[0]  # output logits
            out = F.softmax(out, dim=1)
            mask = out.argmax(dim=1).squeeze().cpu().numpy()

        # ✅ Procesamiento seguro de la máscara
        mask = self.safe_mask_processing(mask)
        
        # Generar máscara color
        color_mask = self.get_colored_mask(mask)

        # Resize back to original camera size
        mask_big = cv2.resize(mask.astype(np.uint8), 
                             (cv_image_original.shape[1], cv_image_original.shape[0]), 
                             interpolation=cv2.INTER_NEAREST)
        color_big = cv2.resize(color_mask, 
                              (cv_image_original.shape[1], cv_image_original.shape[0]), 
                              interpolation=cv2.INTER_NEAREST)

        # 🔥 NUEVO: Crear imagen de overlay
        overlay_image = self.create_overlay_image(cv_image_original, mask_big, color_big)

        # Publish mask
        ros_mask = self.bridge.cv2_to_imgmsg(mask_big, encoding="mono8")
        self.pub_mask.publish(ros_mask)

        # Publish color visualization
        ros_color = self.bridge.cv2_to_imgmsg(color_big, encoding="rgb8")
        self.pub_color.publish(ros_color)

        # 🔥 NUEVO: Publish overlay image
        ros_overlay = self.bridge.cv2_to_imgmsg(overlay_image, encoding="bgr8")
        self.pub_overlay.publish(ros_overlay)

        # Log opcional para debugging
        unique_classes = np.unique(mask_big)
        if len(unique_classes) > 1 or (len(unique_classes) == 1 and unique_classes[0] != 0):
            self.get_logger().info(f"Clases detectadas: {unique_classes}", throttle_duration_sec=2.0)

    # ----------------------------------------------------------------------
    # COLOR MAP
    # ----------------------------------------------------------------------
    def get_colored_mask(self, mask):
        # Official Cityscapes colormap (19 classes)
        colors = np.array([
            [128, 64,128],   # 0 carretera
            [244, 35,232],   # 1 acera
            [ 70, 70, 70],   # 2 edificio
            [102,102,156],   # 3 muro
            [190,153,153],   # 4 valla
            [153,153,153],   # 5 poste
            [250,170, 30],   # 6 semaforo
            [220,220,  0],   # 7 señal de trafico
            [107,142, 35],   # 8 vegetación
            [152,251,152],   # 9 terreno
            [ 70,130,180],   # 10 cielo
            [220, 20, 60],   # 11 persona
            [255,  0,  0],   # 12 ciclista
            [  0,  0,142],   # 13 coche
            [  0,  0, 70],   # 14 camion
            [  0, 60,100],   # 15 autobús
            [  0, 80,100],   # 16 tren
            [  0,  0,230],   # 17 motocicleta
            [119, 11, 32],   # 18 bicicleta
        ], dtype=np.uint8)

        h, w = mask.shape
        color_mask = colors[mask]
        return color_mask



def main(args=None):
    rclpy.init(args=args)
    node = LaneSegmenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Supervisor de cámara apagado por usuario")
        pass
    except Exception as e:
        node.get_logger().error(f"Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown


if __name__ == "__main__":
    main()
"""