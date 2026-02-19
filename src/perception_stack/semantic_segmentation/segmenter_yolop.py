#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import torch
import numpy as np
from sensor_msgs.msg import CompressedImage  # Cambiado de Image a CompressedImage
from cv_bridge import CvBridge
import cv2  # Añadido para decodificar la imagen comprimida

from custom_interfaces.msg import SegmentationData


class YOLOPv2SegmenterNode(Node):

    def __init__(self):
        super().__init__('segmenter_yolop')

        # =========================
        # CONFIG
        # =========================
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.use_fp16 = self.device.type == 'cuda'

        self.input_h = 640
        self.input_w = 640

        self.lane_thresh = 0.5
        self.road_thresh = 0.4
        
        # Parámetro para controlar cada cuántas imágenes procesar
        self.process_every_n_frames = self.declare_parameter('process_every_n_frames', 1).value
        # Calcular FPS aproximado (asumiendo 30 FPS de entrada)
        self.target_fps = 30.0 / self.process_every_n_frames
        self.get_logger().info(f'Procesando cada {self.process_every_n_frames} imágenes (aprox. {self.target_fps:.1f} FPS)')

        self.bridge = CvBridge()

        if self.device.type == 'cuda':
            torch.backends.cudnn.benchmark = True

        # =========================
        # VARIABLES DE CONTROL
        # =========================
        self.is_processing = False
        self.frame_counter = 0
        self.latest_msg = None
        self.processing_future = None
        self.timer = self.create_timer(0.001, self.timer_callback)

        # =========================
        # LOAD MODEL (TorchScript)
        # =========================
        model_path = (
            '/home/raynel/autonomous_navigation/src/'
            'perception_stack/semantic_segmentation/pretrained/yolopv2.pt'
        )

        self.get_logger().info(f'Cargando YOLOPv2 en {self.device}')
        self.model = torch.jit.load(model_path, map_location=self.device)
        self.model.eval()

        if self.use_fp16:
            self.model.half()

        # =========================
        # ROS I/O - MODIFICADO PARA IMAGEN COMPRIMIDA
        # =========================
        # QoS similar al nodo C++ (KeepLast(1) con best_effort)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.sub = self.create_subscription(
            CompressedImage,  # Cambiado a CompressedImage
            '/image_raw/compressed',  # Cambiado el tópico
            self.image_callback,
            qos  # Usando QoS profile
        )

        self.pub = self.create_publisher(
            SegmentationData,
            '/segmentation/data',
            1
        )

        self.get_logger().info('Nodo inicializado para imágenes comprimidas')

    # =====================================================
    # POSTPROCESS (GPU, preciso)
    # =====================================================
    def process_mask(self, seg_out, threshold, out_h, out_w):
        if seg_out.shape[1] == 2:
            mask = torch.softmax(seg_out, dim=1)[:, 1]
        else:
            mask = torch.sigmoid(seg_out[:, 0])

        mask = torch.nn.functional.interpolate(
            mask.unsqueeze(1),
            size=(out_h, out_w),
            mode='bilinear',
            align_corners=False
        ).squeeze(1)

        binary = (mask > threshold).to(torch.uint8)
        return binary[0].cpu().numpy()

    # =====================================================
    # CALLBACK DE IMAGEN COMPRIMIDA - MODIFICADO
    # =====================================================
    def image_callback(self, msg: CompressedImage):  # Tipo cambiado a CompressedImage
        self.frame_counter += 1

        if self.frame_counter % self.process_every_n_frames != 0:
            return

        try:
            # Decodificar imagen comprimida a OpenCV
            # msg.data contiene los bytes de la imagen comprimida (ej. JPEG)
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().warn('No se pudo decodificar la imagen comprimida')
                return
                
            h, w = frame.shape[:2]

            # Método rápido de redimensionamiento (como el original)
            y = np.linspace(0, h - 1, self.input_h).astype(np.int32)
            x = np.linspace(0, w - 1, self.input_w).astype(np.int32)
            resized = frame[y[:, None], x]

            # BGR → RGB
            resized = resized[:, :, [2, 1, 0]]

            tensor = torch.from_numpy(resized).to(self.device)
            tensor = tensor.permute(2, 0, 1).unsqueeze(0).float() / 255.0

            if self.use_fp16:
                tensor = tensor.half()

            with torch.no_grad():
                _, da_seg, ll_seg = self.model(tensor)

            lane_mask = self.process_mask(ll_seg, self.lane_thresh, h, w)
            road_mask = self.process_mask(da_seg, self.road_thresh, h, w)

            combined = np.zeros((h, w), dtype=np.uint8)
            combined[road_mask > 0] = 1
            combined[lane_mask > 0] = 2

            out = SegmentationData()
            out.header = msg.header  # El header se mantiene igual
            out.height = h
            out.width = w
            out.mask_data = combined.flatten().tobytes()

            self.pub.publish(out)
            
            if self.frame_counter % 100 == 0:  # Log cada 100 frames
                self.get_logger().info(f'Procesados {self.frame_counter} frames')

        except Exception as e:
            self.get_logger().error(f'Error en callback: {str(e)}')

    def start_processing(self):
        """Inicia el procesamiento de la última imagen en segundo plano"""
        if self.latest_msg is not None and not self.is_processing:
            self.is_processing = True
            msg_to_process = self.latest_msg
            self.processing_future = self.process_image(msg_to_process)

    async def process_image(self, msg: CompressedImage):  # Tipo cambiado
        """Procesa una imagen comprimida de manera asíncrona"""
        try:
            # Decodificar imagen comprimida
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().warn('No se pudo decodificar la imagen comprimida en process_image')
                return
                
            h, w = frame.shape[:2]

            # -------- FAST INPUT RESIZE --------
            y = np.linspace(0, h - 1, self.input_h).astype(np.int32)
            x = np.linspace(0, w - 1, self.input_w).astype(np.int32)
            resized = frame[y[:, None], x]

            # BGR → RGB
            resized = resized[:, :, [2, 1, 0]]

            tensor = torch.from_numpy(resized).to(self.device)
            tensor = tensor.permute(2, 0, 1).unsqueeze(0).float() / 255.0

            if self.use_fp16:
                tensor = tensor.half()

            # -------- INFERENCE --------
            with torch.no_grad():
                _, da_seg, ll_seg = self.model(tensor)

            # -------- POSTPROCESS --------
            lane_mask = self.process_mask(ll_seg, self.lane_thresh, h, w)
            road_mask = self.process_mask(da_seg, self.road_thresh, h, w)

            combined = np.zeros((h, w), dtype=np.uint8)
            combined[road_mask > 0] = 1
            combined[lane_mask > 0] = 2

            out = SegmentationData()
            out.header = msg.header
            out.height = h
            out.width = w
            out.mask_data = combined.flatten().tobytes()

            self.pub.publish(out)
            
            self.get_logger().debug(f'Imagen comprimida procesada correctamente. Frame: {self.frame_counter}')

        except Exception as e:
            self.get_logger().error(f'Error en procesamiento: {str(e)}')
        finally:
            self.is_processing = False
            if self.latest_msg is not None and self.latest_msg != msg:
                self.get_logger().debug('Procesando siguiente imagen acumulada')
                self.start_processing()

    def timer_callback(self):
        """Timer para manejar el inicio de procesamiento cuando sea posible"""
        if self.latest_msg is not None and not self.is_processing:
            if self.frame_counter % self.process_every_n_frames == 0:
                self.start_processing()

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPv2SegmenterNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
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