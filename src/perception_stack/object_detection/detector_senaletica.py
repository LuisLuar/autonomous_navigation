#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # Cambio de Image a CompressedImage
from custom_interfaces.msg import DetectionArray, Detection
import numpy as np
import time
import os
import cv2 # Necesario para decodificar la compresión
from ultralytics import YOLO
import torch


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_senaletica')

        # ------------ Config Modelo -------------
        self.model_path = "/home/raynel/autonomous_navigation/src/perception_stack/object_detection/models/senaletica2.pt"
        
        self.conf_thres = float(os.getenv("YOLO_CONF", 0.79))
        self.iou_thres = float(os.getenv("YOLO_IOU", 0.45))
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.imgsz = 640
        
        # ------------ Config Tracking -------------
        self.use_tracking = True
        self.tracker_type = "bytetrack.yaml"
        self.next_id = 1 

        # ------------ Load model -------------
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().error(f"Error cargando modelo: {e}")
            raise

        self.frame_count = 0

        # QoS para imágenes: Best Effort es ideal para CompressedImage en redes WiFi/Serial
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Suscriptor cambiado a CompressedImage
        self.sub_image = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', # Asegúrate que este sea el tópico correcto
            self.callback_image, 
            qos_profile
        )
        
        self.pub_results = self.create_publisher(
            DetectionArray, '/detection/results_senaletica', 1
        )

        self.class_names = self.model.names if hasattr(self.model, 'names') else {}
        self.get_logger().info(f"YOLO iniciado en {self.device} con imágenes COMPRIMIDAS")

    def callback_image(self, msg: CompressedImage):
        t0 = time.time()

        # -------- DESCOMPRESIÓN MANUAL (Más rápida y robusta) --------
        try:
            # Convertir el buffer de ROS a un array de numpy
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decodificar la imagen (JPEG/PNG) a formato BGR para OpenCV
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                return
        except Exception as e:
            self.get_logger().error(f"Error decodificando imagen comprimida: {e}")
            return

        # -------- INFERENCE --------
        try:
            if self.use_tracking:
                results = self.model.track(
                    source=cv_image,
                    imgsz=self.imgsz,
                    conf=self.conf_thres,
                    iou=self.iou_thres,
                    device=self.device,
                    persist=True,
                    tracker=self.tracker_type,
                    verbose=False,
                    half=True if self.device == "cuda" else False
                )
            else:
                results = self.model.predict(
                    source=cv_image,
                    imgsz=self.imgsz,
                    conf=self.conf_thres,
                    iou=self.iou_thres,
                    device=self.device,
                    verbose=False,
                    half=True if self.device == "cuda" else False
                )
        except Exception as e:
            return

        inference_time = (time.time() - t0) * 1000.0

        # Crear mensaje ROS
        detection_array = DetectionArray()
        # Nota: Los mensajes CompressedImage también tienen header con stamp
        detection_array.header = msg.header
        detection_array.inference_time_ms = inference_time
        detection_array.backend = "PyTorch+Tracking" if self.use_tracking else "PyTorch"

        if results and len(results) > 0:
            r = results[0]
            if r.boxes is not None:
                boxes = r.boxes
                has_tracking_ids = hasattr(boxes, 'id') and boxes.id is not None
                
                for i in range(len(boxes)):
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    det = Detection()
                    det.class_id = int(boxes.cls[i])
                    det.class_name = self.class_names.get(det.class_id, f"class_{det.class_id}")
                    det.confidence = float(boxes.conf[i])
                    det.x1, det.y1, det.x2, det.y2 = map(int, xyxy)
                    det.center_x = float((xyxy[0] + xyxy[2]) / 2.0)
                    det.center_y = float((xyxy[1] + xyxy[3]) / 2.0)
                    
                    if self.use_tracking and has_tracking_ids:
                        det.track_id = int(boxes.id[i])
                    else:
                        det.track_id = self.next_id
                        self.next_id += 1
                    
                    detection_array.detections.append(det)

        self.pub_results.publish(detection_array)
        self.frame_count += 1

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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