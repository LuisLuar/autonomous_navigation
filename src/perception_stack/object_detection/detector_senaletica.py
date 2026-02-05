#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.msg import DetectionArray, Detection
import numpy as np
import time
import os
from ultralytics import YOLO
import torch


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_senaleticar')

        # ------------ Config Modelo -------------
        self.model_path = "/home/raynel/autonomous_navigation/src/perception_stack/object_detection/models/senaletica.pt"
        
        self.conf_thres = float(os.getenv("YOLO_CONF", 0.5))
        self.iou_thres = float(os.getenv("YOLO_IOU", 0.45))
        self.device = "cuda" if self.check_gpu() else "cpu"
        self.imgsz = 640
        
        # ------------ Config Tracking -------------
        self.use_tracking = True
        self.tracker_type = "bytetrack.yaml"
        
        # Cache para tracks vistos
        self.seen_tracks = set()
        self.next_id = 1  # Para tracks sin ID (fallback)

        # ------------ Load model UNA VEZ -------------
        try:
            self.model = YOLO(self.model_path)
            self.backend = "PyTorch"
        except Exception as e:
            raise

        self.bridge = CvBridge()
        self.frame_count = 0

        # ROS I/O
        self.sub_image = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.callback_image, 1
        )
        self.pub_results = self.create_publisher(
            DetectionArray, '/detection/results_senaletica', 1
        )

        # Nombres de clases
        self.class_names = self.model.names if hasattr(self.model, 'names') else {}


    def check_gpu(self):
        """Verificar si hay GPU disponible"""
        try:
            if torch.cuda.is_available():
                #self.get_logger().info(f"GPU detectada: {torch.cuda.get_device_name(0)}")
                return True
        except:
            pass
        #self.get_logger().warn("GPU no disponible, usando CPU")
        return False

    def callback_image(self, msg: Image):
        t0 = time.time()

        # Convertir imagen
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            #self.get_logger().warn(f"Error en CV bridge: {e}")
            return

        # -------- INFERENCE con TRACKING --------
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
                    max_det=100,
                    half=True if self.device == "cuda" else False,
                    agnostic_nms=False,
                    classes=None
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
            #self.get_logger().warn(f"Error en inferencia: {e}", throttle_duration_sec=5.0)
            return

        inference_time = (time.time() - t0) * 1000.0

        # Crear mensaje ROS
        detection_array = DetectionArray()
        detection_array.header = msg.header
        detection_array.inference_time_ms = inference_time
        detection_array.backend = f"PyTorch+Tracking" if self.use_tracking else "PyTorch"

        # Verificar si hay resultados
        if not results or len(results) == 0:
            self.pub_results.publish(detection_array)
            return

        r = results[0]
        
        # Verificar si hay detecciones
        if r.boxes is None or len(r.boxes) == 0:
            self.pub_results.publish(detection_array)
            return

        # Procesar cada detección
        boxes = r.boxes
        has_tracking_ids = hasattr(boxes, 'id') and boxes.id is not None
        
        for i in range(len(boxes)):
            # Obtener datos del bounding box
            xyxy = boxes.xyxy[i].cpu().numpy()
            conf = float(boxes.conf[i])
            cls = int(boxes.cls[i])
            
            # Calcular centro
            x_center = float((xyxy[0] + xyxy[2]) / 2.0)
            y_center = float((xyxy[1] + xyxy[3]) / 2.0)
            
            # Crear mensaje de detección (SIMPLIFICADO)
            det = Detection()
            det.class_id = cls
            det.class_name = self.class_names.get(cls, f"class_{cls}")
            det.confidence = conf
            det.x1, det.y1, det.x2, det.y2 = map(int, xyxy)
            det.center_x = x_center
            det.center_y = y_center
            
            # SOLO track_id (los otros campos se eliminan)
            if self.use_tracking and has_tracking_ids:
                try:
                    det.track_id = int(boxes.id[i].cpu().numpy())
                except:
                    det.track_id = self.next_id
                    self.next_id += 1
            else:
                det.track_id = self.next_id
                self.next_id += 1
            
            detection_array.detections.append(det)

        # Publicar resultados
        self.pub_results.publish(detection_array)
        
        # Log cada 30 frames
        if self.frame_count % 30 == 0:
            num_objects = len(detection_array.detections)
            num_tracks = len(set(d.track_id for d in detection_array.detections))
            
            """self.get_logger().info(
                f"Frame {self.frame_count}: "
                f"{num_objects} objetos, "
                f"{num_tracks} tracks únicos, "
                f"Tiempo: {inference_time:.1f}ms"
            )"""
        
        self.frame_count += 1
        
        # Limpiar tracks viejos periódicamente
        if self.frame_count % 300 == 0:
            if len(self.seen_tracks) > 1000:
                self.seen_tracks.clear()

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