#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from custom_interfaces.msg import DetectionArray, Detection

import numpy as np
import cv2
import json
import time
import os

# Ultralytics
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # ------------ Config para TensorRT engine -------------
        self.model_path = "/home/raynel/autonomous_navigation/src/perception_stack/object_detection/models/yolo11m.engine"

        # Parámetros
        self.conf_thres = float(os.getenv("YOLO_CONF", 0.25))
        self.iou_thres = float(os.getenv("YOLO_IOU", 0.45))
        
        # Para TensorRT engine
        self.device = "0"  # GPU device ID para TensorRT

        self.imgsz = 640

        #self.get_logger().info(f"Cargando modelo TensorRT engine: {self.model_path}")
        
        try:
            # Cargar el modelo TensorRT engine
            self.model = YOLO(self.model_path, task='detect')
            #self.get_logger().info("Modelo TensorRT engine cargado correctamente")
        except Exception as e:
            #self.get_logger().error(f"Error cargando TensorRT engine: {e}")
            # Fallback a ONNX
            try:
                onnx_path = self.model_path.replace('.engine', '.onnx')
                #self.get_logger().info(f"Fallback a ONNX: {onnx_path}")
                self.model = YOLO(onnx_path, task='detect')
                self.device = "cuda"  # Cambiar a cuda para ONNX
            except Exception as e2:
                #self.get_logger().error(f"Fallback falló: {e2}")
                raise

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber 
        self.sub_image = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.callback_image, 1
        )

        # Publishers
        self.pub_annotated = self.create_publisher(Image, '/detection/annotated_image', 1)
        self.pub_results = self.create_publisher(DetectionArray, '/detection/results', 1)

        # Class names
        self.class_names = {}
        try:
            if hasattr(self.model, 'names'):
                self.class_names = self.model.names
        except Exception:
            self.class_names = {}

        self.start_time = time.time()
        self.count = 0
        self.inference_times = []
        self.backend = "TensorRT" if self.model_path.endswith('.engine') else "ONNX"

    def callback_image(self, msg: Image):
        inference_start = time.time()
        
        # Convert ROS Image -> OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            #self.get_logger().error(f"cv_bridge error: {e}")
            return

        # Run inference
        try:
            results = self.model(
                cv_image,
                imgsz=self.imgsz,
                conf=self.conf_thres,
                iou=self.iou_thres,
                device=self.device,
                verbose=False
            )
        except Exception as e:
            #self.get_logger().error(f"Inference error: {e}")
            return

        inference_time = time.time() - inference_start
        self.inference_times.append(inference_time)

        # Crear mensaje DetectionArray (CAMBIOS AQUÍ)
        detection_array = DetectionArray()
        detection_array.header = msg.header
        detection_array.inference_time_ms = inference_time*1000
        detection_array.backend = self.backend

        # Procesar resultados
        detections = []
        if len(results) > 0:
            r = results[0]
            if hasattr(r, 'boxes') and r.boxes is not None:
                for box in r.boxes:
                    try:
                        xyxy = box.xyxy.cpu().numpy().tolist()[0]
                        conf = float(box.conf.cpu().numpy().item())
                        cls = int(box.cls.cpu().numpy().item())
                    except Exception:
                        xyxy = box.xyxy.tolist()[0]
                        conf = float(box.conf.tolist()[0])
                        cls = int(box.cls.tolist()[0])

                    x1, y1, x2, y2 = map(int, xyxy)
                    name = self.class_names.get(cls, str(cls))

                    # Crear mensaje Detection (CAMBIOS AQUÍ)
                    detection = Detection()
                    detection.class_id = cls
                    detection.class_name = name
                    detection.confidence = float(conf)
                    detection.x1 = x1
                    detection.y1 = y1
                    detection.x2 = x2
                    detection.y2 = y2
                    detection.center_x = (x1 + x2) / 2.0
                    detection.center_y = (y1 + y2) / 2.0
                    
                    detection_array.detections.append(detection)
        # Annotate and publish
        annotated = draw_detections(cv_image.copy(), detection_array.detections)

        # Publish detections (CAMBIOS AQUÍ - MUCHO MÁS SIMPLE)
        self.pub_results.publish(detection_array)
        
        try:
            ros_img = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
            ros_img.header = msg.header
            self.pub_annotated.publish(ros_img)
        except Exception as e:
            #self.get_logger().error(f"Error publicando imagen: {e}")
            pass

        

        # Logging cada 20 frames
        """self.count += 1
        if self.count % 20 == 0:
            elapsed = time.time() - self.start_time
            fps = self.count / elapsed
            avg_inference = np.mean(self.inference_times[-20:]) if self.inference_times else 0
            #self.get_logger().info(f"Frames: {self.count} | FPS: {fps:.1f} | Inference: {avg_inference*1000:.1f}ms | Backend: {self.backend}")"""

def draw_detections(img: np.ndarray, detections: list):
    for det in detections:
        color = class_color_hash(det.class_name)
        cv2.rectangle(img, (det.x1, det.y1), (det.x2, det.y2), color, 2)
        
        label = f"{det.class_name} {det.confidence:.2f}"
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (det.x1, det.y1 - th - baseline - 6), 
                      (det.x1 + tw + 6, det.y1), color, -1)
        cv2.putText(img, label, (det.x1 + 3, det.y1 - baseline - 3), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    return img


def class_color_hash(name: str):
    h = 0
    for ch in name:
        h = (h * 31 + ord(ch)) & 0xFFFFFFFF
    return (int((h & 0xFF0000) >> 16), int((h & 0x00FF00) >> 8), int(h & 0x0000FF))

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Apagando detector YOLO")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()

"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np
import cv2
import json
import time
import os

# Ultralytics
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # ------------ Config (puedes modificar/leer de parámetros ROS) -------------
        # Ruta al .pt o nombre del modelo (ej: "yolov8n.pt"). Ultralytics descargará si no existe.
        #self.model_path = os.getenv("models/yolo11m.pt", "yolo11m.pt")


        # Ruta al modelo dentro de la carpeta "models"
        self.model_path = "/home/raynel/autonomous_navigation/src/perception_stack/object_detection/models/yolo11s.pt"

        # umbral de confianza (0..1)
        self.conf_thres = float(os.getenv("YOLO_CONF", 0.25))
        # nms iou threshold
        self.iou_thres = float(os.getenv("YOLO_IOU", 0.45))
        # tamaño máximo de batch para inferencia (1 image at a time here)
        self.device = "cuda" if (torch_is_cuda_available()) else "cpu"

        # tamaño ENTRADA fijado → evitar error imgsz=None
        self.imgsz = [640, 480]     # puedes usar [512,512] también

        self.get_logger().info(f"YOLO model path: {self.model_path}, device: {self.device}")
        self.get_logger().info("Cargando modelo YOLO... esto puede tardar unos segundos si hay descarga.")
        try:
            self.model = YOLO(self.model_path)  # carga (y descarga si se refiere por nombre)
            # Set device if desired (ultralytics handles CPU/GPU automatically; but we log)
            # self.model.to(self.device)  # not required with ultralytics 8.x - it auto selects
            self.get_logger().info("Modelo YOLO cargado correctamente.")
        except Exception as e:
            self.get_logger().error(f"No se pudo cargar el modelo YOLO: {e}")
            raise

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber (usa mismo topic que segmentación)
        self.sub_image = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.callback_image, 1
        )

        # Publishers
        self.pub_annotated = self.create_publisher(Image, '/detection/annotated_image', 1)
        self.pub_results = self.create_publisher(String, '/detection/results', 1)

        # Pre-compute names from model (if available)
        self.class_names = {}
        try:
            # ultralytics model has .names attribute mapping id->label
            if hasattr(self.model, 'names'):
                self.class_names = self.model.names
        except Exception:
            self.class_names = {}

        self.start_time = time.time()
        self.count = 0

    def callback_image(self, msg: Image):
        t0 = time.time()
        # Convert ROS Image -> OpenCV (numpy) in RGB
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # Ultralytics can accept numpy (HWC) RGB arrays directly.
        # Run inference (single image)
        # We set conf and iou thresholds in the predict call
        try:
            # model returns a Results object; `model()` shorthand performs inference
            # We pass `verbose=False` to suppress prints.
            results = self.model(
                cv_image,
                imgsz=self.imgsz,                # keep original size, model will auto-resize/pad internally
                conf=self.conf_thres,
                iou=self.iou_thres,
                device=self.device,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f"Error during YOLO inference: {e}")
            return

        # results is a list-like (one element per image)
        # We take first result
        if len(results) == 0:
            detections = []
        else:
            r = results[0]
            # r.boxes is a Boxes object (may be empty)
            boxes = []
            if hasattr(r, 'boxes') and r.boxes is not None:
                for box in r.boxes:
                    # For ultralytics v8: box.xyxy, box.conf, box.cls
                    try:
                        xyxy = box.xyxy.cpu().numpy().tolist()[0]  # shape (1,4)
                        conf = float(box.conf.cpu().numpy().item())
                        cls = int(box.cls.cpu().numpy().item())
                    except Exception:
                        # Fallback for other result formats:
                        xyxy = box.xyxy.tolist()[0]
                        conf = float(box.conf.tolist()[0])
                        cls = int(box.cls.tolist()[0])

                    x1, y1, x2, y2 = map(int, xyxy)
                    name = self.class_names.get(cls, str(cls))
                    boxes.append({
                        "class_id": cls,
                        "class_name": name,
                        "confidence": float(conf),
                        "x1": x1, "y1": y1, "x2": x2, "y2": y2
                    })
            else:
                boxes = []

            detections = boxes

        # Annotate image for visualization (draw boxes, labels)
        annotated = cv_image.copy()
        annotated = draw_detections(annotated, detections)

        # Publish annotated image (convert back to ROS Image, rgb8)
        try:
            ros_img = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
            ros_img.header = msg.header  # keep original header/time/frame
            self.pub_annotated.publish(ros_img)
        except Exception as e:
            self.get_logger().error(f"Error publishing annotated image: {e}")

        # Publish detections as JSON string
        try:
            payload = {
                "header": {
                    "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                    "frame_id": msg.header.frame_id
                },
                "detections": detections
            }
            str_msg = String()
            str_msg.data = json.dumps(payload)
            self.pub_results.publish(str_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing JSON results: {e}")

        # Logging FPS (simple)
        self.count += 1
        if self.count % 20 == 0:
            elapsed = time.time() - self.start_time
            fps = self.count / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(f"Processed {self.count} frames -- approximate FPS: {fps:.2f}")

def draw_detections(img: np.ndarray, detections: list):
    h, w, _ = img.shape
    for det in detections:
        x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
        cls_name = det["class_name"]
        conf = det["confidence"]

        # color por clase (hash)
        color = class_color_hash(cls_name)
        # rectángulo
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness=2)
        # label background
        label = f"{cls_name} {conf:.2f}"
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (x1, y1 - th - baseline - 6), (x1 + tw + 6, y1), color, -1)
        # label text (white)
        cv2.putText(img, label, (x1 + 3, y1 - baseline - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

    return img

def class_color_hash(name: str):
    # hash string to color
    h = 0
    for ch in name:
        h = (h * 31 + ord(ch)) & 0xFFFFFFFF
    # map to RGB
    r = (h & 0xFF0000) >> 16
    g = (h & 0x00FF00) >> 8
    b = (h & 0x0000FF)
    return (int(r), int(g), int(b))

def torch_is_cuda_available():
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO detector (user interrupt).")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()
"""