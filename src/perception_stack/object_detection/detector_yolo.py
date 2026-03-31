#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from custom_interfaces.msg import DetectionArray, Detection
import numpy as np
import time
import os
from ultralytics import YOLO
import torch
import cv2

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ------------ Config Modelo -------------
        self.model_path = "/home/raynel/autonomous_navigation/src/perception_stack/object_detection/models/yolo11n.pt"

        self.conf_thres = float(os.getenv("YOLO_CONF", 0.25))
        self.iou_thres = float(os.getenv("YOLO_IOU", 0.45))
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.imgsz = 640

        # PARCHE: Clases esenciales (person, bicycle, car, motorcycle, bus, truck)
        self.target_classes = [0, 1, 2, 3, 5, 7]

        self.process_every_n_frames = self.declare_parameter('process_every_n_frames', 1).value

        self.get_logger().info(f"🚀 YOLOv11n en {self.device} | Clases: {self.target_classes}")

        # ------------ Tracking -------------
        self.use_tracking = True
        self.tracker_type = "bytetrack.yaml"

        # ------------ Load model -------------
        self.model = YOLO(self.model_path)
        self.class_names = self.model.names

        self.frame_count = 0

        # QoS Best Effort para telemetría/imágenes
        best_effort_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub_image = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, best_effort_qos)
        self.pub_results = self.create_publisher(DetectionArray, '/detection/results', 1)

    def image_callback(self, msg: CompressedImage):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        t0 = time.time()

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: return
        except Exception as e:
            self.get_logger().error(f"Error decodificando: {e}")
            return

        # ---------- INFERENCIA CON FILTRO DE CLASES ----------
        try:
            # Al pasar 'classes', YOLO ignora el resto en el post-proceso (más rápido)
            results = self.model.track(
                source=cv_image,
                classes=self.target_classes,
                imgsz=self.imgsz,
                conf=self.conf_thres,
                iou=self.iou_thres,
                device=self.device,
                persist=True,
                tracker=self.tracker_type,
                verbose=False,
                half=(self.device == "cuda")
            )
        except Exception as e:
            self.get_logger().error(f"Error en inferencia: {e}")
            return

        inference_time = (time.time() - t0) * 1000.0

        detection_array = DetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "camera_link"

        # ---------- PROCESAR RESULTADOS ----------
        if results and len(results) > 0:
            r = results[0]
            if r.boxes is not None and len(r.boxes) > 0:
                boxes = r.boxes
                
                all_xyxy = boxes.xyxy.cpu().numpy()
                all_cls = boxes.cls.cpu().numpy()
                all_conf = boxes.conf.cpu().numpy()
                has_ids = boxes.id is not None
                if has_ids:
                    all_ids = boxes.id.cpu().numpy()

                for i in range(len(all_xyxy)):
                    det = Detection()
                    xyxy = all_xyxy[i]
                    
                    # --- CORRECCIÓN AQUÍ: Forzar int() nativo de Python ---
                    det.class_id = int(all_cls[i]) 
                    det.class_name = self.class_names.get(det.class_id, "unknown")
                    det.confidence = float(all_conf[i])

                    # Coordenadas BBox (también forzamos int)
                    det.u1, det.v1, det.u2, det.v2 = map(int, xyxy)
                    
                    det.center_u = float((xyxy[0] + xyxy[2]) / 2)
                    det.center_v = float(xyxy[3]) 

                    if has_ids:
                        det.track_id = int(all_ids[i]) # <-- También aquí
                    else:
                        det.track_id = 4294967295 # UINT32_MAX para "sin ID"

                    detection_array.detections.append(det)

        self.pub_results.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
  0: person --------------------------------------
  1: bicycle --------------------------------------
  2: car --------------------------------------
  3: motorcycle --------------------------------------
  4: airplane
  5: bus --------------------------------------
  6: train --------------------------------------
  7: truck --------------------------------------
  8: boat
  9: traffic light
  10: fire hydrant
  11: stop sign --------------------------------------?
  12: parking meter
  13: bench
  14: bird
  15: cat
  16: dog
  17: horse
  18: sheep
  19: cow
  20: elephant
  21: bear
  22: zebra
  23: giraffe
  24: backpack
  25: umbrella
  26: handbag
  27: tie
  28: suitcase
  29: frisbee
  30: skis
  31: snowboard
  32: sports ball
  33: kite
  34: baseball bat
  35: baseball glove
  36: skateboard
  37: surfboard
  38: tennis racket
  39: bottle
  40: wine glass
  41: cup
  42: fork
  43: knife
  44: spoon
  45: bowl
  46: banana
  47: apple
  48: sandwich
  49: orange
  50: broccoli
  51: carrot
  52: hot dog
  53: pizza
  54: donut
  55: cake
  56: chair
  57: couch
  58: potted plant
  59: bed
  60: dining table
  61: toilet
  62: tv
  63: laptop
  64: mouse
  65: remote
  66: keyboard
  67: cell phone
  68: microwave
  69: oven
  70: toaster
  71: sink
  72: refrigerator
  73: book
  74: clock
  75: vase
  76: scissors
  77: teddy bear
  78: hair drier
  79: toothbrush
"""