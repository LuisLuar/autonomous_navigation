#!/usr/bin/env python3
"""
YOLOP + Canny Candidate Extractor (CON PUBLICACIÓN)
--------------------------------------------------
- Limpia huecos grandes en road mask
- Extrae píxeles candidatos
- Publica PixelPoint.msg
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interfaces.msg import SegmentationData, PixelPoint
from cv_bridge import CvBridge
import cv2
import numpy as np


class YOLOPCannyCandidatesRight(Node):

    def __init__(self):
        super().__init__('yolop_canny_candidates_right')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_seg = None

        # ================= PARÁMETROS =================
        self.declare_parameter('canny_low_road', 100)
        self.declare_parameter('canny_high_road', 150)
        self.declare_parameter('canny_low_lane', 40)
        self.declare_parameter('canny_high_lane', 120)
        self.declare_parameter('road_close_kernel', 70)

        self.declare_parameter('row_step', 1)
        self.declare_parameter('min_segment_width_lane', 2)
        self.declare_parameter('min_segment_width_road', 3)

        # ================= SUBS =================
        self.create_subscription(
            Image, '/camera/rgb/right',
            self.image_cb, 10)

        self.create_subscription(
            SegmentationData, '/segmentation/data_right',
            self.seg_cb, 10)

        # ================= PUB =================
        self.pub_pixels = self.create_publisher(
            PixelPoint,
            '/lane/pixel_candidates_right',
            10
        )

        #cv2.namedWindow('Lane Candidates Debug Right', cv2.WINDOW_NORMAL)
        self.create_timer(0.03, self.process)

        #self.get_logger().info('YOLOP + Canny Candidates iniciado')

    def image_cb(self, msg):
        self.latest_image = msg

    def seg_cb(self, msg):
        self.latest_seg = msg

    def process(self):

        if self.latest_image is None or self.latest_seg is None:
            return

        # ================= IMAGEN =================
        img = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # ================= SEGMENTACIÓN =================
        seg = np.frombuffer(
            self.latest_seg.mask_data, dtype=np.uint8
        ).reshape(self.latest_seg.height,
                  self.latest_seg.width)

        if seg.shape[:2] != (h, w):
            seg = cv2.resize(seg, (w, h), interpolation=cv2.INTER_NEAREST)

        road_mask = (seg == 1).astype(np.uint8) * 255
        lane_mask = (seg == 2).astype(np.uint8) * 255

        # ====== LIMPIAR HUECOS GRANDES EN ROAD ======
        k = self.get_parameter('road_close_kernel').value
        if k > 0:
            kernel = np.ones((k, k), np.uint8)
            road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)

        # ================= ROAD EDGES =================
        low_r = self.get_parameter('canny_low_road').value
        high_r = self.get_parameter('canny_high_road').value
        road_edges = cv2.Canny(road_mask, low_r, high_r)

        road_points = np.zeros_like(road_edges)

        row_step = self.get_parameter('row_step').value
        min_segment_width_road = self.get_parameter('min_segment_width_road').value 

        for y in range(0, h, row_step):
            xs = np.where(road_edges[y] > 0)[0]
            if len(xs) < min_segment_width_road:
                continue

            # dividir bordes en segmentos independientes
            segments = np.split(xs, np.where(np.diff(xs) > 1)[0] + 1)

            for seg in segments:
                if len(seg) >= min_segment_width_road:
                    x_center = int(seg.mean())
                    road_points[y, x_center] = 255

        # ================= LANE EDGES =================
        low_l = self.get_parameter('canny_low_lane').value
        high_l = self.get_parameter('canny_high_lane').value
        lane_roi_gray = cv2.bitwise_and(gray, gray, mask=lane_mask)
        lane_edges = cv2.Canny(lane_roi_gray, low_l, high_l)

        # ================= LANE CENTERLINE =================
        lane_center = np.zeros_like(lane_mask)

        min_segment_width_lane = self.get_parameter('min_segment_width_lane').value   # para líneas reales

        for y in range(0, h, row_step):
            xs = np.where(lane_mask[y] > 0)[0]
            if len(xs) < min_segment_width_lane:
                continue

            # dividir en segmentos contiguos
            segments = np.split(xs, np.where(np.diff(xs) > 1)[0] + 1)

            for seg in segments:
                if len(seg) >= min_segment_width_lane:
                    x_center = int(seg.mean())
                    lane_center[y, x_center] = 255

        # ================= EXTRAER PÍXELES =================
        ys_r, xs_r = np.where(road_points > 0)
        ys_l, xs_l = np.where(lane_center > 0)

        x_coords = np.concatenate([xs_r, xs_l]).astype(np.uint32)
        y_coords = np.concatenate([ys_r, ys_l]).astype(np.uint32)

        confidences = np.concatenate([
            np.full(len(xs_r), 0.7, dtype=np.float32),
            np.full(len(xs_l), 1.0, dtype=np.float32)
        ])

        total_points = len(x_coords)
        is_valid = total_points > 10
        global_quality = min(1.0, total_points / 5000.0)

        # ================= PUBLICAR =================
        msg = PixelPoint()
        msg.header = self.latest_image.header
        msg.x_coordinates = x_coords.tolist()
        msg.y_coordinates = y_coords.tolist()
        msg.confidences = confidences.tolist()
        msg.global_quality = float(global_quality)
        msg.is_valid = bool(is_valid)

        self.pub_pixels.publish(msg)

        # ================= DEBUG VISUAL =================
        """vis = np.zeros((h, w, 3), dtype=np.uint8)
        vis[road_edges > 0] = (50, 50, 50)       # gris: Canny original
        #vis[lane_edges > 0] = (50, 50, 50)

        vis[road_points > 0] = (0, 0, 255)       # rojo: centro por segmento        
        vis[lane_center > 0] = (0, 255, 255)  # amarillo


        txt = f'Pts: {total_points} | Q: {global_quality:.2f}'
        cv2.putText(vis, txt, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)
        
        # En la sección de debug visual
        txt2 = f'Road: {len(xs_r)} | Lane: {len(xs_l)}'
        cv2.putText(vis, txt2, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2) 

        cv2.imshow('Lane Candidates Debug Right', vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()"""
            
    def destroy_node(self):
        #cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = YOLOPCannyCandidatesRight()
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
