#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from custom_interfaces.msg import SegmentationData


class YOLOPv2SegmenterRight(Node):

    def __init__(self):
        super().__init__('segmenter_yolop_right')

        # =========================
        # CONFIG
        # =========================
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.use_fp16 = self.device.type == 'cuda'

        self.input_h = 640
        self.input_w = 640

        self.lane_thresh = 0.6
        self.road_thresh = 0.27

        self.bridge = CvBridge()

        if self.device.type == 'cuda':
            torch.backends.cudnn.benchmark = True

        # =========================
        # LOAD MODEL (TorchScript)
        # =========================
        model_path = (
            '/home/raynel/autonomous_navigation/src/'
            'perception_stack/semantic_segmentation/pretrained/yolopv2.pt'
        )

        #self.get_logger().info(f'Cargando YOLOPv2 en {self.device}')
        self.model = torch.jit.load(model_path, map_location=self.device)
        self.model.eval()

        if self.use_fp16:
            self.model.half()

        # =========================
        # ROS I/O
        # =========================
        self.sub = self.create_subscription(
            Image,
            '/camera/rgb/right_corrected',
            self.image_callback,
            1
        )

        self.pub = self.create_publisher(
            SegmentationData,
            '/segmentation/data_right',
            1
        )

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
    # CALLBACK
    # =====================================================
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
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

        except Exception as e:
            #self.get_logger().error(str(e))
            pass

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPv2SegmenterRight()
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
