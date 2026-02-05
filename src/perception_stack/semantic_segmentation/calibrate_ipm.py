#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import json

from custom_interfaces.msg import PixelPoint


class PitchCalibrationNode(Node):

    def __init__(self):
        super().__init__('pitch_calibration_node')

        calibration_path = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration.json"
        with open(calibration_path, 'r') as f:
            calib = json.load(f)

        self.h  = float(calib['camera_height'])
        self.fx = float(calib['intrinsics']['fx'])
        self.fy = float(calib['intrinsics']['fy'])
        self.cx = float(calib['intrinsics']['cx'])
        self.cy = float(calib['intrinsics']['cy'])

        self.get_logger().info("✅ Pitch calibration node (IPM EXACTA)")

        self.create_subscription(
            PixelPoint,
            '/pixel/right_lane',
            self.pixel_callback,
            10
        )

    # ================= IPM IDÉNTICA =================
    def pixel_to_world(self, u, v, pitch):
        x = (u - self.cx) / self.fx
        y = -(v - self.cy) / self.fy
        ray = np.array([x, y, 1.0])

        cp = math.cos(pitch)
        sp = math.sin(pitch)

        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],
            [0, -sp,  cp]
        ])

        ray_w = R @ ray

        if ray_w[1] >= 0:
            return None

        t = self.h / -ray_w[1]

        Xc = ray_w[0] * t
        Zc = ray_w[2] * t

        # MISMA CONVENCIÓN QUE PixelToMeterTransform
        x_base = Zc
        y_base = -Xc

        return x_base, y_base

    # ============ MÉTRICA CORRECTA ============
    def compute_straightness_error(self, pixels, pitch):
        ys = []

        for u, v in pixels:
            res = self.pixel_to_world(u, v, pitch)
            if res is not None:
                _, y = res
                ys.append(y)

        if len(ys) < 5:
            return np.inf

        # carril recto → y constante
        return np.std(ys)

    def pixel_callback(self, msg: PixelPoint):

        if not msg.is_valid or len(msg.x_coordinates) < 5:
            self.get_logger().warn("No suficientes puntos")
            return

        pixels = list(zip(msg.x_coordinates, msg.y_coordinates))

        pitches = np.linspace(0.05, 0.30, 400)

        errors = []

        for p in pitches:
            errors.append(self.compute_straightness_error(pixels, p))

        idx = np.argmin(errors)
        best_pitch = pitches[idx]

        self.get_logger().info(
            f"🎯 PITCH ÓPTIMO = {best_pitch:.6f} rad  |  "
            f"{math.degrees(best_pitch):.3f}°  |  "
            f"std_y = {errors[idx]:.6f}"
        )


def main():
    rclpy.init()
    node = PitchCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
