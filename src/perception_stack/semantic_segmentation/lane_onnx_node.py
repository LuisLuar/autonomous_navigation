#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit 
import sys
import os
from rclpy.qos import qos_profile_sensor_data

class TwinLiteTRTNode(Node):
    def __init__(self):
        super().__init__('twinlite_trt_node')
        
        # 1. Configuración de TensorRT
        self.logger = trt.Logger(trt.Logger.INFO)
        engine_path = '/home/raynel/UFDV2/TwinLiteNet/pretrained/twinlite_lines_only.engine'
        
        if not os.path.exists(engine_path):
            self.get_logger().error(f"No se encontró el engine en: {engine_path}")
            return

        with open(engine_path, 'rb') as f:
            self.runtime = trt.Runtime(self.logger)
            self.engine = self.runtime.deserialize_cuda_engine(f.read())
        
        # Usamos un nombre que no choque con la API interna
        self.exec_context = self.engine.create_execution_context()
        
        # 2. Reservar Memoria
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers(self.engine)
        
        # 3. ROS Sub
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.callback,
            qos_profile_sensor_data
        )
        
        self.window_name = "TwinLiteNet - Tesis Raynel (TRT)"
        self.get_logger().info("✅ Nodo TensorRT 10 listo. Visualizando...")

    def allocate_buffers(self, engine):
        inputs, outputs, bindings = [], [], []
        stream = cuda.Stream()
        
        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            size = trt.volume(engine.get_tensor_shape(name))
            dtype = trt.nptype(engine.get_tensor_dtype(name))
            
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            bindings.append(int(device_mem))
            
            if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                inputs.append({'host': host_mem, 'device': device_mem, 'name': name})
            else:
                outputs.append({'host': host_mem, 'device': device_mem, 'name': name})
                
        return inputs, outputs, bindings, stream

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        h_orig, w_orig = frame.shape[:2]

        # --- PRE-PROCESAMIENTO ---
        input_img = cv2.resize(frame, (640, 360))
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        # Importante: Tu engine está en FP16
        input_img = input_img.astype(np.float16) / 255.0
        input_img = input_img.transpose(2, 0, 1).ravel()

        # --- INFERENCIA ---
        np.copyto(self.inputs[0]['host'], input_img)
        cuda.memcpy_htod_async(self.inputs[0]['device'], self.inputs[0]['host'], self.stream)
        
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            addr = self.inputs[0]['device'] if i == 0 else self.outputs[0]['device']
            self.exec_context.set_tensor_address(name, int(addr))

        self.exec_context.execute_async_v3(stream_handle=self.stream.handle)
        cuda.memcpy_dtoh_async(self.outputs[0]['host'], self.outputs[0]['device'], self.stream)
        self.stream.synchronize()
        
        # --- POST-PROCESAMIENTO ---
        # Remodelar salida (1, 2, 360, 640) -> Lane Lines están en el canal 1
        output = self.outputs[0]['host'].reshape(1, 2, 360, 640)
        
        # Argmax para obtener la máscara binaria
        ll_mask = np.argmax(output[0], axis=0).astype(np.uint8)
        ll_mask = cv2.resize(ll_mask, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)

        # --- VISUALIZACIÓN (Tu estilo original) ---
        vis_img = frame.copy()
        
        # Color Rojo para Lane Lines (ll_mask == 1)
        vis_img[ll_mask == 1] = [0, 0, 255]

        # Mezclar con la original (transparencia 0.4 - 0.6 como antes)
        result = cv2.addWeighted(vis_img, 0.4, frame, 0.6, 0)

        cv2.imshow(self.window_name, result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TwinLiteTRTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()