#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import zlib
from custom_interfaces.msg import SegmentationData
from custom_interfaces.msg import LaneArray, Lane

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector')
        
        # Suscribirse a datos de segmentación
        self.sub = self.create_subscription(SegmentationData,'/segmentation/data',self.segmentation_callback,10)
        
        # Publicar carriles detectados
        self.pub_lanes = self.create_publisher(LaneArray,'/lanes/detected',10)
        
        # Parámetros configurables
        self.min_lane_width_pixels = 50
        self.roi_height_ratio = 0.6  # Usar solo parte inferior de la imagen
        
    def segmentation_callback(self, msg):
        # 1) Descomprimir máscara
        """if msg.compression_type == SegmentationData.COMPRESSION_ZLIB:
            mask_bytes = zlib.decompress(msg.mask_compressed)
        else:"""
        
        mask_bytes = msg.mask_data
            
        mask = np.frombuffer(mask_bytes, dtype=np.uint8)
        mask = mask.reshape((msg.height, msg.width))
        
        # 2) Extraer ROI (Región de Interés)
        roi_start = int(msg.height * (1 - self.roi_height_ratio))
        roi_mask = mask[roi_start:, :]
        
        # 3) Identificar clases de carril (depende de tu modelo)
        # Ejemplo: clases 6, 7, 8 podrían ser carriles
        lane_mask = np.isin(roi_mask, [0, 7, 11])
        
        # 4) Detectar carriles (algoritmo simplificado)
        lanes = self.detect_lanes(lane_mask)
        
        # 5) Convertir a coordenadas normalizadas
        normalized_lanes = self.normalize_lanes(
            lanes, roi_start, msg.width, msg.height
        )
        
        # 6) Publicar
        self.publish_lanes(normalized_lanes, msg.header)
    
    def detect_lanes(self, binary_mask):
        """Detección básica de carriles usando componentes conectados"""
        lanes = []
        
        # Encontrar componentes conectados
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary_mask.astype(np.uint8) * 255,
            connectivity=8
        )
        
        # Filtrar por tamaño
        for i in range(1, num_labels):  # Saltar fondo (0)
            width = stats[i, cv2.CC_STAT_WIDTH]
            height = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            
            if width > self.min_lane_width_pixels and area > 100:
                # Crear máscara para este componente
                component_mask = (labels == i)
                
                # Extraer puntos del contorno
                y_coords, x_coords = np.where(component_mask)
                
                if len(x_coords) > 10:  # Mínimo puntos para ser válido
                    # Ajustar polinomio (línea recta o curva)
                    coeffs = np.polyfit(y_coords, x_coords, deg=2)
                    lanes.append({
                        'coefficients': coeffs,
                        'confidence': area / (width * height),  # métrica simple
                        'points': list(zip(x_coords, y_coords))
                    })
        
        return lanes
    
    def normalize_lanes(self, lanes, roi_start, img_w, img_h):
        """Convertir coordenadas de píxeles a normalizadas [0,1]"""
        normalized = []
        
        for lane in lanes:
            lane_msg = Lane()
            lane_msg.confidence = lane['confidence']
            
            # Evaluar polinomio en varios puntos de Y
            y_points = np.linspace(0, img_h - roi_start - 1, 10)
            x_points = np.polyval(lane['coefficients'], y_points)
            
            # Normalizar
            lane_msg.points_x = (x_points / img_w).tolist()
            lane_msg.points_y = ((y_points + roi_start) / img_h).tolist()
            
            # Determinar tipo de línea (simplificado)
            # Podrías usar la máscara original para ver si es continua o discontinua
            lane_msg.lane_type = self.determine_lane_type(lane['points'])
            
            normalized.append(lane_msg)
        
        return normalized
    
    def determine_lane_type(self, points):
        """Determinar si línea es sólida o discontinua"""
        # Implementar lógica basada en continuidad de puntos
        # Por ahora retornar desconocido
        return Lane.LANE_TYPE_UNKNOWN
    
    def publish_lanes(self, lanes, header):
        lane_array = LaneArray()
        lane_array.header = header
        lane_array.lanes = lanes
        self.pub_lanes.publish(lane_array)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info("Supervisor de RPLIDAR apagado por usuario")
        pass
    except Exception as e:
        node.get_logger().error(f"Error fatal: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == '__main__':
    main()