#!/usr/bin/env python3
"""
Lane PointCloud Merger - Fusiona pointclouds de 3 cámaras CON MEJOR SINCRONIZACIÓN
=========================================================
- Recibe pointclouds de cámaras front, left, right
- Sincronización adaptativa basada en frecuencias reales
- Publica cuando hay datos de al menos 2 cámaras
- Maneja frecuencias diferentes (especialmente cámara izquierda a ~15Hz)
"""

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
import time
from typing import List, Optional, Dict, Tuple

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2

class LanePointCloudMerger(Node):
    def __init__(self):
        super().__init__('lane_pointcloud_merger')
        
        # Parámetros ajustados para la cámara izquierda
        self.declare_parameters('', [
            ('max_age_left', 0.5),      # Mayor tolerancia para cámara izquierda (500ms)
            ('max_age_others', 0.15),   # Menor tolerancia para cámaras rápidas (150ms)
            ('min_sources', 2),         # Mínimas fuentes requeridas
            ('debug', True),
            ('publish_empty', False),   # Publicar incluso si no hay fusión completa
        ])
        
        # Buffers para sincronización
        self.cloud_buffer: Dict[str, Optional[List[Tuple[float, float, float]]]] = {
            'front': None,
            'left': None,
            'right': None
        }
        self.cloud_buffer_time: Dict[str, float] = {}
        self.cloud_buffer_stamp: Dict[str, Tuple[int, int]] = {}  # Guarda el stamp original
        
        # Estadísticas para diagnóstico
        self.source_stats = {
            'front': {'count': 0, 'last_time': 0},
            'left': {'count': 0, 'last_time': 0},
            'right': {'count': 0, 'last_time': 0}
        }
        
        # Suscriptores
        self.create_subscription(PointCloud2, '/lane/meter_candidates_front',
                                 lambda msg: self.cb_points(msg, source='front'), 10)
        self.create_subscription(PointCloud2, '/lane/meter_candidates_left',
                                 lambda msg: self.cb_points(msg, source='left'), 10)
        self.create_subscription(PointCloud2, '/lane/meter_candidates_right',
                                 lambda msg: self.cb_points(msg, source='right'), 10)
        
        # Publisher para pointcloud fusionado
        self.pub_merged = self.create_publisher(
            PointCloud2, '/lane/merged_candidates', 10
        )
        
        # Timer para limpieza y publicación
        self.cleanup_timer = self.create_timer(0.1, self.check_and_publish)  # 10Hz
        
        #self.get_logger().info(" LanePointCloudMerger inicializado - Fusionando pointclouds de 3 cámaras")
        #self.get_logger().info(" Configuración: left_max_age=0.5s, others_max_age=0.15s, min_sources=2")
    
    def cb_points(self, msg, source):
        """Callback para pointcloud de cada cámara"""
        # Extraer puntos del mensaje
        pts = list(point_cloud2.read_points(msg, ('x','y','z'), skip_nans=True))
        
        # Actualizar estadísticas
        self.source_stats[source]['count'] += 1
        self.source_stats[source]['last_time'] = time.time()
        
        if len(pts) < 3:  # Umbral más bajo para aceptar datos
            """if self.get_parameter('debug').value:
                self.get_logger().debug(f"{source}: solo {len(pts)} puntos, ignorando")"""
            return
        
        # Actualizar buffer con el stamp original
        now = time.time()
        self.cloud_buffer[source] = pts
        self.cloud_buffer_time[source] = now
        self.cloud_buffer_stamp[source] = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        
        if self.get_parameter('debug').value:
            #self.get_logger().debug(f"{source}: recibidos {len(pts)} puntos")
            
            # Mostrar estadísticas periódicamente
            """if self.source_stats[source]['count'] % 10 == 0:
                total_pts = sum(len(p) if p else 0 for p in self.cloud_buffer.values())
                self.get_logger().info(
                    f" Stats - Front: {self.source_stats['front']['count']}, "
                    f"Left: {self.source_stats['left']['count']}, "
                    f"Right: {self.source_stats['right']['count']}, "
                    f"Buffer total: {total_pts} pts"
                )"""
    
    def check_and_publish(self):
        """Verifica buffers y publica si hay datos válidos"""
        current_time = time.time()
        
        # Usar diferentes max_age según la fuente
        max_age_left = self.get_parameter('max_age_left').value
        max_age_others = self.get_parameter('max_age_others').value
        min_sources = self.get_parameter('min_sources').value
        
        # Verificar qué fuentes tienen datos recientes
        valid_sources = []
        ages = {}
        
        for src in ['front', 'left', 'right']:
            if src in self.cloud_buffer_time and self.cloud_buffer[src] is not None:
                age = current_time - self.cloud_buffer_time[src]
                
                # Aplicar max_age según la fuente
                max_age = max_age_left if src == 'left' else max_age_others
                
                if age <= max_age:
                    valid_sources.append(src)
                    ages[src] = age
                """else:
                    if self.get_parameter('debug').value and age < 1.0:  # Solo log si no es muy viejo
                        self.get_logger().debug(f"{src}: datos muy antiguos ({age:.3f}s > {max_age:.3f}s)")"""
        
        
        # Solo proceder si tenemos suficientes fuentes
        if len(valid_sources) < min_sources:
            """if self.get_parameter('debug').value:
                self.get_logger().debug(
                    f"Esperando más fuentes: {len(valid_sources)}/{min_sources}. "
                    f"Válidas: {valid_sources}"
                )"""
            
            # Opcional: publicar datos parciales si está configurado
            if self.get_parameter('publish_empty').value and valid_sources:
                self.publish_partial_data(valid_sources)
            return
        
        # Fusionar puntos de todas las fuentes válidas
        merged_pts = []
        for src in valid_sources:
            points = self.cloud_buffer[src]
            if points is not None and len(points) > 0:
                merged_pts.extend(points)
        
        if len(merged_pts) < 5:
            """if self.get_parameter('debug').value:
                self.get_logger().debug(f"Solo {len(merged_pts)} puntos fusionados, no publicando")"""
            return
        
        # Publicar pointcloud fusionado
        self.publish_merged_cloud(merged_pts)
        
        """if self.get_parameter('debug').value:
            age_str = ", ".join([f"{src}:{ages[src]:.3f}s" for src in valid_sources])
            self.get_logger().info(
                f" Publicando {len(merged_pts)} puntos fusionados de {valid_sources} "
                f"(Edades: {age_str})"
            )"""

    
    def publish_partial_data(self, sources):
        """Publica datos parciales cuando no hay suficientes fuentes"""
        if not self.get_parameter('publish_empty').value:
            return
        
        merged_pts = []
        for src in sources:
            points = self.cloud_buffer[src]
            if points is not None and len(points) > 0:
                merged_pts.extend(points)
        
        if merged_pts:
            self.publish_merged_cloud(merged_pts)
            """if self.get_parameter('debug').value:
                self.get_logger().info(
                    f" Publicando datos parciales: {len(merged_pts)} puntos de {sources}"
                )"""
    
    def publish_merged_cloud(self, pts):
        """Publica el pointcloud fusionado"""
        # Crear header
        header = self.create_header()
        
        # Crear mensaje PointCloud2
        try:
            cloud = point_cloud2.create_cloud_xyz32(header, pts)
            self.pub_merged.publish(cloud)
        except Exception as e:
            #self.get_logger().error(f"Error creando nube de puntos: {e}")
            pass
    
    def create_header(self):
        """Crea header para mensajes"""
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_footprint'
        return header
    
    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    node = LanePointCloudMerger()
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