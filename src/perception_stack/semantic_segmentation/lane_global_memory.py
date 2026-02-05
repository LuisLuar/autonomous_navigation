import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from collections import deque
import numpy as np
from scipy import stats
from scipy.optimize import curve_fit
import struct
from typing import Dict, List, Tuple, Optional
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class RoadLineProcessor(Node):
    def __init__(self):
        super().__init__('road_line_processor')
        
        # Configuración de parámetros
        self.declare_parameter('mem_size', 10)
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('lookbehind_distance', 2.0)
        self.declare_parameter('point_spacing', 0.1)
        self.declare_parameter('fit_method', 'linear')  # 'linear', 'quadratic'
        
        self.mem_size = self.get_parameter('mem_size').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.lookbehind = self.get_parameter('lookbehind_distance').value
        self.point_spacing = self.get_parameter('point_spacing').value
        self.fit_method = self.get_parameter('fit_method').value
        
        # Memorias globales para cada tipo de línea
        self.global_memories = {
            k: deque(maxlen=self.mem_size) for k in ['left_border', 'lane_dividing', 'right_lane', 'right_border']
        }
        
        # Últimas estimaciones de líneas para cada tipo
        self.last_estimations = {
            'left_border': None,
            'lane_dividing': None,
            'right_lane': None,
            'right_border': None
        }
        
        # Contador de tiempo sin actualizaciones
        self.last_update_time = {k: self.get_clock().now() for k in self.global_memories.keys()}
        
        # Suscriptores para cada línea
        self.subs = {}
        for name in self.global_memories.keys():
            # Usar QoS para solo el mensaje más reciente
            qos_profile = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST
            )
            self.subs[name] = self.create_subscription(
                PointCloud2,
                f'/memory_local/{name}',
                lambda msg, n=name: self.cb_memory_fast(msg, n),
                qos_profile
            )
        
        # Publicadores para las nubes de puntos generadas
        self.pubs = {
            k: self.create_publisher(PointCloud2, f'/memory/{k}', 10)
            for k in self.global_memories
        }
        
        # Timer para publicar líneas estimadas periódicamente
        self.timer = self.create_timer(0.1, self.publish_estimated_lines)  # 10 Hz
        
        self.get_logger().info(f'RoadLineProcessor inicializado con método de ajuste: {self.fit_method}')
    
    def cb_memory_fast(self, msg: PointCloud2, line_type: str):
        """Callback rápido para procesar nubes de puntos de líneas viales"""
        try:
            # Extraer puntos de la nube
            points = self.pointcloud2_to_array(msg)
            
            if len(points) == 0:
                return
            
            # Actualizar memoria
            self.global_memories[line_type].append(points)
            self.last_update_time[line_type] = self.get_clock().now()
            
            # Estimar la línea
            estimated_line = self.estimate_line(line_type)
            
            if estimated_line is not None:
                # Guardar última estimación
                self.last_estimations[line_type] = estimated_line
                
                # Generar y publicar nube de puntos
                generated_points = self.generate_points_from_line(estimated_line)
                pointcloud_msg = self.array_to_pointcloud2(generated_points, msg.header)
                self.pubs[line_type].publish(pointcloud_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error en cb_memory_fast para {line_type}: {str(e)}')
    
    def estimate_line(self, line_type: str) -> Optional[Dict]:
        """Estimar la línea (recta o curva) que mejor se ajusta a los puntos en memoria"""
        # Combinar todos los puntos de la memoria
        all_points = []
        for points in self.global_memories[line_type]:
            all_points.extend(points)
        
        all_points = np.array(all_points)
        
        if len(all_points) < 3:
            # Si no hay suficientes puntos, usar la última estimación
            return self.last_estimations[line_type]
        
        # Extraer coordenadas X, Y (asumiendo que Z es aproximadamente constante o irrelevante)
        x = all_points[:, 0]
        y = all_points[:, 1]
        
        # Ordenar por X para mejor procesamiento
        sort_idx = np.argsort(x)
        x_sorted = x[sort_idx]
        y_sorted = y[sort_idx]
        
        try:
            if self.fit_method == 'linear':
                # Ajuste lineal
                slope, intercept, r_value, p_value, std_err = stats.linregress(x_sorted, y_sorted)
                coeffs = [slope, intercept]
                fit_func = self.linear_func
                
            elif self.fit_method == 'quadratic':
                # Ajuste cuadrático (curva de grado 2)
                coeffs = np.polyfit(x_sorted, y_sorted, 2)
                fit_func = self.quadratic_func
            
            else:
                self.get_logger().warn(f'Método de ajuste {self.fit_method} no reconocido. Usando lineal.')
                slope, intercept, r_value, p_value, std_err = stats.linregress(x_sorted, y_sorted)
                coeffs = [slope, intercept]
                fit_func = self.linear_func
            
            # Calcular error RMS
            y_pred = fit_func(x_sorted, *coeffs)
            rms_error = np.sqrt(np.mean((y_sorted - y_pred) ** 2))
            
            # Determinar rango de X para la línea
            x_min = np.min(x_sorted)
            x_max = np.max(x_sorted)
            
            return {
                'type': self.fit_method,
                'coeffs': coeffs,
                'fit_func': fit_func,
                'x_range': (x_min, x_max),
                'rms_error': rms_error,
                'timestamp': self.get_clock().now()
            }
            
        except Exception as e:
            self.get_logger().warn(f'Error ajustando línea para {line_type}: {str(e)}')
            return self.last_estimations[line_type]
    
    def linear_func(self, x: np.ndarray, a: float, b: float) -> np.ndarray:
        """Función lineal: y = ax + b"""
        return a * x + b
    
    def quadratic_func(self, x: np.ndarray, a: float, b: float, c: float) -> np.ndarray:
        """Función cuadrática: y = ax² + bx + c"""
        return a * x**2 + b * x + c
    
    def generate_points_from_line(self, line_info: Dict) -> np.ndarray:
        """Generar puntos a partir de la línea estimada"""
        if line_info is None:
            return np.array([])
        
        # Obtener rango actual de la línea
        x_min, x_max = line_info['x_range']
        
        # Extender el rango según configuración
        x_start = x_min - self.lookbehind
        x_end = x_max + self.lookahead
        
        # Generar puntos espaciados uniformemente
        num_points = int((x_end - x_start) / self.point_spacing) + 1
        x_points = np.linspace(x_start, x_end, num_points)
        
        # Calcular Y usando la función ajustada
        y_points = line_info['fit_func'](x_points, *line_info['coeffs'])
        
        # Asumir Z constante (puedes ajustar esto según tu caso)
        z_points = np.zeros_like(x_points)
        
        # Combinar en array de puntos
        points = np.column_stack([x_points, y_points, z_points])
        
        return points
    
    def publish_estimated_lines(self):
        """Publicar líneas estimadas periódicamente (incluso si no hay nuevas actualizaciones)"""
        current_time = self.get_clock().now()
        
        for line_type in self.global_memories.keys():
            # Verificar si ha pasado mucho tiempo desde la última actualización
            time_since_update = (current_time - self.last_update_time[line_type]).nanoseconds / 1e9
            
            # Si tenemos una estimación previa y han pasado datos en otras memorias
            if self.last_estimations[line_type] is not None and time_since_update < 5.0:  # 5 segundos de timeout
                # Actualizar rango de X basado en otras líneas (para mantener paralelismo)
                updated_line = self.update_line_from_others(line_type, self.last_estimations[line_type])
                
                # Generar y publicar puntos
                generated_points = self.generate_points_from_line(updated_line)
                
                if len(generated_points) > 0:
                    # Crear header con timestamp actual
                    header = Header()
                    header.stamp = current_time.to_msg()
                    header.frame_id = 'odom'  # o tu frame TF correspondiente
                    
                    pointcloud_msg = self.array_to_pointcloud2(generated_points, header)
                    self.pubs[line_type].publish(pointcloud_msg)
    
    def update_line_from_others(self, line_type: str, line_info: Dict) -> Dict:
        """Actualizar rango de la línea basándose en otras líneas (para mantener paralelismo)"""
        # Encontrar el rango máximo de X entre todas las líneas activas
        all_x_min = []
        all_x_max = []
        
        for other_type, other_info in self.last_estimations.items():
            if other_info is not None and other_type != line_type:
                x_min, x_max = other_info['x_range']
                all_x_min.append(x_min)
                all_x_max.append(x_max)
        
        if all_x_min and all_x_max:
            # Usar el rango promedio de las otras líneas
            avg_x_min = np.mean(all_x_min)
            avg_x_max = np.mean(all_x_max)
            
            # Suavizar la transición
            current_min, current_max = line_info['x_range']
            new_min = 0.7 * current_min + 0.3 * avg_x_min
            new_max = 0.7 * current_max + 0.3 * avg_x_max
            
            # Actualizar solo el rango, manteniendo los coeficientes
            updated_info = line_info.copy()
            updated_info['x_range'] = (new_min, new_max)
            return updated_info
        
        return line_info
    
    def pointcloud2_to_array(self, cloud_msg: PointCloud2) -> np.ndarray:
        """Convertir PointCloud2 a array numpy"""
        # Determinar formato de datos
        fmt = self.get_pointcloud2_format(cloud_msg)
        
        # Extraer datos
        points = []
        for i in range(0, len(cloud_msg.data), cloud_msg.point_step):
            point_data = cloud_msg.data[i:i + cloud_msg.point_step]
            
            try:
                if fmt == 'xyz':
                    x, y, z = struct.unpack('fff', point_data[:12])
                elif fmt == 'xyzi':
                    x, y, z, intensity = struct.unpack('ffff', point_data[:16])
                else:
                    # Intento genérico
                    x, y, z = struct.unpack('fff', point_data[:12])
                
                points.append([x, y, z])
            except:
                continue
        
        return np.array(points) if points else np.array([])
    
    def get_pointcloud2_format(self, cloud_msg: PointCloud2) -> str:
        """Determinar el formato de PointCloud2"""
        has_intensity = False
        for field in cloud_msg.fields:
            if field.name == 'intensity':
                has_intensity = True
        
        return 'xyzi' if has_intensity else 'xyz'
    
    def array_to_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:
        """Convertir array numpy a PointCloud2"""
        msg = PointCloud2()
        msg.header = header
        
        if len(points) == 0:
            msg.height = 1
            msg.width = 0
            msg.fields = []
            msg.is_bigendian = False
            msg.point_step = 0
            msg.row_step = 0
            msg.data = bytes()
            msg.is_dense = True
            return msg
        
        # Configurar campos
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Convertir datos a bytes
        msg.data = points.astype(np.float32).tobytes()
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = RoadLineProcessor()
    
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