#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
import csv
import os
from pathlib import Path as PathLib
import time

class GlobalPathRecorder(Node):
    def __init__(self):
        super().__init__('global_path_recorder')
        
        # Estado de logging
        self.is_logging_enabled = False
        self.current_log_path = None
        self.node_name = "global_path"
        
        # Almacena el último path recibido
        self.last_path_data = []
        self.last_path_timestamp = None
        self.path_frame_id = None
        self.has_new_path = False
        
        # Subscripciones
        self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )
        
        # Subscripciones a las señales del manager
        self.create_subscription(
            Bool, '/logging_enabled', self.logging_enabled_cb, 10)
            
        self.create_subscription(
            String, '/current_log_path', self.log_path_cb, 10)
        
        #self.get_logger().info('🗺️  GlobalPathRecorder inicializado')
        #self.get_logger().info('📡 Suscrito a /global_path')
        #self.get_logger().info('⏳ Esperando path global y señal de logging...')
    
    def path_callback(self, msg):
        """Almacena la última versión completa del path"""
        # Extraer datos del path
        path_data = []
        
        for i, pose_stamped in enumerate(msg.poses):
            pose = pose_stamped.pose
            header = pose_stamped.header
            
            timestamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            
            path_data.append({
                'sequence_id': i,  # ID secuencial dentro del path
                'timestamp': timestamp,
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w,
                'frame_id': header.frame_id,
                'pose_id': pose_stamped.header.seq if hasattr(pose_stamped.header, 'seq') else i
            })
        
        # Actualizar el último path
        self.last_path_data = path_data
        self.last_path_timestamp = time.time()
        self.path_frame_id = msg.header.frame_id if msg.poses else "unknown"
        self.has_new_path = True
        
        #self.get_logger().debug(f'📥 Path recibido: {len(path_data)} puntos en frame {self.path_frame_id}')
        
        # Si ya estamos logging y recibimos un nuevo path, podemos guardarlo inmediatamente
        # (opcional, dependiendo de si quieres múltiples versiones)
        #if self.is_logging_enabled and self.current_log_path:
            #self.get_logger().info('🔄 Path actualizado durante logging activo')
            # Opción 1: Guardar inmediatamente (crea nuevo archivo o sobrescribe)
            # Opción 2: Mantener el último y guardar al final (implementado abajo)
    
    def logging_enabled_cb(self, msg):
        """Callback para habilitar/deshabilitar logging"""
        if msg.data != self.is_logging_enabled:
            self.is_logging_enabled = msg.data
            
            if self.is_logging_enabled:
                #self.get_logger().info('🚀 Logging HABILITADO - Preparado para guardar path...')
                # No guardamos inmediatamente, esperamos a que se active el logging
                # El path se guardará cuando se reciba la ruta de logging
            #else:
                #self.get_logger().info('🛑 Logging DESHABILITADO')
                # Podríamos guardar el path final aquí si es necesario
                self._save_final_path()
    
    def log_path_cb(self, msg):
        """Callback para recibir la ruta de logging"""
        if msg.data != self.current_log_path:
            self.current_log_path = msg.data
            #self.get_logger().info(f'📁 Ruta de logging recibida: {self.current_log_path}')
            
            # Si tenemos datos de path y el logging está habilitado, guardamos
            if self.is_logging_enabled and self.last_path_data:
                #self.get_logger().info('💾 Guardando path global actual...')
                self._save_path_to_csv()
    
    def _save_final_path(self):
        """Guarda el path final cuando se detiene el logging"""
        if self.current_log_path and self.last_path_data:
            #self.get_logger().info('💾 Guardando path final...')
            
            # Crear nombre de archivo con sufijo "final"
            log_dir = PathLib(self.current_log_path)
            filename = log_dir / f"{self.node_name}_final.csv"
            
            self._write_path_to_file(filename, self.last_path_data, "final")
    
    def _save_path_to_csv(self):
        """Guarda el path actual a CSV"""
        if not self.current_log_path or not self.last_path_data:
            #self.get_logger().warning('No hay path o ruta para guardar')
            return
        
        try:
            log_dir = PathLib(self.current_log_path)
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # Nombre del archivo - usamos timestamp del path para unicidad
            if self.last_path_timestamp:
                timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.last_path_timestamp))
                filename = log_dir / f"{self.node_name}_{timestamp_str}.csv"
            else:
                filename = log_dir / f"{self.node_name}.csv"
            
            success = self._write_path_to_file(filename, self.last_path_data)
            
            if success:
                #self.get_logger().info(f'✅ Path guardado: {filename} ({len(self.last_path_data)} puntos)')
                self.has_new_path = False
                
                # Opcional: también guardar un archivo de metadatos del path
                self._save_path_metadata(log_dir)
            
        except Exception as e:
            #self.get_logger().error(f'❌ Error guardando path: {e}')
            pass
    
    def _write_path_to_file(self, filename, path_data, suffix=""):
        """Escribe los datos del path a un archivo CSV"""
        try:
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = [
                    'sequence_id', 'timestamp', 'x', 'y', 'z', 
                    'qx', 'qy', 'qz', 'qw', 'frame_id', 'pose_id'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(path_data)
            
            # Crear también un archivo resumen simple (opcional)
            self._create_summary_file(filename, path_data, suffix)
            
            return True
            
        except Exception as e:
            #self.get_logger().error(f'Error escribiendo archivo {filename}: {e}')
            return False
    
    def _create_summary_file(self, csv_filename, path_data, suffix=""):
        """Crea un archivo de resumen del path"""
        try:
            if not path_data:
                return
            
            # Calcular estadísticas básicas
            start_point = path_data[0]
            end_point = path_data[-1]
            
            # Calcular distancia total aproximada (línea quebrada)
            total_distance = 0.0
            for i in range(len(path_data) - 1):
                p1 = path_data[i]
                p2 = path_data[i + 1]
                dx = p2['x'] - p1['x']
                dy = p2['y'] - p1['y']
                dz = p2['z'] - p1['z']
                total_distance += (dx**2 + dy**2 + dz**2)**0.5
            
            # Crear archivo de resumen
            summary_filename = str(csv_filename).replace('.csv', '_summary.txt')
            
            with open(summary_filename, 'w') as f:
                f.write(f"=== RESUMEN DEL PATH GLOBAL ===\n")
                f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Sufijo: {suffix if suffix else 'principal'}\n")
                f.write(f"\n")
                f.write(f"ESTADÍSTICAS:\n")
                f.write(f"  Total puntos: {len(path_data)}\n")
                f.write(f"  Distancia total aproximada: {total_distance:.2f} m\n")
                f.write(f"  Frame ID: {self.path_frame_id}\n")
                f.write(f"\n")
                f.write(f"PUNTO DE INICIO:\n")
                f.write(f"  Coordenadas: [{start_point['x']:.3f}, {start_point['y']:.3f}, {start_point['z']:.3f}]\n")
                f.write(f"  Timestamp: {start_point['timestamp']:.6f} s\n")
                f.write(f"\n")
                f.write(f"PUNTO FINAL:\n")
                f.write(f"  Coordenadas: [{end_point['x']:.3f}, {end_point['y']:.3f}, {end_point['z']:.3f}]\n")
                f.write(f"  Timestamp: {end_point['timestamp']:.6f} s\n")
                f.write(f"\n")
                f.write(f"ARCHIVOS:\n")
                f.write(f"  CSV detallado: {PathLib(csv_filename).name}\n")
                f.write(f"  Resumen: {PathLib(summary_filename).name}\n")
            
            #self.get_logger().debug(f'Resumen creado: {summary_filename}')
            
        except Exception as e:
            #self.get_logger().error(f'Error creando resumen: {e}')
            pass
    
    def _save_path_metadata(self, log_dir):
        """Guarda metadatos adicionales del path"""
        try:
            metadata_file = log_dir / f"{self.node_name}_metadata.txt"
            
            with open(metadata_file, 'w') as f:
                f.write(f"=== METADATOS PATH GLOBAL ===\n")
                f.write(f"Fecha generación: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Nodo: {self.get_name()}\n")
                f.write(f"Topic: /global_path\n")
                f.write(f"Frame ID: {self.path_frame_id}\n")
                f.write(f"Total puntos: {len(self.last_path_data)}\n")
                f.write(f"Timestamp último path: {self.last_path_timestamp}\n")
                f.write(f"\n")
                f.write(f"FORMATO CSV:\n")
                f.write(f"  sequence_id: Índice secuencial en el path\n")
                f.write(f"  timestamp: Timestamp de cada pose\n")
                f.write(f"  x,y,z: Posición en metros\n")
                f.write(f"  qx,qy,qz,qw: Orientación (cuaternión)\n")
                f.write(f"  frame_id: Sistema de referencia\n")
                f.write(f"  pose_id: ID único de la pose\n")
            
        except Exception as e:
            #self.get_logger().error(f'Error guardando metadatos: {e}')
            pass
    
    def _check_and_save_on_shutdown(self):
        """Verifica si hay datos pendientes y los guarda"""
        if self.last_path_data and self.current_log_path:
            #self.get_logger().info('🔄 Guardando path pendiente al apagar...')
            self._save_path_to_csv()
    
    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        self._check_and_save_on_shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('Apagando GlobalPathRecorder...')
        pass
    except Exception as e:
        #node.get_logger().error(f'Error en GlobalPathRecorder: {e}')
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass 

if __name__ == '__main__':
    main()