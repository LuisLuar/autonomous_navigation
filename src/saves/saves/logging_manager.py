#!/usr/bin/env python3
"""
Nodo Manager para Sistema de Logging de Robot Autónomo
Este nodo gestiona la creación de carpetas y envía señales de inicio/parada
a todos los nodos de logging basado en condiciones específicas.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import os
import datetime
import time
from pathlib import Path

class LoggingManager(Node):
    def __init__(self):
        super().__init__('logging_manager')
        
        # Parámetros configurables
        self.declare_parameter('base_log_dir', '~/autonomous_navigation/src/saves/data_logs')
        self.declare_parameter('log_prefix', 'ruta')
        self.declare_parameter('enable_auto_naming', True)
        self.declare_parameter('manual_route_name', '')
        
        # Obtener parámetros
        self.base_log_dir = Path(self.get_parameter('base_log_dir').value).expanduser()
        self.log_prefix = self.get_parameter('log_prefix').value
        self.enable_auto_naming = self.get_parameter('enable_auto_naming').value
        self.manual_route_name = self.get_parameter('manual_route_name').value
        
        # Estado interno
        self.is_logging = False
        self.current_log_dir = None
        self.lid_closed = False
        self.active_osm = False
        self.goal_reached = False
        
        # Variables para detectar transiciones
        self.previous_lid_closed = False  # Estado anterior de lid_closed
        self.lid_closed_transition = False  # Indica si hubo transición F->T
        
        # Contador para nombres únicos
        self.route_counter = self._get_next_route_number()
        
        # Publishers
        self.logging_signal_pub = self.create_publisher(Bool, '/logging_enabled', 10)
        self.logging_path_pub = self.create_publisher(String, '/current_log_path', 10)
        
        # Subscribers
        self.create_subscription(Bool, '/lid_closed', self.lid_closed_callback, 10)
        self.create_subscription(Bool, '/active/osm', self.active_osm_callback, 10)
        self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        
        # Timer para chequeo periódico
        self.timer = self.create_timer(0.1, self.check_conditions)  # 10 Hz
        
        ##self.get_logger().info(f'Logging Manager inicializado. Directorio base: {self.base_log_dir}')
        #self.get_logger().info('Esperando condiciones para iniciar logging...')
        
    def _get_next_route_number(self):
        """Obtiene el siguiente número de ruta basado en carpetas existentes"""
        if not self.base_log_dir.exists():
            return 1
            
        existing_folders = [d for d in self.base_log_dir.iterdir() if d.is_dir()]
        max_num = 0
        
        for folder in existing_folders:
            folder_name = folder.name
            # Busca patrones como "ruta1", "ruta2_", etc.
            if folder_name.startswith(self.log_prefix):
                try:
                    # Extrae el número después del prefijo
                    remaining = folder_name[len(self.log_prefix):]
                    # Encuentra el primer número (puede haber sufijos)
                    num_str = ''
                    for char in remaining:
                        if char.isdigit():
                            num_str += char
                        elif num_str:  # Si ya empezamos números y encontramos no-dígito
                            break
                    if num_str:
                        num = int(num_str)
                        max_num = max(max_num, num)
                except (ValueError, IndexError):
                    continue
        
        return max_num + 1
    
    def _generate_route_name(self):
        """Genera un nombre único para la carpeta de ruta"""
        if not self.enable_auto_naming and self.manual_route_name:
            base_name = self.manual_route_name
        else:
            base_name = f"{self.log_prefix}{self.route_counter}"
        
        # Agregar timestamp para unicidad
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Verificar si ya existe y agregar sufijo si es necesario
        candidate_name = f"{base_name}_{timestamp}"
        full_path = self.base_log_dir / candidate_name
        
        counter = 1
        while full_path.exists():
            candidate_name = f"{base_name}_{timestamp}_{counter}"
            full_path = self.base_log_dir / candidate_name
            counter += 1
            if counter > 100:  # Límite de seguridad
                raise RuntimeError("No se pudo generar nombre único para la carpeta")
        
        return candidate_name
    
    def _create_log_directory(self, route_name):
        """Crea la carpeta de logging con subcarpetas si es necesario"""
        log_dir = self.base_log_dir / route_name
        
        try:
            log_dir.mkdir(parents=True, exist_ok=False)
            
            # Crear archivo de metadatos
            metadata_file = log_dir / "metadata.txt"
            with open(metadata_file, 'w') as f:
                f.write(f"Ruta: {route_name}\n")
                f.write(f"Fecha: {datetime.datetime.now().isoformat()}\n")
                f.write(f"Manager Node: {self.get_name()}\n")
                f.write("Estado inicial:\n")
                f.write(f"  lid_closed: {self.lid_closed}\n")
                f.write(f"  active_osm: {self.active_osm}\n")
                f.write(f"  goal_reached: {self.goal_reached}\n")
            
            #self.get_logger().info(f'Directorio de logging creado: {log_dir}')
            return log_dir
            
        except FileExistsError:
            #self.get_logger().error(f'El directorio {log_dir} ya existe')
            return None
        except Exception as e:
            #self.get_logger().error(f'Error creando directorio: {e}')
            return None
    
    def lid_closed_callback(self, msg):
        """Callback para el topic /lid_closed"""
        new_value = msg.data
        
        # Detectar transición de False a True
        if not self.previous_lid_closed and new_value:
            self.lid_closed_transition = True
            #self.get_logger().info('📈 Transición detectada: lid_closed False -> True')
        elif self.previous_lid_closed and not new_value:
            #self.get_logger().info('📉 Transición detectada: lid_closed True -> False')
            pass
        
        # Actualizar estados
        self.previous_lid_closed = self.lid_closed
        self.lid_closed = new_value
        
    def active_osm_callback(self, msg):
        """Callback para el topic /active/osm"""
        self.active_osm = msg.data
        
    def goal_reached_callback(self, msg):
        """Callback para el topic /goal_reached"""
        self.goal_reached = msg.data
        
    def start_logging(self):
        """Inicia el proceso de logging"""
        if self.is_logging:
            return False
        
        try:
            # Generar nombre y crear carpeta
            route_name = self._generate_route_name()
            self.current_log_dir = self._create_log_directory(route_name)
            
            if not self.current_log_dir:
                return False
            
            # Publicar ruta actual a todos los nodos
            path_msg = String()
            path_msg.data = str(self.current_log_dir)
            self.logging_path_pub.publish(path_msg)
            
            # Esperar un momento para que los nodos reciban la ruta
            time.sleep(0.1)
            
            # Enviar señal de inicio
            self._publish_logging_signal(True)
            self.is_logging = True
            self.route_counter += 1
            
            # Resetear flag de transición
            self.lid_closed_transition = False
            
            #self.get_logger().info('=== LOGGING INICIADO ===')
            #self.get_logger().info(f'Ruta: {route_name}')
            #self.get_logger().info(f'Directorio: {self.current_log_dir}')
            
            return True
            
        except Exception as e:
            #self.get_logger().error(f'Error iniciando logging: {e}')
            return False
    
    def stop_logging(self):
        """Detiene el proceso de logging"""
        if not self.is_logging:
            #self.get_logger().warning('Logging no está activo')
            return False
        
        try:
            # Enviar señal de parada
            self._publish_logging_signal(False)
            self.is_logging = False
            
            # Actualizar archivo de metadatos con información final
            if self.current_log_dir:
                metadata_file = self.current_log_dir / "metadata.txt"
                with open(metadata_file, 'a') as f:
                    f.write(f"\nLogging finalizado: {datetime.datetime.now().isoformat()}\n")
                    f.write(f"Motivo: {'goal_reached' if self.goal_reached else 'condicion_falsa'}\n")
                    f.write(f"Estado final:\n")
                    f.write(f"  lid_closed: {self.lid_closed}\n")
                    f.write(f"  active_osm: {self.active_osm}\n")
                    f.write(f"  goal_reached: {self.goal_reached}\n")
            
            #self.get_logger().info('=== LOGGING DETENIDO ===')
            if self.current_log_dir:
                #self.get_logger().info(f'Datos guardados en: {self.current_log_dir}')
                pass
            
            self.current_log_dir = None
            return True
            
        except Exception as e:
            #self.get_logger().error(f'Error deteniendo logging: {e}')
            return False
    
    def _publish_logging_signal(self, enable):
        """Publica la señal de logging a todos los nodos"""
        msg = Bool()
        msg.data = enable
        self.logging_signal_pub.publish(msg)
        #self.get_logger().debug(f'Señal de logging publicada: {enable}')
    
    def check_conditions(self):
        """Verifica condiciones periódicamente para iniciar/detener logging"""
        
        # CONDICIÓN PARA INICIAR: 
        # 1. Hubo transición de False a True en lid_closed EN ESTE CICLO
        # 2. active_osm está en True
        # 3. No estamos ya logging
        #self.active_osm and              # active_osm debe estar True
        start_condition = (
            self.lid_closed_transition and  # Transición F->T detectada
            self.lid_closed and              # lid_closed debe estar True (redundante pero segura)
            not self.is_logging              # No debe estar ya logging
        )
        
        # CONDICIONES PARA DETENER: 
        # 1. lid_closed pasa a False
        # 2. active_osm pasa a False  
        # 3. goal_reached llega a True
        #(not self.active_osm) or 
        stop_condition = (
            (not self.lid_closed) or             
            self.goal_reached
        )
        
        # Lógica de transición de estados
        if start_condition:
            #self.get_logger().info('🚀 Condiciones de inicio cumplidas:')
            #self.get_logger().info(f'   - Transición lid_closed F->T: {self.lid_closed_transition}')
            #self.get_logger().info(f'   - active_osm: {self.active_osm}')
            #self.get_logger().info(f'   - lid_closed actual: {self.lid_closed}')
            #self.get_logger().info('Iniciando logging...')
            self.start_logging()
            
        elif self.is_logging and stop_condition:
            reason = []
            if not self.lid_closed:
                reason.append('lid_closed=False')
            if not self.active_osm:
                reason.append('active_osm=False')
            if self.goal_reached:
                reason.append('goal_reached=True')
            
            #self.get_logger().info(f'🛑 Condición de parada detectada: {", ".join(reason)}')
            self.stop_logging()
    
    def destroy_node(self):
        """Cleanup al destruir el nodo"""
        if self.is_logging:
            self.stop_logging()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        logging_manager = LoggingManager()
        rclpy.spin(logging_manager)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if 'logging_manager' in locals():
                logging_manager.destroy_node()
            rclpy.shutdown()
        except:
            pass 

if __name__ == '__main__':
    main()