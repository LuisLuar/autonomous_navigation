#!/usr/bin/env python3
"""
websocket_bridge.py - Puente WebSocket para control remoto y monitoreo de diagnóstico.
Versión con manejo correcto del handler.
"""
import asyncio
import json
import os
import websockets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus
import socket
import netifaces
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import threading
import time
import functools

LINEAR_SCALE = float(os.environ.get("LINEAR_SCALE", "1.0"))
ANGULAR_SCALE = float(os.environ.get("ANGULAR_SCALE", "1.0"))
WS_HOST = os.environ.get("WS_HOST", "0.0.0.0")
WS_PORT = int(os.environ.get("WS_PORT", "8765"))

def obtener_ip_local():
    """Obtiene la IP local de la máquina"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_socket = s.getsockname()[0]
        s.close()
        if ip_socket and not ip_socket.startswith("127."):
            return ip_socket
    except Exception:
        pass
    
    try:
        interfaces = netifaces.interfaces()
        for interface in interfaces:
            if interface == 'lo':
                continue
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for link in addrs[netifaces.AF_INET]:
                    ip = link['addr']
                    if (ip and not ip.startswith("127.") and not ip.startswith("169.254.")):
                        return ip
    except Exception:
        pass
    
    return "127.0.0.1"

class WebSocketROSBridge(Node):
    def __init__(self):
        super().__init__('websocket')
        
        self.ws_host = WS_HOST
        self.ws_port = WS_PORT
        
        # Publishers
        self.pub_motor_left = self.create_publisher(Bool, '/start/motor_left', 10)
        self.pub_motor_right = self.create_publisher(Bool, '/start/motor_right', 10)
        self.pub_light_stop = self.create_publisher(Bool, '/light/stop', 10)
        self.pub_light_left = self.create_publisher(Bool, '/light/left', 10)
        self.pub_light_right = self.create_publisher(Bool, '/light/right', 10)
        self.pub_light_safety = self.create_publisher(Bool, '/light/safety', 10)
        self.pub_capture = self.create_publisher(Bool, '/capture', 10)
        self.pub_manual = self.create_publisher(Bool, '/manual', 10)
        self.pub_emergency = self.create_publisher(Bool, '/emergency', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publicación inicial
        self.pub_manual.publish(Bool(data=False))

        # Estados compartidos
        self.diagnostic_status = {}
        self.additional_status = {}
        
        # Conexiones WebSocket
        self.connected_websockets = set()
        self.connections_lock = threading.Lock()
        
        # Control de rate limiting
        self.last_diagnostic_send = 0
        self.last_additional_send = 0
        self.diagnostic_update_interval = 2.0  # 0.5 Hz
        self.additional_update_interval = 0.1  # 10 Hz
        
        # Cola de mensajes para enviar a clientes
        self.message_queue = []
        self.queue_lock = threading.Lock()
        
        # Flag para controlar el servidor
        self.server_running = True
        
        # Event loop del servidor
        self.server_loop = None
        
        # Suscripciones a temas de diagnóstico
        diagnostic_topics = [
            '/status/battery_12v',
            '/status/camera', 
            '/status/esp32_control',
            '/status/esp32_safety',
            '/status/gps',
            '/status/microros_agent',
            '/status/motor_left',
            '/status/motor_right',
            '/status/rplidar',
            '/status/voltage_5v',
            '/status/cpu_temperature',
            '/status/gpu_temperature',
            '/status/battery_laptop',
            '/status/ram',
            '/status/cpu_usage',
            '/status/gpu_usage',
            '/status/disk_temperature',
            '/status/uptime'
        ]
        
        for topic in diagnostic_topics:
            self.create_subscription(
                DiagnosticStatus,
                topic,
                self.create_diagnostic_callback(topic),
                10
            )

        # Suscripciones adicionales para información de navegación
        additional_topics = [
            ('/goal_reached', Bool, self.goal_reached_callback),
            ('/omega/lane', Float32, self.omega_lane_callback),
            ('/active/lane', Bool, self.active_lane_callback),
            ('/omega/planner', Float32, self.omega_planner_callback),
            ('/active/planner', Bool, self.active_planner_callback),
            ('/omega/lidar', Float32, self.omega_lidar_callback),
            ('/active/lidar_lateral', Bool, self.active_lidar_lateral_callback),
            ('/alpha/lidar', Float32, self.alpha_lidar_callback),
            ('/active/lidar_front', Bool, self.active_lidar_front_callback),
            ('/alpha/vision', Float32, self.alpha_vision_callback),
            ('/active/vision', Bool, self.active_vision_callback),
            ('/alpha/osm', Float32, self.alpha_osm_callback),
            ('/active/osm', Bool, self.active_osm_callback),
            ('/alpha/osm_obstacles', Float32, self.alpha_osm_obstacles_callback),
            ('/active/osm_obstacles', Bool, self.active_osm_obstacles_callback),
            ('/cmd_vel', Twist, self.cmd_vel_callback),
            ('/odom/unfiltered', Odometry, self.odom_unfiltered_callback),
            ('/odometry/local', Odometry, self.odometry_local_callback),
            ('/odometry/global', Odometry, self.odometry_global_callback)
        ]

        for topic, msg_type, callback in additional_topics:
            self.create_subscription(msg_type, topic, callback, 10)
        
        # Iniciar servidor WebSocket en un thread separado
        self.server_thread = threading.Thread(target=self.run_websocket_server, daemon=True)
        self.server_thread.start()
        
        # Timer para enviar actualizaciones
        self.create_timer(0.05, self.update_timer_callback)  # 20 Hz para checkear
        
        #self.get_logger().info("WebSocket bridge iniciado")

    def run_websocket_server(self):
        """Ejecuta el servidor WebSocket en un thread separado"""
        try:
            # Crear un nuevo event loop para este thread
            self.server_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.server_loop)
            
            # Ejecutar el servidor
            self.server_loop.run_until_complete(self.websocket_server_main())
        except Exception as e:
            #self.get_logger().error(f"Error en servidor WebSocket: {e}")
            pass

    async def websocket_server_main(self):
        """Función principal del servidor WebSocket"""
        ip_local = obtener_ip_local()
        
        # Intentar diferentes puertos si el predeterminado está ocupado
        port = self.ws_port
        max_attempts = 5
        
        for attempt in range(max_attempts):
            try:
                #self.get_logger().info(f"Intentando iniciar WebSocket en ws://{ip_local}:{port}")
                
                # Crear handler con functools.partial para bindear 'self'
                handler = functools.partial(self.websocket_handler)
                
                # Configurar servidor
                async with websockets.serve(
                    handler, 
                    self.ws_host, 
                    port,
                    ping_interval=30,
                    ping_timeout=30
                ) as server:
                    
                    self.ws_port = port
                    #self.get_logger().info(f"✅ Servidor WebSocket iniciado en ws://{ip_local}:{port}")
                    
                    # Tarea para procesar cola de mensajes
                    asyncio.create_task(self.process_message_queue())
                    
                    # Mantener el servidor corriendo
                    await server.wait_closed()
                    break
                    
            except OSError as e:
                if "address already in use" in str(e) and attempt < max_attempts - 1:
                    #self.get_logger().warn(f"Puerto {port} ocupado. Probando puerto {port + 1}...")
                    port += 1
                    await asyncio.sleep(1)
                else:
                    #self.get_logger().error(f"❌ No se pudo iniciar servidor WebSocket: {e}")
                    return
            except Exception as e:
                #self.get_logger().error(f"❌ Error inesperado: {e}")
                return

    async def websocket_handler(self, websocket, path=None):
        """Manejador de conexiones WebSocket"""
        client_ip = websocket.remote_address[0] if websocket.remote_address else "desconocido"
        
        try:
            # Agregar conexión
            with self.connections_lock:
                self.connected_websockets.add(websocket)
            
            #self.get_logger().info(f"👤 Cliente conectado desde {client_ip}")
            
            # Enviar estados iniciales
            await self.send_initial_data(websocket)
            
            # Procesar mensajes entrantes
            async for message in websocket:
                await self.handle_websocket_message(websocket, message)
                    
        except websockets.exceptions.ConnectionClosed:
            #self.get_logger().info(f"📴 Cliente desconectado: {client_ip}")
            pass
        except Exception as e:
            #self.get_logger().debug(f"Error en handler: {e}")
            pass
        finally:
            # Remover conexión
            with self.connections_lock:
                if websocket in self.connected_websockets:
                    self.connected_websockets.remove(websocket)

    async def send_initial_data(self, websocket):
        """Envía datos iniciales al cliente"""
        try:
            # Enviar diagnóstico
            diagnostic_data = self.get_diagnostic_data()
            if diagnostic_data:
                message = {
                    'type': 'diagnostic_status',
                    'data': diagnostic_data
                }
                await websocket.send(json.dumps(message))
            
            # Enviar información adicional
            additional_data = self.get_additional_data()
            if additional_data:
                message = {
                    'type': 'additional_status',
                    'data': additional_data,
                    'timestamp': self.get_clock().now().nanoseconds
                }
                await websocket.send(json.dumps(message))
                
        except Exception as e:
            #self.get_logger().debug(f"Error enviando datos iniciales: {e}")
            pass

    async def handle_websocket_message(self, websocket, message):
        """Procesa un mensaje WebSocket entrante"""
        if isinstance(message, bytes):
            return
            
        try:
            data = json.loads(message)
        except Exception:
            return

        mtype = data.get("type", "")
        
        # Booleanos
        if mtype in ["motor_left", "motor_right", "light_stop", "light_left", 
                   "light_right", "light_safety", "capture", "emergency"]:
            val = bool(data.get("value", False))
            msg = Bool()
            msg.data = val
            
            if mtype == "motor_left":
                self.pub_motor_left.publish(msg)
            elif mtype == "motor_right":
                self.pub_motor_right.publish(msg)
            elif mtype == "light_stop":
                self.pub_light_stop.publish(msg)
            elif mtype == "light_left":
                self.pub_light_left.publish(msg)
            elif mtype == "light_right":
                self.pub_light_right.publish(msg)
            elif mtype == "light_safety":
                self.pub_light_safety.publish(msg)
            elif mtype == "capture":
                self.pub_capture.publish(msg)
            elif mtype == "emergency":
                self.pub_emergency.publish(msg)
        
        # Joystick
        elif mtype == "joy":
            try:
                x = float(data.get("x", 0.0))
                y = float(data.get("y", 0.0))
            except Exception:
                x, y = 0.0, 0.0
                
            twist = Twist()
            twist.linear.x = x * LINEAR_SCALE
            twist.angular.z = y * ANGULAR_SCALE
            self.cmd_vel_pub.publish(twist)

            if x != 0.0 or y != 0.0:
                self.pub_manual.publish(Bool(data=True))
            else:
                self.pub_manual.publish(Bool(data=False))
        
        # Comando directo
        elif mtype == "cmd_vel":
            try:
                linear = float(data.get("linear", 0.0))
                angular = float(data.get("angular", 0.0))
            except Exception:
                linear, angular = 0.0, 0.0
                
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.cmd_vel_pub.publish(twist)
        
        # Solicitudes especiales
        elif mtype == "request_diagnostic":
            await self.send_diagnostic_to_client(websocket)
        elif mtype == "request_additional":
            await self.send_additional_to_client(websocket)

    async def send_diagnostic_to_client(self, websocket):
        """Envía datos de diagnóstico a un cliente específico"""
        try:
            diagnostic_data = self.get_diagnostic_data()
            if diagnostic_data:
                message = {
                    'type': 'diagnostic_status',
                    'data': diagnostic_data
                }
                await websocket.send(json.dumps(message))
        except Exception as e:
            #self.get_logger().debug(f"Error enviando diagnóstico: {e}")
            pass

    async def send_additional_to_client(self, websocket):
        """Envía datos adicionales a un cliente específico"""
        try:
            additional_data = self.get_additional_data()
            if additional_data:
                message = {
                    'type': 'additional_status',
                    'data': additional_data,
                    'timestamp': self.get_clock().now().nanoseconds
                }
                await websocket.send(json.dumps(message))
        except Exception as e:
            #self.get_logger().debug(f"Error enviando adicional: {e}")
            pass

    # =====================================================
    # Callbacks - SOLO actualizan datos, NO envían
    # =====================================================
    def create_diagnostic_callback(self, topic):
        def callback(msg):
            try:
                level_int = int(msg.level) if not isinstance(msg.level, bytes) else int.from_bytes(msg.level, 'little')
                
                level_str = "OK"
                if level_int == 1:
                    level_str = "WARNING"
                elif level_int == 2:
                    level_str = "ERROR"
                elif level_int == 3:
                    level_str = "STALE"
                
                self.diagnostic_status[topic] = {
                    'name': msg.name,
                    'level': level_str,
                    'message': msg.message,
                    'timestamp': self.get_clock().now().nanoseconds
                }
            except Exception as e:
                #self.get_logger().debug(f"Error en callback de diagnóstico: {e}")
                pass
        
        return callback

    def goal_reached_callback(self, msg):
        self.additional_status['goal_reached'] = bool(msg.data)

    def omega_lane_callback(self, msg):
        self.additional_status['omega_lane'] = float(msg.data)

    def active_lane_callback(self, msg):
        self.additional_status['active_lane'] = bool(msg.data)

    def omega_planner_callback(self, msg):
        self.additional_status['omega_planner'] = float(msg.data)

    def active_planner_callback(self, msg):
        self.additional_status['active_planner'] = bool(msg.data)

    def omega_lidar_callback(self, msg):
        self.additional_status['omega_lidar'] = float(msg.data)

    def active_lidar_lateral_callback(self, msg):
        self.additional_status['active_lidar_lateral'] = bool(msg.data)

    def alpha_lidar_callback(self, msg):
        self.additional_status['alpha_lidar'] = float(msg.data)

    def active_lidar_front_callback(self, msg):
        self.additional_status['active_lidar_front'] = bool(msg.data)

    def alpha_vision_callback(self, msg):
        self.additional_status['alpha_vision'] = float(msg.data)

    def active_vision_callback(self, msg):
        self.additional_status['active_vision'] = bool(msg.data)

    def alpha_osm_callback(self, msg):
        self.additional_status['alpha_osm'] = float(msg.data)

    def active_osm_callback(self, msg):
        self.additional_status['active_osm'] = bool(msg.data)
    
    def alpha_osm_obstacles_callback(self, msg):
        self.additional_status['alpha_osm_obstacles'] = float(msg.data)

    def active_osm_obstacles_callback(self, msg):
        self.additional_status['active_osm_obstacles'] = bool(msg.data)

    def cmd_vel_callback(self, msg):
        self.additional_status['cmd_vel'] = {
            'linear_x': float(msg.linear.x),
            'angular_z': float(msg.angular.z)
        }

    def odom_unfiltered_callback(self, msg):
        self.additional_status['odom_unfiltered'] = {
            'linear_x': float(msg.twist.twist.linear.x),
            'angular_z': float(msg.twist.twist.angular.z)
        }

    def odometry_local_callback(self, msg):
        self.additional_status['odometry_local'] = {
            'linear_x': float(msg.twist.twist.linear.x),
            'angular_z': float(msg.twist.twist.angular.z)
        }

    def odometry_global_callback(self, msg):
        self.additional_status['odometry_global'] = {
            'linear_x': float(msg.twist.twist.linear.x),
            'angular_z': float(msg.twist.twist.angular.z)
        }

    # =====================================================
    # Funciones de utilidad
    # =====================================================
    def update_timer_callback(self):
        """Timer que checkea cuándo enviar actualizaciones"""
        current_time = time.time()
        
        # Enviar diagnóstico periódicamente
        if current_time - self.last_diagnostic_send >= self.diagnostic_update_interval:
            self.queue_diagnostic_message()
            self.last_diagnostic_send = current_time
        
        # Enviar información adicional más frecuentemente
        if current_time - self.last_additional_send >= self.additional_update_interval:
            self.queue_additional_message()
            self.last_additional_send = current_time

    def get_diagnostic_data(self):
        """Obtiene datos de diagnóstico formateados"""
        diagnostic_list = []
        for topic, status in self.diagnostic_status.items():
            diagnostic_list.append({
                'topic': topic,
                'name': status['name'],
                'level': status['level'],
                'message': status['message']
            })
        return diagnostic_list

    def get_additional_data(self):
        """Obtiene datos adicionales formateados"""
        # Crear copia para evitar problemas de concurrencia
        return self.additional_status.copy()

    def queue_diagnostic_message(self):
        """Pone mensaje de diagnóstico en la cola"""
        diagnostic_data = self.get_diagnostic_data()
        if not diagnostic_data:
            return
            
        message = {
            'type': 'diagnostic_status',
            'data': diagnostic_data
        }
        
        with self.queue_lock:
            self.message_queue.append(('diagnostic', message))

    def queue_additional_message(self):
        """Pone mensaje adicional en la cola"""
        additional_data = self.get_additional_data()
        if not additional_data:
            return
            
        message = {
            'type': 'additional_status',
            'data': additional_data,
            'timestamp': self.get_clock().now().nanoseconds
        }
        
        with self.queue_lock:
            self.message_queue.append(('additional', message))

    async def process_message_queue(self):
        """Procesa la cola de mensajes y los envía a los clientes"""
        while self.server_running:
            try:
                # Obtener mensajes de la cola
                messages_to_send = []
                with self.queue_lock:
                    if self.message_queue:
                        messages_to_send = self.message_queue.copy()
                        self.message_queue.clear()
                
                # Si no hay mensajes, esperar un poco
                if not messages_to_send:
                    await asyncio.sleep(0.01)
                    continue
                
                # Obtener conexiones activas
                with self.connections_lock:
                    connections = list(self.connected_websockets)
                
                if not connections:
                    continue
                
                # Enviar cada mensaje a todas las conexiones
                for msg_type, message in messages_to_send:
                    message_json = json.dumps(message)
                    
                    # Enviar a cada conexión
                    disconnected = []
                    for websocket in connections:
                        try:
                            await websocket.send(message_json)
                        except (websockets.exceptions.ConnectionClosed, 
                               websockets.exceptions.WebSocketException):
                            disconnected.append(websocket)
                        except Exception:
                            disconnected.append(websocket)
                    
                    # Limpiar conexiones desconectadas
                    if disconnected:
                        with self.connections_lock:
                            for ws in disconnected:
                                if ws in self.connected_websockets:
                                    self.connected_websockets.remove(ws)
                
                # Pequeña pausa para no saturar
                await asyncio.sleep(0.005)
                
            except Exception as e:
                #self.get_logger().debug(f"Error procesando cola: {e}")
                await asyncio.sleep(0.1)

    def destroy_node(self):
        """Limpia recursos"""
        self.server_running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WebSocketROSBridge()
        
        # Ejecutar ROS en el thread principal
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()