#!/usr/bin/env python3
"""
websocket_bridge.py - Puente WebSocket para control remoto y monitoreo de diagnóstico.
Versión mejorada con manejo robusto de conexiones.
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
import traceback
from custom_interfaces.msg import LaneModel

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
        self.pub_motor = self.create_publisher(Bool, '/start/motors', 10)
        self.pub_light_stop = self.create_publisher(Bool, '/light/stop', 10)
        self.pub_light_left = self.create_publisher(Bool, '/light/left', 10)
        self.pub_light_right = self.create_publisher(Bool, '/light/right', 10)
        self.pub_light_safety = self.create_publisher(Bool, '/light/safety', 10)
        self.pub_capture = self.create_publisher(Bool, '/capture', 10)
        self.pub_manual = self.create_publisher(Bool, '/manual', 10)
        self.pub_emergency = self.create_publisher(Bool, '/emergency', 10)
        self.pub_stop = self.create_publisher(Bool, '/safe_stop', 10)
        self.pub_record = self.create_publisher(Bool, '/record', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)

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
            '/status/cpu_temperature',
            '/status/gpu_temperature',
            '/status/soc_temperature', 
            '/status/power_usage',
            '/status/input_voltage',
            '/status/gpu_usage',
            '/status/cpu_usage',
            '/status/ram_usage',
            '/status/fan_speed',
            
        ]
                # Publishers con el prefijo /status/ solicitado

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
            ('/alpha/vision', Float32, self.alpha_vision_callback),
            ('/active/vision', Bool, self.active_vision_callback),
            ('/cmd_vel', Twist, self.cmd_vel_callback),
            ('/odom/unfiltered', Odometry, self.odom_unfiltered_callback),
            ('/odometry/local', Odometry, self.odometry_local_callback),
            ('/odometry/global', Odometry, self.odometry_global_callback),
            ('/lane/model_filtered', LaneModel, self.lane_model_callback)
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
            self.get_logger().error(f"Error en servidor WebSocket: {e}")

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
                
                # Configurar servidor con parámetros optimizados
                async with websockets.serve(
                    handler, 
                    self.ws_host, 
                    port,
                    ping_interval=20,
                    ping_timeout=10,
                    close_timeout=5,
                    max_size=10_000_000  # 10 MB máximo
                ) as server:
                    
                    self.ws_port = port
                    #self.get_logger().info(f" Servidor WebSocket iniciado en ws://{ip_local}:{port}")
                    
                    # Tarea para procesar cola de mensajes
                    asyncio.create_task(self.process_message_queue())
                    
                    # Mantener el servidor corriendo
                    await server.wait_closed()
                    break
                    
            except OSError as e:
                if "address already in use" in str(e) and attempt < max_attempts - 1:
                    self.get_logger().warn(f"Puerto {port} ocupado, probando {port + 1}")
                    port += 1
                    await asyncio.sleep(1)
                else:
                    self.get_logger().error(f"No se pudo iniciar servidor: {e}")
                    return
            except Exception as e:
                self.get_logger().error(f"Error inesperado: {e}")
                return

    async def websocket_handler(self, websocket, path=None):
        """Manejador de conexiones WebSocket"""
        client_ip = websocket.remote_address[0] if websocket.remote_address else "desconocido"
        
        try:
            # Agregar conexión
            with self.connections_lock:
                self.connected_websockets.add(websocket)
            
            #self.get_logger().info(f" Cliente conectado desde {client_ip} (total: {len(self.connected_websockets)})")
            
            # Enviar estados iniciales con timeout
            await self.send_initial_data(websocket)
            
            # Procesar mensajes entrantes
            try:
                async for message in websocket:
                    await self.handle_websocket_message(websocket, message)
            except asyncio.TimeoutError:
                self.get_logger().debug(f"Timeout con cliente {client_ip}")
            except websockets.exceptions.ConnectionClosed as e:
                self.get_logger().debug(f"Conexión cerrada con {client_ip}: código {e.code}")
                    
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().debug(f"Cliente desconectado: {client_ip}")
        except Exception as e:
            self.get_logger().error(f"Error con cliente {client_ip}: {e}")
        finally:
            # Remover conexión
            with self.connections_lock:
                if websocket in self.connected_websockets:
                    self.connected_websockets.remove(websocket)
                    #self.get_logger().info(f"Cliente desconectado: {client_ip} (quedan {len(self.connected_websockets)})")

    async def send_initial_data(self, websocket):
        """Envía datos iniciales al cliente con timeout"""
        try:
            # Enviar diagnóstico
            diagnostic_data = self.get_diagnostic_data()
            if diagnostic_data:
                message = {
                    'type': 'diagnostic_status',
                    'data': diagnostic_data
                }
                await asyncio.wait_for(websocket.send(json.dumps(message)), timeout=5.0)
            
            # Enviar información adicional
            additional_data = self.get_additional_data()
            if additional_data:
                message = {
                    'type': 'additional_status',
                    'data': additional_data,
                    'timestamp': self.get_clock().now().nanoseconds
                }
                await asyncio.wait_for(websocket.send(json.dumps(message)), timeout=5.0)
                
        except asyncio.TimeoutError:
            self.get_logger().warning("Timeout enviando datos iniciales")
        except Exception as e:
            self.get_logger().debug(f"Error enviando datos iniciales: {e}")

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
        if mtype in ["motor", "light_stop", "light_left", 
                   "light_right", "light_safety", "capture", "emergency", "safe_stop", "record"]:
            val = bool(data.get("value", False))
            msg = Bool()
            msg.data = val
            
            if mtype == "motor":
                self.pub_motor.publish(msg)
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
            elif mtype == "safe_stop":
                self.pub_stop.publish(msg)
            elif mtype == "record":
                self.pub_record.publish(msg)
        
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
                await asyncio.wait_for(websocket.send(json.dumps(message)), timeout=5.0)
        except asyncio.TimeoutError:
            self.get_logger().debug("Timeout enviando diagnóstico")
        except Exception as e:
            self.get_logger().debug(f"Error enviando diagnóstico: {e}")

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
                await asyncio.wait_for(websocket.send(json.dumps(message)), timeout=5.0)
        except asyncio.TimeoutError:
            self.get_logger().debug("Timeout enviando adicional")
        except Exception as e:
            self.get_logger().debug(f"Error enviando adicional: {e}")

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
                pass
        
        return callback

    def goal_reached_callback(self, msg):
        self.additional_status['goal_reached'] = bool(msg.data)

    def alpha_vision_callback(self, msg):
        self.additional_status['alpha_vision'] = float(msg.data)

    def active_vision_callback(self, msg):
        self.additional_status['active_vision'] = bool(msg.data)

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
    
    def lane_model_callback(self, msg):
        """Callback para procesar el modelo de carril filtrado"""
        
        self.additional_status['lane_model'] = {
            'lane_width': float(msg.lane_width),
            'd_lat': float(msg.d_lat),
            'yaw': float(msg.yaw),
            'curvature': float(msg.curvature),
            'confidence': float(msg.confidence)
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