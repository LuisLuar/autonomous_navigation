#!/usr/bin/env python3
"""
websocket_bridge.py - Puente WebSocket para control remoto y monitoreo de diagnóstico.
Permite controlar el robot mediante mensajes WebSocket y envía estados de diagnóstico en tiempo real.
Características principales:
 - Publica comandos de movimiento y control basados en mensajes WebSocket entrantes
 - Envía estados de diagnóstico ROS2 a todos los clientes WebSocket conectados
"""
import asyncio
import json
import os
import websockets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import inspect
import socket
import netifaces

LINEAR_SCALE = float(os.environ.get("LINEAR_SCALE", "0.5"))
ANGULAR_SCALE = float(os.environ.get("ANGULAR_SCALE", "1.0"))
WS_HOST = os.environ.get("WS_HOST", "192.168.100.169")
WS_PORT = int(os.environ.get("WS_PORT", "8765"))

def obtener_ip_local():
    """Obtiene la IP local de la máquina - versión mejorada para hotspot"""
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_socket = s.getsockname()[0]
        s.close()
        if ip_socket and not ip_socket.startswith("127."):
            ips.append(ip_socket)
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
                        ips.append(ip)
    except Exception as e:
        #print(f"Error con netifaces: {e}")
        pass
    
    for ip in ips:
        if ip.startswith("192.168.") or ip.startswith("10.") or ip.startswith("172."):
            return ip
    
    for ip in ips:
        if not ip.startswith("127."):
            return ip
    
    return ips[0] if ips else "127.0.0.1"

def obtener_todas_las_ips():
    """Obtiene todas las IPs disponibles para debugging"""
    todas_ips = []
    try:
        interfaces = netifaces.interfaces()
        for interface in interfaces:
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for link in addrs[netifaces.AF_INET]:
                    ip_info = {
                        'interface': interface,
                        'ip': link['addr'],
                        'netmask': link.get('netmask', 'N/A')
                    }
                    todas_ips.append(ip_info)
    except Exception as e:
        #print(f"Error obteniendo todas las IPs: {e}")
        pass
    return todas_ips

class WebSocketROSBridge(Node):
    def __init__(self):
        super().__init__('websocket')
        global WS_HOST
        
        ip_local = obtener_ip_local()
        todas_ips = obtener_todas_las_ips()
        
        #self.get_logger().info("=== DETECCIÓN DE RED ===")
        #for ip_info in todas_ips:
            #self.get_logger().info(f"Interfaz: {ip_info['interface']} -> IP: {ip_info['ip']}")
        
        if ip_local and not ip_local.startswith("127."):
            WS_HOST = ip_local
            #self.get_logger().info(f"IP seleccionada para WebSocket: {WS_HOST}")
        else:
            #self.get_logger().error("No se pudo detectar una IP válida")
            pass

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

        #Publicación inicial
        self.pub_manual.publish(Bool(data=False))

        # Lista de conexiones WebSocket activas
        self.connected_websockets = set()
        
        # Estado de diagnóstico
        self.diagnostic_status = {}
        
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
        
        try:
            self.asyncio_loop = asyncio.get_event_loop()
        except RuntimeError:
            self.asyncio_loop = None
        
        #self.get_logger().info(f"Inicializando WebSocket ws://{WS_HOST}:{WS_PORT}")
        asyncio.ensure_future(self.start_server())

    def create_diagnostic_callback(self, topic):
        """Crea un callback para cada tema de diagnóstico"""
        def callback(msg):
            try:
                # DEBUG: Mostrar información completa del mensaje
                #self.get_logger().info(f"Mensaje recibido en {topic}:")
                #self.get_logger().info(f"   - Name: {msg.name}")
                #self.get_logger().info(f"   - Level: {msg.level} (raw type: {type(msg.level)})")
                #self.get_logger().info(f"   - Message: {msg.message}")
                
                # CONVERTIR EL NIVEL CORRECTAMENTE - EL PROBLEMA ESTÁ AQUÍ
                # El nivel viene como bytes (b'\x02') en lugar de int
                level_int = 0
                if isinstance(msg.level, bytes):
                    # Convertir bytes a int
                    level_int = int.from_bytes(msg.level, byteorder='little', signed=False)
                    #self.get_logger().info(f"   - Level convertido desde bytes: {level_int}")
                else:
                    # Ya es un int
                    level_int = int(msg.level)
                    #self.get_logger().info(f"   - Level como int: {level_int}")
                
                # Convertir el nivel a string legible según el estándar ROS2 Diagnostic
                # DiagnosticStatus levels: 0=OK, 1=WARN, 2=ERROR, 3=STALE
                level_str = "OK"
                if level_int == 1:
                    level_str = "WARNING"
                elif level_int == 2:
                    level_str = "ERROR"
                elif level_int == 3:
                    level_str = "STALE"
                
                #self.get_logger().info(f"   - Level final: {level_str}")
                
                # Actualizar estado
                self.diagnostic_status[topic] = {
                    'name': msg.name,
                    'level': level_str,
                    'message': msg.message,
                    'timestamp': self.get_clock().now().nanoseconds
                }
                

                # Programar broadcast en el event loop
                if self.asyncio_loop and self.asyncio_loop.is_running():
                    # Método thread-safe para agregar tareas al event loop
                    asyncio.run_coroutine_threadsafe(
                        self.broadcast_diagnostic_status(),
                        self.asyncio_loop
                    )
                else:
                    # Fallback
                    # Enviar actualización a todos los clientes conectados
                    asyncio.ensure_future(self.broadcast_diagnostic_status())
          
            except Exception as e:
                #self.get_logger().error(f"Error en callback de diagnóstico {topic}: {e}")
                pass
        return callback

    async def broadcast_diagnostic_status(self):
        """Envía el estado de diagnóstico a todos los clientes conectados"""
        if not self.connected_websockets:
            return
            
        try:
            diagnostic_list = []
            for topic, status in self.diagnostic_status.items():
                diagnostic_list.append({
                    'topic': topic,
                    'name': status['name'],
                    'level': status['level'],
                    'message': status['message']
                })
            
            message = {
                'type': 'diagnostic_status',
                'data': diagnostic_list
            }
            
            # DEBUG: Mostrar qué se está enviando
            #self.get_logger().info(f"Enviando diagnóstico: {len(diagnostic_list)} items")
            #for item in diagnostic_list:
                #self.get_logger().info(f"   - {item['name']}: {item['level']} - {item['message']}")
            
            await self.broadcast(json.dumps(message))
            
        except Exception as e:
            #self.get_logger().error(f"Error enviando diagnóstico: {e}")
            pass

    async def broadcast(self, message):
        """Envía un mensaje a todos los clientes WebSocket conectados"""
        if not self.connected_websockets:
            return
            
        disconnected = set()
        for websocket in self.connected_websockets:
            try:
                await websocket.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(websocket)
            except Exception as e:
                #self.get_logger().error(f"Error enviando mensaje: {e}")
                disconnected.add(websocket)
        
        # Remover conexiones desconectadas
        self.connected_websockets -= disconnected

    async def handler(self, websocket):
        try:
            remote = getattr(websocket, "remote_address", None)
            path = getattr(websocket, "path", None)
            #self.get_logger().info(f"Cliente conectado: {remote} path={path}")
            
            # Agregar a la lista de conexiones activas
            self.connected_websockets.add(websocket)
            
            # Enviar estado actual de diagnóstico al nuevo cliente
            await self.broadcast_diagnostic_status()
            
            async for message in websocket:
                if isinstance(message, bytes):
                    #self.get_logger().warn("Recibido binario (ignorando).")
                    continue
                    
                try:
                    data = json.loads(message)
                except Exception as e:
                    #self.get_logger().warn(f"JSON inválido: {e} -> {message}")
                    continue

                mtype = data.get("type", "")
                
                # ------------------- Booleanos -------------------
                if mtype in ["motor_left", "motor_right", "light_stop", "light_left", "light_right", "light_safety", "capture","emergency"]:
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
                        
                    #self.get_logger().debug(f"Publicado Bool {mtype}={val}")
                
                # ------------------- Joystick -------------------
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

                    #self.get_logger().debug(f"Publicado Twist linear={twist.linear.x} angular={twist.angular.z}")
                
                # ------------------- Comando directo -------------------
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
                    #self.get_logger().debug(f"Publicado cmd_vel directo linear={linear} angular={angular}")
                
                elif mtype == "emergency":
                    self.get_logger().warn("Emergency received from UI")
                
                elif mtype == "request_diagnostic":
                    # Enviar estado actual de diagnóstico inmediatamente
                    await self.broadcast_diagnostic_status()
                
                else:
                    self.get_logger().warn(f"Tipo no reconocido: {mtype}")
                    
        except websockets.exceptions.ConnectionClosed:
            #self.get_logger().info("Cliente desconectado")
            pass
        except Exception as e:
            #self.get_logger().error(f"Excepción en handler: {e}")
            pass
        finally:
            # Remover de conexiones activas al desconectarse
            self.connected_websockets.discard(websocket)

    async def start_server(self):
        #self.get_logger().info("Iniciando servidor WebSocket...")
        server = await websockets.serve(
            self.handler, 
            WS_HOST, 
            WS_PORT, 
            ping_interval=20, 
            ping_timeout=20
        )
        #self.get_logger().info(f"WebSocket LISTO en ws://{WS_HOST}:{WS_PORT}")
        await server.wait_closed()
    
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketROSBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    loop = asyncio.get_event_loop()
    
    def ros_spin():
        try:
            executor.spin()
        except Exception as e:
            node.get_logger().error(f"Error en ros_spin: {e}")
    
    import threading
    t = threading.Thread(target=ros_spin, daemon=True)
    t.start()
    
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()