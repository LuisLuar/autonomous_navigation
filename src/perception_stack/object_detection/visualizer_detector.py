#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_interfaces.msg import ObjectInfoArray, ObjectInfo
import cv2
import numpy as np
import time
from collections import defaultdict


class ObjectVisualizerNode(Node):
    def __init__(self):
        super().__init__('object_visualizer')
        
        # ============ CONFIGURACIÓN VISUAL ============
        self.declare_parameter('show_confidence', True)
        self.declare_parameter('show_track_id', True)
        self.declare_parameter('show_distance', True)
        self.declare_parameter('show_speed', True)
        self.declare_parameter('show_trajectory', True)
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('font_scale', 0.5)
        self.declare_parameter('trajectory_length', 15)  # Reducido
        self.declare_parameter('max_track_age', 2.0)     # Segundos para eliminar track
        self.declare_parameter('fps_display', False)
        
        self.bridge = CvBridge()
        self.last_objects = None
        self.last_objects_time = None
        
        # Historial para trayectorias con timestamp
        self.trajectories = defaultdict(list)  # track_id -> [(center_x, center_y, timestamp)]
        self.track_colors = {}
        self.track_last_seen = {}  # Para limpieza
        
        # Colores por clase
        self.class_colors = {
            'person': (0, 220, 255),      # Amarillo
            'car': (0, 127, 255),         # Naranja
            'bicycle': (255, 191, 0),     # Azul claro
            'motorcycle': (255, 64, 0),   # Azul
            'bus': (0, 0, 255),           # Rojo
            'truck': (0, 0, 255),         # Rojo
            'dog': (255, 0, 255),         # Magenta
            'cat': (255, 128, 255),       # Rosa
        }
        
        # ============ SUBSCRIPTORES ============
        self.sub_image = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.callback_image, 10
        )
        
        self.sub_objects = self.create_subscription(
            ObjectInfoArray, '/objects/fused_info', self.callback_objects, 10
        )
        
        # ============ PUBLICADORES ============
        self.pub_annotated = self.create_publisher(
            Image, '/detection/annotated_image', 10
        )
        
        # Timer para limpiar tracks antiguos
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_old_tracks)
        
        # Para FPS
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        self.get_logger().info("Visualizador de objetos inicializado")

    def callback_objects(self, msg: ObjectInfoArray):
        """Almacenar objetos fusionados más recientes"""
        self.last_objects = msg
        self.last_objects_time = time.time()
        
        current_time = time.time()
        trajectory_length = self.get_parameter('trajectory_length').value
        active_tracks = set()
        
        # Actualizar trayectorias
        for obj in msg.objects:
            track_id = obj.track_id
            if track_id > 0:
                active_tracks.add(track_id)
                
                # Actualizar tiempo de última vista
                self.track_last_seen[track_id] = current_time
                
                # Añadir punto actual a la trayectoria
                center_x = obj.center_x
                center_y = obj.center_y
                
                if track_id not in self.trajectories:
                    self.trajectories[track_id] = []
                
                # Añadir con timestamp
                self.trajectories[track_id].append((center_x, center_y, current_time))
                
                # Mantener solo los últimos puntos y limpiar muy antiguos
                self.trajectories[track_id] = [
                    point for point in self.trajectories[track_id] 
                    if current_time - point[2] < 5.0  # 5 segundos máximo
                ]
                
                # Asegurar longitud máxima
                if len(self.trajectories[track_id]) > trajectory_length:
                    self.trajectories[track_id] = self.trajectories[track_id][-trajectory_length:]
                
                # Asignar color si no tiene
                if track_id not in self.track_colors:
                    self.track_colors[track_id] = self.get_track_color(track_id, obj.class_name)
        
        # Marcar tracks inactivos (pero no eliminarlos aún)
        for track_id in list(self.track_last_seen.keys()):
            if track_id not in active_tracks:
                # Incrementar contador de tiempo perdido
                pass

    def cleanup_old_tracks(self):
        """Limpiar tracks que no se han visto por mucho tiempo"""
        current_time = time.time()
        max_age = self.get_parameter('max_track_age').value
        
        # Identificar tracks a eliminar
        tracks_to_remove = []
        for track_id, last_seen in list(self.track_last_seen.items()):
            if current_time - last_seen > max_age:
                tracks_to_remove.append(track_id)
        
        # Eliminar tracks antiguos
        for track_id in tracks_to_remove:
            if track_id in self.trajectories:
                del self.trajectories[track_id]
            if track_id in self.track_colors:
                del self.track_colors[track_id]
            if track_id in self.track_last_seen:
                del self.track_last_seen[track_id]
        
        # Limpieza adicional periódica
        if len(self.trajectories) > 50:  # Si hay demasiados tracks
            # Eliminar los más antiguos
            sorted_tracks = sorted(
                self.track_last_seen.items(), 
                key=lambda x: x[1]
            )
            for track_id, _ in sorted_tracks[:-30]:  # Mantener solo 30 más recientes
                if track_id in self.trajectories:
                    del self.trajectories[track_id]
                if track_id in self.track_colors:
                    del self.track_colors[track_id]
                if track_id in self.track_last_seen:
                    del self.track_last_seen[track_id]

    def callback_image(self, msg: Image):
        """Procesar y anotar imagen con objetos fusionados"""
        if self.last_objects is None:
            # Publicar imagen original si no hay objetos
            try:
                self.pub_annotated.publish(msg)
            except:
                pass
            return
        
        # Convertir imagen
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = img.shape[:2]
        except Exception as e:
            self.get_logger().warn(f"Error CV bridge: {e}", throttle_duration_sec=5.0)
            return
        
        # Optimización: crear una copia solo si es necesario
        if self.get_parameter('show_trajectory').value:
            img_with_trajectories = img.copy()
            img_with_trajectories = self.draw_trajectories(img_with_trajectories)
        else:
            img_with_trajectories = img
        
        # Dibujar cada objeto
        current_time = time.time()
        active_tracks = set()
        
        for obj in self.last_objects.objects:
            if obj.track_id > 0:
                active_tracks.add(obj.track_id)
            
            # Verificar si el objeto es demasiado antiguo (para no dibujar tracks perdidos)
            object_time = getattr(obj, 'timestamp', current_time)
            if current_time - object_time > 1.0:
                continue  # Saltar objetos muy antiguos
            
            img_with_trajectories = self.draw_object(
                img_with_trajectories, obj, height, width
            )
        
        # Dibujar información global
        img_with_trajectories = self.draw_global_info(
            img_with_trajectories, self.last_objects, len(active_tracks)
        )
        
        # Publicar imagen anotada
        try:
            out_msg = self.bridge.cv2_to_imgmsg(img_with_trajectories, encoding="bgr8")
            out_msg.header = msg.header
            self.pub_annotated.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"Error publicando imagen: {e}", throttle_duration_sec=5.0)
        
        # Calcular FPS si está habilitado
        if self.get_parameter('fps_display').value:
            self.frame_count += 1
            current_time_fps = time.time()
            if current_time_fps - self.last_fps_time >= 1.0:
                fps = self.frame_count / (current_time_fps - self.last_fps_time)
                self.get_logger().info(f"FPS: {fps:.1f} | Tracks activos: {len(active_tracks)}")
                self.frame_count = 0
                self.last_fps_time = current_time_fps

    def draw_trajectories(self, img):
        """Dibujar trayectorias de los objetos trackeados (optimizado)"""
        current_time = time.time()
        
        for track_id, points in list(self.trajectories.items()):
            if len(points) < 2:
                continue
            
            # Filtrar puntos muy antiguos
            recent_points = [
                point for point in points 
                if current_time - point[2] < 3.0  # Solo últimos 3 segundos
            ]
            
            if len(recent_points) < 2:
                continue
            
            # Obtener color del track
            color = self.track_colors.get(track_id, (200, 200, 200))
            
            # Dibujar línea conectando los puntos
            for i in range(len(recent_points) - 1):
                pt1 = (int(recent_points[i][0]), int(recent_points[i][1]))
                pt2 = (int(recent_points[i+1][0]), int(recent_points[i+1][1]))
                
                # Grosor decreciente (más reciente = más grueso)
                thickness = max(1, 2 - i // 8)  # Más delgado
                alpha = max(0.2, 1.0 - i / len(recent_points) * 0.8)
                
                # Crear color con transparencia
                line_color = (
                    int(color[0] * alpha),
                    int(color[1] * alpha),
                    int(color[2] * alpha)
                )
                
                cv2.line(img, pt1, pt2, line_color, thickness, cv2.LINE_AA)
            
            # Dibujar punto solo en la posición más reciente
            last_point = (int(recent_points[-1][0]), int(recent_points[-1][1]))
            cv2.circle(img, last_point, 3, color, -1)
            cv2.circle(img, last_point, 5, (255, 255, 255), 1)
        
        return img

    def draw_object(self, img, obj: ObjectInfo, img_height, img_width):
        """Dibujar un objeto individual con su información (optimizado)"""
        # Obtener parámetros
        show_conf = self.get_parameter('show_confidence').value
        show_track = self.get_parameter('show_track_id').value
        show_dist = self.get_parameter('show_distance').value
        show_speed = self.get_parameter('show_speed').value
        thickness = self.get_parameter('line_thickness').value
        font_scale = self.get_parameter('font_scale').value
        
        # Coordenadas del bounding box
        x1, y1, x2, y2 = obj.x1, obj.y1, obj.x2, obj.y2
        
        # Verificar que el bbox sea válido
        if x1 >= x2 or y1 >= y2 or x2 > img_width or y2 > img_height:
            return img
        
        # Obtener color basado en la clase
        base_color = self.get_class_color(obj.class_name)
        
        # Determinar color del bbox según el estado
        bbox_color = base_color  # Por defecto
        if obj.is_static:
            bbox_color = (0, 200, 0)  # Verde para estáticos
        elif obj.is_moving_toward and obj.relative_speed < -0.3:
            bbox_color = (0, 0, 255)  # Rojo para acercándose
        elif obj.is_moving_away and obj.relative_speed > 0.3:
            bbox_color = (255, 165, 0)  # Naranja para alejándose
        
        # Dibujar bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), bbox_color, thickness)
        
        # Solo dibujar centro si es útil
        center_x = int(obj.center_x)
        center_y = int(obj.center_y)
        cv2.circle(img, (center_x, center_y), 2, bbox_color, -1)
        
        # Preparar etiqueta principal
        label_parts = []
        
        # Nombre de clase (abreviado)
        class_display = obj.class_name
        if len(class_display) > 10:
            class_display = class_display[:8] + ".."
        label_parts.append(class_display)
        
        # ID de track
        if show_track and obj.track_id > 0:
            label_parts.append(f"#{obj.track_id}")
        
        # Confianza
        if show_conf:
            conf_percent = int(obj.confidence * 100)
            if conf_percent >= 50:  # Solo mostrar si es confiable
                label_parts.append(f"{conf_percent}%")
        
        label = " ".join(label_parts) if label_parts else class_display
        
        # Calcular tamaño del texto
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1
        )
        
        # Posición de la etiqueta (arriba del bbox)
        label_y = max(y1 - 3, th + 3)
        label_x = max(x1, 5)
        
        # Asegurar que la etiqueta no se salga de la imagen
        if label_x + tw + 6 > img_width:
            label_x = img_width - tw - 6
        
        # Fondo de etiqueta (solo si hay texto)
        if label.strip():
            cv2.rectangle(
                img,
                (label_x - 2, label_y - th - 2),
                (label_x + tw + 2, label_y + 2),
                bbox_color,
                -1
            )
            
            # Texto de etiqueta
            cv2.putText(
                img,
                label,
                (label_x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (255, 255, 255),  # Blanco
                1,
                cv2.LINE_AA
            )
        
        # Información adicional (solo si hay espacio y está habilitada)
        if y2 + 30 < img_height and (show_dist or show_speed):
            info_text = ""
            
            # Distancia
            if show_dist and obj.distance_valid and obj.distance > 0:
                source_symbol = self.get_distance_source_symbol(obj.distance_source)
                info_text += f"{obj.distance:.1f}m{source_symbol}"
                
                # Velocidad relativa
                if show_speed and abs(obj.relative_speed) > 0.1:
                    direction = "←" if obj.relative_speed < 0 else "→"
                    info_text += f" {direction}{abs(obj.relative_speed):.1f}"
            elif show_speed and abs(obj.relative_speed) > 0.1:
                direction = "←" if obj.relative_speed < 0 else "→"
                info_text = f"{direction}{abs(obj.relative_speed):.1f}m/s"
            
            # Estado de movimiento (solo si es relevante)
            if obj.is_static and info_text:
                info_text += " S"
            elif obj.is_moving_toward and info_text:
                info_text += " T"
            elif obj.is_moving_away and info_text:
                info_text += " A"
            
            # Dibujar si hay información
            if info_text:
                (info_tw, info_th), _ = cv2.getTextSize(
                    info_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.7, 1
                )
                
                # Posición
                info_x = max(x1, 5)
                info_y = min(y2 + info_th + 5, img_height - 5)
                
                # Fondo
                cv2.rectangle(
                    img,
                    (info_x - 2, info_y - info_th - 2),
                    (info_x + info_tw + 2, info_y + 2),
                    (0, 0, 0),
                    -1
                )
                
                # Texto
                cv2.putText(
                    img,
                    info_text,
                    (info_x, info_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale * 0.7,
                    (255, 255, 200),  # Amarillo claro
                    1,
                    cv2.LINE_AA
                )
        
        return img

    def draw_global_info(self, img, objects_msg: ObjectInfoArray, active_tracks_count=0):
        """Dibujar información global en la esquina (optimizado)"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        
        # Información de estadísticas
        stats = []
        if len(objects_msg.objects) > 0:
            stats.append(f"Objs: {len(objects_msg.objects)}")
            if objects_msg.num_static_objects > 0:
                stats.append(f"Static: {objects_msg.num_static_objects}")
            if objects_msg.num_moving_objects > 0:
                stats.append(f"Moving: {objects_msg.num_moving_objects}")
            if active_tracks_count > 0:
                stats.append(f"Tracks: {active_tracks_count}")
            if abs(objects_msg.robot_speed) > 0.01:
                stats.append(f"Robot: {objects_msg.robot_speed:.1f}m/s")
        
        if not stats:
            return img
        
        # Posición inicial (esquina superior izquierda)
        x, y = 10, 25
        line_spacing = 20
        
        # Calcular tamaño del panel
        max_text_width = 0
        for stat in stats:
            (tw, th), _ = cv2.getTextSize(stat, font, font_scale, thickness)
            max_text_width = max(max_text_width, tw)
        
        panel_width = max_text_width + 20
        panel_height = len(stats) * line_spacing + 10
        
        # Fondo semi-transparente para el panel
        overlay = img.copy()
        cv2.rectangle(
            overlay, 
            (5, 5), 
            (panel_width, panel_height), 
            (0, 0, 0), 
            -1
        )
        img = cv2.addWeighted(overlay, 0.6, img, 0.4, 0)
        
        # Dibujar cada línea
        for i, stat in enumerate(stats):
            cv2.putText(
                img,
                stat,
                (x, y + i * line_spacing),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA
            )
        
        return img

    def get_class_color(self, class_name):
        """Obtener color para una clase (optimizado)"""
        class_lower = class_name.lower()
        
        # Buscar coincidencia exacta o parcial
        for key, color in self.class_colors.items():
            if key in class_lower:
                return color
        
        # Generar color basado en hash
        return self.color_from_hash(class_lower)

    def get_track_color(self, track_id, class_name):
        """Obtener color único para un track"""
        class_color = self.get_class_color(class_name)
        
        # Modificar basado en track_id para diferenciar
        variation = track_id % 4
        
        if variation == 0:
            return class_color
        elif variation == 1:
            return (
                min(255, class_color[0] + 40),
                max(0, class_color[1] - 30),
                class_color[2]
            )
        elif variation == 2:
            return (
                class_color[0],
                min(255, class_color[1] + 40),
                max(0, class_color[2] - 30)
            )
        else:
            return (
                max(0, class_color[0] - 30),
                class_color[1],
                min(255, class_color[2] + 40)
            )

    @staticmethod
    def color_from_hash(name: str):
        """Generar color consistente basado en hash del nombre"""
        h = hash(name) & 0xFFFFFFFF
        
        r = (h >> 16) & 0xFF
        g = (h >> 8) & 0xFF
        b = h & 0xFF
        
        # Asegurar que no sea demasiado oscuro o claro
        r = max(80, min(220, r))
        g = max(80, min(220, g))
        b = max(80, min(220, b))
        
        return (b, g, r)  # BGR para OpenCV

    @staticmethod
    def get_distance_source_symbol(source: int):
        """Obtener símbolo para la fuente de distancia"""
        symbols = ["", "D", "I", "S"]
        return symbols[source] if 0 <= source < len(symbols) else ""

    def destroy_node(self):
        """Cleanup"""
        self.cleanup_timer.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectVisualizerNode()
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