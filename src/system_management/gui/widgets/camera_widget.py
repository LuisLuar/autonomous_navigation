from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                               QGroupBox, QPushButton)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
import cv2
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus

from utils.styles_light import get_app_style, get_theme_colors

class CameraWidget(QWidget):
    def __init__(self, ros_node=None):
        super().__init__()
        self.ros_node = ros_node
        self.cv_bridge = CvBridge()
        
        # Estados de la cámara
        self.camera_status = None
        self.rgb_image = None
        self.depth_image = None
        
        self.setup_ui()
        self.setStyleSheet(get_app_style())
        
        # Timer para actualizar desde ROS2
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_from_ros)
        self.update_timer.start(20)  # Actualizar cada 500ms
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Título
        title = QLabel("VISIÓN POR COMPUTADORA - CÁMARA RGB-D")
        title.setProperty("class", "sensor-title")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Layout horizontal para las vistas
        views_layout = QHBoxLayout()
        
        # Vista RGB
        self.rgb_group = self.create_camera_view("CÁMARA RGB", "rgb")
        views_layout.addWidget(self.rgb_group)
        
        # Vista Profundidad
        self.depth_group = self.create_camera_view("CÁMARA PROFUNDIDAD", "depth") 
        views_layout.addWidget(self.depth_group)
        
        layout.addLayout(views_layout)
        
        # Información de detecciones
        info_group = QGroupBox("INFORMACIÓN DE DETECCIONES")
        info_layout = QVBoxLayout()
        
        self.detection_info = QLabel("Sistema de visión no inicializado")
        self.detection_info.setProperty("class", "sensor-value-off")
        self.detection_info.setStyleSheet("""
            QLabel {
                min-height: 170px;
            }
        """)
        self.detection_info.setAlignment(Qt.AlignCenter)
        
        info_layout.addWidget(self.detection_info)
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
    
    def create_camera_view(self, title, view_type):
        group = QGroupBox(title)
        layout = QVBoxLayout()
        
        # Placeholder para el video
        video_placeholder = QLabel("VIDEO NO DISPONIBLE")
        video_placeholder.setStyleSheet("""
            QLabel {
                background-color: #2c3e50;
                color: #7f8c8d;
                font-size: 14px;
                font-weight: bold;
                border: 2px dashed #34495e;
                min-height: 500px;
            }
        """)
        video_placeholder.setAlignment(Qt.AlignCenter)
        
        # Estado
        status_label = QLabel("")
        
        # Guardar referencias para actualización
        if view_type == "rgb":
            self.rgb_video_label = video_placeholder
            self.rgb_status_label = status_label
        else:  # depth
            self.depth_video_label = video_placeholder
            self.depth_status_label = status_label

        icon, color, background, border_color = get_theme_colors()['diagnostic']['error']
        status_label.setText(f"{icon} Desconectado")        
        status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
        
        layout.addWidget(video_placeholder)
        layout.addWidget(status_label)
        group.setLayout(layout)
        
        return group

    def _get_message_level(self, msg):
        """Extrae el nivel de un mensaje de forma segura."""
        if not msg:
            return 2
            
        level_raw = msg.level
        if isinstance(level_raw, bytes):
            return int.from_bytes(level_raw, byteorder='little', signed=False)
        else:
            try:
                return int(level_raw)
            except Exception:
                return 2

    def update_from_ros(self):
        """Actualiza los datos desde ROS2"""
        if not self.ros_node:
            return

        try:
            # Verificar el bridge
            if not hasattr(self.ros_node, 'ros_bridge') or self.ros_node.ros_bridge is None:
                return

            bridge = self.ros_node.ros_bridge

            # Actualizar estado de la cámara
            if bridge.camera_status:
                self.camera_status = bridge.camera_status
                level = self._get_message_level(bridge.camera_status)
                
                # Actualizar estados visuales
                status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
                icon, color, background, border_color = get_theme_colors()['diagnostic'][status_type]
                
                status_text = f"{icon} Conectado" if level == 0 else f"{icon} Advertencia" if level == 1 else f"{icon} Error"
                
                self.rgb_status_label.setText(status_text)
                self.rgb_status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
                self.depth_status_label.setText(status_text)
                self.depth_status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
                
                # Actualizar información de detecciones
                if level == 0:
                    self.detection_info.setText("Cámara funcionando correctamente\nLista para procesamiento de visión")
                    self.detection_info.setProperty("class", "sensor-value-on")
                elif level == 1:
                    self.detection_info.setText(f"Advertencia: {bridge.camera_status.message}")
                    self.detection_info.setProperty("class", "sensor-value-warning")
                else:
                    self.detection_info.setText(f"Error: {bridge.camera_status.message}")
                    self.detection_info.setProperty("class", "sensor-value-off")
            else:
                # Estado por defecto si no hay información
                icon, color, background, border_color = get_theme_colors()['diagnostic']['error']
                self.rgb_status_label.setText(f"{icon} Desconectado")
                self.rgb_status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
                self.depth_status_label.setText(f"{icon} Desconectado")
                self.depth_status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
                self.detection_info.setText("Esperando conexión con la cámara...")
                self.detection_info.setProperty("class", "sensor-value-off")

            # Procesar imagen RGB si está disponible y la cámara está conectada
            # En CameraWidget.__init__ cambia:

            # (dentro de update_from_ros, donde compruebas bridge)
            if hasattr(bridge, 'camera_rgb_qpixmap') and bridge.camera_rgb_qpixmap and self.camera_status and self._get_message_level(self.camera_status) == 0:
                try:
                    self.rgb_video_label.setPixmap(bridge.camera_rgb_qpixmap)
                    self.rgb_video_label.setScaledContents(True)
                except Exception as e:
                    print(f"Error mostrando QPixmap RGB: {e}")
            else:
                self.rgb_video_label.clear()
                self.rgb_video_label.setText("VIDEO NO DISPONIBLE")

            if hasattr(bridge, 'camera_depth_qpixmap') and bridge.camera_depth_qpixmap and self.camera_status and self._get_message_level(self.camera_status) == 0:
                try:
                    self.depth_video_label.setPixmap(bridge.camera_depth_qpixmap)
                    self.depth_video_label.setScaledContents(True)
                except Exception as e:
                    print(f"Error mostrando QPixmap Depth: {e}")
            else:
                self.depth_video_label.clear()
                self.depth_video_label.setText("VIDEO NO DISPONIBLE")



        except Exception as e:
            print(f"Error en update_from_ros de CameraWidget: {e}")