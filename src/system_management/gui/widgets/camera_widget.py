from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                               QGroupBox, QPushButton)
from PySide6.QtCore import Qt
from utils.styles_light import get_app_style, get_theme_colors

class CameraWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.setStyleSheet(get_app_style()) 
    
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
        rgb_group = self.create_camera_view("CÁMARA RGB", "rgb")
        views_layout.addWidget(rgb_group)
        
        # Vista Profundidad
        depth_group = self.create_camera_view("CÁMARA PROFUNDIDAD", "depth") 
        views_layout.addWidget(depth_group)
        
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

        icon, color, background, border_color = get_theme_colors()['diagnostic']['error']
        status_label .setText(f"{icon} Desconectado")        
        status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
        
        layout.addWidget(video_placeholder)
        layout.addWidget(status_label)
        group.setLayout(layout)
        
        return group