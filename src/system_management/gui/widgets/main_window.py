#!/usr/bin/env python3
import sys
import os
from PySide6.QtWidgets import (QMainWindow, QTabWidget, QVBoxLayout, 
                               QWidget, QHBoxLayout, QLabel, QStatusBar)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QIcon

from config.constants import Constants
from utils.styles_light import get_app_style
from widgets.sensor_monitor import SensorMonitorWidget
from widgets.map_widget import MapWidget
from widgets.camera_widget import CameraWidget

class MainWindow(QMainWindow):
    def __init__(self, ros_node=None):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle(f"{Constants.APP_TITLE} v{Constants.APP_VERSION}")
        self.setGeometry(100, 100, Constants.WINDOW_WIDTH, Constants.WINDOW_HEIGHT)
        self.setStyleSheet(get_app_style())  # Aplicar estilos generales
        
        self.setup_ui()
        
    def setup_ui(self):
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Barra de título personalizada
        title_bar = self.create_title_bar()
        main_layout.addWidget(title_bar)
        
        # Widget de pestañas
        self.tab_widget = QTabWidget()
        self.setup_tabs()
        main_layout.addWidget(self.tab_widget)
        
        # Barra de estado
        self.setup_status_bar()
    
    def create_title_bar(self):
        title_widget = QWidget()
        title_widget.setFixedHeight(60)
        title_widget.setProperty("class", "title-bar")
        
        layout = QHBoxLayout(title_widget)
        layout.setContentsMargins(20, 5, 20, 5)
        
        # Título de la aplicación
        title_label = QLabel(Constants.APP_TITLE)
        title_label.setProperty("class", "title-bar-title")
        
        # Estado del sistema
        status_label = QLabel("🔴 SISTEMA NO INICIALIZADO")
        status_label.setProperty("class", "system-status")
        
        layout.addWidget(title_label)
        layout.addStretch()
        layout.addWidget(status_label)
        
        return title_widget
    
    def setup_tabs(self):
        # Pestaña 1: Monitoreo de Sensores
        self.sensor_tab = SensorMonitorWidget(ros_node=self.ros_node)
        self.tab_widget.addTab(self.sensor_tab, "📊 MONITOREO SENSORES")
        
        # Pestaña 2: Mapa y Navegación
        self.map_tab = MapWidget(ros_node=self.ros_node)
        self.tab_widget.addTab(self.map_tab, "🗺️ MAPA Y NAVEGACIÓN")
        
        # Pestaña 3: Cámara y Visión (placeholder por ahora)
        self.camera_tab = CameraWidget()
        self.tab_widget.addTab(self.camera_tab, "📷 CÁMARA Y VISIÓN")
        
        # Pestaña 4: Control (placeholder)
        control_placeholder = QWidget()
        control_placeholder.setStyleSheet("background-color: #2c3e50;")
        layout = QVBoxLayout(control_placeholder)
        title = QLabel("CONTROL Y TELEOPERACIÓN")
        title.setStyleSheet("color: #3498db; font-size: 16px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        self.tab_widget.addTab(control_placeholder, "🎮 CONTROL")
    
    def setup_status_bar(self):
        status_bar = QStatusBar()
        status_bar.setStyleSheet(get_app_style())  # Aplicar estilos de barra de estado
        
        # Mensaje permanente
        status_bar.showMessage("Sistema listo - Desconectado de ROS2")
        
        # Indicadores en la barra de estado
        self.connection_indicator = QLabel("🔴 ROS2")
        self.connection_indicator.setProperty("class", "ros-indicator ros-disconnected")
        status_bar.addPermanentWidget(self.connection_indicator)
        self.setStatusBar(status_bar)

        # Timer para actualizar indicador de conexión
        if self.ros_node:
            timer = QTimer(self)
            timer.timeout.connect(self.update_ros_connection_indicator)
            timer.start(500)
    
    def update_ros_connection_indicator(self):
        if self.ros_node and getattr(self.ros_node, 'any_msg_received', False):
            self.connection_indicator.setText("🟢 ROS2")
            self.connection_indicator.setProperty("class", "ros-indicator ros-connected")
        else:
            self.connection_indicator.setText("🔴 ROS2")
            self.connection_indicator.setProperty("class", "ros-indicator ros-disconnected")