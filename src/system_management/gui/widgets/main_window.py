import sys
from PySide6.QtWidgets import (QMainWindow, QTabWidget, QVBoxLayout, 
                               QWidget, QHBoxLayout, QLabel, QStatusBar)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from config.constants import Constants
from utils.styles import get_app_style
from widgets.sensor_monitor import SensorMonitorWidget
from widgets.map_widget import MapWidget
from widgets.camera_widget import CameraWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{Constants.APP_TITLE} v{Constants.APP_VERSION}")
        self.setGeometry(100, 100, Constants.WINDOW_WIDTH, Constants.WINDOW_HEIGHT)
        self.setStyleSheet(get_app_style())
        
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
        title_widget.setStyleSheet("background-color: #2c3e50; border-bottom: 2px solid #3498db;")
        
        layout = QHBoxLayout(title_widget)
        layout.setContentsMargins(20, 5, 20, 5)
        
        # Título de la aplicación
        title_label = QLabel(Constants.APP_TITLE)
        title_label.setStyleSheet("""
            color: #3498db; 
            font-size: 18px; 
            font-weight: bold;
            background-color: transparent;
        """)
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        
        # Estado del sistema
        status_label = QLabel("🔴 SISTEMA NO INICIALIZADO")
        status_label.setStyleSheet("""
            color: #e74c3c; 
            font-size: 12px; 
            font-weight: bold;
            background-color: transparent;
            padding: 5px 10px;
            border: 1px solid #e74c3c;
            border-radius: 3px;
        """)
        
        layout.addWidget(title_label)
        layout.addStretch()
        layout.addWidget(status_label)
        
        return title_widget
    
    def setup_tabs(self):
        # Pestaña 1: Monitoreo de Sensores
        self.sensor_tab = SensorMonitorWidget()
        self.tab_widget.addTab(self.sensor_tab, "📊 MONITOREO SENSORES")
        
        # Pestaña 2: Mapa y Navegación
        self.map_tab = MapWidget()
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
        status_bar.setStyleSheet("""
            QStatusBar {
                background-color: #2c3e50;
                color: #bdc3c7;
                border-top: 1px solid #34495e;
            }
        """)
        
        # Mensaje permanente
        status_bar.showMessage("Sistema listo - Desconectado de ROS2")
        
        # Indicadores en la barra de estado
        connection_indicator = QLabel("🔴 ROS2")
        connection_indicator.setStyleSheet("color: #e74c3c; font-weight: bold;")
        
        status_bar.addPermanentWidget(connection_indicator)
        self.setStatusBar(status_bar)