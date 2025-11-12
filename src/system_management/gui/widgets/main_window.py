#!/usr/bin/env python3
import sys
import os
from PySide6.QtWidgets import (QMainWindow, QTabWidget, QVBoxLayout, 
                               QWidget, QHBoxLayout, QLabel, QStatusBar)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QIcon

from config.constants import Constants
from utils.styles_light import get_app_style, get_theme_colors
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

        # Timer para actualizar UI desde ros_node
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_from_ros)
        self.update_timer.start(500)  # cada 500 ms
        
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
        self.status_label = QLabel("🔴 SISTEMA NO INICIALIZADO")
        self.status_label.setProperty("class", "system-status")
        
        layout.addWidget(title_label)
        layout.addStretch()
        layout.addWidget(self.status_label)
        
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
    
    def setup_status_bar(self):
        status_bar = QStatusBar()
        status_bar.setStyleSheet(get_app_style())  # Aplicar estilos de barra de estado
        
        # Mensaje permanente
        status_bar.showMessage("Sistema listo - Desconectado de ROS2")
        
        # Indicadores en la barra de estado
        self.connection_indicator = QLabel("🔴 ROS2")
        self.connection_indicator.setProperty("class", "ros-indicator")
        status_bar.addPermanentWidget(self.connection_indicator)
        self.setStatusBar(status_bar)

        # Timer para actualizar indicador de conexión
        if self.ros_node:
            timer = QTimer(self)
            timer.timeout.connect(self.update_ros_connection_indicator)
            timer.start(500)
    
    def update_ros_connection_indicator(self):
        """Actualiza el indicador de conexión ROS2 - VERSIÓN CORREGIDA"""
        status_type = 'error'  # Valor por defecto
        
        if not self.ros_node:
            # Si no hay nodo, directamente error
            icon, color, *_ = get_theme_colors()['diagnostic']['error']
            self.connection_indicator.setText(f"{icon} ROS2")
            self.connection_indicator.setStyleSheet(f"color: {color};")
            return
        
        # Verificar si el contexto ROS2 está activo
        try:
            import rclpy
            if not rclpy.ok():
                status_type = 'error'
            else:
                status_type = 'warning'  # ROS2 activo pero verificar datos
        except:
            status_type = 'error'
        
        # Verificar si hay datos recibidos recientemente
        has_data = False
        
        # Método 1: Verificar si hay mensajes en el bridge
        if hasattr(self.ros_node, 'ros_bridge') and self.ros_node.ros_bridge:
            bridge = self.ros_node.ros_bridge
            if (hasattr(bridge, 'global_status') and bridge.global_status is not None):
                has_data = True
        
        # Método 2: Verificar contador de mensajes (si existe)
        if hasattr(self.ros_node, 'message_count') and getattr(self.ros_node, 'message_count', 0) > 0:
            has_data = True
        
        # Método 3: Verificar timestamp del último mensaje
        if hasattr(self.ros_node, 'last_message_time'):
            import time
            last_time = getattr(self.ros_node, 'last_message_time', 0)
            if time.time() - last_time < 5.0:  # Mensajes en últimos 5 segundos
                has_data = True
        
        # Determinar estado final
        if has_data:
            status_type = 'success'
        elif status_type == 'warning': 
            # ROS2 está activo pero sin datos
            status_type = 'warning'
        # else mantiene 'error' del contexto ROS2
        
        icon, color, *_ = get_theme_colors()['diagnostic'][status_type]
        self.connection_indicator.setText(f"{icon} ROS2")
        self.connection_indicator.setStyleSheet(f"color: {color};")


    def _get_message_level(self, msg):
        """Extrae el nivel de un mensaje de forma segura."""
        level_raw = msg.level
        if isinstance(level_raw, bytes):
            return int.from_bytes(level_raw, byteorder='little', signed=False)
        else:
            try:
                return int(level_raw)
            except Exception:
                return 2

    # ==============================================================
    # ACTUALIZACIÓN DESDE ROS2
    # ==============================================================
    def update_from_ros(self):
        """Actualiza los datos desde ROS2 - VERSIÓN OPTIMIZADA."""
        if not self.ros_node:
            return

        rn = self.ros_node

        try:
            # Verificar el bridge
            if not hasattr(rn, 'ros_bridge') or rn.ros_bridge is None:
                self._show_no_bridge_error()
                return

            bridge = rn.ros_bridge

            if bridge.global_status:
                level = self._get_message_level(bridge.global_status)
                status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
                icon, color, background, border_color = get_theme_colors()['diagnostic'][status_type]

                if level == 2:
                    self.status_label.setText(f"{icon} SISTEMA NO INICIALIZADO")
                elif level == 1:
                    self.status_label.setText(f"{icon} SISTEMA CON ADVERTENCIAS")
                else:
                    self.status_label.setText(f"{icon} SISTEMA LISTO")
                
                self.status_label.setStyleSheet(f"color: {color}; border: 2px solid {color};")


        except AttributeError as e:
            print(f"⚠️ Error accediendo a datos ROS: {e}")
        except Exception as e:
            print(f"❌ Error inesperado en update_from_ros: {e}")


    def _show_no_bridge_error(self):
        """Muestra un mensaje de error cuando no hay bridge disponible."""
        print("❌ No hay bridge ROS disponible")