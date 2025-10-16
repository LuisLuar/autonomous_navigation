from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                               QGroupBox, QLabel, QProgressBar, QPushButton)
from PySide6.QtCore import Qt, QTimer
import random

from config.constants import Constants

class SensorMonitorWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.sensor_states = {}
        self.setup_ui()
        self.setup_simulation()
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Título de la sección
        title = QLabel("MONITOREO DE SENSORES Y SISTEMA")
        title.setStyleSheet("color: #3498db; font-size: 16px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Grid principal para sensores
        grid_layout = QGridLayout()
        grid_layout.setSpacing(15)
        
        # Sección 1: Sensores Principales
        main_sensors_group = self.create_sensor_group("SENSORES PRINCIPALES", [
            ("LIDAR 360°", "lidar"),
            ("Cámara RGB-D", "camera"),
            ("GPS", "gps"),
            ("IMU", "imu")
        ])
        grid_layout.addWidget(main_sensors_group, 0, 0)
        
        # Sección 2: Sensores de Seguridad
        safety_sensors_group = self.create_sensor_group("SENSORES DE SEGURIDAD", [
            ("Sharp 1", "sharp_1"),
            ("Sharp 2", "sharp_2"), 
            ("Sharp 3", "sharp_3"),
            ("Pulsador Emergencia", "emergency")
        ])
        grid_layout.addWidget(safety_sensors_group, 0, 1)
        
        # Sección 3: Sistema de Potencia
        power_group = self.create_power_group()
        grid_layout.addWidget(power_group, 1, 0)
        
        # Sección 4: Comunicaciones
        comms_group = self.create_comms_group()
        grid_layout.addWidget(comms_group, 1, 1)
        
        layout.addLayout(grid_layout)
        
        # Botones de control
        control_layout = QHBoxLayout()
        self.calibrate_btn = QPushButton("🔧 CALIBRAR IMU")
        self.calibrate_btn.setStyleSheet("""
            QPushButton {
                background-color: #f39c12;
                color: white;
                font-weight: bold;
                padding: 10px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #e67e22;
            }
        """)
        
        self.emergency_btn = QPushButton("🛑 EMERGENCIA")
        self.emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-weight: bold;
                padding: 10px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        
        control_layout.addWidget(self.calibrate_btn)
        control_layout.addStretch()
        control_layout.addWidget(self.emergency_btn)
        layout.addLayout(control_layout)
    
    def create_sensor_group(self, title, sensors):
        group = QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox {
                color: #3498db;
                font-weight: bold;
                font-size: 12px;
            }
        """)
        
        layout = QGridLayout()
        layout.setSpacing(10)
        
        for i, (sensor_name, sensor_id) in enumerate(sensors):
            # Nombre del sensor
            name_label = QLabel(sensor_name)
            name_label.setStyleSheet("color: #ecf0f1; font-weight: bold;")
            
            # Estado del sensor (LED virtual)
            status_label = QLabel("●")
            status_label.setStyleSheet("color: #e74c3c; font-size: 20px;")
            self.sensor_states[sensor_id] = status_label
            
            # Valor simulado
            value_label = QLabel("--")
            value_label.setStyleSheet("color: #bdc3c7; font-size: 11px;")
            self.sensor_states[f"{sensor_id}_value"] = value_label
            
            layout.addWidget(name_label, i, 0)
            layout.addWidget(status_label, i, 1)
            layout.addWidget(value_label, i, 2)
        
        group.setLayout(layout)
        return group
    
    def create_power_group(self):
        group = QGroupBox("SISTEMA DE POTENCIA")
        layout = QVBoxLayout()
        
        # Batería
        batt_layout = QHBoxLayout()
        batt_label = QLabel("Batería:")
        batt_label.setStyleSheet("color: #ecf0f1; font-weight: bold;")
        
        self.batt_progress = QProgressBar()
        self.batt_progress.setRange(0, 100)
        self.batt_progress.setValue(0)
        self.batt_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #34495e;
                border-radius: 5px;
                text-align: center;
                color: white;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #27ae60;
                border-radius: 3px;
            }
        """)
        
        self.batt_value = QLabel("0%")
        self.batt_value.setStyleSheet("color: #ecf0f1; font-weight: bold; min-width: 40px;")
        
        batt_layout.addWidget(batt_label)
        batt_layout.addWidget(self.batt_progress)
        batt_layout.addWidget(self.batt_value)
        
        # Voltaje y Corriente
        voltage_label = QLabel("Voltaje: -- V")
        voltage_label.setStyleSheet("color: #bdc3c7;")
        self.sensor_states["voltage"] = voltage_label
        
        current_label = QLabel("Corriente: -- A") 
        current_label.setStyleSheet("color: #bdc3c7;")
        self.sensor_states["current"] = current_label
        
        layout.addLayout(batt_layout)
        layout.addWidget(voltage_label)
        layout.addWidget(current_label)
        
        group.setLayout(layout)
        return group
    
    def create_comms_group(self):
        group = QGroupBox("COMUNICACIONES")
        layout = QVBoxLayout()
        
        comms = [
            ("ROS2 Master", "ros2"),
            ("ESP32", "esp32"),
            ("Drivers Motores", "motor_drivers"),
            ("Sensores I2C", "i2c")
        ]
        
        for comm_name, comm_id in comms:
            comm_layout = QHBoxLayout()
            
            name_label = QLabel(comm_name)
            name_label.setStyleSheet("color: #ecf0f1;")
            
            status_label = QLabel("🔴")
            status_label.setStyleSheet("font-size: 16px;")
            self.sensor_states[comm_id] = status_label
            
            comm_layout.addWidget(name_label)
            comm_layout.addStretch()
            comm_layout.addWidget(status_label)
            layout.addLayout(comm_layout)
        
        group.setLayout(layout)
        return group
    
    def setup_simulation(self):
        # Timer para simular actualizaciones de sensores
        self.sim_timer = QTimer()
        self.sim_timer.timeout.connect(self.update_sensor_simulation)
        self.sim_timer.start(1000)  # Actualizar cada segundo
    
    def update_sensor_simulation(self):
        # Simular cambios en los sensores
        for sensor_id, widget in self.sensor_states.items():
            if "value" in sensor_id:
                if "batt" in sensor_id:
                    value = random.randint(85, 100)
                    self.batt_progress.setValue(value)
                    self.batt_value.setText(f"{value}%")
                elif "voltage" in sensor_id:
                    widget.setText(f"Voltaje: {random.uniform(11.8, 12.5):.1f} V")
                elif "current" in sensor_id:
                    widget.setText(f"Corriente: {random.uniform(0.5, 2.5):.1f} A")
                else:
                    widget.setText(f"{random.uniform(0, 100):.1f}")
            
            elif isinstance(widget, QLabel) and widget.text() in ["●", "🔴", "🟢"]:
                # Cambiar estado aleatorio de sensores
                if random.random() > 0.3:  # 70% de probabilidad de estar OK
                    if widget.text() == "●":
                        widget.setStyleSheet("color: #27ae60; font-size: 20px;")
                    else:
                        widget.setText("🟢")
                        widget.setStyleSheet("font-size: 16px;")
                else:
                    if widget.text() == "●":
                        widget.setStyleSheet("color: #e74c3c; font-size: 20px;")
                    else:
                        widget.setText("🔴")
                        widget.setStyleSheet("font-size: 16px;")