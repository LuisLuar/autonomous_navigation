from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                               QGroupBox, QLabel, QProgressBar, QListWidget, QListWidgetItem)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtCore import QSize

from config.constants import Constants
from utils.styles_light import get_app_style, get_theme_colors

class SensorMonitorWidget(QWidget):
    def __init__(self, ros_node=None):
        super().__init__()
        self.ros_node = ros_node
        self.sensor_states = {}
        self.diagnostic_messages = []
        self.setStyleSheet(get_app_style())  # Aplicar estilos generales
        self.setup_ui()
        self.theme_colors = get_theme_colors()

        # Timer para actualizar UI desde ros_node
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_from_ros)
        self.update_timer.start(500)  # cada 500 ms
    
    # ==============================================================
    # CONFIGURACIÓN UI
    # ==============================================================
    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Título de la sección
        title = QLabel("MONITOREO DE SENSORES Y SISTEMA")
        title.setProperty("class", "sensor-title")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Grid principal para sensores
        grid_layout = QGridLayout()
        grid_layout.setSpacing(15)

        # === LISTAS DE DIAGNÓSTICO ===
        self.main_diagnostics_list = self.create_diagnostic_list("COMPONENTES PRINCIPALES")
        self.safety_diagnostics_list = self.create_diagnostic_list("SENSORES DE SEGURIDAD")
        grid_layout.addWidget(self.main_diagnostics_list, 0, 0)
        grid_layout.addWidget(self.safety_diagnostics_list, 0, 1)
        
        # === GRUPOS DE SENSORES ===
        main_sensors_group = self.create_device_group("SENSORES", [
                                ("LIDAR 360°", "lidar"),
                                ("Cámara RGB-D", "camera"),
                                ("GPS", "gps"),
                            ])
        grid_layout.addWidget(main_sensors_group, 2, 0)
        
        """("IMU", "imu"),
                                ("Encoder", "encoder")"""
        safety_sensors_group = self.create_sensor_group("SENSORES DE SEGURIDAD", [
            ("Sharp frontal", "sharp_front"),
            ("Sharp izquierdo", "sharp_left"),
            ("Sharp derecho", "sharp_right")
        ])
        grid_layout.addWidget(safety_sensors_group, 1, 1)

        # === SISTEMA DE POTENCIA ===
        power_group = self.create_power_group()
        grid_layout.addWidget(power_group, 1, 0)
        
        # === COMUNICACIONES ===
        comms_group = self.create_device_group("COMUNICACIONES", [
                        ("micro-ROS Agent", "microros_agent"),
                        ("ESP32 Seguridad", "esp32_safety"),
                        ("ESP32 Control", "esp32_control"),
                    ])
        grid_layout.addWidget(comms_group, 2, 1)
        
        layout.addLayout(grid_layout)

    # ==============================================================
    # LISTADO DE MENSAJES DIAGNÓSTICO MEJORADO
    # ==============================================================
    def create_diagnostic_list(self, title):
        group = QGroupBox(f"📋 {title} - ESTADOS DEL SISTEMA")
        layout = QVBoxLayout()
        
        # Contador de estados
        status_count_label = QLabel("⚪ 0 | ⚠️ 0 | 🔴 0")
        status_count_label.setProperty("class", "diagnostic-counter")
        status_count_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(status_count_label)
        
        list_widget = QListWidget()
        list_widget.setProperty("class", "diagnostic-list")
        list_widget.setSpacing(3)  # Espacio entre items
        list_widget.setUniformItemSizes(False)  # Permite diferentes alturas
        
        layout.addWidget(list_widget)
        group.setLayout(layout)
        group.list_widget = list_widget
        group.status_count_label = status_count_label
        
        return group

    def update_diagnostic_list(self, list_widget, diagnostics):
        list_widget.clear()
        
        # Contadores para el resumen
        error_count = warn_count = ok_count = 0

        # Ordenar por prioridad: ERROR (2) → WARN (1) → OK (0)
        sorted_msgs = sorted(diagnostics, key=lambda d: d.level, reverse=True)

        for msg in sorted_msgs:
            # Crear item personalizado
            item = QListWidgetItem()
            
            # Determinar nivel
            level = self._get_message_level(msg)

            # Configurar texto y estilo según el nivel
            icon, color, background, border_color = self._get_diagnostic_style(level)
            
            # Actualizar contadores
            if level == 2:  # ERROR
                error_count += 1
            elif level == 1:  # WARN
                warn_count += 1
            else:  # OK
                ok_count += 1

            # Texto formateado
            item_text = f"{icon} [{msg.name}] {msg.message}"
            item.setText(item_text)
            
            # Estilo completo del item
            item.setForeground(QColor(color))
            item.setBackground(QColor(background))
            
            # Tooltip con información adicional
            item.setToolTip(f"Sistema: {msg.name}\nNivel: {level}\nMensaje: {msg.message}\n\nHora: {msg.header.stamp.sec if hasattr(msg, 'header') else 'N/A'}")

            # Hacer el item más alto para mejor separación visual
            item.setSizeHint(QSize(item.sizeHint().width(), 45))
            
            # Estilo personalizado para el item
            item.setData(Qt.UserRole, level)  # Guardar nivel para filtrado
            
            list_widget.addItem(item)
            
            # Agregar separador visual después de cada grupo de nivel
            if sorted_msgs.index(msg) < len(sorted_msgs) - 1:
                next_msg = sorted_msgs[sorted_msgs.index(msg) + 1]
                next_level = self._get_message_level(next_msg)
                if level != next_level:
                    self._add_separator_item(list_widget)

        # Actualizar contador de estados
        if hasattr(list_widget.parent(), 'status_count_label'):
            count_text = f"⚪ {ok_count} | ⚠️ {warn_count} | 🔴 {error_count}"
            list_widget.parent().status_count_label.setText(count_text)
            
        # Mostrar mensaje si no hay diagnósticos
        if not diagnostics:
            item = QListWidgetItem("📭 No hay mensajes de diagnóstico")
            item.setForeground(QColor("#95a5a6"))
            item.setBackground(QColor("#2c3e50"))
            item.setTextAlignment(Qt.AlignCenter)
            list_widget.addItem(item)

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

    def _get_diagnostic_style(self, level):
        """Obtiene el estilo visual para un nivel de diagnóstico."""
        level_map = {2: 'error', 1: 'warning', 0: 'ok'}
        level_key = level_map.get(level, 'ok')
        return self.theme_colors['diagnostic'][level_key]


    def _add_separator_item(self, list_widget):
        """Agrega un separador visual entre grupos de niveles."""
        separator = QListWidgetItem()
        separator.setFlags(Qt.NoItemFlags)  # Hacerlo no seleccionable
        separator.setBackground(QColor("#34495e"))
        separator.setSizeHint(QSize(separator.sizeHint().width(), 2))
        list_widget.addItem(separator)
    
    # === GRUPOS DE SENSORES ===
    def create_sensor_group(self, title, sensors):
        group = QGroupBox(title)
        layout = QGridLayout()
        layout.setSpacing(10)
        
        for i, (sensor_name, sensor_id) in enumerate(sensors):
            # Nombre del sensor
            name_label = QLabel(sensor_name)
            name_label.setProperty("class", "sensor-label")
            
            # Estado del sensor (LED virtual)
            icon, *_ = get_theme_colors()['diagnostic']['error']
            status_label = QLabel(icon)
            status_label.setProperty("class", "sensor-status")
            self.sensor_states[sensor_id] = status_label
            
            # Valor simulado
            value_label = QLabel("--")
            value_label.setProperty("class", "sensor-value-off")
            self.sensor_states[f"{sensor_id}_value"] = value_label
            
            layout.addWidget(name_label, i, 0)
            layout.addWidget(status_label, i, 1)
            layout.addWidget(value_label, i, 2)
        
        group.setLayout(layout)
        return group
    
    # === SISTEMA DE POTENCIA ===
    def create_power_group(self):
        group = QGroupBox("SISTEMA DE POTENCIA")
        layout = QVBoxLayout()

        # Batería 12V
        batt_layout = QHBoxLayout()
        batt_label = QLabel("Batería 12V:")
        batt_label.setProperty("class", "power-label")
        
        # Barra de progreso de la batería
        self.batt_progress = QProgressBar()
        self.batt_progress.setRange(0, 100)
        self.batt_progress.setValue(0)
        self.batt_progress.setProperty("class", "battery-progress")

        batt_layout.addWidget(batt_label)
        batt_layout.addWidget(self.batt_progress)

        # Contenedor para los tres valores alineados
        values_container = QWidget()
        values_layout = QHBoxLayout(values_container)
        values_layout.setContentsMargins(0, 10, 0, 10)
        
        # Layout para las etiquetas (izquierda)
        labels_layout = QVBoxLayout()
        labels_layout.setSpacing(8)
        
        # Layout para los valores (derecha/centrados)
        values_right_layout = QVBoxLayout()
        values_right_layout.setSpacing(8)
        
        # Etiquetas a la izquierda
        voltage_label = QLabel("Voltaje 5V:")
        motor_left_text = QLabel("Corriente Motor Izquierdo:")
        motor_right_text = QLabel("Corriente Motor Derecho:")
        
        for lbl in [voltage_label, motor_left_text, motor_right_text]:
            lbl.setProperty("class", "power-label")
            labels_layout.addWidget(lbl)
        
        # Valores a la derecha (centrados)
        self.voltage_5v_label = QLabel("-- V")
        self.motor_left_label = QLabel("-- A")
        self.motor_right_label = QLabel("-- A")
        
        for lbl in [self.voltage_5v_label, self.motor_left_label, self.motor_right_label]:
            lbl.setProperty("class", "sensor-value-off")
            lbl.setAlignment(Qt.AlignCenter)
            values_right_layout.addWidget(lbl)
        
        # Agregar ambos layouts al layout horizontal
        values_layout.addLayout(labels_layout)
        values_layout.addStretch(120)
        values_layout.addLayout(values_right_layout)
        values_layout.addStretch(100)
        
        layout.addLayout(batt_layout)
        layout.addWidget(values_container)
        
        group.setLayout(layout)
        return group
            
    # === COMUNICACIONES Y SENSORES ===
    def create_device_group(self, group_title, devices):
        group = QGroupBox(group_title)
        layout = QVBoxLayout()
        
        for device_name, device_id in devices:
            device_layout = QHBoxLayout()
            
            # Nombre del dispositivo
            name_label = QLabel(device_name)
            name_label.setProperty("class", "comms-label")
            
            # Estado del dispositivo
            icon, *_ = get_theme_colors()['diagnostic']['error']
            status_label = QLabel(icon)
            status_label.setProperty("class", "comms-status")
            self.sensor_states[device_id] = status_label
            
            device_layout.addWidget(name_label)
            device_layout.addStretch()
            device_layout.addWidget(status_label)
            layout.addLayout(device_layout)
        
        group.setLayout(layout)
        return group


    def _update_sensors_from_components(self, components_msg, values=None):
        """Actualiza los sensores basado en el mensaje de diagnóstico de componentes."""
        try:
            # Extraer información del mensaje de componentes
            values_dict = {kv.key: kv.value for kv in components_msg.values}
            
            # Actualizar sensores Sharp
            sharp_sensors = {
                'sharp_front': 'Sharp Front_status',
                'sharp_left': 'Sharp Left_status', 
                'sharp_right': 'Sharp Right_status'
            }
            
            for sensor_id, status_key in sharp_sensors.items():
                status = values_dict.get(status_key, 'NUNCA_RECIBIDO')

                if status == 'FUNCIONANDO_OK':
                    self._update_sensor_status(sensor_id, 'success', f"{values[sensor_id]:.2f} m")
                elif status == 'DATOS_INVALIDOS':
                    self._update_sensor_status(sensor_id, 'warning', "--")
                else:  # TIMEOUT o NUNCA_RECIBIDO
                    self._update_sensor_status(sensor_id, 'error', "-- m")
            
            # Actualizar IMU
            """imu_status = values_dict.get('IMU_status', 'NUNCA_RECIBIDO')
            if imu_status == 'FUNCIONANDO_OK':
                self._update_sensor_status('imu', 'success', None)
            elif imu_status == 'DATOS_INVALIDOS':
                self._update_sensor_status('imu', 'warning', None)
            else:
                self._update_sensor_status('imu', 'error', None)"""
                
            # Log de cambios de estado
            if not hasattr(self, '_last_components_state'):
                self._last_components_state = {}
                
            current_state = {
                'sharp_front': values_dict.get('Sharp Front_status'),
                'sharp_left': values_dict.get('Sharp Left_status'),
                'sharp_right': values_dict.get('Sharp Right_status'),
                'imu': values_dict.get('IMU_status')
            }
            
            if current_state != self._last_components_state:
                """print(f"Estado Componentes: Front={current_state['sharp_front']}, "
                    f"Left={current_state['sharp_left']}, Right={current_state['sharp_right']}, "
                    f"IMU={current_state['imu']}")"""
                self._last_components_state = current_state
                
        except Exception as e:
            #print(f"Error procesando componentes: {e}")
            pass
    
    def _update_sensor_status(self, sensor_id, status_type, value=None):
        """Actualiza el estado visual de un sensor."""
        # Obtener color según el tipo de estado
        icon, color, background, border_color = get_theme_colors()['diagnostic'][status_type]
        self.sensor_states[sensor_id].setText(icon)
        self.sensor_states[sensor_id].setStyleSheet(f"color: {color};")
        
        # Actualizar valor si se proporciona
        if value is not None and f"{sensor_id}_value" in self.sensor_states:
            self.sensor_states[f"{sensor_id}_value"].setText(value)
            if status_type == 'success':
                self.sensor_states[f"{sensor_id}_value"].setProperty("class", "sensor-value-on")
            else:
                self.sensor_states[f"{sensor_id}_value"].setProperty("class", "sensor-value-off")
    
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

            # Recolectar mensajes DiagnosticStatus
            diagnostics = []

            topics = [
                bridge.battery_12v_status,
                bridge.voltage_5v_status,
                bridge.motor_left_status,
                bridge.motor_right_status,
                bridge.esp32_safety_status,
                bridge.esp32_control_status,
                bridge.microros_agent_status,
                bridge.global_status,
                bridge.camera_status,
                bridge.gps_status,
                bridge.rplidar_status,
                bridge.cpu_temperature_status,
                bridge.gpu_temperature_status,
                bridge.battery_laptop_status,
                bridge.ram_status,
                bridge.cpu_usage_status,
                bridge.gpu_usage_status,
                bridge.disk_temperature_status,
                bridge.uptime_status,
            ]

            for msg in topics:
                if msg:
                    diagnostics.append(msg)

            # Dividir los mensajes por tipo
            safety_msgs = [m for m in diagnostics if "Voltaje" in m.name 
                           or "Corriente" in m.name
                           or "Temperatura" in m.name
                           or "Tiempo" in m.name
                           or "Laptop" in m.name
                           or "Uso" in m.name]
            main_msgs = [m for m in diagnostics if "ESP32" in m.name 
                         or "MICRO" in m.name 
                         or "Sistema" in m.name 
                         or "Cámara" in m.name 
                         or "GPS" in m.name 
                         or "RPLIDAR" in m.name]

            # Actualizar listas
            self.update_diagnostic_list(self.safety_diagnostics_list.list_widget, safety_msgs)
            self.update_diagnostic_list(self.main_diagnostics_list.list_widget, main_msgs)

            # Actualizar sensores Sharp
            sharp_values = {}
            if bridge.range_front:
                sharp_values['sharp_front'] = bridge.range_front.range
            if bridge.range_left:
                sharp_values['sharp_left'] = bridge.range_left.range
            if bridge.range_right:
                sharp_values['sharp_right'] = bridge.range_right.range

            if bridge.components_status:
                self._update_sensors_from_components(bridge.components_status, sharp_values)

            # Actualizar estado del micro-ROS Agent
            if bridge.microros_agent_status:
                level = self._get_message_level(bridge.microros_agent_status)
                status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
                self._update_sensor_status('microros_agent', status_type)

            # Actualizar sistema de potencia
            self._update_power_system(bridge)

            # Actualizar estado ESP32
            self._update_esp32_status(bridge)

            # Actualizar estado de la camara:
            if bridge.camera_status:
                level = self._get_message_level(bridge.camera_status)
                status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
                self._update_sensor_status('camera', status_type)
            
            # Actualizar estado del GPS:
            if bridge.gps_status:
                level = self._get_message_level(bridge.gps_status)
                status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
                self._update_sensor_status('gps', status_type)

            # Actualizar estado del RPLIDAR:
            if bridge.rplidar_status:
                level = self._get_message_level(bridge.rplidar_status)
                status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
                self._update_sensor_status('lidar', status_type)

        except AttributeError as e:
            #print(f"Error accediendo a datos ROS: {e}")
            pass
        except Exception as e:
            #print(f"Error inesperado en update_from_ros: {e}")
            pass

    def _update_power_system(self, bridge):
        """Actualiza el sistema de potencia (batería y motores)."""
        if bridge.battery_array:
            data = bridge.battery_array.data
            if len(data) >= 2:
                voltage_12v = float(data[0])
                voltage_5v = float(data[1])
                percent = self._battery_percent(voltage_12v)

                self.batt_progress.setValue(int(percent))
                
                # Determinar color según nivel de voltaje
                level = self._get_message_level(bridge.voltage_5v_status)
                voltage_color = '#e74c3c' if level == 2 else \
                               '#f1c40f' if level == 1 else \
                               '#ecf0f1'

                # Actualizar etiqueta de voltaje
                mensaje = bridge.voltage_5v_status.message
                if "CRÍTICO" in mensaje or "BAJO" in mensaje or "NORMAL" in mensaje:
                    self.voltage_5v_label.setText(f"{voltage_5v:.2f} V")
                else:
                    self.voltage_5v_label.setText("-- V")
                
                # Aplicar clase dinámica según el estado
                if level == 2:
                    self.voltage_5v_label.setProperty("class", "value-error")
                elif level == 1:
                    self.voltage_5v_label.setProperty("class", "value-warning")
                else:
                    self.voltage_5v_label.setProperty("class", "value-success")

                # Cambiar color según nivel
                if percent < Constants.BATTERY_PERCENTAGE_MIN:
                    chunk_color = Constants.COLORS['error'] # Rojo
                elif percent < Constants.BATTERY_PERCENTAGE_LIMIT:
                    chunk_color = Constants.COLORS['warning'] # Naranja
                else:
                    chunk_color = Constants.COLORS['success'] # Verde


                self.batt_progress.setStyleSheet(f"""
                    QProgressBar::chunk {{
                    background-color: {chunk_color};
                    }}
                """)

        else:
            self.voltage_5v_label.setText("-- V")
            self.voltage_5v_label.setProperty("class", "sensor-value-off")
            self.batt_progress.setValue(0)

        if bridge.motors_array:
            data = bridge.motors_array.data
            if len(data) >= 2:
                current_left = data[0]
                current_right = data[1]

                # Determinar colores según nivel
                left_level = self._get_message_level(bridge.motor_left_status)
                right_level = self._get_message_level(bridge.motor_right_status)

                # Actualizar etiquetas de corriente
                left_mensaje = bridge.motor_left_status.message
                right_mensaje = bridge.motor_right_status.message

                if "CRÍTICA" in left_mensaje or "ELEVADA" in left_mensaje or "NORMAL" in left_mensaje:
                    self.motor_left_label.setText(f"{current_left:.2f} A")
                else:
                    self.motor_left_label.setText("-- A")

                if "CRÍTICA" in right_mensaje or "ELEVADA" in right_mensaje or "NORMAL" in right_mensaje:
                    self.motor_right_label.setText(f"{current_right:.2f} A")
                else:
                    self.motor_right_label.setText("-- A")

                # Aplicar clases dinámicas según el estado
                if left_level == 2:
                    self.motor_left_label.setProperty("class", "value-error")
                elif left_level == 1:
                    self.motor_left_label.setProperty("class", "value-warning")
                else:
                    self.motor_left_label.setProperty("class", "value-success")

                if right_level == 2:
                    self.motor_right_label.setProperty("class", "value-error")
                elif right_level == 1:
                    self.motor_right_label.setProperty("class", "value-warning")
                else:
                    self.motor_right_label.setProperty("class", "value-success")
        else:
            self.motor_left_label.setText("-- A")
            self.motor_right_label.setText("-- A")
            self.motor_left_label.setProperty("class", "sensor-value-off")
            self.motor_right_label.setProperty("class", "sensor-value-off")

    def _update_esp32_status(self, bridge):
        """Actualiza el estado de las comunicaciones ESP32."""
        # Actualizar ESP32 Seguridad
        if bridge.esp32_safety_status:
            level = self._get_message_level(bridge.esp32_safety_status)
            status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
            self._update_sensor_status('esp32_safety', status_type)
            
            if not hasattr(self, '_last_esp1_level') or self._last_esp1_level != level:
                level_text = "OK" if level == 0 else "WARN" if level == 1 else "ERROR"
                #print(f"ESP32 Safety: {level_text} - {bridge.esp32_safety_status.message}")
                self._last_esp1_level = level
        else:
            self._update_sensor_status('esp32_safety', 'error')
            if not hasattr(self, '_last_esp1_level') or self._last_esp1_level != -1:
                #print(f"ESP32 Safety: SIN DATOS")
                self._last_esp1_level = -1

        # Actualizar ESP32 Control
        if bridge.esp32_control_status:
            level = self._get_message_level(bridge.esp32_control_status)
            status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
            self._update_sensor_status('esp32_control', status_type)
            
            if not hasattr(self, '_last_esp2_level') or self._last_esp2_level != level:
                level_text = "OK" if level == 0 else "WARN" if level == 1 else "ERROR"
                #print(f"ESP32 Control: {level_text} - {bridge.esp32_control_status.message}")
                self._last_esp2_level = level
        else:
            self._update_sensor_status('esp32_control', 'error')
            if not hasattr(self, '_last_esp2_level') or self._last_esp2_level != -1:
                #print(f"ESP32 Control: SIN DATOS")
                self._last_esp2_level = -1

    def _battery_percent(self, voltage):
        """Calcula el porcentaje de batería basado en el voltaje."""
        vmin = Constants.BATTERY_12V_MIN
        vmax = Constants.BATTERY_12V_MAX
        pct = (voltage - vmin) / (vmax - vmin) * 100
        return max(0.0, min(100.0, pct))

    def _show_no_bridge_error(self):
        """Muestra un mensaje de error cuando no hay bridge disponible."""
        #print("No hay bridge ROS disponible")