from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                               QGroupBox, QLabel, QProgressBar, QPushButton, QListWidget, QListWidgetItem)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtCore import QSize

from config.constants import Constants

class SensorMonitorWidget(QWidget):
    def __init__(self, ros_node=None):
        """
        Widget principal para el monitoreo de sensores y sistema.
        
        Args:
            ros_node: Nodo ROS2 para la comunicación
        """
        super().__init__()
        self.ros_node = ros_node
        self.sensor_states = {}  # Diccionario para almacenar estados de sensores
        self.diagnostic_messages = []  # Almacena los últimos mensajes DiagnosticStatus
        self.setup_ui()

        # Timer para actualizar UI desde ros_node
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_from_ros)
        self.update_timer.start(500)  # cada 500 ms
    
    # ==============================================================
    # CONFIGURACIÓN UI
    # ==============================================================
    def setup_ui(self):
        """Configura la interfaz de usuario principal."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Título de la sección
        title = QLabel("MONITOREO DE SENSORES Y SISTEMA")
        title.setStyleSheet(f"color: {Constants.COLORS['accent']}; font-size: {Constants.FONT_SIZE['title']}; font-weight: bold;")
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
        main_sensors_group = self.create_sensor_group("SENSORES PRINCIPALES", [
            ("LIDAR 360°", "lidar"),
            ("Cámara RGB-D", "camera"),
            ("GPS", "gps"),
            ("IMU", "imu")
        ])
        grid_layout.addWidget(main_sensors_group, 1, 0)
        
        safety_sensors_group = self.create_sensor_group("SENSORES DE SEGURIDAD", [
            ("Sharp frontal", "sharp_front"),
            ("Sharp izquierdo", "sharp_left"),
            ("Sharp derecho", "sharp_right"),
            ("Pulsador Emergencia", "emergency")
        ])
        grid_layout.addWidget(safety_sensors_group, 1, 1)

        # === SISTEMA DE POTENCIA ===
        power_group = self.create_power_group()
        grid_layout.addWidget(power_group, 2, 0)
        
        # === COMUNICACIONES ===
        comms_group = self.create_comms_group()
        grid_layout.addWidget(comms_group, 2, 1)
        
        layout.addLayout(grid_layout)

    # ==============================================================
    # LISTADO DE MENSAJES DIAGNÓSTICO MEJORADO
    # ==============================================================
    def create_diagnostic_list(self, title):
        """
        Crea un grupo con lista para mostrar mensajes de diagnóstico.
        
        Args:
            title: Título del grupo
            
        Returns:
            QGroupBox: Contenedor con la lista de diagnóstico
        """
        group = QGroupBox(f"📋 {title} - ESTADOS DEL SISTEMA")
        group.setStyleSheet(Constants.STYLES['group_title'])
        layout = QVBoxLayout()
        
        # Contador de estados
        status_count_label = QLabel("⚪ 0 | ⚠️ 0 | 🔴 0")
        status_count_label.setStyleSheet(Constants.STYLES['count_label'])
        status_count_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(status_count_label)
        
        list_widget = QListWidget()
        list_widget.setStyleSheet(Constants.STYLES['list_style'])
        list_widget.setSpacing(3)  # Espacio entre items
        list_widget.setUniformItemSizes(False)  # Permite diferentes alturas
        
        layout.addWidget(list_widget)
        group.setLayout(layout)
        group.list_widget = list_widget
        group.status_count_label = status_count_label
        
        return group

    def update_diagnostic_list(self, list_widget, diagnostics):
        """
        Actualiza la lista de diagnóstico con los mensajes recibidos.
        
        Args:
            list_widget: QListWidget a actualizar
            diagnostics: Lista de mensajes DiagnosticStatus
        """
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
        """
        Extrae el nivel de un mensaje de forma segura.
        
        Args:
            msg: Mensaje DiagnosticStatus
            
        Returns:
            int: Nivel del mensaje (0=OK, 1=WARNING, 2=ERROR)
        """
        level_raw = msg.level
        if isinstance(level_raw, bytes):
            return int.from_bytes(level_raw, byteorder='little', signed=False)
        else:
            try:
                return int(level_raw)
            except Exception:
                return 2

    def _get_diagnostic_style(self, level):
        """
        Obtiene el estilo visual para un nivel de diagnóstico.
        
        Args:
            level: Nivel de diagnóstico (0=OK, 1=WARNING, 2=ERROR)
            
        Returns:
            tuple: (icon, color, background, border_color)
        """
        if level == 2:  # ERROR
            return "🔴", Constants.COLORS['error'], "#2d1b1b", "#c0392b"
        elif level == 1:  # WARN
            return "⚠️", Constants.COLORS['warning'], "#2d2b1b", "#d35400"
        else:  # OK
            return "⚪", Constants.COLORS['text_primary'], "#1b2d1b", Constants.COLORS['text_primary']

    def _add_separator_item(self, list_widget):
        """
        Agrega un separador visual entre grupos de niveles.
        
        Args:
            list_widget: QListWidget donde agregar el separador
        """
        separator = QListWidgetItem()
        separator.setFlags(Qt.NoItemFlags)  # Hacerlo no seleccionable
        separator.setBackground(QColor("#34495e"))
        separator.setSizeHint(QSize(separator.sizeHint().width(), 2))
        list_widget.addItem(separator)
    
    # === GRUPOS DE SENSORES ===
    def create_sensor_group(self, title, sensors):
        """
        Crea un grupo de sensores con su estado y valor.
        
        Args:
            title: Título del grupo
            sensors: Lista de tuplas (nombre_sensor, id_sensor)
            
        Returns:
            QGroupBox: Contenedor con los sensores
        """
        group = QGroupBox(title)
        group.setStyleSheet(Constants.STYLES['group_title'])
        
        layout = QGridLayout()
        layout.setSpacing(10)
        
        for i, (sensor_name, sensor_id) in enumerate(sensors):
            # Nombre del sensor
            name_label = QLabel(sensor_name)
            name_label.setStyleSheet(Constants.STYLES['sensor_label'])
            
            # Estado del sensor (LED virtual)
            status_label = QLabel("●")
            status_label.setStyleSheet(Constants.STYLES['sensor_status'])
            self.sensor_states[sensor_id] = status_label
            
            # Valor simulado
            value_label = QLabel("--")
            value_label.setStyleSheet(Constants.STYLES['value_label_off'])
            self.sensor_states[f"{sensor_id}_value"] = value_label
            
            layout.addWidget(name_label, i, 0)
            layout.addWidget(status_label, i, 1)
            layout.addWidget(value_label, i, 2)
        
        group.setLayout(layout)
        return group
    
    # === SISTEMA DE POTENCIA ===
    def create_power_group(self):
        """
        Crea el grupo para el sistema de potencia.
        
        Returns:
            QGroupBox: Contenedor con el sistema de potencia
        """
        group = QGroupBox("SISTEMA DE POTENCIA")
        group.setStyleSheet(Constants.STYLES['group_title'])
        layout = QVBoxLayout()

        # Batería 12V
        batt_layout = QHBoxLayout()
        batt_label = QLabel("Batería 12V:")
        batt_label.setStyleSheet(Constants.STYLES['power_label'])
        
        # Barra de progreso de la batería
        self.batt_progress = QProgressBar()
        self.batt_progress.setRange(0, 100)
        self.batt_progress.setValue(0)
        self.batt_progress.setStyleSheet(Constants.STYLES['batt_progress'])

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
            lbl.setStyleSheet(Constants.STYLES['power_label'])
            labels_layout.addWidget(lbl)
        
        # Valores a la derecha (centrados)
        self.voltage_5v_label = QLabel("-- V")
        self.motor_left_label = QLabel("-- A")
        self.motor_right_label = QLabel("-- A")
        
        for lbl in [self.voltage_5v_label, self.motor_left_label, self.motor_right_label]:
            lbl.setStyleSheet(Constants.STYLES['value_label_off'])
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
            
    # === COMUNICACIONES ===
    def create_comms_group(self):
        """
        Crea el grupo para las comunicaciones.
        
        Returns:
            QGroupBox: Contenedor con las comunicaciones
        """
        group = QGroupBox("COMUNICACIONES")
        group.setStyleSheet(Constants.STYLES['group_title'])
        layout = QVBoxLayout()
        
        comms = [
            ("micro-ROS Agent", "microros_agent"),
            ("ESP32 Seguridad", "esp32_safety"),
            ("ESP32 Control", "esp32_control"),
        ]
        
        for comm_name, comm_id in comms:
            comm_layout = QHBoxLayout()
            
            # Nombre de comunicación
            name_label = QLabel(comm_name)
            name_label.setStyleSheet(Constants.STYLES['comms_label'])
            
            # Estado de comunicación
            status_label = QLabel("🔴")
            status_label.setStyleSheet(Constants.STYLES['comms_status'])
            self.sensor_states[comm_id] = status_label
            
            comm_layout.addWidget(name_label)
            comm_layout.addStretch()
            comm_layout.addWidget(status_label)
            layout.addLayout(comm_layout)
        
        group.setLayout(layout)
        return group
    
    def _update_sensors_from_components(self, components_msg, values=None):
        """
        Actualiza los sensores basado en el mensaje de diagnóstico de componentes.
        
        Args:
            components_msg: Mensaje de diagnóstico de componentes
            values: Diccionario con valores de los sensores Sharp
        """
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
            imu_status = values_dict.get('IMU_status', 'NUNCA_RECIBIDO')
            if imu_status == 'FUNCIONANDO_OK':
                self._update_sensor_status('imu', 'success', None)
            elif imu_status == 'DATOS_INVALIDOS':
                self._update_sensor_status('imu', 'warning', None)
            else:
                self._update_sensor_status('imu', 'error', None)
                
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
                print(f"🔧 Estado Componentes: Front={current_state['sharp_front']}, "
                    f"Left={current_state['sharp_left']}, Right={current_state['sharp_right']}, "
                    f"IMU={current_state['imu']}")
                self._last_components_state = current_state
                
        except Exception as e:
            print(f"⚠️ Error procesando componentes: {e}")
    
    def _update_sensor_status(self, sensor_id, status_type, value=None):
        """
        Actualiza el estado visual de un sensor.
        
        Args:
            sensor_id: ID del sensor
            status_type: Tipo de estado ('success', 'warning', 'error')
            value: Valor a mostrar (opcional)
        """
        # Obtener color según el tipo de estado
        color_map = {
            'success': Constants.COLORS['success'],
            'warning': Constants.COLORS['warning'],
            'error': Constants.COLORS['error']
        }
        
        color = color_map.get(status_type, Constants.COLORS['text_primary'])
        
        # Actualizar estado
        icon_map = {
            'success': '🟢',
            'warning': '⚠️',
            'error': '❌'
        }
        
        icon = icon_map.get(status_type, '●')
        self.sensor_states[sensor_id].setText(icon)
        self.sensor_states[sensor_id].setStyleSheet(f"font-size: {Constants.FONT_SIZE['icons']}; color: {color};")
        
        # Actualizar valor si se proporciona
        if value is not None and f"{sensor_id}_value" in self.sensor_states:
            self.sensor_states[f"{sensor_id}_value"].setText(value)
            if status_type == 'success':
                self.sensor_states[f"{sensor_id}_value"].setStyleSheet(Constants.STYLES['value_label_on'])
            else:
                self.sensor_states[f"{sensor_id}_value"].setStyleSheet(Constants.STYLES['value_label_off'])
    
    # ==============================================================
    # ACTUALIZACIÓN DESDE ROS2
    # ==============================================================
    def update_from_ros(self):
        """
        Actualiza los datos desde ROS2 - VERSIÓN OPTIMIZADA.
        """
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
                bridge.components_status, 
            ]

            for msg in topics:
                if msg:
                    diagnostics.append(msg)

            # Dividir los mensajes por tipo
            safety_msgs = [m for m in diagnostics if "Voltaje" in m.name or "Corriente" in m.name]
            main_msgs = [m for m in diagnostics if "ESP32" in m.name or "MICRO" in m.name  or "Componentes" in m.name]

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

        except AttributeError as e:
            print(f"⚠️ Error accediendo a datos ROS: {e}")
        except Exception as e:
            print(f"❌ Error inesperado en update_from_ros: {e}")

    def _update_power_system(self, bridge):
        """
        Actualiza el sistema de potencia (batería y motores).
        
        Args:
            bridge: Objeto ROSBridge con los datos
        """
        if bridge.battery_array:
            data = bridge.battery_array.data
            if len(data) >= 2:
                voltage_12v = float(data[0])
                voltage_5v = float(data[1])
                percent = self._battery_percent(voltage_12v)

                self.batt_progress.setValue(int(percent))
                
                # Determinar color según nivel de voltaje
                level = self._get_message_level(bridge.voltage_5v_status)
                voltage_color = Constants.COLORS['error'] if level == 2 else \
                               Constants.COLORS['warning'] if level == 1 else \
                               Constants.COLORS['text_primary']

                # Actualizar etiqueta de voltaje
                mensaje = bridge.voltage_5v_status.message
                if "CRÍTICO" in mensaje or "BAJO" in mensaje or "NORMAL" in mensaje:
                    self.voltage_5v_label.setText(f"{voltage_5v:.2f} V")
                else:
                    self.voltage_5v_label.setText("-- V")
                
                self.voltage_5v_label.setStyleSheet(f"color: {voltage_color}; font-weight: bold; font-size: {Constants.FONT_SIZE['text_small']};")

                # Cambiar color de la barra de progreso según el porcentaje
                chunk_color = Constants.COLORS['error'] if percent < Constants.BATTERY_PERCENTAGE_MIN else \
                            Constants.COLORS['warning'] if percent < Constants.BATTERY_PERCENTAGE_LIMIT else \
                            Constants.COLORS['success']
                            
                self.batt_progress.setStyleSheet(f"""
                    QProgressBar {{ 
                        border: 2px solid #34495e; 
                        border-radius: 5px;
                        text-align: center; 
                        color: white; 
                        font-weight: bold; 
                    }}
                    QProgressBar::chunk {{ 
                        background-color: {chunk_color}; 
                        border-radius: 3px; 
                    }}
                """)
        else:
            self.voltage_5v_label.setText("-- V")
            self.voltage_5v_label.setStyleSheet(Constants.STYLES['value_label_on'])
            self.batt_progress.setValue(0)

        if bridge.motors_array:
            data = bridge.motors_array.data
            if len(data) >= 2:
                current_left = data[0]
                current_right = data[1]

                # Determinar colores según nivel
                left_level = self._get_message_level(bridge.motor_left_status)
                left_color = Constants.COLORS['error'] if left_level == 2 else \
                            Constants.COLORS['warning'] if left_level == 1 else \
                            Constants.COLORS['text_primary']

                right_level = self._get_message_level(bridge.motor_right_status)
                right_color = Constants.COLORS['error'] if right_level == 2 else \
                             Constants.COLORS['warning'] if right_level == 1 else \
                             Constants.COLORS['text_primary']

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

                self.motor_left_label.setStyleSheet(f"color: {left_color}; font-weight: bold; font-size: {Constants.FONT_SIZE['text_small']};")
                self.motor_right_label.setStyleSheet(f"color: {right_color}; font-weight: bold; font-size: {Constants.FONT_SIZE['text_small']};")
        else:
            self.motor_left_label.setText("-- A")
            self.motor_right_label.setText("-- A")
            self.motor_left_label.setStyleSheet(Constants.STYLES['value_label_off'])
            self.motor_right_label.setStyleSheet(Constants.STYLES['value_label_off'])

    def _update_esp32_status(self, bridge):
        """
        Actualiza el estado de las comunicaciones ESP32.
        
        Args:
            bridge: Objeto ROSBridge con los datos
        """
        # Actualizar ESP32 Seguridad
        if bridge.esp32_safety_status:
            level = self._get_message_level(bridge.esp32_safety_status)
            status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
            self._update_sensor_status('esp32_safety', status_type)
            
            if not hasattr(self, '_last_esp1_level') or self._last_esp1_level != level:
                level_text = "OK" if level == 0 else "WARN" if level == 1 else "ERROR"
                print(f"📡 ESP32 Safety: {level_text} - {bridge.esp32_safety_status.message}")
                self._last_esp1_level = level
        else:
            self._update_sensor_status('esp32_safety', 'error')
            if not hasattr(self, '_last_esp1_level') or self._last_esp1_level != -1:
                print(f"📡 ESP32 Safety: SIN DATOS")
                self._last_esp1_level = -1

        # Actualizar ESP32 Control
        if bridge.esp32_control_status:
            level = self._get_message_level(bridge.esp32_control_status)
            status_type = 'success' if level == 0 else 'warning' if level == 1 else 'error'
            self._update_sensor_status('esp32_control', status_type)
            
            if not hasattr(self, '_last_esp2_level') or self._last_esp2_level != level:
                level_text = "OK" if level == 0 else "WARN" if level == 1 else "ERROR"
                print(f"📡 ESP32 Control: {level_text} - {bridge.esp32_control_status.message}")
                self._last_esp2_level = level
        else:
            self._update_sensor_status('esp32_control', 'error')
            if not hasattr(self, '_last_esp2_level') or self._last_esp2_level != -1:
                print(f"📡 ESP32 Control: SIN DATOS")
                self._last_esp2_level = -1

    def _battery_percent(self, voltage):
        """
        Calcula el porcentaje de batería basado en el voltaje.
        
        Args:
            voltage: Voltaje medido
            
        Returns:
            float: Porcentaje de batería (0-100)
        """
        vmin = Constants.BATTERY_12V_MIN
        vmax = Constants.BATTERY_12V_MAX
        pct = (voltage - vmin) / (vmax - vmin) * 100
        return max(0.0, min(100.0, pct))

    def _show_no_bridge_error(self):
        """
        Muestra un mensaje de error cuando no hay bridge disponible.
        """
        print("❌ No hay bridge ROS disponible")