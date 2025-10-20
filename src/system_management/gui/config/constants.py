# Configuraciones constantes de la aplicación
class Constants:
    APP_TITLE = "Robot Autónomo Universitario - Control Interface"
    APP_VERSION = "1.0.0"
    WINDOW_WIDTH = 1400
    WINDOW_HEIGHT = 900

    # Battery mapping (configurable)
    BATTERY_12V_MIN = 11.0   # 0%
    BATTERY_12V_MAX = 13.0   # 100%  (ajustable)

    BATTERY_PERCENTAGE_MIN = 20
    BATTERY_PERCENTAGE_LIMIT = 30

    MOTOR_CURRENT_MAX = 30
    MOTOR_CURRENT_LIMIT = 20

    CONTROL_VOLTAGE_MIN = 3
    CONTROL_VOLTAGE_LIMIT = 4

    # Paleta de colores estilo HMI industrial
    COLORS = {
        'primary': '#2c3e50',
        'secondary': '#34495e',
        'accent': '#3498db',
        'success': '#27ae60',
        'warning': '#f1c40f',
        'error': '#e74c3c',
        'dark': '#2c3e50',
        'text_primary': '#ecf0f1',
        'text_secondary': '#bdc3c7',
        'background': '#1e272e'
    }

    # Tamaños de fuente
    FONT_SIZE = {
        'text_small': '12px',  # Letras pequeñas
        'icons': '16px',
        'title': '16px',
        'text_normal': '14px'
    }
    
    # Estilos de alertas
    ALERT_LEVELS = {
        'CRITICAL': 'danger',
        'WARNING': 'warning',
        'INFO': 'success',
        'DEBUG': 'accent'
    }
    
    # Estilos unificados para la interfaz
    STYLES = {
        # Títulos de grupos (SENSORES PRINCIPALES, SISTEMA DE POTENCIA, etc.)
        'group_title': """
            QGroupBox {
                color: {COLORS['accent']};
                font-weight: bold;
                font-size: {FONT_SIZE['text_normal']};
            }
        """,
        
        # Labels de sensores (LIDAR 360°, Cámara RGB-D, etc.)
        'sensor_label': f"""
            color: {COLORS['text_primary']}; 
            font-weight: bold;
            font-size: {FONT_SIZE['text_normal']};
        """,
        
        # Labels de valores (--, 12.45V, 1.2A, etc.)
        'value_label_off': f"""
            color: {COLORS['text_secondary']}; 
            font-size: {FONT_SIZE['text_normal']};
        """,
        'value_label_on': f"""
            color: {COLORS['text_primary']}; 
            font-size: {FONT_SIZE['text_normal']};
        """,
        
        # Labels de sistema de potencia (Batería 12V, Voltaje 5V, etc.)
        'power_label': f"""
            color: {COLORS['text_primary']}; 
            font-weight: bold;
            font-size: {FONT_SIZE['text_normal']};
        """,
        
        # Labels de comunicaciones (micro-ROS Agent, ESP32 Seguridad, etc.)
        'comms_label': f"""
            color: {COLORS['text_primary']}; 
            font-weight: bold;
            font-size: {FONT_SIZE['text_normal']};
        """,
        
        # Estado de sensores (LED virtual ●)
        'sensor_status': f"""
            color: {COLORS['error']}; 
            font-size: {FONT_SIZE['icons']};
        """,
        
        # Estado de comunicaciones (🔴 🟢 ⚠️ ❌)
        'comms_status': f"""
            font-size: {FONT_SIZE['icons']};
        """,

        # LISTADO DE MENSAJES DIAGNÓSTICO 
        'list_style': f"""
            QListWidget {{
                background-color: {COLORS['background']};
                color: {COLORS['text_primary']};
                font-size: {FONT_SIZE['text_normal']};
                border: 1px solid #2f3640;
                border-radius: 6px;
                outline: none;
            }}
            QListWidget::item {{
                border-bottom: 1px solid #2f3640;
                padding: 8px 5px;
                margin: 2px 5px;
                border-radius: 4px;
            }}
            QListWidget::item:selected {{
                background-color: #34495e;
                border: 1px solid {COLORS['accent']};
            }}
            QListWidget::item:hover {{
                background-color: #2c3e50;
            }}
        """,

        # Contador de estados
        'count_label': f"""
            QLabel {{
                color: {COLORS['text_secondary']};
                font-size: {FONT_SIZE['text_small']};
                font-weight: bold;
                padding: 5px;
                background-color: #2c3e50;
                border-radius: 4px;
            }}
        """,

        # Barra de progreso de la batería
        'batt_progress': """
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
        """,
        
        # Estilo para elementos de estado (LEDs, iconos)
        'status_icon': f"""
            font-size: {FONT_SIZE['icons']};
        """,
        
        # Estilo para etiquetas de valor con color dinámico
        'value_label_dynamic': f"""
            font-size: {FONT_SIZE['text_normal']};
            font-weight: bold;
        """
    }