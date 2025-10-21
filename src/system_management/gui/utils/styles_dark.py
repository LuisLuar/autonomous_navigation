# Estilos CSS para la interfaz HMI
def get_app_style():
    return """
    /* Estilos generales de la aplicación */
    QMainWindow {
        background-color: #2c3e50;
        color: #ecf0f1;
    }
    
    QWidget {
        background-color: #34495e;
        color: #ecf0f1;
        font-family: 'Segoe UI', Arial, sans-serif;
    }
    
    QLabel {
        color: #ecf0f1;
        font-size: 14px;
    }
    
    /* Pestañas */
    QTabWidget::pane {
        border: 2px solid #34495e;
        background-color: #34495e;
        border-radius: 4px;
    }
    
    QTabWidget::tab-bar {
        alignment: center;
    }
    
    QTabBar::tab {
        background-color: #34495e;
        color: #bdc3c7;
        padding: 12px 24px;
        margin: 2px;
        border: none;
        border-top-left-radius: 4px;
        border-top-right-radius: 4px;
        font-weight: bold;
        font-size: 12px;
    }
    
    QTabBar::tab:selected {
        background-color: #3498db;
        color: #ffffff;
    }
    
    QTabBar::tab:hover {
        background-color: #2980b9;
        color: #ffffff;
    }
    
    /* Botones */
    QPushButton {
        background-color: #3498db;
        color: white;
        border: none;
        padding: 8px 16px;
        border-radius: 4px;
        font-weight: bold;
        min-width: 80px;
    }
    
    QPushButton:hover {
        background-color: #2980b9;
    }
    
    QPushButton:pressed {
        background-color: #21618c;
    }
    
    /* Grupos de widgets */
    QGroupBox {
        color: #3498db;
        font-weight: bold;
        border: 2px solid #3498db;
        border-radius: 5px;
        margin-top: 10px;
        padding-top: 10px;
    }
    
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 5px 0 5px;
        color: #3498db;
    }
    
    /* Barra de estado */
    QStatusBar {
        background-color: #2c3e50;
        color: #bdc3c7;
        border-top: 1px solid #34495e;
    }
    
    /* Widget de monitoreo de sensores */
    .sensor-title {
        color: #3498db;
        font-size: 16px;
        font-weight: bold;
    }
    
    .sensor-label {
        color: #ecf0f1;
        font-weight: bold;
        font-size: 14px;
    }
    
    .sensor-status {
        color: #e74c3c;
        font-size: 16px;
    }
    
    .sensor-value {
        color: #bdc3c7;
        font-size: 14px;
    }
    
    .sensor-value-on {
        color: #ecf0f1;
        font-size: 14px;
        font-weight: bold;
    }
    
    .sensor-value-off {
        color: #bdc3c7;
        font-size: 14px;
    }
    
    .power-label {
        color: #ecf0f1;
        font-weight: bold;
        font-size: 14px;
    }
    
    .power-value {
        color: #ecf0f1;
        font-size: 14px;
        font-weight: bold;
    }
    
    .battery-progress {
        border: 2px solid #34495e;
        border-radius: 5px;
        text-align: center;
        color: white;
        font-weight: bold;
    }
    
    .battery-progress::chunk {
        background-color: #27ae60;
        border-radius: 3px;
    }
    
    .comms-label {
        color: #ecf0f1;
        font-weight: bold;
        font-size: 14px;
    }
    
    .comms-status {
        font-size: 16px;
    }
    
    /* Lista de diagnóstico */
    .diagnostic-list {
        background-color: #1e272e;
        color: #ecf0f1;
        font-size: 14px;
        border: 1px solid #2f3640;
        border-radius: 6px;
        outline: none;
    }
    
    .diagnostic-list::item {
        border-bottom: 1px solid #2f3640;
        padding: 8px 5px;
        margin: 2px 5px;
        border-radius: 4px;
    }
    
    .diagnostic-list::item:selected {
        background-color: #34495e;
        border: 1px solid #3498db;
    }
    
    .diagnostic-list::item:hover {
        background-color: #2c3e50;
    }
    
    .diagnostic-counter {
        color: #bdc3c7;
        font-size: 12px;
        font-weight: bold;
        padding: 5px;
        background-color: #2c3e50;
        border-radius: 4px;
    }
    
    /* Indicadores de estado */
    .status-icon {
        font-size: 16px;
        font-weight: bold;
    }
    
    .status-success {
        color: #27ae60;
    }
    
    .status-warning {
        color: #f1c40f;
    }
    
    .status-error {
        color: #e74c3c;
    }
    
    /* Barra de título */
    .title-bar {
        background-color: #2c3e50;
        border-bottom: 2px solid #3498db;
    }
    
    .title-bar-title {
        color: #3498db;
        font-size: 18px;
        font-weight: bold;
        background-color: transparent;
    }
    
    .system-status {
        color: #e74c3c;
        font-size: 12px;
        font-weight: bold;
        background-color: transparent;
        padding: 5px 10px;
        border: 1px solid #e74c3c;
        border-radius: 3px;
    }
    
    .ros-indicator {
        font-weight: bold;
    }
    
    .ros-connected {
        color: #27ae60;
    }
    
    .ros-disconnected {
        color: #e74c3c;
    }
    
    /* Estilos para valores dinámicos */
    .value-success {
        color: #27ae60;
        font-weight: bold;
    }
    
    .value-warning {
        color: #f1c40f;
        font-weight: bold;
    }
    
    .value-error {
        color: #e74c3c;
        font-weight: bold;
    }
    """

def get_map_widget_style():
    """Estilos específicos para el widget del mapa"""
    return """
    /* Estilos específicos para el widget del mapa */
    #info_label {
        color: #ecf0f1; 
        font-weight: bold;
        font-size: 13px;
        padding: 5px;
    }
    
    #set_dest_btn {
        background-color: #27ae60;
        color: white;
        font-weight: bold;
        padding: 8px 16px;
        border-radius: 4px;
    }
    
    #set_dest_btn:hover {
        background-color: #229954;
    }
    
    #set_dest_btn:disabled {
        background-color: #7f8c8d;
        color: #bdc3c7;
    }
    
    #clear_dest_btn {
        background-color: #e74c3c;
        color: white;
        font-weight: bold;
        padding: 8px 16px;
        border-radius: 4px;
    }
    
    #clear_dest_btn:hover {
        background-color: #c0392b;
    }
    """

def get_theme_colors():
    """Devuelve todos los colores del tema."""
    return {
        'diagnostic': {
            'error': ("🔴", "#e74c3c", "#fadbd8", "#e74c3c"),
            'warning': ("⚠️", "#f1c40f", "#fef9e7", "#f1c40f"),
            'ok': ("⚪", "#ecf0f1", "#34495e", "#bdc3c7")
        },
        'sensors': {
            'success': '#27ae60',
            'warning': '#f1c40f', 
            'error': '#e74c3c'
        },
        'map': {
            'primary': '#516B82',
            'secondary': '#d8d8d8',
            'accent': '#27ae60',
            'danger': '#e74c3c',
            'info': '#ecf0f1'
        }
    }