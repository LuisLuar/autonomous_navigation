# styles_light.py
# Estilos CSS para la interfaz HMI 
def get_app_style():
    return """
    /* Estilos generales de la aplicación */
    QMainWindow {
        background-color: #cbcbcb;
        color: #1c1c1c;
    }
    
    QWidget {
        background-color: #d8d8d8;
        color: #1c1c1c;
        font-family: 'Segoe UI', Arial, sans-serif;
    }
    
    QLabel {
        color: #2c3e50;
        font-size: 14px;
    }
    
    /* Pestañas */
    QTabWidget::pane {
        border: 2px solid #aaaaaa;
        background-color: #fafafa;
        border-radius: 4px;
    }
    
    QTabWidget::tab-bar {
        alignment: center;
    }
    
    QTabBar::tab {
        background-color: #e8e8e8;
        color: #555555;
        padding: 12px 24px;
        margin: 2px;
        border: none;
        border-top-left-radius: 4px;
        border-top-right-radius: 4px;
        font-weight: bold;
        font-size: 12px;
    }
    
    QTabBar::tab:selected {
        background-color: #516B82;
        color: #d8d8d8;
    }
    
    QTabBar::tab:hover {
        background-color: #f0f0f0;
        color: #000000;
    }
    
    /* Botones */
    QPushButton {
        background-color: #e0e0e0;
        color: #000000;
        border: none;
        padding: 8px 16px;
        border-radius: 4px;
        font-weight: bold;
        min-width: 80px;
    }
    
    QPushButton:hover {
        background-color: #d6d6d6;
    }
    
    QPushButton:pressed {
        background-color: #bfbfbf;
    }
    
    /* Grupos de widgets */
    QGroupBox {
        color: #516B82;
        font-weight: bold;
        border: 2px solid #516B82;
        border-radius: 5px;
        margin-top: 10px;
        padding-top: 10px;
    }
    
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 5px 0 5px;
        color: #516B82;
    }
    
    /* Barra de estado */
    QStatusBar {
        background-color: #cbcbcb;
        color: #555555;
        border-top: 1px solid #d8d8d8;
    }
    
    /* Widget de monitoreo de sensores */
    .sensor-title {
        color: #516B82;
        font-size: 16px;
        font-weight: bold;
    }
    
    .sensor-label {
        color: #1c1c1c;
        font-weight: bold;
        font-size: 14px;
    }
    
    .sensor-status {
        color: #e74c3c;
        font-size: 16px;
    }
    
    .sensor-value {
        color: #555555;
        font-size: 14px;
    }
    
    .sensor-value-on {
        color: #1c1c1c;
        font-size: 14px;
        font-weight: bold;
    }
    
    .sensor-value-off {
        color: #555555;
        font-size: 14px;
    }
    
    .power-label {
        color: #1c1c1c;
        font-weight: bold;
        font-size: 14px;
    }
    
    .power-value {
        color: #1c1c1c;
        font-size: 14px;
        font-weight: bold;
    }
    
    .battery-progress {
        border: 2px solid #bababa;
        border-radius: 5px;
        text-align: center;
        color: #000000;
        font-weight: bold;
    }
    
    .battery-progress::chunk {
        background-color: #27ae60;
        border-radius: 3px;
    }
    
    .comms-label {
        color: #1c1c1c;
        font-weight: bold;
        font-size: 14px;
    }
    
    .comms-status {
        font-size: 16px;
    }
    
    /* Lista de diagnóstico */
    .diagnostic-list {
        background-color: #d8d8d8;
        color: #333333;
        font-size: 14px;
        border: 1px solid #aaaaaa;
        border-radius: 6px;
        outline: none;
    }
    
    .diagnostic-list::item {
        border-bottom: 1px solid #aaaaaa;
        padding: 8px 5px;
        margin: 2px 5px;
        border-radius: 4px;
    }
    
    .diagnostic-list::item:selected {
        background-color: #e6ebef;
        border: 1px solid #516B82;
    }
    
    .diagnostic-list::item:hover {
        background-color: #eef4f7;
    }
    
    .diagnostic-counter {
        color: #555555;
        font-size: 12px;
        font-weight: bold;
        padding: 5px;
        background-color: #cbcbcb;
        border-radius: 4px;
    }
    
    /* Indicadores de estado */
    .status-icon {
        font-size: 16px;
        font-weight: bold;
    }
    
    .status-success {
        color: #1e7d22;
    }
    
    .status-warning {
        color: #d9a300;
    }
    
    .status-error {
        color: #e74c3c;
    }
    
    /* Barra de título */
    .title-bar {
        background-color: #cbcbcb;
        border-bottom: 2px solid #516B82;
    }
    
    .title-bar-title {
        color: #516B82;
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
        color: #1e7d22;
    }
    
    .ros-disconnected {
        color: #e74c3c;
    }
    
    /* Estilos para valores dinámicos */
    .value-success {
        color: #1e7d22;
        font-weight: bold;
    }
    
    .value-warning {
        color: #d9a300;
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
        color: #666a6b; 
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
            'warning': ("⚠️", "#cfb408", "#fef9e7", "#f1c40f"),
            'ok': ("⚪", "#737f80", "#ecf0f1", "#bdc3c7")
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