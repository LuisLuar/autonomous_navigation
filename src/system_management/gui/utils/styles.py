# Estilos CSS para la interfaz HMI
def get_app_style():
    return """
    QMainWindow {
        background-color: #2c3e50;
        color: #ecf0f1;
    }
    
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
    
    QWidget {
        background-color: #34495e;
        color: #ecf0f1;
        font-family: 'Segoe UI', Arial, sans-serif;
    }
    
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
    
    QLabel {
        color: #ecf0f1;
        font-size: 12px;
    }
    
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
    """