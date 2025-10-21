# Configuraciones constantes de la aplicación
class Constants:
    APP_TITLE = "Robot Autónomo - Interfaz de Control"
    APP_VERSION = "1.0.0"
    WINDOW_WIDTH = 1600
    WINDOW_HEIGHT = 1100

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