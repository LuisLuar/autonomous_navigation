import sys
import os

# Añadir el directorio actual al path para imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from widgets.main_window import MainWindow

def main():
    # Crear aplicación
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Autonomo GUI")
    
    # Crear ventana principal
    window = MainWindow()
    window.show()
    
    # Timer para prevenir cierre inmediato
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)
    
    print("✅ Interfaz GUI iniciada correctamente")
    print("📊 Panel de sensores funcionando con simulación")
    print("🗺️  Mapa interactivo cargado - Haz clic para seleccionar destino")
    print("🚀 Sistema listo para pruebas visuales")
    
    # Ejecutar aplicación
    sys.exit(app.exec())

if __name__ == "__main__":
    main()