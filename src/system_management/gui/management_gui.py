#!/usr/bin/env python3
import sys
import os
import signal

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from widgets.main_window import MainWindow
import rclpy
from rclpy.node import Node
from ros_bridge import ROSBridge
import threading


# ⚠️ CONFIGURAR EL BACKEND ANTES de importar Qt
os.environ['QT_QPA_PLATFORM'] = 'xcb'


class ManagementGUINode(Node):
    def __init__(self):
        super().__init__('management_gui')
        self.ros_bridge = ROSBridge(self)


class ApplicationManager:
    def __init__(self):
        self.app = None
        self.node = None
        self.window = None
        self.spin_timer = None
        self._shutting_down = False

    def shutdown(self):
        if self._shutting_down:
            return
            
        self._shutting_down = True
        print("🧹 Cerrando aplicación...")
        
        if self.spin_timer:
            self.spin_timer.stop()
            
        if self.node:
            try:
                self.node.destroy_node()
            except:
                pass
                
        try:
            rclpy.shutdown()
        except:
            pass
            
        if self.app:
            self.app.quit()


def signal_handler(signum, frame):
    print(f"\n🛑 Señal {signum} recibida")
    app_manager.shutdown()


def main():
    global app_manager
    app_manager = ApplicationManager()

    # Registrar manejador de señales
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Inicializar ROS2
    rclpy.init()

    # Crear aplicación Qt
    app_manager.app = QApplication(sys.argv)
    app_manager.app.setApplicationName("Autonomous Robot - Panel de Control")

    # Crear nodo ROS2
    app_manager.node = ManagementGUINode()

    # 🔥🔥🔥 INICIAR SPIN EN HILO SEPARADO 🔥🔥🔥
    def ros_spin_thread():
        rclpy.spin(app_manager.node)

    spin_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    spin_thread.start()
    # -------------------------------------------------------------------

    # Crear ventana principal
    app_manager.window = MainWindow(ros_node=app_manager.node)
    app_manager.window.show()

    # ❌ No usar spin_once, así que no activamos ningún QTimer

    # Cierre por ventana
    def on_window_close(event):
        print("🔒 Ventana cerrada por usuario")
        app_manager.shutdown()
        event.accept()

    app_manager.window.closeEvent = on_window_close

    print("✅ Interfaz GUI iniciada correctamente")
    print("📡 Nodo ROS2 activo: /management_gui")
    print("💡 Usa Ctrl+C para cerrar")

    # Ejecutar
    try:
        exit_code = app_manager.app.exec()
        print(f"🔚 Aplicación terminada con código: {exit_code}")
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C detectado")
        app_manager.shutdown()
    finally:
        try:
            app_manager.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
