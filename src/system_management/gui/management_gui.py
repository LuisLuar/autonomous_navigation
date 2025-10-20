#!/usr/bin/env python3
import sys
import os
import atexit
import signal
import threading

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from widgets.main_window import MainWindow
import rclpy
from rclpy.node import Node
from ros_bridge import ROSBridge


class ManagementGUINode(Node):
    def __init__(self):
        super().__init__('management_gui')
        self.get_logger().info('🟢 Nodo ROS2 de la GUI iniciado')
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
    app_manager.app.setApplicationName("Robot Autónomo - Panel de Control")

    # Crear nodo ROS2
    app_manager.node = ManagementGUINode()

    # Crear ventana principal
    app_manager.window = MainWindow(ros_node=app_manager.node)
    app_manager.window.showMaximized()
    #showFullScreen()

    # Timer para ROS2
    app_manager.spin_timer = QTimer()
    
    def spin_once():
        if not app_manager._shutting_down:
            try:
                rclpy.spin_once(app_manager.node, timeout_sec=0.01)
            except Exception as e:
                if not app_manager._shutting_down:
                    print(f"[WARN] Error en spin_once: {e}")

    app_manager.spin_timer.timeout.connect(spin_once)
    app_manager.spin_timer.start(50)

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