#!/usr/bin/env python3
import sys
import os
import signal

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtWidgets import QApplication
from widgets.main_window import MainWindow
import rclpy
from rclpy.node import Node
from ros_bridge import ROSBridge
import threading
import time


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
        self.spin_thread = None
        self._shutting_down = False
        self._spin_thread_event = threading.Event()

    def shutdown(self):
        if self._shutting_down:
            return
            
        self._shutting_down = True
        self._spin_thread_event.set()  # Señalar al hilo que se detenga
        
        if self.spin_thread:
            self.spin_thread.join(timeout=2.0)
        
        if self.node:
            try:
                self.node.destroy_node()
            except:
                pass
                
        if self.app:
            self.app.quit()


def signal_handler(signum, frame):
    app_manager.shutdown()


def ros_spin_thread(app_manager):
    """Hilo separado para manejar ROS 2 spin"""
    try:
        # Usar executor con timeout para poder salir limpiamente
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(app_manager.node)
        
        # Spin con timeout para verificar periódicamente si debemos detenernos
        while rclpy.ok() and not app_manager._shutting_down:
            executor.spin_once(timeout_sec=0.1)
            
        # Limpiar
        executor.remove_node(app_manager.node)
        executor.shutdown()
        #print("Hilo ROS finalizado limpiamente")  # Opcional: comentar para menos logs
        
    except KeyboardInterrupt:
        # Ctrl+C - cierre normal, no mostrar error
        pass
    except Exception as e:
        # Solo imprimir si hay un mensaje real y no es cierre normal
        if not app_manager._shutting_down and str(e).strip():
            print(f"Error en hilo ROS: {e}")

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

    # Iniciar hilo para ROS spin
    app_manager.spin_thread = threading.Thread(
        target=ros_spin_thread, 
        args=(app_manager,), 
        daemon=True
    )
    app_manager.spin_thread.start()

    # Crear ventana principal
    app_manager.window = MainWindow(ros_node=app_manager.node)
    app_manager.window.show()

    # Cierre por ventana
    def on_window_close(event):
        app_manager.shutdown()
        event.accept()
    app_manager.window.closeEvent = on_window_close

    # Ejecutar aplicación Qt
    try:
        exit_code = app_manager.app.exec()
        # Asegurarse de limpiar antes de salir
        if not app_manager._shutting_down:
            app_manager.shutdown()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        app_manager.shutdown()
    except Exception as e:
        print(f"Error en aplicación Qt: {e}")
        app_manager.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()