#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from diagnostic_msgs.msg import DiagnosticStatus
import serial
import threading

class SimpleSafetyNode(Node):
    def __init__(self):
        super().__init__('safety_unifier_simple')

        # --- Configuración Serial ---
        self.port = '/dev/ttyESP32Safety' # <-- Asegúrate que este sea tu puerto
        self.ser = None
        self.connect_serial()
        
        # --- Estado Interno ---
        self.rele_bits = [False] * 8 
        self.last_voltage = 0.0
        self.connection_error = "Iniciando..."

        # --- Suscriptores ---
        self.create_subscription(Bool, '/light/stop',  lambda m: self.set_bit(0, m.data), 10)
        self.create_subscription(Bool, '/light/left',  lambda m: self.set_bit(1, m.data), 10)
        self.create_subscription(Bool, '/light/right', lambda m: self.set_bit(2, m.data), 10)
        self.create_subscription(Bool, '/light/safety',lambda m: self.set_bit(3, m.data), 10)
        self.create_subscription(Bool, '/start/motor_left', lambda m: self.set_bit(4, m.data), 10)
        

        # --- Publicadores ---
        self.diag_pub = self.create_publisher(DiagnosticStatus, 'status/esp32_safety', 10)
        self.batt_pub = self.create_publisher(Float32MultiArray, 'battery_array', 10)

        # Timers (10Hz)
        self.create_timer(0.1, self.control_loop)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05)
            self.connection_error = ""
        except Exception as e:
            self.connection_error = str(e)

    def set_bit(self, index, value):
        self.rele_bits[index] = value

    def control_loop(self):
        diag = DiagnosticStatus()
        diag.name = "ESP32 Safety"
        diag.hardware_id = "ESP32_Robot"

        # 1. Intentar Reconectar si se perdió el puerto
        if self.ser is None or not self.ser.is_open:
            self.connect_serial()
            diag.level = DiagnosticStatus.ERROR
            diag.message = f"Desconectado"
            self.diag_pub.publish(diag)
            return

        # 2. Enviar Bits a ESP32
        try:
            byte_val = 0
            for i, bit in enumerate(self.rele_bits):
                if bit:
                    byte_val |= (1 << i)
            
            # --- LÍNEA DE DEBUG ---
            # Muestra el comando, el valor decimal y el binario (ej: 00010110)
            #self.get_logger().info(f"Enviando: CMD,{byte_val} | Binario: {byte_val:08b}")
            
            cmd_string = f"CMD,{byte_val}\n"
            self.ser.write(cmd_string.encode('utf-8'))
            
        except Exception as e:
            self.ser = None # Forzar reconexión

        # 3. Leer Datos (Formato: DATA,0.00,)
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                # Separamos por coma: ['DATA', '0.00', '']
                parts = line.split(',')
                if len(parts) >= 2 and parts[0] == "DATA":
                    self.last_voltage = float(parts[1])
                    
                    batt_msg = Float32MultiArray()
                    batt_msg.data = [self.last_voltage]
                    self.batt_pub.publish(batt_msg)
            
            diag.level = DiagnosticStatus.OK
            diag.message = f"Voltaje: {self.last_voltage}V"
        except Exception as e:
            diag.level = DiagnosticStatus.WARN
            diag.message = f"Error de lectura"

        self.diag_pub.publish(diag)

def main():
    rclpy.init()
    node = SimpleSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()