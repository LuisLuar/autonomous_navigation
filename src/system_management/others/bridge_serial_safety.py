#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray, UInt8, Bool
import time


class SerialSafetyBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_safety')
        self.declare_parameter('serial_port', '/dev/ttyESP32Safety')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('reconnect_interval', 3.0)  # 3 segundos entre intentos
        self.declare_parameter('connection_timeout', 3.0)  # segundos para considerar la conexión muerta
        self.declare_parameter('periodic_update_interval', 1.0)  # segundos para updates periódicos
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        self.periodic_update_interval = self.get_parameter('periodic_update_interval').value

        self.serial_conn = None
        self.last_successful_comm = 0.0
        self.last_reconnect_attempt = 0.0
        self.is_connected = False
        
        # Connection state publisher
        #self.connection_state_pub = self.create_publisher(Bool, 'serial_safety_connection_state', 10)

        # Publishers (serial-sourced) - datos de la ESP32 safety
        self.battery_array_pub = self.create_publisher(Float32MultiArray, 'battery_array/serial', 10)
        self.motors_array_pub = self.create_publisher(Float32MultiArray, 'motors_array/serial', 10)
        self.serial_hb_pub = self.create_publisher(Bool, 'serial_safety/heartbeat', 10)

        # Subscribe to rele_control commands to send via serial
        self.rele_sub = self.create_subscription(UInt8, 'rele_control', self.rele_callback, 10)

        # Subscribe to unifier directive: use_serial_safety (Bool). If True -> forward commands via serial.
        self.use_serial_sub = self.create_subscription(Bool, 'use_serial_safety', self.use_serial_cb, 10)
        self.use_serial = False  # default: don't forward by serial

        # Control variables
        self.last_rele_time = 0.0
        self.last_periodic_update_time = 0.0
        self.current_rele_cmd = 0
        self.update_rele = False  # Controla si se actualizan los relés en el ESP32

        # buffer
        self.serial_buffer = ""
        
        # Timers
        self.read_timer = self.create_timer(0.01, self.read_serial)  # 100Hz for reading
        self.connection_timer = self.create_timer(1.0, self.check_connection)  # 1Hz for connection monitoring
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop
        
        # Initial connection attempt
        self.connect_serial()

    def get_current_time(self):
        """Get current time in seconds"""
        return time.time()

    def connect_serial(self):
        """Attempt to connect to serial port"""
        current_time = self.get_current_time()
        self.last_reconnect_attempt = current_time
        
        try:
            # Close existing connection if any
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.is_connected = True
                self.last_successful_comm = self.get_current_time()
                
                #self.get_logger().info(f'✅ Connected to safety serial {self.serial_port}')
                
                # Publish connection state
                conn_state = Bool()
                conn_state.data = True
                #self.connection_state_pub.publish(conn_state)
                
                return True
            else:
                self.is_connected = False
                return False
                
        except Exception as e:
            self.is_connected = False
            #self.get_logger().warning(f'⚠️ Safety serial connection failed: {str(e)}')
            return False

    def disconnect_serial(self):
        """Safely disconnect serial connection"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = None
            self.is_connected = False
            
            # Publish disconnected state
            conn_state = Bool()
            conn_state.data = False
            #self.connection_state_pub.publish(conn_state)
            
            #self.get_logger().info('🔌 Safety serial connection closed')
        except Exception as e:
            #self.get_logger().warning(f'Error closing safety serial: {e}')
            pass

    def check_connection(self):
        """Monitor connection health and attempt reconnection if needed"""
        current_time = self.get_current_time()
        
        if self.is_connected:
            # Check if we've received data recently
            time_since_last_comm = current_time - self.last_successful_comm
            if time_since_last_comm > self.connection_timeout:
                #self.get_logger().warning(f'🔄 Safety serial connection timeout ({time_since_last_comm:.1f}s), reconnecting...')
                self.disconnect_serial()
        else:
            # Not connected - ALWAYS attempt reconnect every reconnect_interval seconds
            time_since_last_attempt = current_time - self.last_reconnect_attempt
            if time_since_last_attempt >= self.reconnect_interval:
                #self.get_logger().info(f'🔄 Attempting to reconnect to safety serial...')
                self.connect_serial()

    def use_serial_cb(self, msg: Bool):
        new_use_serial = bool(msg.data)
        if new_use_serial != self.use_serial:
            self.use_serial = new_use_serial
            if self.use_serial:
                #self.get_logger().info('🔛 use_serial_safety enabled - switching to rele_control frequency')
                # Reset rele update flag when switching to rele_control mode
                self.update_rele = True
            else:
                #self.get_logger().info('🔚 use_serial_safety disabled - switching to periodic updates')
                # Send zero rele command when disabling serial control
                self.current_rele_cmd = 0
                self.update_rele = False
                self.send_serial_cmd(0, update_rele=False)

    def rele_callback(self, msg: UInt8):
        # Store current rele command and update timestamp
        self.current_rele_cmd = msg.data
        self.last_rele_time = self.get_current_time()
        
        # Set rele update flag when use_serial is enabled
        if self.use_serial:
            self.update_rele = True

    def control_loop(self):
        """Control loop to handle serial communication frequency"""
        if not self.is_connected:
            return
            
        current_time = self.get_current_time()
        
        if self.use_serial:
            # Mode 1: use_serial_safety = True - Send at rele_control frequency with rele updates
            time_since_last_cmd = current_time - self.last_rele_time
            
            # If we received a rele_control recently (within 0.5 seconds), send update
            if time_since_last_cmd < 0.5 and self.update_rele:
                self.send_serial_cmd(self.current_rele_cmd, update_rele=True)
                self.update_rele = False  # Reset flag after sending
        else:
            # Mode 2: use_serial_safety = False - Send periodic updates every 1 second without rele updates
            if current_time - self.last_periodic_update_time >= self.periodic_update_interval:
                self.send_serial_cmd(0, update_rele=False)
                self.last_periodic_update_time = current_time

    def send_serial_cmd(self, rele_cmd, update_rele=True):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Format: CMD,rele_command,update_rele
                cmd_str = f"CMD,{int(rele_cmd)},{int(update_rele)}\n"
                self.serial_conn.write(cmd_str.encode('ascii'))
                self.serial_conn.flush()
                self.last_successful_comm = self.get_current_time()
                
                # Log only occasionally to avoid spam
                if self.get_clock().now().nanoseconds % 10 == 0:  # Log ~10% of messages
                    #self.get_logger().debug(f'📤 Sent safety: {cmd_str.strip()}')
                    pass
                    
            except Exception as e:
                #self.get_logger().error(f'❌ Error sending safety serial CMD: {e}')
                self.is_connected = False
                self.disconnect_serial()

    def read_serial(self):
        if not self.is_connected or not self.serial_conn or not self.serial_conn.is_open:
            return
            
        try:
            bytes_to_read = self.serial_conn.in_waiting
            if bytes_to_read > 0:
                raw = self.serial_conn.read(bytes_to_read)
                data = raw.decode('ascii', errors='replace')
                self.serial_buffer += data
                self.last_successful_comm = self.get_current_time()  # Update on any data received
                
                while '\n' in self.serial_buffer:
                    line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.process_serial_line(line)
        except serial.SerialException as e:
            #self.get_logger().error(f'❌ Safety serial read error: {e}')
            self.is_connected = False
            self.disconnect_serial()
        except Exception as e:
            # Other errors (decode, etc) - don't disconnect for these
            #self.get_logger().warning(f'⚠️ Safety serial processing error: {e}')
            pass

    def safe_float(self, s):
        try:
            # Handle common serial data issues
            s = s.replace('..', '.').strip()
            # Remove any non-numeric characters except minus sign and decimal point
            s = ''.join(c for c in s if c in '0123456789.-')
            return float(s)
        except (ValueError, TypeError):
            return 0.0

    def process_serial_line(self, line):
        if line.startswith('DATA,'):
            parts = line.split(',')
            if len(parts) >= 5:
                try:
                    # Parse data: l, r, vo12, vo5
                    current_left = self.safe_float(parts[1])
                    current_right = self.safe_float(parts[2])
                    voltage_12v = self.safe_float(parts[3])
                    voltage_5v = self.safe_float(parts[4])

                    # Publish battery array (voltage_12v, voltage_5v)
                    battery_array = Float32MultiArray()
                    battery_array.data = [voltage_12v, voltage_5v]
                    self.battery_array_pub.publish(battery_array)

                    # Publish motors array (current_left, current_right)
                    motors_array = Float32MultiArray()
                    motors_array.data = [current_left, current_right]
                    self.motors_array_pub.publish(motors_array)

                    # Publish serial heartbeat
                    hb = Bool()
                    hb.data = True
                    self.serial_hb_pub.publish(hb)
                    
                    #self.get_logger().debug(f'📊 Safety data: 12V={voltage_12v:.2f}V, 5V={voltage_5v:.2f}V, I_L={current_left:.2f}A, I_R={current_right:.2f}A')
                    
                except Exception as e:
                    #self.get_logger().warning(f'⚠️ Error processing safety DATA line: {e}')
                    pass
            else:
                #self.get_logger().debug(f'⚠️ Incomplete safety DATA line: {line}')
                pass
        elif line.startswith('CMD,'):
            # Echo of our own command - can be ignored or logged for debugging
            pass
        else:
            #self.get_logger().debug(f'📨 Unknown safety serial line: {line}')
            pass

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        #self.get_logger().info('🛑 Shutting down safety serial bridge...')
        # Send stop command before shutting down
        if self.is_connected:
            self.send_serial_cmd(0, update_rele=False)
        self.disconnect_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialSafetyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('👋 Safety serial bridge stopped by user')
    except Exception as e:
        node.get_logger().error(f'🚨 Fatal error in safety serial bridge: {e}')
    finally:        
        try:
            node.disconnect_serial()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()