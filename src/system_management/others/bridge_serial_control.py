#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from std_srvs.srv import SetBool
from tf_transformations import quaternion_from_euler
import time


class SerialROSBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_control')
        self.declare_parameter('serial_port', '/dev/ttyESP32Control')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('reconnect_interval', 5.0)  # seconds between reconnect attempts
        self.declare_parameter('connection_timeout', 3.0)  # seconds to consider connection dead
        self.declare_parameter('periodic_update_interval', 1.0)  # seconds for periodic updates when use_serial=False
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        self.periodic_update_interval = self.get_parameter('periodic_update_interval').value

        self.serial_conn = None
        self.last_successful_comm = 0.0
        self.last_reconnect_attempt = 0.0
        self.is_connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 10
        
        # Connection state publisher
        self.connection_state_pub = self.create_publisher(Bool, 'serial_connection_state', 10)

        # Publishers (serial-sourced)
        self.odom_pub = self.create_publisher(Odometry, 'odom/serial', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/serial', 10)
        self.range_front_pub = self.create_publisher(Range, 'range/front/serial', 10)
        self.range_left_pub = self.create_publisher(Range, 'range/left/serial', 10)
        self.range_right_pub = self.create_publisher(Range, 'range/right/serial', 10)
        self.serial_hb_pub = self.create_publisher(Bool, 'serial/heartbeat', 10)

        # Service to perform reset over serial (fallback)
        self.reset_service = self.create_service(SetBool, 'robot_control_reset_serial', self.handle_reset_serial)

        # Subscribe to unified /cmd_vel (unifier publishes final /cmd_vel).
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Subscribe to unifier directive: use_serial (Bool). If True -> forward commands via serial.
        self.use_serial_sub = self.create_subscription(Bool, 'use_serial_control', self.use_serial_cb, 10)
        self.use_serial = False  # default: don't forward by serial

        # Control variables
        self.last_cmd_vel_time = 0.0
        self.last_periodic_update_time = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.update_velocity = False  # Controla si se actualizan las velocidades en el ESP32

        # buffer
        self.serial_buffer = ""
        
        # Timers
        self.read_timer = self.create_timer(0.01, self.read_serial)  # 100Hz for reading
        self.connection_timer = self.create_timer(1.0, self.check_connection)  # 1Hz for connection monitoring
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop
        
        # Initial connection attempt
        self.connect_serial()
        
        #self.get_logger().info('Serial bridge initialized with periodic updates')

    def get_current_time(self):
        """Get current time in seconds"""
        return time.time()

    def connect_serial(self):
        """Attempt to connect to serial port with error handling"""
        current_time = self.get_current_time()
        
        # Resetear attempts si ha pasado mucho tiempo desde el último intento exitoso
        if current_time - self.last_successful_comm > 300.0:  # 5 minutos
            self.reconnect_attempts = 0
            #self.get_logger().info('Resetting reconnection attempts after long period')
        
        self.last_reconnect_attempt = current_time
        self.reconnect_attempts += 1
        
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
                self.reconnect_attempts = 0  # Reset solo en conexión exitosa
                
                #self.get_logger().info(f'Connected to serial {self.serial_port}')
                
                # Publish connection state
                conn_state = Bool()
                conn_state.data = True
                self.connection_state_pub.publish(conn_state)
                
                return True
            else:
                self.is_connected = False
                return False
                
        except Exception as e:
            self.is_connected = False
            # Limitar logs para evitar spam - solo loggear cada 5 intentos después de los primeros 3
            if self.reconnect_attempts <= 3 or self.reconnect_attempts % 5 == 0:
                #self.get_logger().warning(f'Serial connection attempt {self.reconnect_attempts} failed: {str(e)[:100]}')
                pass
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
            self.connection_state_pub.publish(conn_state)
            
            #self.get_logger().info('Serial connection closed')
        except Exception as e:
            #self.get_logger().warning(f'Error closing serial: {e}')
            pass

    def check_connection(self):
        """Monitor connection health and attempt reconnection if needed"""
        current_time = self.get_current_time()
        
        if self.is_connected:
            # Check if we've received data recently
            time_since_last_comm = current_time - self.last_successful_comm
            if time_since_last_comm > self.connection_timeout:
                #self.get_logger().warning(f'Serial connection timeout ({time_since_last_comm:.1f}s), attempting reconnect...')
                self.disconnect_serial()
        else:
            # Not connected - ALWAYS attempt reconnect with exponential backoff
            base_interval = self.reconnect_interval
            max_interval = 60.0  # Máximo 60 segundos entre intentos
            
            # Calcular delay exponencial basado en intentos
            current_delay = min(base_interval * (2 ** min(self.reconnect_attempts, 6)), max_interval)
            
            if current_time - self.last_reconnect_attempt >= current_delay:
                if self.reconnect_attempts >= self.max_reconnect_attempts:
                    #self.get_logger().info(f'Continuing reconnection attempts (interval: {current_delay:.1f}s)')
                    pass
                self.connect_serial()
            else:
                # Solo loggear ocasionalmente para evitar spam
                remaining = current_delay - (current_time - self.last_reconnect_attempt)
                if remaining > 1.0 and current_time % 10 < 1:  # Log cada ~10 segundos
                    #self.get_logger().debug(f'Next reconnection attempt in {remaining:.1f}s')
                    pass

    def use_serial_cb(self, msg: Bool):
        new_use_serial = bool(msg.data)
        if new_use_serial != self.use_serial:
            self.use_serial = new_use_serial
            if self.use_serial:
                #self.get_logger().info('use_serial enabled - switching to cmd_vel frequency')
                # Reset velocity update flag when switching to cmd_vel mode
                self.update_velocity = True
            else:
                #self.get_logger().info('use_serial disabled - switching to periodic updates')
                # Send zero velocity when disabling serial control
                self.current_linear = 0.0
                self.current_angular = 0.0
                self.update_velocity = False
                self.send_serial_cmd(0.0, 0.0, reset=0, update_velocity=False)

    def cmd_vel_callback(self, msg: Twist):
        # Store current velocities and update timestamp
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        self.last_cmd_vel_time = self.get_current_time()
        
        # Set velocity update flag when use_serial is enabled
        if self.use_serial:
            self.update_velocity = True

    def control_loop(self):
        """Control loop to handle serial communication frequency"""
        if not self.is_connected:
            return
            
        current_time = self.get_current_time()
        
        if self.use_serial:
            # Mode 1: use_serial = True - Send at cmd_vel frequency with velocity updates
            time_since_last_cmd = current_time - self.last_cmd_vel_time
            
            # If we received a cmd_vel recently (within 0.5 seconds), send update
            if time_since_last_cmd < 0.5 and self.update_velocity:
                self.send_serial_cmd(self.current_linear, self.current_angular, reset=0, update_velocity=True)
                self.update_velocity = False  # Reset flag after sending
        else:
            # Mode 2: use_serial = False - Send periodic updates every 1 second without velocity updates
            if current_time - self.last_periodic_update_time >= self.periodic_update_interval:
                self.send_serial_cmd(0.0, 0.0, reset=0, update_velocity=False)
                self.last_periodic_update_time = current_time

    def send_serial_cmd(self, linear, angular, reset=0, update_velocity=True):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Format: CMD,linear,angular,reset,update_velocity
                cmd_str = f"CMD,{linear:.4f},{angular:.4f},{int(reset)},{int(update_velocity)}\n"
                self.serial_conn.write(cmd_str.encode('ascii'))
                #self.serial_conn.flush()
                self.last_successful_comm = self.get_current_time()
                
                # Log only occasionally to avoid spam
                if self.get_clock().now().nanoseconds % 10 == 0:  # Log ~10% of messages
                    #self.get_logger().debug(f'Sent: {cmd_str.strip()}')
                    pass
                    
            except Exception as e:
                #self.get_logger().error(f'Error sending serial CMD: {e}')
                self.is_connected = False
                self.disconnect_serial()

    def handle_reset_serial(self, request, response):
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                self.send_serial_cmd(0.0, 0.0, reset=1, update_velocity=False)
                response.success = True
                response.message = 'Reset command sent via serial'
                return response
            except Exception as e:
                response.success = False
                response.message = f'Error sending serial reset: {e}'
                return response
        else:
            response.success = False
            response.message = 'Serial not connected'
            return response

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
            #self.get_logger().error(f'Serial read error: {e}')
            self.is_connected = False
            self.disconnect_serial()
        except Exception as e:
            # Other errors (decode, etc) - don't disconnect for these
            #self.get_logger().warning(f'Serial processing error: {e}')
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
            if len(parts) >= 18:
                try:
                    vx = self.safe_float(parts[1])
                    wz = self.safe_float(parts[2])
                    x_pos = self.safe_float(parts[3])
                    y_pos = self.safe_float(parts[4])
                    yaw_enc = self.safe_float(parts[5])
                    ax = self.safe_float(parts[6])
                    ay = self.safe_float(parts[7])
                    az = self.safe_float(parts[8])
                    gx = self.safe_float(parts[9])
                    gy = self.safe_float(parts[10])
                    gz = self.safe_float(parts[11])
                    roll = self.safe_float(parts[12])
                    pitch = self.safe_float(parts[13])
                    yaw = self.safe_float(parts[14])
                    rfront = self.safe_float(parts[15])
                    rleft = self.safe_float(parts[16])
                    rright = self.safe_float(parts[17])

                    current_time = self.get_clock().now().to_msg()

                    # publish odom/serial
                    odom = Odometry()
                    odom.header.stamp = current_time
                    odom.header.frame_id = "odom"
                    odom.child_frame_id = self.frame_id
                    odom.pose.pose.position.x = x_pos
                    odom.pose.pose.position.y = y_pos
                    odom.pose.pose.position.z = 0.0
                    q = quaternion_from_euler(0, 0, yaw_enc)
                    odom.pose.pose.orientation.x = q[0]
                    odom.pose.pose.orientation.y = q[1]
                    odom.pose.pose.orientation.z = q[2]
                    odom.pose.pose.orientation.w = q[3]
                    odom.twist.twist.linear.x = vx
                    odom.twist.twist.angular.z = wz
                    odom.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,     
                                            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
                    
                    odom.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
                    self.odom_pub.publish(odom)

                    # publish IMU / serial
                    imu = Imu()
                    imu.header.stamp = current_time
                    imu.header.frame_id = "imu_link"
                    q2 = quaternion_from_euler(roll, pitch, yaw)
                    imu.orientation.x = q2[0]
                    imu.orientation.y = q2[1]
                    imu.orientation.z = q2[2]
                    imu.orientation.w = q2[3]
                    imu.angular_velocity.x = gx
                    imu.angular_velocity.y = gy
                    imu.angular_velocity.z = gz
                    imu.linear_acceleration.x = ax
                    imu.linear_acceleration.y = ay
                    imu.linear_acceleration.z = az
                    imu.orientation_covariance = [0.01, 0.0, 0.0,
                                                  0.0, 0.01, 0.0,
                                                  0.0, 0.0, 0.01]
                    imu.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                                       0.0, 0.01, 0.0,
                                                       0.0, 0.0, 0.01]
                    imu.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                                          0.0, 0.01, 0.0,
                                                          0.0, 0.0, 0.01]
                    self.imu_pub.publish(imu)

                    # ranges
                    rf = Range()
                    rf.header.stamp = current_time
                    rf.header.frame_id = "range_front_link"
                    rf.range = rfront
                    
                    rl = Range()
                    rl.header.stamp = current_time
                    rl.header.frame_id = "range_left_link"
                    rl.range = rleft
                    
                    rr = Range()
                    rr.header.stamp = current_time
                    rr.header.frame_id = "range_right_link"
                    rr.range = rright
                    
                    self.range_front_pub.publish(rf)
                    self.range_left_pub.publish(rl)
                    self.range_right_pub.publish(rr)

                    # publish serial heartbeat
                    hb = Bool()
                    hb.data = True
                    self.serial_hb_pub.publish(hb)
                    
                except Exception as e:
                    #self.get_logger().warning(f'Error processing DATA line: {e}')
                    pass
            else:
                #self.get_logger().debug(f'Incomplete DATA line: {line}')
                pass
        elif line.startswith('CMD,'):
            # Echo of our own command - can be ignored or logged for debugging
            pass
        else:
            #self.get_logger().debug(f'Unknown serial line: {line}')
            pass

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        #self.get_logger().info('Shutting down serial bridge...')
        # Send stop command before shutting down
        if self.is_connected:
            self.send_serial_cmd(0.0, 0.0, reset=0, update_velocity=False)
        self.disconnect_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialROSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('Serial bridge stopped by user')
        pass
    except Exception as e:
        node.get_logger().error(f'🚨 Fatal error in serial bridge: {e}')
    finally:        
        try:
            node.disconnect_serial()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()