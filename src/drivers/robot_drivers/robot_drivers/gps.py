#!/usr/bin/env python3
"""
gps.py - Nodo ROS2 optimizado para BU-3535N5 y módulos GNSS similares.

Características principales:
 - Reconexión automática del puerto serial si se desconecta el GPS
 - Verificación de checksum NMEA para validar integridad de datos
 - Parseo robusto de sentencias GGA, RMC, GSA, GSV
 - Conversión de coordenadas NMEA a grados decimales
 - Sincronización con timestamp GNSS (RMC date + GGA time) y conversión a hora Ecuador (UTC-5)
 - Cálculo de velocidad mediante diferencia de posición (Haversine) y/o SOG de RMC
 - Suavizado de velocidad con promedio móvil y umbral para detectar estado estático
 - Publicación de múltiples tópicos ROS2 con información GPS completa
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String, Float64
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from datetime import datetime, timezone, timedelta
import serial
import math
from collections import deque
import time

# Zona horaria de Ecuador (sin horario de verano): UTC-5
ECUADOR_TZ = timezone(timedelta(hours=-5))

# Constantes para reconexión
MAX_RECONNECT_ATTEMPTS = 5  # Intentos antes de aumentar el intervalo
INITIAL_RECONNECT_DELAY = 1.0  # Segundos de espera inicial
MAX_RECONNECT_DELAY = 30.0  # Máximo tiempo de espera entre intentos


def verify_nmea_checksum(sentence: str) -> bool:
    """
    Verifica el checksum de una sentencia NMEA.
    
    Args:
        sentence: Sentencia NMEA completa (ej: $GPGGA,...*XX)
        
    Returns:
        True si el checksum es válido, False en caso contrario
    """
    try:
        if not sentence.startswith('$') or '*' not in sentence:
            return False
        
        # Separar el cuerpo del mensaje y el checksum
        body, cs = sentence[1:].split('*', 1)
        cs = cs.strip()
        
        # Calcular checksum: XOR de todos los caracteres del cuerpo
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        
        hex_cs = f"{calc:02X}"
        return hex_cs == cs.upper()
    except Exception:
        return False


def nmea_latlon_to_decimal(lat_str: str, lat_dir: str, lon_str: str, lon_dir: str):
    """
    Convierte coordenadas NMEA (grados + minutos) a grados decimales.
    
    Args:
        lat_str: Latitud en formato NMEA (ddmm.mmmm)
        lat_dir: Dirección latitud ('N' o 'S')
        lon_str: Longitud en formato NMEA (dddmm.mmmm)
        lon_dir: Dirección longitud ('E' o 'W')
        
    Returns:
        Tupla (latitud, longitud) en grados decimales o (None, None) si hay error
    """
    lat = None
    lon = None
    try:
        # Parsear latitud: primeros 2 dígitos son grados
        if lat_str and lat_dir and len(lat_str) >= 4:
            deg = int(lat_str[:2])
            minutes = float(lat_str[2:])
            lat = deg + minutes / 60.0
            if lat_dir.upper() == 'S':
                lat = -lat
        
        # Parsear longitud: primeros 3 dígitos son grados
        if lon_str and lon_dir and len(lon_str) >= 5:
            deg = int(lon_str[:3])
            minutes = float(lon_str[3:])
            lon = deg + minutes / 60.0
            if lon_dir.upper() == 'W':
                lon = -lon
    except Exception:
        return None, None
    return lat, lon


def haversine_m(lat1, lon1, lat2, lon2):
    """
    Calcula la distancia entre dos puntos GPS usando la fórmula de Haversine.
    
    Args:
        lat1, lon1: Coordenadas del primer punto en grados decimales
        lat2, lon2: Coordenadas del segundo punto en grados decimales
        
    Returns:
        Distancia en metros entre los dos puntos
    """
    R = 6371000.0  # Radio de la Tierra en metros
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    
    a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


class MultiGNSSPublisher(Node):
    """
    Nodo ROS2 para publicar datos de módulos GNSS con reconexión automática.
    """
    
    def __init__(self):
        super().__init__('multi_gnss_publisher')

        # ================= PARÁMETROS ROS2 =================
        self.declare_parameter('serial_port', '/dev/ttyGPS')
        self.declare_parameter('baudrate', 4800)
        self.declare_parameter('frame_id', 'gnss_frame')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('speed_threshold', 0.2)  # m/s - umbral para considerar movimiento real
        self.declare_parameter('smoothing_window', 5)  # ventana para promedio móvil de velocidad

        # Obtener valores de parámetros
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = int(self.get_parameter('baudrate').get_parameter_value().integer_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self.speed_threshold = float(self.get_parameter('speed_threshold').get_parameter_value().double_value)
        self.smoothing_window = int(self.get_parameter('smoothing_window').get_parameter_value().integer_value)
        
        if self.smoothing_window < 1:
            self.smoothing_window = 1

        # ================= PUBLICADORES ROS2 =================
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/gps/vel', 10)
        self.vel_cov_pub = self.create_publisher(TwistWithCovarianceStamped, '/gps/vel_with_cov', 10)
        self.course_pub = self.create_publisher(Float64, '/gps/course', 10)
        self.raw_pub = self.create_publisher(String, '/gps/raw', 50)
        self.info_pub = self.create_publisher(String, '/gps/info', 10)

        # ================= ESTADO DEL PUERTO SERIAL =================
        self.ser = None
        self.is_connected = False
        self.reconnect_attempts = 0
        self.reconnect_delay = INITIAL_RECONNECT_DELAY
        self.last_reconnect_time = 0.0

        # ================= VARIABLES DE ESTADO GPS =================
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.hdop = None
        self.fix_quality = 0
        self.gps_sats = 0
        self.satellites_visible = 0

        # Variables para cálculo de velocidad
        self.last_lat = None
        self.last_lon = None
        self.last_fix_time_utc = None  # datetime UTC desde GNSS
        self.prev_fix_time_utc = None  # para calcular delta de tiempo
        self.last_rmc_date = None  # (día, mes, año) desde RMC

        # Buffer para suavizado de velocidad (promedio móvil)
        self.speed_buffer = deque(maxlen=self.smoothing_window)

        # Control de rate limiting para publicación
        self.last_publish_time = self.get_clock().now()

        # ================= CONEXIÓN INICIAL =================
        self.connect_to_gps()

        # ================= TIMER PRINCIPAL =================
        # Timer que lee datos del GPS y gestiona reconexión
        timer_period = 1.0 / max(self.publish_rate, 1.0)
        self.create_timer(timer_period, self.read_gnss_data)
        
        #self.get_logger().info('Nodo GPS iniciado correctamente')

    def connect_to_gps(self):
        """
        Intenta conectarse al puerto serial del GPS.
        
        Returns:
            True si la conexión fue exitosa, False en caso contrario
        """
        try:
            # Cerrar conexión previa si existe
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
                self.ser = None
            
            # Intentar abrir el puerto serial
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.is_connected = True
            self.reconnect_attempts = 0
            self.reconnect_delay = INITIAL_RECONNECT_DELAY
            
            #self.get_logger().info(f'✓ Conectado al GPS en {self.port} @ {self.baud} bps')
            return True
            
        except serial.SerialException as e:
            self.is_connected = False
            #self.get_logger().warning(f'✗ No se pudo conectar a {self.port}: {e}')
            return False
        except Exception as e:
            self.is_connected = False
            #self.get_logger().error(f'✗ Error inesperado al conectar GPS: {e}')
            return False

    def attempt_reconnection(self):
        """
        Gestiona el proceso de reconexión automática con backoff exponencial.
        Intenta reconectar solo si ha pasado suficiente tiempo desde el último intento.
        """
        current_time = time.time()
        
        # Verificar si es momento de intentar reconectar
        if current_time - self.last_reconnect_time < self.reconnect_delay:
            return
        
        self.last_reconnect_time = current_time
        self.reconnect_attempts += 1
        
        #self.get_logger().info(
        #    f'Intentando reconexión #{self.reconnect_attempts} al GPS...'
        #)
        
        # Intentar reconectar
        if self.connect_to_gps():
            #self.get_logger().info('¡Reconexión exitosa!')
            return
        
        # Aumentar el delay con backoff exponencial
        if self.reconnect_attempts >= MAX_RECONNECT_ATTEMPTS:
            self.reconnect_delay = min(self.reconnect_delay * 2, MAX_RECONNECT_DELAY)
            """self.get_logger().warning(
                f'Múltiples intentos fallidos. Siguiente intento en {self.reconnect_delay:.1f}s'
            )"""

    def parse_gga(self, parts):
        """
        Parsea sentencia NMEA GGA (Global Positioning System Fix Data).
        Contiene: posición, altitud, calidad de fix, número de satélites, HDOP.
        
        Args:
            parts: Lista de campos de la sentencia GGA separados por comas
            
        Returns:
            True si el parseo fue exitoso, False en caso contrario
        """
        try:
            # Extraer campos relevantes
            time_str = parts[1] if len(parts) > 1 else ''
            lat_str = parts[2] if len(parts) > 2 else ''
            lat_dir = parts[3] if len(parts) > 3 else ''
            lon_str = parts[4] if len(parts) > 4 else ''
            lon_dir = parts[5] if len(parts) > 5 else ''
            fix_q = int(parts[6]) if len(parts) > 6 and parts[6] else 0
            num_sats = int(parts[7]) if len(parts) > 7 and parts[7] else 0
            hdop = float(parts[8]) if len(parts) > 8 and parts[8] else None
            alt = float(parts[9]) if len(parts) > 9 and parts[9] else None

            # Convertir coordenadas a grados decimales
            lat, lon = nmea_latlon_to_decimal(lat_str, lat_dir, lon_str, lon_dir)
            
            if lat is not None and lon is not None:
                self.latitude = lat
                self.longitude = lon
                self.altitude = alt if alt is not None else 0.0
                self.fix_quality = fix_q
                self.gps_sats = num_sats
                
                if hdop is not None:
                    self.hdop = hdop

                # Crear timestamp GNSS usando fecha almacenada de RMC
                gnss_dt = self.make_datetime_from_gnss(time_str, self.last_rmc_date)
                if gnss_dt:
                    self.last_fix_time_utc = gnss_dt.astimezone(timezone.utc)
                return True
            return False
            
        except Exception as e:
            #self.get_logger().warning(f'Error parseando GGA: {e}')
            return False

    def parse_rmc(self, parts):
        """
        Parsea sentencia NMEA RMC (Recommended Minimum Specific GNSS Data).
        Contiene: posición, fecha, velocidad sobre el suelo (SOG), rumbo.
        
        Args:
            parts: Lista de campos de la sentencia RMC separados por comas
            
        Returns:
            Tupla (éxito, velocidad_m/s, rumbo_grados)
        """
        try:
            # Extraer campos relevantes
            time_str = parts[1] if len(parts) > 1 else ''
            status = parts[2] if len(parts) > 2 else ''
            lat_str = parts[3] if len(parts) > 3 else ''
            lat_dir = parts[4] if len(parts) > 4 else ''
            lon_str = parts[5] if len(parts) > 5 else ''
            lon_dir = parts[6] if len(parts) > 6 else ''
            sog = float(parts[7]) if len(parts) > 7 and parts[7] else None  # nudos
            course = float(parts[8]) if len(parts) > 8 and parts[8] else None  # grados
            date_str = parts[9] if len(parts) > 9 else ''

            # Almacenar fecha para usarla con GGA
            if date_str and len(date_str) == 6:
                day = int(date_str[0:2])
                month = int(date_str[2:4])
                year = int(date_str[4:6]) + 2000
                self.last_rmc_date = (day, month, year)

            # Convertir coordenadas
            lat, lon = nmea_latlon_to_decimal(lat_str, lat_dir, lon_str, lon_dir)
            if lat is not None and lon is not None:
                self.latitude = lat
                self.longitude = lon

            # Publicar rumbo si está disponible (para diagnóstico)
            if course is not None:
                self.course_pub.publish(Float64(data=course))

            # Crear timestamp GNSS
            gnss_dt = self.make_datetime_from_gnss(time_str, self.last_rmc_date)
            if gnss_dt:
                self.last_fix_time_utc = gnss_dt.astimezone(timezone.utc)

            # Convertir velocidad de nudos a m/s
            speed_m_s = None
            if sog is not None:
                speed_m_s = sog * 0.514444  # 1 nudo = 0.514444 m/s

            return True, speed_m_s, course
            
        except Exception as e:
            #self.get_logger().warning(f'Error parseando RMC: {e}')
            return False, None, None

    def parse_gsa(self, parts):
        """
        Parsea sentencia NMEA GSA (GNSS DOP and Active Satellites).
        Contiene: modo de operación, satélites activos, PDOP, HDOP, VDOP.
        
        Args:
            parts: Lista de campos de la sentencia GSA separados por comas
            
        Returns:
            True si el parseo fue exitoso, False en caso contrario
        """
        try:
            # Extraer HDOP si está presente (campo 16)
            if len(parts) >= 17:
                try:
                    hdop = float(parts[16]) if parts[16] else None
                    if hdop:
                        self.hdop = hdop
                except Exception:
                    pass
            
            # Contar satélites activos en campos 3 a 14
            active = 0
            for i in range(3, 15):
                if i < len(parts) and parts[i] and parts[i].strip().isdigit():
                    active += 1
            self.gps_sats = active
            return True
            
        except Exception as e:
            #self.get_logger().warning(f'Error parseando GSA: {e}')
            return False

    def parse_gsv(self, parts):
        """
        Parsea sentencia NMEA GSV (GNSS Satellites in View).
        Contiene: información sobre satélites visibles.
        
        Args:
            parts: Lista de campos de la sentencia GSV separados por comas
            
        Returns:
            True si el parseo fue exitoso, False en caso contrario
        """
        try:
            # Número total de satélites visibles (campo 3)
            if len(parts) >= 4 and parts[3]:
                self.satellites_visible = int(parts[3])
            return True
        except Exception:
            return False

    def make_datetime_from_gnss(self, time_str, date_tuple):
        """
        Construye un objeto datetime a partir de tiempo GNSS y fecha.
        
        Args:
            time_str: Hora en formato HHMMSS.sss
            date_tuple: Tupla (día, mes, año) o None
            
        Returns:
            Objeto datetime en UTC o None si hay error
        """
        try:
            if not time_str:
                return None
            
            # Parsear tiempo: HHMMSS.sss
            hh = int(time_str[0:2])
            mm = int(time_str[2:4])
            ss = int(time_str[4:6])
            micro = 0
            
            # Parsear microsegundos si existen
            if '.' in time_str:
                frac = time_str.split('.', 1)[1]
                micro = int(float('0.' + frac) * 1e6)
            
            # Usar fecha proporcionada o fecha UTC actual como fallback
            if date_tuple:
                day, month, year = date_tuple
            else:
                now_utc = datetime.now(timezone.utc)
                day, month, year = now_utc.day, now_utc.month, now_utc.year
            
            dt = datetime(year, month, day, hh, mm, ss, micro, tzinfo=timezone.utc)
            return dt
            
        except Exception:
            return None

    def read_gnss_data(self):
        """
        Lee y procesa datos del GPS en cada iteración del timer.
        Gestiona la reconexión automática si el GPS se desconecta.
        """
        # Si no está conectado, intentar reconectar
        if not self.is_connected:
            self.attempt_reconnection()
            return

        try:
            # Verificar si hay datos disponibles
            if self.ser is None or not self.ser.is_open:
                self.is_connected = False
                #self.get_logger().warning('Puerto serial cerrado, intentando reconectar...')
                return

            if self.ser.in_waiting <= 0:
                return

            # Leer línea del GPS
            raw = self.ser.readline().decode('ascii', errors='ignore').strip()
            if not raw:
                return

            # Publicar datos crudos
            self.raw_pub.publish(String(data=raw))

            # Validar formato NMEA
            if not raw.startswith('$'):
                return

            # Verificar checksum
            if not verify_nmea_checksum(raw):
                #self.get_logger().warning(f'Checksum inválido: {raw[:80]}')
                return

            # Separar campos de la sentencia
            parts = raw.split(',')
            msg_type = parts[0][1:6]  # Ej: GNGGA, GPRMC, GLGSA, etc.

            # Parsear según el tipo de sentencia
            if 'GGA' in msg_type:
                self.parse_gga(parts)
            elif 'RMC' in msg_type:
                self.parse_rmc(parts)
            elif 'GSA' in msg_type:
                self.parse_gsa(parts)
            elif 'GSV' in msg_type:
                self.parse_gsv(parts)

            # Publicar fix y velocidad con rate limiting
            now = self.get_clock().now()
            elapsed = (now - self.last_publish_time).nanoseconds / 1e9
            interval = 1.0 / max(self.publish_rate, 1.0)
            
            if elapsed >= interval:
                if self.latitude is not None and self.longitude is not None:
                    self.publish_gps_fix_and_velocity()
                self.last_publish_time = now

        except (serial.SerialException, OSError) as e:
            # Error de conexión serial (incluye errno 5: I/O error) - marcar como desconectado
            self.is_connected = False
            #self.get_logger().error(f'Error de conexión serial: {e}. Intentando reconectar...')
            # Intentar cerrar el puerto corrupto
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except:
                pass
            self.ser = None
            
        except Exception as e:
            # Otros errores no relacionados con el serial
            #self.get_logger().error(f'Error inesperado leyendo datos GNSS: {e}')
            pass

    def publish_gps_fix_and_velocity(self):
        """
        Publica mensaje NavSatFix con la posición actual y mensajes de velocidad.
        Calcula la velocidad usando diferencia de posición o SOG de RMC.
        """
        # ================= PUBLICAR NavSatFix =================
        gps_msg = NavSatFix()
        gps_msg.header.frame_id = self.frame_id
        gps_msg.header.stamp = self.get_clock().now().to_msg()

        # Configurar estado del fix
        gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
        if self.fix_quality and self.fix_quality >= 1:
            gps_msg.status.status = NavSatStatus.STATUS_FIX
        
        # Tipo de servicio GNSS (GPS + GLONASS)
        try:
            gps_msg.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS)
        except Exception:
            gps_msg.status.service = 0

        # Coordenadas
        gps_msg.latitude = float(self.latitude)
        gps_msg.longitude = float(self.longitude)
        gps_msg.altitude = float(self.altitude) if self.altitude is not None else 0.0

        # Covarianza de posición basada en HDOP
        if self.hdop and self.hdop > 0:
            scale = 5.0  # Factor de escala empírico
            sigma = self.hdop * scale
            var = sigma * sigma
            cov = [0.0] * 9
            cov[0] = var  # varianza en latitud
            cov[4] = var  # varianza en longitud
            cov[8] = var * 9.0  # varianza en altitud (menos precisa)
            gps_msg.position_covariance = cov
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(gps_msg)

        # ================= CALCULAR VELOCIDAD =================
        speed_m_s = None
        dt_s = None
        
        # Calcular velocidad por diferencia de posición si hay datos previos
        if self.last_fix_time_utc and self.last_lat is not None and self.last_lon is not None:
            try:
                current_gnss_time = self.last_fix_time_utc
                
                # Calcular delta de tiempo entre fixes
                if self.prev_fix_time_utc:
                    dt = (current_gnss_time - self.prev_fix_time_utc).total_seconds()
                    dt_s = max(dt, 1e-3)  # evitar división por cero
                else:
                    dt_s = 1.0 / max(self.publish_rate, 1.0)

                # Calcular distancia con Haversine
                dist = haversine_m(self.last_lat, self.last_lon, self.latitude, self.longitude)
                speed_m_s = dist / dt_s if dt_s and dt_s > 0 else 0.0
                
            except Exception:
                speed_m_s = None

        # Fallback: estimar velocidad con intervalo del timer
        if speed_m_s is None:
            if self.last_lat is not None and self.last_lon is not None:
                dist = haversine_m(self.last_lat, self.last_lon, self.latitude, self.longitude)
                dt_s = 1.0 / max(self.publish_rate, 1.0)
                speed_m_s = dist / dt_s
            else:
                speed_m_s = 0.0
                dt_s = 1.0 / max(self.publish_rate, 1.0)

        # ================= SUAVIZADO DE VELOCIDAD =================
        # Agregar velocidad al buffer y calcular promedio móvil
        self.speed_buffer.append(speed_m_s)
        smoothed_speed = sum(self.speed_buffer) / len(self.speed_buffer)

        # Aplicar umbral: velocidades menores se consideran como estático
        publish_speed = smoothed_speed if smoothed_speed >= self.speed_threshold else 0.0

        # ================= PUBLICAR TwistStamped =================
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.frame_id
        twist.twist.linear.x = float(publish_speed)
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.vel_pub.publish(twist)

        # ================= PUBLICAR TwistWithCovarianceStamped =================
        twc = TwistWithCovarianceStamped()
        twc.header = twist.header
        twc.twist.twist = twist.twist

        # Estimar covarianza de velocidad basada en HDOP y delta tiempo
        if self.hdop and self.hdop > 0 and dt_s:
            pos_sigma = self.hdop * 5.0
            vel_sigma = pos_sigma / max(dt_s, 0.1)
            vel_var = vel_sigma * vel_sigma
        else:
            vel_var = 0.05  # varianza por defecto (m/s)²

        # Matriz de covarianza 6x6 para twist (lineal + angular)
        cov = [0.0] * 36
        cov[0] = vel_var  # varianza velocidad en X
        cov[7] = vel_var  # varianza velocidad en Y
        cov[14] = vel_var  # varianza velocidad en Z
        twc.twist.covariance = cov
        self.vel_cov_pub.publish(twc)

        # ================= ACTUALIZAR ESTADO PREVIO =================
        self.prev_fix_time_utc = self.last_fix_time_utc
        self.last_lat = self.latitude
        self.last_lon = self.longitude

        # ================= PUBLICAR INFORMACIÓN DIAGNÓSTICA =================
        gnss_local_str = 'desconocido'
        if self.last_fix_time_utc:
            gnss_local_str = self.last_fix_time_utc.astimezone(ECUADOR_TZ).isoformat(timespec='seconds')

        info = (
            f"lat:{self.latitude:.6f} lon:{self.longitude:.6f} alt:{(self.altitude if self.altitude is not None else 0.0):.2f}m "
            f"fix_q:{self.fix_quality} sats_usado:{self.gps_sats} sats_visibles:{self.satellites_visible} "
            f"hdop:{(self.hdop if self.hdop is not None else -1):.2f} "
            f"vel_raw:{speed_m_s:.3f}m/s suavizada:{smoothed_speed:.3f}m/s publicada:{publish_speed:.3f}m/s "
            f"timestamp_ec:{gnss_local_str}"
        )
        self.info_pub.publish(String(data=info))

    def destroy_node(self):
        """
        Limpieza al destruir el nodo: cierra el puerto serial.
        """
        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
                #self.get_logger().info('Puerto serial cerrado correctamente')
        except Exception as e:
            #self.get_logger().error(f'Error al cerrar puerto serial: {e}')
            pass
        super().destroy_node()


def main(args=None):
    """
    Función principal para iniciar el nodo GPS.
    """
    rclpy.init(args=args)
    node = None
    
    try:
        node = MultiGNSSPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupción por teclado - cerrando nodo GPS...")
    except Exception as e:
        print(f"Error crítico en nodo GNSS: {e}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()