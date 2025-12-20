import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from pyproj import Proj, Transformer

class UTMMapOrigin(Node):

    def __init__(self):
        super().__init__("utm_map_origin")

        self.origin_set = False
        self.origin_utm = None
        
        # Declarar parámetros configurables
        self.declare_parameter('utm_zone', 17)  # Zona UTM (17 para Ecuador)
        self.declare_parameter('southern_hemisphere', True)  # True para hemisferio sur
        
        # Obtener parámetros
        utm_zone = self.get_parameter('utm_zone').value
        southern_hemisphere = self.get_parameter('southern_hemisphere').value
        
        # Inicializar transformador UTM con pyproj
        # pyproj maneja automáticamente las zonas y hemisferios
        self.utm_proj = Proj(
            proj='utm', 
            zone=utm_zone, 
            south=southern_hemisphere, 
            ellps='WGS84',
            preserve_units=True
        )
        
        # Transformer para conversión bidireccional (opcional)
        self.transformer = Transformer.from_proj(
            'EPSG:4326',  # WGS84 (lat/lon)
            self.utm_proj.crs,  # UTM
            always_xy=True
        )
        
        # Calcular zona UTM automáticamente basado en longitud (opcional)
        self.auto_calculate_zone = True

        self.sub = self.create_subscription(
            NavSatFix,
            "/gps/filtered",
            self.gps_cb,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            "/utm_map_origin",
            1
        )

        hemisphere = "S" if southern_hemisphere else "N"
        self.get_logger().info(f"UTM Map Origin node initialized")
        self.get_logger().info(f"Configuration: Zone {utm_zone}{hemisphere}")
        self.get_logger().info("Waiting for GPS data...")

    def calculate_utm_zone(self, longitude):
        """Calcula la zona UTM automáticamente basado en la longitud"""
        # Fórmula estándar para calcular zona UTM
        zone = int((longitude + 180) / 6) + 1
        
        # Zonas especiales
        if 56 <= longitude < 64 and 72 <= longitude < 84:
            # Zonas especiales de Noruega y Svalbard
            zone_boundaries = [(32, 56, 64), (31, 72, 84)]
            for z, lon_min, lon_max in zone_boundaries:
                if lon_min <= longitude < lon_max:
                    return z
        
        return zone

    def latlon_to_utm_pyproj(self, latitude, longitude):
        """Convierte latitud/longitud a UTM usando pyproj (más preciso)"""
        
        # Opcional: calcular zona automáticamente
        if self.auto_calculate_zone:
            calculated_zone = self.calculate_utm_zone(longitude)
            current_zone = self.get_parameter('utm_zone').value
            
            if calculated_zone != current_zone:
                self.get_logger().warn(
                    f"UTM zone mismatch! Calculated: {calculated_zone}, "
                    f"Configured: {current_zone}. Using configured zone."
                )
        
        try:
            # Método 1: Usar Proj directamente (simple)
            # utm_x, utm_y = self.utm_proj(longitude, latitude)
            
            # Método 2: Usar Transformer (más robusto)
            utm_x, utm_y = self.transformer.transform(longitude, latitude)
            
            return utm_x, utm_y
            
        except Exception as e:
            self.get_logger().error(f"pyproj conversion error: {e}")
            raise

    def gps_cb(self, msg):
        if self.origin_set:
            # Opcional: puedes publicar periódicamente o solo una vez
            # Para solo una vez, mantener return
            # Para publicación periódica, quitar el return
            return

        if msg.status.status < 0:
            self.get_logger().warn("GPS status negative, waiting for valid fix...")
            return
            
        # Verificar calidad GPS
        if msg.position_covariance[0] > 1.0 or msg.position_covariance[4] > 1.0:
            self.get_logger().warn("GPS covariance too high, waiting for better fix...")
            return

        try:
            utm_x, utm_y = self.latlon_to_utm_pyproj(msg.latitude, msg.longitude)
            
            # Validaciones adicionales
            utm_zone = self.get_parameter('utm_zone').value
            
            # Validar coordenada Este (x)
            if not (160000 <= utm_x <= 834000):
                self.get_logger().error(
                    f"UTM Easting out of range: {utm_x:.2f}. "
                    f"Expected 160,000-834,000 for zone {utm_zone}"
                )
                return
                
            # Validar coordenada Norte (y) según hemisferio
            southern = self.get_parameter('southern_hemisphere').value
            if southern and not (0 <= utm_y <= 10000000):
                self.get_logger().error(
                    f"UTM Northing out of range: {utm_y:.2f}. "
                    f"Expected 0-10,000,000 for southern hemisphere"
                )
                return
            elif not southern and not (0 <= utm_y <= 10000000):
                self.get_logger().error(
                    f"UTM Northing out of range: {utm_y:.2f}. "
                    f"Expected 0-10,000,000 for northern hemisphere"
                )
                return

            self.origin_utm = (utm_x, utm_y, msg.altitude)
            self.origin_set = True

            # Crear mensaje PointStamped
            p = PointStamped()
            p.header.stamp = self.get_clock().now().to_msg()
            
            hemisphere = "S" if southern else "N"
            p.header.frame_id = f"utm_{utm_zone}{hemisphere}"
            
            p.point.x = utm_x
            p.point.y = utm_y
            p.point.z = msg.altitude  # Usar altitud real

            self.pub.publish(p)

            self.get_logger().info("=" * 50)
            self.get_logger().info("UTM ORIGIN SUCCESSFULLY SET")
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"GPS Coordinates:")
            self.get_logger().info(f"  Latitude:  {msg.latitude:.8f}°")
            self.get_logger().info(f"  Longitude: {msg.longitude:.8f}°")
            self.get_logger().info(f"  Altitude:  {msg.altitude:.2f} m")
            self.get_logger().info(f"UTM Coordinates (Zone {utm_zone}{hemisphere}):")
            self.get_logger().info(f"  Easting (x):  {utm_x:.3f} m")
            self.get_logger().info(f"  Northing (y): {utm_y:.3f} m")
            self.get_logger().info(f"  Frame ID:     {p.header.frame_id}")
            self.get_logger().info("=" * 50)
            
            # Opcional: puedes deshabilitar la suscripción después de establecer el origen
            # self.destroy_subscription(self.sub)
            
        except Exception as e:
            self.get_logger().error(f"Error in GPS callback: {e}")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UTMMapOrigin()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UTM Map Origin node...")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass  # Ignorar error si ya se hizo shutdown

if __name__ == "__main__":
    main()