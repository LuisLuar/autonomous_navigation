#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from collections import deque

# Mensajes ROS
from custom_interfaces.msg import ObjectInfoArray, ObjectInfo
from std_msgs.msg import Float32, Bool

class VisionAlphaController(Node):
    def __init__(self):
        super().__init__('vision_alpha_controller')
        
        # ============ PARÁMETROS CONFIGURABLES ============
        self.declare_parameters(namespace='', parameters=[
            ('bump_crosswalk_max_distance', 8.0),      # metros
            ('bump_crosswalk_min_distance', 2.0),      # metros
            ('bump_alpha_min', 0.5),                   # valor mínimo de alpha para badenes/cruces
            ('person_distance_threshold', 3.0),        # metros en eje X
            ('person_lateral_threshold', 2.0),         # metros en eje Y (absoluto)
            ('person_alpha_zero_time', 5.0),           # segundos con alpha a 0
            ('vehicle_distance_start', 4.0),           # metros donde empieza a reducir
            ('vehicle_distance_critical', 3.0),        # metros donde alpha=0
            ('vehicle_lateral_threshold', 1.0),        # metros en eje Y (absoluto)
            ('vehicle_classes', ['car', 'truck', 'bus', 'motorcycle', 'bicycle']),  # clases de vehículos
            ('bump_classes', ['speed bump']),          # clases de badenes
            ('crosswalk_classes', ['crosswalk']),      # clases de cruces peatonales
            ('update_rate', 20.0),                     # Hz de actualización
            ('default_alpha', 1.0),                    # valor por defecto
        ])
        
        # ============ VARIABLES DE ESTADO ============
        self.last_objects_info = None
        self.last_objects_time = None
        self.alpha_value = self.get_parameter('default_alpha').value
        self.active_state = False
        
        # Para manejo de tiempos con personas
        self.person_zero_start_time = None
        self.in_person_zero_mode = False
        
        # Historial para suavizado
        self.alpha_history = deque(maxlen=10)
        self.alpha_history.append(self.alpha_value)
        
        # ============ SUSCRIPTORES ============
        self.create_subscription(
            ObjectInfoArray,
            '/objects/fused_info',
            self.objects_callback,
            10
        )
        
        # ============ PUBLICADORES ============
        self.alpha_pub = self.create_publisher(
            Float32,
            '/alpha/vision',
            10
        )
        
        self.active_pub = self.create_publisher(
            Bool,
            '/active/vision',
            10
        )
        
        # ============ TIMER PARA PUBLICACIÓN ============
        self.timer = self.create_timer(
            1.0 / self.get_parameter('update_rate').value,
            self.publish_alpha
        )
        
        #self.get_logger().info("Vision Alpha Controller iniciado")
    
    def objects_callback(self, msg: ObjectInfoArray):
        """Procesar información de objetos fusionados"""
        self.last_objects_info = msg
        self.last_objects_time = time.time()
        
        # Actualizar estado activo
        self.active_state = True
        
        # Calcular nuevo valor de alpha
        new_alpha = self.calculate_alpha(msg)
        
        # Suavizar cambios bruscos
        self.alpha_history.append(new_alpha)
        self.alpha_value = sum(self.alpha_history) / len(self.alpha_history)
    
    def calculate_alpha(self, msg: ObjectInfoArray) -> float:
        """
        Calcular valor de alpha basado en objetos detectados
        Prioridades:
        1. Personas cercanas (alpha=0.0 por 5s, luego 0.5)
        2. Vehículos cercanos
        3. Badenes/cruces peatonales
        4. Valor por defecto (1.0)
        """
        # Obtener parámetros
        default_alpha = self.get_parameter('default_alpha').value
        bump_classes = self.get_parameter('bump_classes').value
        crosswalk_classes = self.get_parameter('crosswalk_classes').value
        vehicle_classes = self.get_parameter('vehicle_classes').value
        
        # Variables para cálculos
        min_alpha = default_alpha
        has_person_near = False
        person_info = None
        
        # Verificar cada objeto
        for obj in msg.objects:
            # Ignorar objetos sin distancia válida
            if not obj.distance_valid:
                continue
            
            distance = obj.distance  # Eje X (adelante)
            lateral = obj.lateral_offset  # Eje Y (lateral)
            
            # ============ 1. DETECCIÓN DE PERSONAS ============
            if 'person' in obj.class_name.lower() or 'pedestrian' in obj.class_name.lower():
                person_dist_thresh = self.get_parameter('person_distance_threshold').value
                person_lat_thresh = self.get_parameter('person_lateral_threshold').value
                
                if (distance <= person_dist_thresh and 
                    abs(lateral) <= person_lat_thresh):
                    has_person_near = True
                    person_info = obj
                    
                    # Manejar el modo de 5 segundos con alpha=0.0
                    current_time = time.time()
                    
                    if not self.in_person_zero_mode:
                        # Primera detección cercana de persona
                        self.in_person_zero_mode = True
                        self.person_zero_start_time = current_time
                        """self.get_logger().warn(
                            f"Persona detectada cerca: d={distance:.1f}m, lat={lateral:.1f}m. "
                            f"Alpha=0.0 por 5 segundos."
                        )"""
                        return 0.0
                    else:
                        # Ya estábamos en modo persona cercana
                        time_in_mode = current_time - self.person_zero_start_time
                        person_zero_time = self.get_parameter('person_alpha_zero_time').value
                        
                        if time_in_mode < person_zero_time:
                            # Seguimos en los primeros 5 segundos
                            return 0.0
                        else:
                            # Pasados 5 segundos, alpha=0.5
                            """self.get_logger().info(
                                f"Persona persistente detectada. Alpha=0.5"
                            )"""
                            return 0.5
            
            # ============ 2. DETECCIÓN DE VEHÍCULOS ============
            if any(vehicle_class in obj.class_name.lower() for vehicle_class in vehicle_classes):
                vehicle_start = self.get_parameter('vehicle_distance_start').value
                vehicle_critical = self.get_parameter('vehicle_distance_critical').value
                vehicle_lat_thresh = self.get_parameter('vehicle_lateral_threshold').value
                
                if (abs(lateral) <= vehicle_lat_thresh and 
                    vehicle_critical <= distance <= vehicle_start):
                    
                    # Reducción progresiva de alpha
                    alpha_range = vehicle_start - vehicle_critical
                    distance_from_start = vehicle_start - distance
                    
                    if alpha_range > 0:
                        alpha_reduction = (distance_from_start / alpha_range) * default_alpha
                        vehicle_alpha = max(0.0, default_alpha - alpha_reduction)
                        min_alpha = min(min_alpha, vehicle_alpha)
                        
                        """self.get_logger().info(
                            f"Vehículo detectado: {obj.class_name} "
                            f"d={distance:.1f}m, lat={lateral:.1f}m. Alpha={vehicle_alpha:.2f}"
                        )"""
                    
                elif (abs(lateral) <= vehicle_lat_thresh and 
                      distance < vehicle_critical):
                    # Vehículo muy cerca, alpha=0.0
                    """self.get_logger().warn(
                        f"Vehículo MUY CERCA: {obj.class_name} "
                        f"d={distance:.1f}m, lat={lateral:.1f}m. Alpha=0.0"
                    )"""
                    return 0.0
            
            # ============ 3. DETECCIÓN DE BADENES/CRUCES ============
            is_bump = any(bump_class in obj.class_name.lower() for bump_class in bump_classes)
            is_crosswalk = any(cross_class in obj.class_name.lower() for cross_class in crosswalk_classes)
            
            if is_bump or is_crosswalk:
                bump_max = self.get_parameter('bump_crosswalk_max_distance').value
                bump_min = self.get_parameter('bump_crosswalk_min_distance').value
                bump_alpha_min = self.get_parameter('bump_alpha_min').value
                
                if bump_min <= distance <= bump_max:
                    # Reducción progresiva de alpha
                    bump_range = bump_max - bump_min
                    distance_from_max = bump_max - distance
                    
                    if bump_range > 0:
                        alpha_reduction = (distance_from_max / bump_range) * (default_alpha - bump_alpha_min)
                        bump_alpha = max(bump_alpha_min, default_alpha - alpha_reduction)
                        min_alpha = min(min_alpha, bump_alpha)
                        
                        obj_type = "Badén" if is_bump else "Cruce peatonal"
                        """self.get_logger().info(
                            f"{obj_type} detectado: d={distance:.1f}m. Alpha={bump_alpha:.2f}"
                        )"""
                
                elif distance < bump_min:
                    # Muy cerca del badén/cruce
                    min_alpha = min(min_alpha, bump_alpha_min)
                    """self.get_logger().info(
                        f"{'Badén' if is_bump else 'Cruce'} MUY CERCA: "
                        f"d={distance:.1f}m. Alpha={bump_alpha_min:.2f}"
                    )"""
        
        # ============ MANEJO DE FIN DE MODO PERSONA ============
        if self.in_person_zero_mode and not has_person_near:
            # Ya no hay personas cercanas, salir del modo
            self.in_person_zero_mode = False
            self.person_zero_start_time = None
            #self.get_logger().info("Personas ya no detectadas cerca. Volviendo a alpha normal.")
        
        # ============ RETORNAR EL VALOR FINAL ============
        if self.in_person_zero_mode:
            # Si estamos en modo persona, ya se manejó en la lógica anterior
            # Esto es solo para seguridad
            if time.time() - self.person_zero_start_time < self.get_parameter('person_alpha_zero_time').value:
                return 0.0
            else:
                return 0.5
        
        return min_alpha
    
    def publish_alpha(self):
        """Publicar valores de alpha y estado activo"""
        # Publicar estado activo
        active_msg = Bool()
        active_msg.data = self.active_state
        self.active_pub.publish(active_msg)
        
        # Publicar valor de alpha
        alpha_msg = Float32()
        alpha_msg.data = float(self.alpha_value)
        self.alpha_pub.publish(alpha_msg)
        
        # Si no hay datos recientes, volver a estado inactivo
        if self.last_objects_time and (time.time() - self.last_objects_time > 1.0):
            self.active_state = False
            self.alpha_value = self.get_parameter('default_alpha').value
            self.alpha_history.clear()
            self.alpha_history.append(self.alpha_value)
            
            # Resetear modo persona
            self.in_person_zero_mode = False
            self.person_zero_start_time = None
    
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionAlphaController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()