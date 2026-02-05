#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from cv_bridge import CvBridge


class LaneDebugNode(Node):
    """Debug node que muestra TODAS las líneas incluyendo el centro de calzada"""

    def __init__(self):
        super().__init__('lane_debug_node')
        self.bridge = CvBridge()

        # Todas las líneas disponibles
        self.bev = None
        self.center_path = None      # /lane/tracked_path (centro de calzada)
        self.ego_path = None         # /lane/ego_center (centro carril derecho)
        self.left_path = None        # /lane/left (borde izquierdo calzada)
        self.right_path = None       # /lane/right (borde derecho calzada)
        
        self.scale = 40.0  # ESCALA ESTÁNDAR DEL SISTEMA

        # Subscriptores para TODAS las líneas
        self.create_subscription(Image, '/ipm/bev', self.bev_cb, 1)
        self.create_subscription(Path, '/lane/edge_path', self.center_cb, 1)  
        self.create_subscription(Path, '/lane/offset_path', self.ego_cb, 1)
        self.create_subscription(Path, '/lane/tracked_path', self.left_cb, 1)
        #self.create_subscription(Path, '/lane/right_tracked', self.right_cb, 1)

        #self.get_logger().info(" Lane Debug Node READY (Scale: 40.0 px/m)")

    def bev_cb(self, msg):
        self.bev = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.draw_all()

    def center_cb(self, msg):
        self.center_path = msg
        self.draw_all()

    def ego_cb(self, msg):
        self.ego_path = msg
        self.draw_all()

    def left_cb(self, msg):
        self.left_path = msg
        self.draw_all()

    """def right_cb(self, msg):
        self.right_path = msg
        self.draw_all()"""

    def draw_path(self, vis, path, label, color, thickness, H, W):
        """Función auxiliar para dibujar cualquier path"""
        if path and path.poses:
            points = []
            for pose in path.poses[:20]:  # Primeros 20 puntos
                # **FÓRMULA ESTÁNDAR**: 
                # Y positivo = izquierda, negativo = derecha
                # X positivo = adelante
                px = int(W/2 - pose.pose.position.y * self.scale)
                py = int(H - pose.pose.position.x * self.scale)
                
                if 0 <= px < W and 0 <= py < H:
                    cv2.circle(vis, (px, py), thickness+1, color, -1)
                    points.append((px, py))
            
            # Conectar puntos
            if len(points) > 1:
                pts_array = np.array(points, dtype=np.int32)
                cv2.polylines(vis, [pts_array], False, color, thickness)
            
            # Etiqueta con valor Y
            if points:
                y_val = path.poses[0].pose.position.y
                cv2.putText(vis, f"{label} (Y={y_val:+.2f}m)", 
                           (points[0][0] + 5, points[0][1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def draw_all(self):
        if self.bev is None:
            return

        H, W = self.bev.shape
        vis = cv2.cvtColor(self.bev, cv2.COLOR_GRAY2BGR)

        # Eje central (robot)
        cv2.line(vis, (W//2, H), (W//2, 0), (0, 255, 0), 1)

        # Marcas métricas
        max_dist = int(H / self.scale)
        for z in range(1, min(8, max_dist + 1)):
            y = int(H - z * self.scale)
            if y > 0:
                cv2.line(vis, (0, y), (W, y), (50, 50, 50), 1)
                if z % 2 == 0:
                    cv2.putText(vis, f"{z}m", (5, y-5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # **DIBUJAR TODOS LOS PATHS EN ORDEN DE IMPORTANCIA**
        paths_to_draw = [
            (self.center_path, 'RIGHT', (255, 255, 0), 3),    # Amarillo - Centro calzada
            (self.right_path, 'OFFSET', (255, 0, 0), 2),        # Azul - Borde derecho
            (self.left_path, 'RIGHT_EKF', (0, 0, 255), 2),          # Rojo - Borde izquierdo
            #(self.ego_path, 'EGO', (0, 255, 0), 4),            # Verde - Centro carril derecho
        ]
        
        for path, label, color, thickness in paths_to_draw:
            self.draw_path(vis, path, label, color, thickness, H, W)

        # Robot (coche) en la parte inferior
        cv2.rectangle(vis, (W//2-15, H-10), (W//2+15, H), (0, 255, 255), -1)

        # Información general del sistema
        cv2.putText(vis, f"BEV: {W}x{H} | Scale: {self.scale} px/m", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Leyenda completa
        legend = [
            ("RIGHT", (255, 255, 0), "Borde de la calzada"),
            ("OFFSET", (0, 255, 0), "Carril derecho - Centro vehículo"),
            ("RIGHT_EKF", (255, 0, 0), "Carril derecho - EKF"),
            #("LEFT", (0, 0, 255), "Borde izquierdo calzada")
        ]
        
        y_offset = 60
        for label, color, desc in legend:
            cv2.putText(vis, f"{label}: {desc}", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            y_offset += 25

        # Mostrar ventana
        cv2.imshow("LANE DEBUG - Complete System", vis)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = LaneDebugNode()
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


if __name__ == "__main__":
    main()