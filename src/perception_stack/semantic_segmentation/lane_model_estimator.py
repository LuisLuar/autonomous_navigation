#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import struct
import colorsys
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from custom_interfaces.msg import LaneModel
from sklearn.cluster import DBSCAN

class QuadraticLaneEstimator(Node):
    def __init__(self):
        super().__init__('quadratic_lane_estimator')
        
        # Parámetros
        self.expected_width = 3.0      
        self.min_points = 5           
        self.cluster_tol = 0.8         # Aumentado un poco para curvas
        self.max_yaw_angle = 0.69       # Límite de orientación inicial

        # Filtro temporal
        self.d_lat_last = 10000
        self.max_d_lat_change = 1.5  # Máximo cambio permitido en d_lat entre frames (en metros)

        # Suscripción
        self.sub = self.create_subscription(PointCloud2, '/lane/meter_candidates', self.cb_points, 10)
        
        # Publicadores
        self.pub_model = self.create_publisher(LaneModel, '/lane/model_raw', 10)
        #self.pub_debug_clusters = self.create_publisher(PointCloud2, '/lane/debug_clusters', 10)
        #self.pub_debug_lines = self.create_publisher(PointCloud2, '/lane/debug_fitted_lines', 10)

    def create_pc2_msg(self, header, points_rgb):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points_rgb)

    def get_rgb_uint32(self, r, g, b):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

    def cb_points(self, msg):
        pts = np.array([(p[0], p[1]) for p in point_cloud2.read_points(msg, field_names=['x', 'y'], skip_nans=True)])
        if len(pts) < self.min_points: return

        # eps: Distancia máxima entre dos muestras para que se consideren vecinas
        # min_samples: El número de puntos necesarios para formar un "cluster"
        # Creamos una copia para el clustering
        pts_scaled = pts.copy()

        # Comprimimos el eje X logarítmicamente o con una raíz
        # Esto hace que 1 metro de diferencia a los 10m se vea "más corto" 
        # que 1 metro de diferencia a los 2m.
        pts_scaled[:, 0] = np.sqrt(pts[:, 0]) 

        clustering = DBSCAN(eps=0.3, min_samples=5).fit(pts_scaled)
        labels = clustering.labels_
        
        unique_labels = set(labels)
        # ------------------------------------

        candidates = []
        #cluster_cloud_pts = []
        mean = []
        
        for label in unique_labels:
            if label == -1: continue  # Saltar el ruido detectado por DBSCAN
            
            # Máscara para obtener solo los puntos de este cluster
            cluster_mask = (labels == label)
            cluster_pts = pts[cluster_mask]
            
            if len(cluster_pts) < self.min_points: 
                continue

            # --- LÓGICA DE CLASIFICACIÓN POR PORCENTAJE ---
            # Contamos cuántos puntos están a la izquierda (y > 0) y cuántos a la derecha (y < 0)
            puntos_izq = np.sum(cluster_pts[:, 1] > 0)
            puntos_der = np.sum(cluster_pts[:, 1] < 0)
            total = len(cluster_pts)
            
            porcentaje_izq = (puntos_izq / total) * 100
            # Decidimos el lado por mayoría simple (más del 50%)
            lado_predominante = "izq" if porcentaje_izq > 50 else "der"
            # ----------------------------------------------------
            
            # Color para debug (mantenemos tu lógica de colores)
            """hue = (label * 0.618) % 1.0
            rgb = colorsys.hsv_to_rgb(hue, 0.8, 1.0)
            color_int = self.get_rgb_uint32(int(rgb[0]*255), int(rgb[1]*255), int(rgb[2]*255))
            
            for p in cluster_pts:
                cluster_cloud_pts.append([float(p[0]), float(p[1]), 0.0, color_int])"""

            # AJUSTE DE POLINOMIO (Se mantiene igual)
            try:
                if max(cluster_pts[:, 0]) > 7.0 and len(cluster_pts) > 50:
                    # Usamos x como variable independiente y 'y' como dependiente
                    coeffs = np.polyfit(cluster_pts[:, 0], cluster_pts[:, 1], 2)
                    c2, c1, c0 = coeffs
                else:
                    # Para pocos puntos, ajustamos una línea recta (grado 1)
                    coeffs = np.polyfit(cluster_pts[:, 0], cluster_pts[:, 1], 1)
                    c1, c0 = coeffs
                    c2 = 0.0
                
                if abs(np.arctan(c1)) < self.max_yaw_angle and len(cluster_pts) > 10:
                    candidates.append({'lado': lado_predominante,'mean': np.mean(cluster_pts[:, 1]), 'c0': c0, 'c1': c1, 'c2': c2, 'pts': cluster_pts})
            except np.RankWarning:
                continue

        #if cluster_cloud_pts:
        #    self.pub_debug_clusters.publish(self.create_pc2_msg(msg.header, cluster_cloud_pts))

        if not candidates: return

        # 3. Selección de los mejores candidatos usando la nueva etiqueta 'lado'
        l_cands = [c for c in candidates if c['lado'] == "izq"]
        r_cands = [c for c in candidates if c['lado'] == "der"]

        best_l = min(l_cands, key=lambda x: abs(x['mean'])) if l_cands else None
        best_r = min(r_cands, key=lambda x: abs(x['mean'])) if r_cands else None

        """self.get_logger().info(f"lineas: {len(candidates)}")

        if best_l and best_r:
            self.get_logger().info(f"IZQUIERDA: puntos: {len(best_l['pts'])} c0: {best_l['c0']:.2f} mean: {best_l['mean']:.2f} | DERECHA: puntos{len(best_r['pts'])} c0: {best_r['c0']:.2f} mean: {best_r['mean']:.2f} | ANCHO: {abs(best_l['c0'] - best_r['c0']):.2f}")
        elif best_l:
            self.get_logger().info(f"IZQUIERDA:{len(best_l['pts'])} c0: {best_l['c0']:.2f} mean: {best_l['mean']:.2f} | DERECHA: NO VISTA")
        elif best_r:
            self.get_logger().info(f"IZQUIERDA: NO VISTA | DERECHA:{len(best_r['pts'])} c0: {best_r['c0']:.2f} mean: {best_r['mean']:.2f}")"""

        if best_l and best_r and abs(best_l['c0'] - best_r['c0']) > 3.5:
            if abs(best_l['c0']) < abs(best_r['c0']):
                best_r = None
            else:                
                best_l = None

        # 4. Construcción del Mensaje y Debug Visual
        out = LaneModel()
        out.header = msg.header
        #line_debug_pts = []
        #c_left = self.get_rgb_uint32(0, 255, 0)
        #c_right = self.get_rgb_uint32(0, 0, 255)
        #c_center = self.get_rgb_uint32(255, 0, 0)

        """def add_curve_to_debug(c0, c1, c2, color, max_distance=10, num_points=250):
            Dibuja una curva con más puntos cerca de la cámara.
            
            Args:
                c0, c1, c2: Coeficientes de la curva
                color: Color para el debug
                max_distance: Distancia máxima en metros
                num_points: Número total de puntos a generar
            
            # Distribución exponencial: más puntos cerca, menos lejos
            # Usamos logspace para crear espaciado logarítmico
            distances = np.logspace(-2, np.log10(max_distance), num_points)
            
            # Filtrar distancias que nos interesan (opcional)
            # distances = distances[distances <= max_distance]
            
            for x in distances:
                y = c2*(x**2) + c1*x + c0
                # El grosor también podría variar con la distancia si quieres
                # thickness = max(0.01, 0.1 - x/1000)  # Más grueso cerca
                line_debug_pts.append([float(x), float(y), 0.05, color])"""

        if best_l and best_r:
            out.lane_width = float(abs(best_l['c0'] - best_r['c0']))
            out.d_lat = float((best_l['c0'] + best_r['c0']) / 2.0)
            # Promediamos el ángulo local y la curvatura para la línea central
            avg_yaw = (best_l['c1'] + best_r['c1']) / 2.0

            if best_l['c2'] != 0 and best_r['c2'] != 0:
                avg_curv = (best_l['c2'] + best_r['c2']) / 2.0
            elif best_l['c2'] != 0:
                avg_curv = best_l['c2']
            elif best_r['c2'] != 0:
                avg_curv = best_r['c2']
            else:
                avg_curv = 0.0

            out.curvature = float(2.0 * avg_curv)
            out.yaw = float(np.arctan(avg_yaw))
            out.confidence = 1.0
            
            #add_curve_to_debug(best_l['c0'], best_l['c1'], best_l['c2'], c_left)
            #add_curve_to_debug(best_r['c0'], best_r['c1'], best_r['c2'], c_right)
            # Línea central
            #add_curve_to_debug(out.d_lat, avg_yaw, avg_curv, c_center)
            
        elif best_l or best_r:
            ref = best_l if best_l else best_r
            sign = -1 if best_l else 1

            out.lane_width = self.expected_width
            out.yaw = float(np.arctan(ref['c1']))
            out.curvature = float(2.0 * ref['c2']) 
            out.confidence = 0.5

            if not best_l and best_r and abs(best_r['c0']) > 3.0:
                # Forzamos el offset para mantenernos en el carril central
                # aunque no veamos la línea izquierda.
                distancia_al_centro = 4.5 
                out.d_lat = float(ref['c0'] + distancia_al_centro)
                self.get_logger().info("MODO ENTRADA DERECHO: Usando offset de 4.5m")
            elif best_l and not best_r and abs(best_l['c0']) > 3.0:
                distancia_al_centro = 4.5
                out.d_lat = float(ref['c0'] - distancia_al_centro)
                self.get_logger().info("MODO ENTRADA IZQUIERDO: Usando offset de 4.5m")
            else:
                # Comportamiento normal
                out.d_lat = float(ref['c0'] + sign * (self.expected_width / 2.0))
            
            #color = c_left if best_l else c_right
            #add_curve_to_debug(ref['c0'], ref['c1'], ref['c2'], color)
            # Línea central asumiendo misma curvatura que la única línea vista
            #add_curve_to_debug(out.d_lat, ref['c1'], ref['c2'], c_center)

        else:
            out.lane_width = self.expected_width
            out.d_lat = float(0.0)
            out.yaw = float(0.0)
            out.curvature = float(0.0)
            out.confidence = float(0.0)
            #add_curve_to_debug(0.0, 0.0, 0.0, c_center)

        # 5. Validación de Salto de Carril y Re-escaneo
        if ( abs(out.d_lat - self.d_lat_last) > self.max_d_lat_change or out.lane_width < 1.0 ) and self.d_lat_last != 10000:
            
            # Intento de recuperación 1: ¿Y si solo usamos la línea Izquierda?
            if best_l:
                d_lat_solo_l = float(best_l['c0'] + (self.expected_width / 2.0))
                if abs(d_lat_solo_l - self.d_lat_last) < self.max_d_lat_change:
                    out.lane_width = self.expected_width
                    out.yaw = float(np.arctan(best_l['c1']))
                    out.curvature = float(2.0 * best_l['c2']) 
                    out.d_lat = d_lat_solo_l
                    out.confidence = 0.7  # Confianza media por ser recuperación
                    #self.get_logger().warn("SALTO DETECTADO: Recuperando usando solo línea IZQUIERDA")
            
            # Intento de recuperación 2: ¿Y si solo usamos la línea Derecha?
            if best_r:
                d_lat_solo_r = float(best_r['c0'] - (self.expected_width / 2.0))
                if abs(d_lat_solo_r - self.d_lat_last) < self.max_d_lat_change:
                    out.lane_width = self.expected_width
                    out.yaw = float(np.arctan(best_r['c1']))
                    out.curvature = float(2.0 * best_r['c2']) 
                    out.d_lat = d_lat_solo_r
                    out.confidence = 0.7
                    #self.get_logger().warn("SALTO DETECTADO: Recuperando usando solo línea DERECHA")

            # Si después de intentar recuperarse sigue habiendo un salto, ahora sí invalidamos
            if abs(out.d_lat - self.d_lat_last) > self.max_d_lat_change:
                out.lane_width = self.expected_width
                out.confidence = 0.0
                out.yaw = float(0.0)
                out.curvature = float(0.0)
                out.d_lat = self.d_lat_last # O podrías mantener d_lat_last para que el Kalman no sufra
                #line_debug_pts = []
                #self.get_logger().error("SALTO IRRECUPERABLE: Modelo descartado")
        
        self.d_lat_last = out.d_lat if out.confidence > 0 else self.d_lat_last

        self.pub_model.publish(out)
        #self.pub_debug_lines.publish(self.create_pc2_msg(msg.header, line_debug_pts))
        

def main(args=None):
    rclpy.init(args=args)
    node = QuadraticLaneEstimator()

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