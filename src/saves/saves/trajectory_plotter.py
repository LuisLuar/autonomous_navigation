#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math
import numpy as np
from matplotlib.patches import Ellipse

class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__('trajectory_plotter')

        # ======================
        # Almacenamiento datos
        # ======================
        self.raw = []      # odom/unfiltered
        self.local = []    # EKF local
        self.global_ = []  # EKF global
        
        # Nuevo: almacenar datos del último punto EKF para la elipse
        self.last_local_point = None
        self.last_local_covariance = None
        self.last_local_orientation = None

        #Almacenar datos del ultimo punto EKF para la elipse global
        self.last_global_point = None
        self.last_global_covariance = None
        self.last_global_orientation = None
        
        # Variables para alinear ground truth
        self.has_received_ekf = False
        self.initial_ekf_orientation = None
        self.rotated_gt = None  # Ground truth rotado

        # ======================
        # Ground truth (generarlo pero no mostrarlo aún)
        # ======================
        self.original_gt = self.generate_ground_truth()
        self.gt_orientations = self.generate_gt_orientations()

        # ======================
        # Subscripciones
        # ======================
        self.create_subscription(
            Odometry, '/odom/unfiltered', self.raw_cb, 10)

        self.create_subscription(
            Odometry, '/odometry/local', self.local_cb, 10)

        self.create_subscription(
            Odometry, '/odometry/global', self.global_cb, 10)

        # ======================
        # Plot en tiempo real
        # ======================
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.timer = self.create_timer(0.5, self.update_plot)
        
        # Parámetros para la elipse
        self.ellipse_scale = 3.0  # 3-sigma

        #self.get_logger().info('📈 Trayectorias + Ground Truth + RMS en tiempo real')
        #self.get_logger().info('🎯 Elipse de covarianza solo en el último punto')

    # =========================================================
    # Ground truth geométrico con orientaciones
    # =========================================================
    def generate_ground_truth(self):
        path = [
            ("forward", 9.0),
            ("turn", math.pi/2),
            ("forward", 3.9),
            ("turn", -math.pi/2),
            ("forward", 8.0),
            ("turn", math.pi/2),
            ("forward", 3.6),
            ("turn", math.pi/2),
            ("forward", 17.0),
            ("turn", math.pi/2),
            ("forward", 7.5),
            ("turn", math.pi/2),
        ]

        resolution = 0.05  # 5 cm
        x, y, theta = 0.0, 0.0, 0.0
        gt = [(x, y)]

        for action, value in path:
            if action == "forward":
                steps = int(value / resolution)
                for _ in range(steps):
                    x += resolution * math.cos(theta)
                    y += resolution * math.sin(theta)
                    gt.append((x, y))
            elif action == "turn":
                theta += value

        return gt
    
    def generate_gt_orientations(self):
        """Genera las orientaciones para cada punto del ground truth"""
        path = [
            ("forward", 9.0),
            ("turn", math.pi/2),
            ("forward", 3.9),
            ("turn", -math.pi/2),
            ("forward", 8.0),
            ("turn", math.pi/2),
            ("forward", 3.6),
            ("turn", math.pi/2),
            ("forward", 17.0),
            ("turn", math.pi/2),
            ("forward", 7.5),
            ("turn", math.pi/2),
        ]

        resolution = 0.05
        theta = 0.0
        orientations = [theta]
        
        for action, value in path:
            if action == "forward":
                steps = int(value / resolution)
                for _ in range(steps):
                    orientations.append(theta)
            elif action == "turn":
                theta += value
                orientations.append(theta)
        
        return orientations

    def rotate_ground_truth(self, ekf_initial_yaw):
        """Rota el ground truth para que coincida con la orientación inicial del EKF"""
        # Calcular diferencia entre orientación inicial del EKF y del ground truth (0°)
        theta_offset = ekf_initial_yaw  # Ground truth empieza en 0°, así que offset = ekf_yaw
        
        # Rotar todos los puntos del ground truth
        rotated_points = []
        for x, y in self.original_gt:
            # Matriz de rotación
            c, s = math.cos(theta_offset), math.sin(theta_offset)
            x_rot = x * c - y * s
            y_rot = x * s + y * c
            rotated_points.append((x_rot, y_rot))
        
        # Rotar las orientaciones
        rotated_orientations = []
        for theta in self.gt_orientations:
            rotated_orientations.append(theta + theta_offset)
        
        return rotated_points, rotated_orientations

    # ======================
    # Callbacks
    # ======================
    def raw_cb(self, msg):
        self.raw.append((msg.pose.pose.position.x,
                         msg.pose.pose.position.y))

    def local_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Guardar posición
        self.local.append((x, y))
        
        # Extraer orientación del quaternion
        q = msg.pose.pose.orientation
        # Calcular yaw (simplificado)
        yaw = 2.0 * math.atan2(q.z, q.w)
        
        # Si es el primer dato del EKF, guardar orientación inicial y rotar ground truth
        if not self.has_received_ekf:
            self.has_received_ekf = True
            self.initial_ekf_orientation = yaw
            
            # Rotar ground truth para alinearlo con el EKF
            self.rotated_gt, self.rotated_gt_orientations = self.rotate_ground_truth(yaw)
            
            #self.get_logger().info(f'✅ Recibido primer dato del EKF')
            #self.get_logger().info(f'   Orientación inicial EKF: {math.degrees(yaw):.1f}°')
            #self.get_logger().info(f'   Ground Truth rotado para coincidir')
        
        self.last_local_orientation = yaw
        self.last_local_point = (x, y)
        
        # Extraer covarianza de posición (x, y)
        cov_matrix = msg.pose.covariance
        # Índices: 0=xx, 7=yy, 1=xy
        cov_xx = cov_matrix[0] if cov_matrix[0] > 0 else 0.001
        cov_yy = cov_matrix[7] if cov_matrix[7] > 0 else 0.001
        cov_xy = cov_matrix[1]
        
        self.last_local_covariance = (cov_xx, cov_yy, cov_xy)

    def global_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.global_.append((x,y))

        self.last_global_point = (x, y)
        # Extraer covarianza de posición (x, y)
        cov_matrix = msg.pose.covariance
        # Índices: 0=xx, 7=yy, 1=xy
        cov_xx = cov_matrix[0] if cov_matrix[0] > 0 else 0.001
        cov_yy = cov_matrix[7] if cov_matrix[7] > 0 else 0.001
        cov_xy = cov_matrix[1]
        self.last_global_covariance = (cov_xx, cov_yy, cov_xy)

    # ======================
    # Dibujar elipse en último punto
    # ======================
    def draw_last_point_ellipse(self):
        """Dibuja solo la elipse del último punto EKF con flecha de orientación"""
        if self.last_local_point is None or self.last_local_covariance is None:
            return
        
        x, y = self.last_local_point
        cov_xx, cov_yy, cov_xy = self.last_local_covariance
        
        # Crear matriz de covarianza 2x2
        cov_matrix = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])
        
        # Calcular autovalores y autovectores
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Ordenar autovalores de mayor a menor
        order = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]
        
        # Calcular ángulo de rotación de la elipse
        angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
        
        # Calcular dimensiones de la elipse (k-sigma)
        width = 2 * self.ellipse_scale * np.sqrt(eigenvalues[0])
        height = 2 * self.ellipse_scale * np.sqrt(eigenvalues[1])
        
        # Dibujar elipse (SOLO contorno, sin relleno)
        ellipse = Ellipse(xy=(x, y), width=width, height=height,
                         angle=angle, edgecolor='red', 
                         facecolor='none',  # Sin relleno
                         linewidth=1.5, alpha=0.8,
                         label='Covarianza (3σ)' if len(self.local) == 1 else "")
        self.ax.add_patch(ellipse)
        
        # Dibujar flecha roja (dirección de máxima incertidumbre)
        axis_length = self.ellipse_scale * np.sqrt(eigenvalues[0])
        dx = axis_length * eigenvectors[0, 0]
        dy = axis_length * eigenvectors[1, 0]
        
        self.ax.arrow(x, y, dx, dy, 
                     head_width=0.1, head_length=0.15, 
                     fc='red', ec='red', linewidth=1.5,
                     alpha=0.8, label='Dir. σ máx.' if len(self.local) == 1 else "")
        
        # Dibujar punto central
        self.ax.plot(x, y, 'ro', markersize=5, alpha=0.8)

    def draw_last_point_ellipse_global(self):
        """Dibuja la elipse del último punto EKF GLOBAL"""
        if self.last_global_point is None or self.last_global_covariance is None:
            return
        
        x, y = self.last_global_point
        cov_xx, cov_yy, cov_xy = self.last_global_covariance
        
        # Crear matriz de covarianza 2x2
        cov_matrix = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])
        
        # Calcular autovalores y autovectores
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Ordenar autovalores de mayor a menor
        order = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]
        
        # Calcular ángulo de rotación de la elipse
        angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
        
        # Calcular dimensiones de la elipse (k-sigma)
        width = 2 * self.ellipse_scale * np.sqrt(eigenvalues[0])
        height = 2 * self.ellipse_scale * np.sqrt(eigenvalues[1])
        
        # Dibujar elipse GLOBAL (con diferente color y estilo)
        ellipse = Ellipse(xy=(x, y), width=width, height=height,
                        angle=angle, edgecolor='purple', 
                        facecolor='none',  # Sin relleno
                        linewidth=1.5, alpha=0.8,
                        linestyle='--',  # Línea punteada
                        label='Covarianza global (3σ)' if len(self.global_) == 1 else "")
        self.ax.add_patch(ellipse)
        
        # Dibujar flecha magenta (dirección de máxima incertidumbre global)
        axis_length = self.ellipse_scale * np.sqrt(eigenvalues[0])
        dx = axis_length * eigenvectors[0, 0]
        dy = axis_length * eigenvectors[1, 0]
        
        self.ax.arrow(x, y, dx, dy, 
                    head_width=0.1, head_length=0.15, 
                    fc='magenta', ec='magenta', linewidth=1.5,
                    alpha=0.8, linestyle='--',
                    label='Dir. σ máx. global' if len(self.global_) == 1 else "")
        
        # Dibujar punto central global
        self.ax.plot(x, y, 'mo', markersize=5, alpha=0.8)  # magenta

    # ======================
    # Plot en tiempo real
    # ======================
    def update_plot(self):
        self.ax.clear()

        # ======================
        # 1. Ground Truth (solo si ya hay EKF y está rotado)
        # ======================
        if self.has_received_ekf and self.rotated_gt is not None:
            x, y = zip(*self.rotated_gt)
            self.ax.plot(x, y, 'g-.', linewidth=2, label='Ground Truth')
            
            # Mostrar orientación inicial del ground truth rotado
            if len(self.rotated_gt_orientations) > 0:
                init_x, init_y = self.rotated_gt[0]
                init_theta = self.rotated_gt_orientations[0]
                arrow_len = 1.0
                dx = arrow_len * math.cos(init_theta)
                dy = arrow_len * math.sin(init_theta)
                

                
                # Punto inicial
                self.ax.plot(init_x, init_y, 'go', markersize=8, alpha=0.7)
                
        elif not self.has_received_ekf:
            # Mostrar mensaje de espera
            self.ax.text(0.5, 0.5, 'Esperando primer dato del EKF...\nGround Truth oculto hasta entonces',
                        transform=self.ax.transAxes, fontsize=12,
                        ha='center', va='center',
                        bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))

        # ======================
        # 2. Odometría cruda
        # ======================
        if self.raw:
            x, y = zip(*self.raw)
            self.ax.plot(x, y, 'k--', label='Odom raw', linewidth=1.5, alpha=0.7)

        # ======================
        # 3. EKF local (línea completa)
        # ======================
        if self.local:
            x, y = zip(*self.local)
            self.ax.plot(x, y, 'b-', label='EKF local', linewidth=2, alpha=0.8)
            
        # ======================
        # 4. EKF global (si existe)
        # ======================
        if self.global_:
            x, y = zip(*self.global_)
            self.ax.plot(x, y, 'r-', label='EKF global', linewidth=2, alpha=0.8)

        # ======================
        # 5. Elipse en último punto
        # ======================
        self.draw_last_point_ellipse()

        # ======================
        # 6. Elipse en último punto GLOBAL (NUEVO)
        # ======================
        if self.last_global_point and self.last_global_covariance:
            self.draw_last_point_ellipse_global()

        # ======================
        # Configurar gráfico
        # ======================
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # Leyenda (solo si hay datos)
        if self.has_received_ekf or self.raw or self.local:
            handles, labels = self.ax.get_legend_handles_labels()
            # Eliminar duplicados en la leyenda
            by_label = dict(zip(labels, handles))
            self.ax.legend(by_label.values(), by_label.keys(), 
                          loc='upper left', fontsize=9, ncol=2)
        
        # Título informativo
        title = 'Comparación de trayectorias (tiempo real)'
        if self.initial_ekf_orientation is not None:
            title += f'\nOrientación inicial EKF: {math.degrees(self.initial_ekf_orientation):.1f}°'
        if self.last_local_covariance:
            title += f' | Elipse: {self.ellipse_scale}σ'
        self.ax.set_title(title, fontsize=11)
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        
        # Mostrar info de covarianza actual y estado
        """info_text = ''
        if self.has_received_ekf:
            info_text += f'✓ EKF activo\n'
            info_text += f'Orient. inicial: {math.degrees(self.initial_ekf_orientation):.1f}°\n'
        else:
            info_text += f'⏳ Esperando EKF\n'
        
        if self.last_local_covariance:
            cov_xx, cov_yy, cov_xy = self.last_local_covariance
            info_text += f'σ_x = {math.sqrt(cov_xx):.3f} m\n'
            info_text += f'σ_y = {math.sqrt(cov_yy):.3f} m\n'
            info_text += f'Puntos EKF: {len(self.local)}'
            
        if info_text:
            self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                        fontsize=8, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))"""

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # ======================
    # Cálculo RMS (usando ground truth rotado)
    # ======================
    def compute_rms(self, traj):
        if not traj or not self.rotated_gt:
            return None

        min_len = min(len(traj), len(self.rotated_gt))
        errors = []

        for i in range(min_len):
            dx = traj[i][0] - self.rotated_gt[i][0]
            dy = traj[i][1] - self.rotated_gt[i][1]
            errors.append(dx*dx + dy*dy)

        return math.sqrt(sum(errors) / len(errors))

    # ======================
    # Al cerrar nodo
    # ======================
    def finalize(self):
        # Calcular RMSs (solo si hay ground truth rotado)
        if self.has_received_ekf and self.rotated_gt:
            rms_raw   = self.compute_rms(self.raw)
            rms_local = self.compute_rms(self.local)
            rms_global= self.compute_rms(self.global_)

            print('\n' + '='*50)
            print('RESULTADOS RMS (Ground Truth rotado)')
            print('='*50)
            if rms_raw is not None:
                print(f'Odom raw     RMS: {rms_raw:.3f} m')
            if rms_local is not None:
                print(f'EKF local    RMS: {rms_local:.3f} m')
            if rms_global is not None:
                print(f'EKF global   RMS: {rms_global:.3f} m')
            
            # Info de alineación
            print(f'\nOrientación inicial EKF: {math.degrees(self.initial_ekf_orientation):.1f}°')
            print(f'Ground Truth rotado para coincidir')
            
            # Info adicional sobre covarianza final
            if self.last_local_covariance:
                cov_xx, cov_yy, cov_xy = self.last_local_covariance
                print(f'\nCovarianza final EKF:')
                print(f'  σ_x: {math.sqrt(cov_xx):.4f} m')
                print(f'  σ_y: {math.sqrt(cov_yy):.4f} m')
                print(f'  Puntos totales EKF: {len(self.local)}')
            
            print('='*50 + '\n')
        else:
            print('\n⚠️  No se recibieron datos del EKF - No se calculó RMS')
            print('   El ground truth no fue rotado ni mostrado')
            print('='*50 + '\n')

        plt.ioff()
        plt.show()
    
    def destroy_node(self):
        super().destroy_node()

# ======================
# Main
# ======================
def main():
    rclpy.init()
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.finalize()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()