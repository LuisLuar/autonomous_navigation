#!/usr/bin/env python3
import numpy as np
import cv2
import math
import os
import glob
import sys

class IPMProcessor:
    def __init__(self):
        # ===== PARAMETROS FÍSICOS (igual que en ROS) =====
        self.h = 0.38      # altura de la cámara en metros
        self.pitch = 0.35  # ángulo de inclinación en radianes (~ -20°)
        self.roi_ratio = 0.65  # ratio de ROI (solo procesar parte inferior)

        # ===== INTRÍNSECOS REALES (de tu cámara) =====
        self.fx = 574.0527954101562  # focal x
        self.fy = 574.0527954101562  # focal y
        self.cx = 319.5  # centro óptico x
        self.cy = 239.5  # centro óptico y

        # ===== BEV =====
        self.bev_w = 400
        self.bev_h = 600
        self.scale = 80.0  # px/m

        print("=" * 50)
        print("PARÁMETROS DE CALIBRACIÓN REALES:")
        print(f"  Altura: {self.h} m")
        print(f"  Pitch: {self.pitch} rad (~{math.degrees(self.pitch):.1f}°)")
        print(f"  ROI ratio: {self.roi_ratio}")
        print(f"  fx: {self.fx:.2f}, fy: {self.fy:.2f}")
        print(f"  cx: {self.cx:.1f}, cy: {self.cy:.1f}")
        print(f"  BEV: {self.bev_w}x{self.bev_h}, Scale: {self.scale} px/m")
        print("=" * 50)

    def process_image(self, seg_image):
        """Procesa una imagen de segmentación y la transforma a vista de pájaro"""
        if seg_image is None:
            return None

        H, W = seg_image.shape
        
        # Crear imagen BEV
        bev = np.zeros((self.bev_h, self.bev_w), dtype=np.uint8)
        
        # Calcular ROI (solo procesar parte inferior)
        v0 = int(H * self.roi_ratio)
        
        # Pre-calcular valores de rotación para eficiencia
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        
        # Matriz de rotación EXACTAMENTE como cuando funcionaba (pero invertido)
        R = np.array([
            [1,  0,   0],
            [0,  cp,  sp],     # sp sin signo negativo
            [0, -sp,  cp]      # -sp aquí
        ])
        
        # Procesar solo ROI
        for v in range(v0, H):
            for u in range(0, W, 2):  # Saltar cada 2 píxeles en horizontal para velocidad
                cls = seg_image[v, u]
                if cls == 0:
                    continue
                
                # Coordenadas de rayo (normalizadas)
                x = (u - self.cx) / self.fx
                y = -(v - self.cy) / self.fy  # Negativo porque Y crece hacia abajo en imagen
                ray = np.array([x, y, 1.0])
                
                # Rotar rayo según pitch de la cámara
                ray_w = R @ ray

                
                
                # ⚠️ CAMBIO CLAVE: invertir condición del rayo
                if ray_w[1] >= 0:  # Antes era <= 0
                    continue
                
                # ⚠️ CAMBIO CLAVE: añadir signo negativo
                t = self.h / -ray_w[1]  # Antes era / ray_w[1]
                
                # Coordenadas 3D en el suelo
                X = ray_w[0] * t   # Derecha/izquierda
                Z = -ray_w[2] * t   # ⚠️ CAMBIO CLAVE: NEGATIVO! Esto corrige la inversión
                
                # Convertir a coordenadas BEV
                bx = int(self.bev_w / 2 + X * self.scale)
                by = int(self.bev_h - Z * self.scale)
                
                # Solo dibujar si está dentro de los límites
                if 0 <= bx < self.bev_w and 0 <= by < self.bev_h:
                    if cls == 2:  # Líneas de carril
                        bev[by, bx] = 255
                    elif cls == 1:  # Área transitable
                        bev[by, bx] = 180
                    else:  # Otros
                        bev[by, bx] = 100
        
        return bev

    def process_from_folders(self, base_path):
        """Procesa todas las imágenes desde la estructura de carpetas"""
        # Construir rutas según la estructura que mostraste
        image_folder = os.path.join(base_path, "perception/images")
        seg_folder = os.path.join(base_path, "perception/segmentation")
        
        print(f"\nBuscando datos en:")
        print(f"  Imágenes: {image_folder}")
        print(f"  Segmentación: {seg_folder}")
        
        # Verificar carpetas
        if not os.path.exists(image_folder):
            print(f"ERROR: No existe el folder de imágenes: {image_folder}")
            return
        
        if not os.path.exists(seg_folder):
            print(f"ERROR: No existe el folder de segmentaciones: {seg_folder}")
            return
        
        # Buscar archivos
        image_files = sorted(glob.glob(os.path.join(image_folder, "*_image.jpg")))
        seg_files = sorted(glob.glob(os.path.join(seg_folder, "*_seg.png")))
        
        # Si no encuentra con ese patrón, intentar otros
        if not image_files:
            image_files = sorted(glob.glob(os.path.join(image_folder, "*.jpg")))
        
        if not seg_files:
            seg_files = sorted(glob.glob(os.path.join(seg_folder, "*.png")))
        
        print(f"\nEncontrados:")
        print(f"  {len(image_files)} imágenes")
        print(f"  {len(seg_files)} segmentaciones")
        
        if not image_files or not seg_files:
            print("ERROR: No se encontraron archivos para procesar")
            return
        
        # Crear ventanas
        cv2.namedWindow("Segmentación con ROI", cv2.WINDOW_NORMAL)
        cv2.namedWindow("IPM BEV", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Segmentación con ROI", 640, 480)
        cv2.resizeWindow("IPM BEV", 400, 600)
        
        # Procesar cada par de imágenes
        frame_count = 0
        min_files = min(len(image_files), len(seg_files))
        
        for i in range(min_files):
            img_path = image_files[i]
            seg_path = seg_files[i]
            
            frame_count += 1
            print(f"\n[{frame_count}/{min_files}] Procesando:")
            print(f"  Imagen: {os.path.basename(img_path)}")
            print(f"  Segmentación: {os.path.basename(seg_path)}")
            
            # Leer imágenes
            img = cv2.imread(img_path)
            seg = cv2.imread(seg_path, cv2.IMREAD_GRAYSCALE)
            
            if img is None or seg is None:
                print(f"  ERROR: No se pudo leer alguna imagen")
                continue
            
            print(f"  Tamaño imagen: {img.shape}")
            print(f"  Tamaño segmentación: {seg.shape}")
            
            # Asegurar que las imágenes tienen el tamaño esperado
            if img.shape[:2] != (480, 640):
                print(f"  ADVERTENCIA: Imagen RGB no es 640x480, es {img.shape[1]}x{img.shape[0]}")
                img = cv2.resize(img, (640, 480))
            
            if seg.shape != (480, 640):
                print(f"  ADVERTENCIA: Segmentación no es 640x480, es {seg.shape[1]}x{seg.shape[0]}")
                seg = cv2.resize(seg, (640, 480))
            
            # Procesar transformación IPM
            bev = self.process_image(seg)
            
            if bev is None:
                print("  ERROR: No se pudo generar BEV")
                continue
            
            # Contar puntos mapeados
            points_mapped = np.count_nonzero(bev)
            print(f"  Puntos mapeados en BEV: {points_mapped}")
            
            # ===== VISUALIZACIÓN SEGMENTACIÓN =====
            seg_vis = cv2.cvtColor(seg * 80, cv2.COLOR_GRAY2BGR)  # Escalar para visualización
            
            # Dibujar línea de ROI
            v0 = int(seg.shape[0] * self.roi_ratio)
            cv2.line(seg_vis, (0, v0), (seg.shape[1], v0), (0, 255, 0), 2)
            
            # Texto informativo
            cv2.putText(seg_vis, f"ROI: y > {v0}px", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(seg_vis, f"Frame: {frame_count}/{min_files}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Mostrar valores únicos para depuración
            unique_vals = np.unique(seg)
            cv2.putText(seg_vis, f"Classes: {unique_vals}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # ===== VISUALIZACIÓN BEV =====
            bev_vis = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
            
            # Dibujar eje del robot
            cv2.line(bev_vis,
                     (self.bev_w // 2, self.bev_h),
                     (self.bev_w // 2, self.bev_h - 120),
                     (0, 255, 0), 2)
            
            # Dibujar línea delantera del vehículo
            cv2.line(bev_vis,
                     (self.bev_w // 2 - 30, self.bev_h),
                     (self.bev_w // 2 + 30, self.bev_h),
                     (0, 0, 255), 2)
            
            # Dibujar rejilla de distancia
            for z in range(1, 6):  # 1m a 5m
                y = int(self.bev_h - z * self.scale)
                cv2.line(bev_vis, (0, y), (self.bev_w, y), (50, 50, 50), 1)
                cv2.putText(bev_vis, f"{z}m", (5, y - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            
            # Texto informativo en BEV
            cv2.putText(bev_vis, f"BEV: {points_mapped} puntos", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(bev_vis, f"Scale: {self.scale} px/m", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # ===== MOSTRAR RESULTADOS =====
            cv2.imshow("Segmentación con ROI", seg_vis)
            cv2.imshow("IPM BEV", bev_vis)
            
            # También mostrar imagen original pequeña
            img_small = cv2.resize(img, (320, 240))
            cv2.imshow("Imagen Original", img_small)
            
            # ===== CONTROLES INTERACTIVOS =====
            print("\n  CONTROLES:")
            print("    [ESPACIO] - Siguiente imagen")
            print("    [q] - Salir del programa")
            print("    [s] - Guardar BEV actual")
            print("    [+] - Aumentar escala (+10%)")
            print("    [-] - Disminuir escala (-10%)")
            print("    [r] - Resetear parámetros")
            print("    [d] - Mostrar/ocultar depuración")
            
            while True:
                key = cv2.waitKey(0) & 0xFF
                
                if key == ord(' '):  # Espacio - siguiente imagen
                    print("  >> Siguiente imagen")
                    break
                    
                elif key == ord('q'):  # q - salir
                    print("\nSaliendo por comando del usuario")
                    cv2.destroyAllWindows()
                    return
                    
                elif key == ord('s'):  # s - guardar
                    bev_filename = f"bev_{os.path.basename(seg_path)}"
                    cv2.imwrite(bev_filename, bev)
                    print(f"  BEV guardado como: {bev_filename}")
                    
                elif key == ord('+'):  # + - aumentar escala
                    self.scale = min(self.scale * 1.1, 200.0)
                    print(f"  Nueva escala: {self.scale:.1f} px/m")
                    # Reprocesar con nueva escala
                    bev = self.process_image(seg)
                    if bev is not None:
                        bev_vis = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
                        # Redibujar elementos
                        cv2.line(bev_vis, (self.bev_w // 2, self.bev_h),
                                 (self.bev_w // 2, self.bev_h - 120), (0, 255, 0), 2)
                        cv2.line(bev_vis, (self.bev_w // 2 - 30, self.bev_h),
                                 (self.bev_w // 2 + 30, self.bev_h), (0, 0, 255), 2)
                        for z in range(1, 6):
                            y = int(self.bev_h - z * self.scale)
                            cv2.line(bev_vis, (0, y), (self.bev_w, y), (50, 50, 50), 1)
                            cv2.putText(bev_vis, f"{z}m", (5, y - 5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                        cv2.putText(bev_vis, f"BEV: {np.count_nonzero(bev)} puntos", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.putText(bev_vis, f"Scale: {self.scale:.1f} px/m", (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                        cv2.imshow("IPM BEV", bev_vis)
                    
                elif key == ord('-'):  # - - disminuir escala
                    self.scale = max(self.scale * 0.9, 20.0)
                    print(f"  Nueva escala: {self.scale:.1f} px/m")
                    # Reprocesar con nueva escala
                    bev = self.process_image(seg)
                    if bev is not None:
                        bev_vis = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
                        # Redibujar elementos
                        cv2.line(bev_vis, (self.bev_w // 2, self.bev_h),
                                 (self.bev_w // 2, self.bev_h - 120), (0, 255, 0), 2)
                        cv2.line(bev_vis, (self.bev_w // 2 - 30, self.bev_h),
                                 (self.bev_w // 2 + 30, self.bev_h), (0, 0, 255), 2)
                        for z in range(1, 6):
                            y = int(self.bev_h - z * self.scale)
                            cv2.line(bev_vis, (0, y), (self.bev_w, y), (50, 50, 50), 1)
                            cv2.putText(bev_vis, f"{z}m", (5, y - 5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                        cv2.putText(bev_vis, f"BEV: {np.count_nonzero(bev)} puntos", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.putText(bev_vis, f"Scale: {self.scale:.1f} px/m", (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                        cv2.imshow("IPM BEV", bev_vis)
                    
                elif key == ord('r'):  # r - reset
                    self.scale = 80.0
                    print(f"  Parámetros reseteados")
                    print(f"  Escala: {self.scale} px/m")
                    # Reprocesar
                    bev = self.process_image(seg)
                    if bev is not None:
                        bev_vis = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
                        cv2.line(bev_vis, (self.bev_w // 2, self.bev_h),
                                 (self.bev_w // 2, self.bev_h - 120), (0, 255, 0), 2)
                        cv2.line(bev_vis, (self.bev_w // 2 - 30, self.bev_h),
                                 (self.bev_w // 2 + 30, self.bev_h), (0, 0, 255), 2)
                        for z in range(1, 6):
                            y = int(self.bev_h - z * self.scale)
                            cv2.line(bev_vis, (0, y), (self.bev_w, y), (50, 50, 50), 1)
                            cv2.putText(bev_vis, f"{z}m", (5, y - 5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                        cv2.putText(bev_vis, f"BEV: {np.count_nonzero(bev)} puntos", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.putText(bev_vis, f"Scale: {self.scale} px/m", (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                        cv2.imshow("IPM BEV", bev_vis)
        
        cv2.destroyAllWindows()
        print(f"\n{'='*50}")
        print(f"PROCESAMIENTO COMPLETADO")
        print(f"Frames procesados: {frame_count}")
        print(f"{'='*50}")

def main():
    # Verificar argumentos de línea de comandos
    if len(sys.argv) < 2:
        print("USO: python3 ipm_offline.py <ruta_base>")
        print("\nEjemplos:")
        print("  python3 ipm_offline.py data_logs/ruta30_20260107_173647/")
        print("  python3 ipm_offline.py data_logs/ruta5_20260106_134920/")
        print("  python3 ipm_offline.py data_logs/ruta6_20260106_153301/")
        print("\nSe asume la estructura:")
        print("  <ruta_base>/perception/images/*_image.jpg")
        print("  <ruta_base>/perception/segmentation/*_seg.png")
        return
    
    base_path = sys.argv[1].rstrip('/')
    
    # Verificar que la ruta existe
    if not os.path.exists(base_path):
        print(f"ERROR: La ruta no existe: {base_path}")
        return
    
    print(f"\nIniciando procesamiento IPM offline")
    print(f"Ruta base: {base_path}")
    
    # Crear y ejecutar procesador
    processor = IPMProcessor()
    processor.process_from_folders(base_path)

if __name__ == "__main__":
    main()