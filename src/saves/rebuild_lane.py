#!/usr/bin/env python3
"""
rebuild_lanes_simple_overlay.py

Muestra la imagen RGB original como fondo con las líneas de carril reconstruidas superpuestas.
"""

import numpy as np
import cv2
from pathlib import Path
import argparse
import json
import sys
from typing import List, Dict, Tuple, Optional, Any

# ---------- Configuración ----------
class Config:
    DELAY_MS = 30  # ms entre frames
    LINE_WIDTH = 3
    POINT_SIZE = 4
    SHOW_POINTS = False

# ---------- Funciones de carga ----------
def load_analysis_data(analysis_file: Path) -> Dict[str, Any]:
    """Carga los datos del análisis."""
    try:
        data = np.load(analysis_file, allow_pickle=True)
        
        return {
            'route_names': data['route_names'],
            'row_positions': data['row_positions'],
            'all_left': data['all_left'],
            'all_right': data['all_right'],
            'route_start_indices': data['route_start_indices'],
            'route_end_indices': data['route_end_indices']
        }
    except Exception as e:
        print(f"⚠️  Error cargando análisis: {e}")
        return {}

def find_route_index(analysis_data: Dict[str, Any], route_name: str) -> Optional[int]:
    """Encuentra el índice de una ruta."""
    if 'route_names' not in analysis_data:
        return None
    
    for i, name in enumerate(analysis_data['route_names']):
        if route_name in str(name):
            return i
    return None

def get_rgb_and_mask_paths(route_dir: Path) -> Tuple[List[Path], List[Path]]:
    """Obtiene rutas de imágenes RGB y máscaras."""
    # Buscar en perception/rgb y perception/segmentation
    perception_dir = route_dir / "perception"
    
    if not perception_dir.exists():
        print(f"❌ No existe perception/ en {route_dir}")
        return [], []
    
    rgb_dir = perception_dir / "images"
    seg_dir = perception_dir / "segmentation"
    
    if not rgb_dir.exists():
        print(f"❌ No existe perception/rgb/")
        return [], []
    
    if not seg_dir.exists():
        print(f"❌ No existe perception/segmentation/")
        return [], []
    
    # Obtener archivos
    rgb_files = sorted(rgb_dir.glob("*.jpg"))
    seg_files = sorted(seg_dir.glob("*_seg.png"))
    
    if not rgb_files:
        print(f"❌ No hay imágenes RGB en {rgb_dir}")
        return [], []
    
    if not seg_files:
        print(f"❌ No hay máscaras en {seg_dir}")
        return [], []
    
    print(f"✓ {len(rgb_files)} imágenes RGB")
    print(f"✓ {len(seg_files)} máscaras")
    
    # Emparejar por nombre (asumiendo mismos nombres excepto _seg)
    rgb_dict = {}
    for f in rgb_files:
        name = f.stem  # Sin extensión
        rgb_dict[name] = f
    
    seg_dict = {}
    for f in seg_files:
        name = f.stem.replace("_seg", "_image")  # Quitar _seg
        seg_dict[name] = f
    
    # Encontrar archivos comunes
    common_names = sorted(set(rgb_dict.keys()) & set(seg_dict.keys()))
    
    if not common_names:
        print("⚠️  No se pudieron emparejar archivos por nombre")
        print(f"  Nombres RGB: {list(rgb_dict.keys())[:5]}...")
        print(f"  Nombres seg: {list(seg_dict.keys())[:5]}...")
        return [], []
    
    print(f"✓ {len(common_names)} archivos emparejados")
    
    rgb_matched = [rgb_dict[name] for name in common_names]
    seg_matched = [seg_dict[name] for name in common_names]
    
    return rgb_matched, seg_matched

# ---------- Reconstrucción ----------
def reconstruct_lanes_from_mask_and_analysis(
    mask: np.ndarray,
    row_positions: np.ndarray,
    left_widths: np.ndarray,
    right_widths: np.ndarray
) -> Dict[str, np.ndarray]:
    """Reconstruye líneas a partir de máscara y análisis."""
    H, W = mask.shape
    
    # Extraer road y lane markings
    road_mask = (mask == 1).astype(np.uint8)
    lane_mask = (mask == 2).astype(np.uint8)
    
    left_points = []
    center_points = []
    right_points = []
    valid_points = []
    
    for i, y in enumerate(row_positions):
        if y >= H:
            continue
            
        # Encontrar bordes de carretera en esta fila
        road_indices = np.where(road_mask[y, :] > 0)[0]
        
        if len(road_indices) < 20:
            left_points.append((np.nan, float(y)))
            center_points.append((np.nan, float(y)))
            right_points.append((np.nan, float(y)))
            valid_points.append(False)
            continue
        
        left_edge = float(road_indices[0])
        right_edge = float(road_indices[-1])
        road_width = right_edge - left_edge
        
        if road_width < 30:
            left_points.append((np.nan, float(y)))
            center_points.append((np.nan, float(y)))
            right_points.append((np.nan, float(y)))
            valid_points.append(False)
            continue
        
        # Calcular línea central usando análisis
        if i < len(left_widths) and not np.isnan(left_widths[i]):
            # Usar ancho del carril izquierdo desde el análisis
            center_x = left_edge + left_widths[i]
        else:
            # Si no hay análisis, buscar lane markings
            lane_indices = np.where(lane_mask[y, :] > 0)[0]
            if len(lane_indices) > 2:
                center_x = np.median(lane_indices)
            else:
                center_x = (left_edge + right_edge) / 2.0
        
        # Asegurar que esté dentro de la carretera
        center_x = max(left_edge + 5, min(right_edge - 5, center_x))
        
        # Validar
        if left_edge < center_x < right_edge:
            left_points.append((left_edge, float(y)))
            center_points.append((center_x, float(y)))
            right_points.append((right_edge, float(y)))
            valid_points.append(True)
        else:
            left_points.append((np.nan, float(y)))
            center_points.append((np.nan, float(y)))
            right_points.append((np.nan, float(y)))
            valid_points.append(False)
    
    return {
        'left': np.array(left_points, dtype=np.float32),
        'center': np.array(center_points, dtype=np.float32),
        'right': np.array(right_points, dtype=np.float32),
        'valid': np.array(valid_points, dtype=bool),
        'n_valid': np.sum(valid_points)
    }

# ---------- Visualización ----------
def draw_lanes_on_image(
    rgb_image: np.ndarray,
    lane_data: Dict[str, np.ndarray],
    config: Config
) -> np.ndarray:
    """Dibuja las líneas de carril sobre la imagen RGB."""
    overlay = rgb_image.copy()
    H, W = overlay.shape[:2]
    
    # Colores para las líneas
    colors = {
        'left': (0, 0, 255),      # Rojo - borde izquierdo
        'center': (255, 0, 0),    # Azul - línea central
        'right': (0, 255, 0)      # Verde - borde derecho
    }
    
    # Dibujar cada línea
    for line_type in ['left', 'center', 'right']:
        points = lane_data[line_type]
        valid = lane_data['valid']
        color = colors[line_type]
        
        # Dibujar puntos si está habilitado
        if config.SHOW_POINTS:
            for i, (x, y) in enumerate(points):
                if valid[i] and not np.isnan(x):
                    cv2.circle(overlay, (int(x), int(y)), 
                              config.POINT_SIZE, color, -1)
        
        # Dibujar líneas conectando puntos válidos consecutivos
        valid_indices = np.where(valid)[0]
        
        for i in range(len(valid_indices) - 1):
            idx1 = valid_indices[i]
            idx2 = valid_indices[i + 1]
            
            # Solo conectar si son adyacentes o cercanos
            if abs(points[idx1, 1] - points[idx2, 1]) < 20:
                pt1 = (int(points[idx1, 0]), int(points[idx1, 1]))
                pt2 = (int(points[idx2, 0]), int(points[idx2, 1]))
                
                # Línea central más gruesa
                thickness = config.LINE_WIDTH + 1 if line_type == 'center' else config.LINE_WIDTH
                cv2.line(overlay, pt1, pt2, color, thickness)
    
    return overlay

def add_info_panel(image: np.ndarray, info: Dict[str, str]) -> np.ndarray:
    """Añade panel de información a la imagen."""
    H, W = image.shape[:2]
    panel_height = 80
    panel = np.zeros((panel_height, W, 3), dtype=np.uint8)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # Información principal
    y = 30
    for key, value in info.items():
        text = f"{key}: {value}"
        cv2.putText(panel, text, (10, y), font, 0.6, (255, 255, 255), 1)
        y += 25
    
    # Leyenda
    legend_x = W - 200
    legend_y = 20
    
    legend_items = [
        ("Borde Izquierdo", (0, 0, 255)),
        ("Línea Central", (255, 0, 0)),
        ("Borde Derecho", (0, 255, 0))
    ]
    
    for i, (text, color) in enumerate(legend_items):
        # Cuadro de color
        cv2.rectangle(panel, (legend_x, legend_y + i*20 - 10),
                     (legend_x + 15, legend_y + i*20 + 5), color, -1)
        # Texto
        cv2.putText(panel, text, (legend_x + 20, legend_y + i*20),
                    font, 0.5, (255, 255, 255), 1)
    
    # Combinar panel con imagen
    result = np.vstack([panel, image])
    
    return result

# ---------- Visualización principal ----------
def visualize_route_with_overlay(
    route_dir: Path,
    analysis_data: Dict[str, Any],
    config: Config
):
    """Visualiza la ruta con imagen RGB de fondo y líneas superpuestas."""
    route_name = route_dir.name
    print(f"\n🎬 Visualizando: {route_name}")
    
    # Obtener archivos RGB y máscaras
    rgb_paths, seg_paths = get_rgb_and_mask_paths(route_dir)
    
    if not rgb_paths or not seg_paths:
        print("❌ No se pudieron obtener archivos")
        return
    
    print(f"✓ {len(rgb_paths)} frames para visualizar")
    
    # Obtener datos de análisis para esta ruta
    route_idx = find_route_index(analysis_data, route_name)
    
    if route_idx is not None:
        start_idx = analysis_data['route_start_indices'][route_idx]
        end_idx = analysis_data['route_end_indices'][route_idx]
        
        route_left = analysis_data['all_left'][start_idx:end_idx, :]
        route_right = analysis_data['all_right'][start_idx:end_idx, :]
        row_positions = analysis_data['row_positions']
        
        print(f"✓ Datos de análisis disponibles: {len(route_left)} frames")
    else:
        print("⚠️  No hay datos de análisis para esta ruta")
        print("⚠️  Usando solo información de máscaras")
        route_left = []
        route_right = []
        row_positions = np.arange(200, 480, 4)  # Valores por defecto
    
    # Crear ventana
    window_name = f"Carriles - {route_name}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)
    
    print("\n🎮 Controles:")
    print("   ESPACIO: Pausa/Reanudar")
    print("   ESC: Salir")
    print("   →: Siguiente frame")
    print("   ←: Frame anterior")
    print("   +: Aumentar velocidad")
    print("   -: Disminuir velocidad")
    print("   P: Mostrar/ocultar puntos")
    
    # Estado
    current_frame = 0
    paused = False
    delay = config.DELAY_MS
    
    while current_frame < len(rgb_paths):
        if not paused:
            try:
                # Cargar imagen RGB
                rgb_img = cv2.imread(str(rgb_paths[current_frame]))
                if rgb_img is None:
                    print(f"⚠️  Error cargando RGB {current_frame}")
                    current_frame += 1
                    continue
                
                # Cargar máscara
                mask = cv2.imread(str(seg_paths[current_frame]), cv2.IMREAD_UNCHANGED)
                if mask is None:
                    print(f"⚠️  Error cargando máscara {current_frame}")
                    current_frame += 1
                    continue
                
                if len(mask.shape) == 3:
                    mask = mask[:, :, 0]
                
                # Preparar datos de análisis para este frame
                if route_idx is not None and current_frame < len(route_left):
                    left_widths = route_left[current_frame]
                    right_widths = route_right[current_frame]
                else:
                    # Datos dummy
                    left_widths = np.full(len(row_positions), 160.0)
                    right_widths = np.full(len(row_positions), 160.0)
                
                # Reconstruir líneas
                lane_data = reconstruct_lanes_from_mask_and_analysis(
                    mask=mask,
                    row_positions=row_positions,
                    left_widths=left_widths,
                    right_widths=right_widths
                )
                
                # Dibujar líneas sobre imagen RGB
                overlay_img = draw_lanes_on_image(rgb_img, lane_data, config)
                
                # Añadir información
                info = {
                    "Ruta": route_name,
                    "Frame": f"{current_frame}/{len(rgb_paths)}",
                    "Puntos válidos": str(lane_data['n_valid'])
                }
                
                final_img = add_info_panel(overlay_img, info)
                
                # Mostrar
                cv2.imshow(window_name, final_img)
                
                # Info en consola
                print(f"\r▶ Frame {current_frame+1}/{len(rgb_paths)} | "
                      f"Válidos: {lane_data['n_valid']} | "
                      f"Delay: {delay}ms", end="")
                
                current_frame += 1
                
            except Exception as e:
                print(f"\n⚠️  Error en frame {current_frame}: {e}")
                current_frame += 1
        
        # Manejo de teclas
        key = cv2.waitKey(delay if not paused else 0) & 0xFF
        
        if key == 27:  # ESC
            print("\n\n⏹️  Salir")
            break
        elif key == 32:  # ESPACIO
            paused = not paused
            print(f"\n{'⏸️  Pausa' if paused else '▶ Reanudar'}")
        elif key == 83 or key == 54:  # → (flecha derecha o 6 en teclado numérico)
            current_frame = min(current_frame + 1, len(rgb_paths) - 1)
            print(f"\n⏭️  Frame {current_frame}")
        elif key == 81 or key == 52:  # ← (flecha izquierda o 4 en teclado numérico)
            current_frame = max(current_frame - 1, 0)
            print(f"\n⏮️  Frame {current_frame}")
        elif key == ord('+'):
            delay = max(1, delay - 5)
            print(f"\n⚡ Velocidad: {delay}ms")
        elif key == ord('-'):
            delay += 5
            print(f"\n🐌 Velocidad: {delay}ms")
        elif key == ord('p') or key == ord('P'):
            config.SHOW_POINTS = not config.SHOW_POINTS
            print(f"\n{'🔵 Mostrar puntos' if config.SHOW_POINTS else '⚪ Ocultar puntos'}")
    
    cv2.destroyAllWindows()
    print("\n✅ Visualización completada")

def main():
    parser = argparse.ArgumentParser(
        description="Muestra imagen RGB con líneas de carril superpuestas"
    )
    parser.add_argument(
        "route_path",
        type=str,
        help="Ruta al directorio de la ruta"
    )
    parser.add_argument(
        "--analysis",
        type=str,
        default="all_routes_lane_analysis.npz",
        help="Archivo de análisis (opcional)"
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Modo rápido (menos delay)"
    )
    parser.add_argument(
        "--points",
        action="store_true",
        help="Mostrar puntos individuales"
    )
    
    args = parser.parse_args()
    
    # Configurar
    config = Config()
    if args.fast:
        config.DELAY_MS = 10
    if args.points:
        config.SHOW_POINTS = True
    
    # Verificar ruta
    route_dir = Path(args.route_path)
    if not route_dir.exists():
        # Buscar en data_logs
        alt_path = Path("data_logs") / route_dir.name
        if alt_path.exists():
            route_dir = alt_path
        else:
            print(f"❌ No existe: {args.route_path}")
            return
    
    print("=" * 60)
    print("🖼️  IMAGEN RGB CON LÍNEAS DE CARRIL")
    print("=" * 60)
    print(f"Ruta: {route_dir.absolute()}")
    
    # Cargar análisis si existe
    analysis_file = Path(args.analysis)
    analysis_data = {}
    
    if analysis_file.exists():
        try:
            print(f"\n📊 Cargando análisis...")
            analysis_data = load_analysis_data(analysis_file)
            if analysis_data:
                print(f"✅ Análisis cargado: {len(analysis_data['route_names'])} rutas")
        except Exception as e:
            print(f"⚠️  No se pudo cargar análisis: {e}")
    else:
        print("ℹ️  No se encontró archivo de análisis, continuando sin él")
    
    # Visualizar
    visualize_route_with_overlay(route_dir, analysis_data, config)

if __name__ == "__main__":
    main()