#!/usr/bin/env python3
"""
analyze_seg_arrays.py

Analiza TODAS las rutas dentro de ./data_logs/
Extrae geometría del carril desde máscaras YOLOP.

- Se ejecuta sin argumentos
- No guarda imágenes (solo cv.imshow en vivo)
- Guarda resultados .npz junto al script
"""

import numpy as np
import cv2
from pathlib import Path
from tqdm import tqdm

# ================= CONFIG =================
ROW_STEP = 5
ROI_RATIO = 0.55
MIN_WIDTH_PX = 30
LANE_WIDTH_PX_DEFAULT = 160
CENTRE_TOLERANCE_RATIO = 0.30

SHOW_DEBUG = True   # solo visualización, NO guarda imágenes

# =========================================

def load_seg_mask(path: Path):
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Cannot load {path}")
    if img.ndim == 3:
        img = img[:, :, 0]
    return img.astype(np.uint8)


def analyze_frame(mask, row_positions):
    H, W = mask.shape
    road = (mask == 1)
    lane = (mask == 2)

    two_w, right_w, left_w, centres, flags = [], [], [], [], []

    for y in row_positions:
        xs = np.where(road[y])[0]
        if xs.size < MIN_WIDTH_PX:
            two_w.append(np.nan)
            right_w.append(np.nan)
            left_w.append(np.nan)
            centres.append(np.nan)
            flags.append(0)
            continue

        l_edge, r_edge = xs[0], xs[-1]
        width = r_edge - l_edge
        center_road = (l_edge + r_edge) / 2

        lane_xs = np.where(lane[y])[0]

        if lane_xs.size > 0:
            lx = int(np.median(lane_xs))
            rel = abs(lx - center_road) / width

            if rel <= CENTRE_TOLERANCE_RATIO:
                # línea divisoria
                lw = lx - l_edge
                rw = r_edge - lx
                flag = 1
                c = lx
            else:
                # borde detectado
                if abs(lx - l_edge) < abs(r_edge - lx):
                    # borde izquierdo
                    rw = min(LANE_WIDTH_PX_DEFAULT, r_edge - lx)
                    lw = width - rw
                    flag = 2
                    c = lx
                else:
                    # borde derecho
                    lw = min(LANE_WIDTH_PX_DEFAULT, lx - l_edge)
                    rw = width - lw
                    flag = 3
                    c = lx
        else:
            # sin lane → mitad
            c = int(center_road)
            lw = c - l_edge
            rw = r_edge - c
            flag = 0

        if lw <= 0 or rw <= 0:
            c = int(center_road)
            lw = c - l_edge
            rw = r_edge - c
            flag = 0

        two_w.append(width)
        left_w.append(lw)
        right_w.append(rw)
        centres.append(c)
        flags.append(flag)

    return {
        "two_w": np.array(two_w, np.float32),
        "left_w": np.array(left_w, np.float32),
        "right_w": np.array(right_w, np.float32),
        "centres": np.array(centres, np.float32),
        "flags": np.array(flags, np.int8)
    }


def visualize(mask, res, rows):
    H, W = mask.shape
    vis = np.zeros((H, W, 3), np.uint8)

    vis[mask == 1] = (60, 60, 60)
    vis[mask == 2] = (255, 0, 0)

    for y, c, lw, rw in zip(rows, res["centres"], res["left_w"], res["right_w"]):
        if np.isnan(c):
            continue
        c = int(c)
        cv2.rectangle(vis, (c - int(lw), y - 2),
                           (c + int(rw), y + 2), (0, 255, 0), -1)
        cv2.circle(vis, (c, y), 2, (0, 0, 255), -1)

    cv2.imshow("YOLOP Corridor Analysis", vis)
    cv2.waitKey(1)


def main():
    script_dir = Path(__file__).resolve().parent
    data_logs = script_dir / "data_logs"
    out_dir = script_dir / "segmentation_analysis"
    out_dir.mkdir(exist_ok=True)

    routes = sorted([r for r in data_logs.iterdir() if r.is_dir()])

    print(f"Found {len(routes)} routes")

    for route in routes:
        seg_dir = route / "perception" / "segmentation"
        if not seg_dir.exists():
            continue

        seg_files = sorted(seg_dir.glob("*_seg.png"))
        if not seg_files:
            continue

        mask0 = load_seg_mask(seg_files[0])
        H, W = mask0.shape
        rows = list(range(int(H * ROI_RATIO), H, ROW_STEP))

        nF, nR = len(seg_files), len(rows)
        out = {
            "two_w": np.full((nF, nR), np.nan),
            "left_w": np.full((nF, nR), np.nan),
            "right_w": np.full((nF, nR), np.nan),
            "centres": np.full((nF, nR), np.nan),
            "flags": np.zeros((nF, nR), np.int8),
        }

        print(f"Processing {route.name} ({nF} frames)")

        for i, p in enumerate(tqdm(seg_files)):
            mask = load_seg_mask(p)
            res = analyze_frame(mask, rows)

            for k in out:
                out[k][i] = res[k]

            if SHOW_DEBUG:
                visualize(mask, res, rows)

        np.savez_compressed(out_dir / f"{route.name}_analysis.npz", **out)
        print(f"Saved → {route.name}_analysis.npz")

    cv2.destroyAllWindows()
    print("DONE")


if __name__ == "__main__":
    main()

