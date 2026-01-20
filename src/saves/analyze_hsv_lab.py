import cv2
import numpy as np
import glob
import os
import time

# =========================
# CARGA DE IMÁGENES
# =========================
img_dir = os.path.expanduser(
    "~/autonomous_navigation/src/saves/data_logs/"
    "ruta65_20260118_084718/perception/images"
)

images = sorted(glob.glob(os.path.join(img_dir, "*.jpg")))
assert len(images) > 0, "No se encontraron imágenes"

idx = 0
paused = False

# =========================
# PARÁMETROS AJUSTABLES
# =========================
use_clahe = False
use_gain_l = False
use_hsv_v = False
use_hsv_s = False

clahe_clip = 2.0
gain_l = 1.0
gain_v = 1.0
gain_s = 1.0
blur_k = 0  # 0 = off

active_param = "clahe"

# =========================
# UTILS
# =========================
def apply_processing(img):
    out = img.copy()

    # ---- LAB ----
    if use_clahe or use_gain_l:
        lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)

        if use_clahe:
            clahe = cv2.createCLAHE(
                clipLimit=clahe_clip,
                tileGridSize=(8, 8)
            )
            l = clahe.apply(l)

        if use_gain_l:
            l = np.clip(l.astype(np.float32) * gain_l, 0, 255).astype(np.uint8)

        lab = cv2.merge([l, a, b])
        out = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # ---- HSV ----
    if use_hsv_v or use_hsv_s:
        hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        if use_hsv_v:
            v = np.clip(v.astype(np.float32) * gain_v, 0, 255).astype(np.uint8)

        if use_hsv_s:
            s = np.clip(s.astype(np.float32) * gain_s, 0, 255).astype(np.uint8)

        hsv = cv2.merge([h, s, v])
        out = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # ---- BLUR ----
    if blur_k >= 3 and blur_k % 2 == 1:
        out = cv2.GaussianBlur(out, (blur_k, blur_k), 0)

    return out


def draw_status(img):
    txt = [
        f"[1] CLAHE-L: {'ON' if use_clahe else 'OFF'} (clip={clahe_clip:.2f})",
        f"[2] Gain L : {'ON' if use_gain_l else 'OFF'} ({gain_l:.2f})",
        f"[3] HSV-V  : {'ON' if use_hsv_v else 'OFF'} ({gain_v:.2f})",
        f"[4] HSV-S  : {'ON' if use_hsv_s else 'OFF'} ({gain_s:.2f})",
        f"Blur K     : {blur_k}",
        f"Active +/- : {active_param}",
        "[SPACE] Play/Pause | n/p frame | r reset | q quit"
    ]

    y = 25
    for t in txt:
        cv2.putText(img, t, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0), 1)
        y += 22


# =========================
# LOOP PRINCIPAL
# =========================
while True:
    img = cv2.imread(images[idx])
    proc = apply_processing(img)

    vis = np.hstack([img, proc])
    draw_status(vis)

    cv2.imshow("Perception Lab | Original vs Processed", vis)

    key = cv2.waitKey(40 if not paused else 0) & 0xFF

    if not paused:
        idx = (idx + 1) % len(images)

    if key == ord('q'):
        break
    elif key == ord(' '):
        paused = not paused
    elif key == ord('n'):
        idx = (idx + 1) % len(images)
    elif key == ord('p'):
        idx = (idx - 1) % len(images)
    elif key == ord('1'):
        use_clahe = not use_clahe
        active_param = "clahe"
    elif key == ord('2'):
        use_gain_l = not use_gain_l
        active_param = "gain_l"
    elif key == ord('3'):
        use_hsv_v = not use_hsv_v
        active_param = "gain_v"
    elif key == ord('4'):
        use_hsv_s = not use_hsv_s
        active_param = "gain_s"
    elif key == ord('+'):
        if active_param == "clahe":
            clahe_clip += 0.2
        elif active_param == "gain_l":
            gain_l += 0.05
        elif active_param == "gain_v":
            gain_v += 0.05
        elif active_param == "gain_s":
            gain_s += 0.05
    elif key == ord('-'):
        if active_param == "clahe":
            clahe_clip = max(0.2, clahe_clip - 0.2)
        elif active_param == "gain_l":
            gain_l = max(0.5, gain_l - 0.05)
        elif active_param == "gain_v":
            gain_v = max(0.5, gain_v - 0.05)
        elif active_param == "gain_s":
            gain_s = max(0.5, gain_s - 0.05)
    elif key == ord('r'):
        use_clahe = use_gain_l = use_hsv_v = use_hsv_s = False
        clahe_clip = 2.0
        gain_l = gain_v = gain_s = 1.0
        blur_k = 0

cv2.destroyAllWindows()

