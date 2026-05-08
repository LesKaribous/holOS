"""
robot_tracker.py — coeur du tracking ArUco (utilisable en bibliothèque)
=======================================================================

Convention monde (mm) :
  - origine au coin (0, 0) de la table
  - X horizontal [0, 3000]
  - Y profondeur [0, 2000]   (mais les robots peuvent être hors table en Y > 2000)
  - Z = 0 plan du sol

Tags fixes au sol (Z=0) :
  ID 20 : (600, 1400)    ID 21 : (2400, 1400)
  ID 22 : (600, 600)     ID 23 : (2400, 600)

Tags robots (Z = configurable, par défaut 530 mm) :
  ID 2 = robot user
  ID 7 = adversaire
"""

import cv2
import numpy as np
from collections import deque


# ---------------------------------------------------------------------------
# Configuration par défaut
# ---------------------------------------------------------------------------
TABLE_WIDTH_MM  = 3000.0
TABLE_DEPTH_MM  = 2000.0
Z_ROBOT_MM      = 530.0
ARUCO_DICT      = cv2.aruco.DICT_4X4_50

TAGS_FIXED = {
    20: (600.0, 1400.0),
    21: (2400.0, 1400.0),
    22: (600.0, 600.0),
    23: (2400.0, 600.0),
}
TAGS_ROBOT = {
    2: ('USER',     (0, 255, 0)),     # vert
    7: ('OPPONENT', (0, 0, 255)),     # rouge
}

BEV_PX_PER_MM = 0.20


# ---------------------------------------------------------------------------
# Détection
# ---------------------------------------------------------------------------
# OpenCV ≥ 4.7 has cv2.aruco.ArucoDetector; older builds (Jetson NVIDIA
# cv2 4.5.x) use the legacy functional API. detect_tags() handles both.
_HAS_NEW_ARUCO_API = hasattr(cv2.aruco, 'ArucoDetector')


def make_detector():
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    else:
        d = cv2.aruco.Dictionary_get(ARUCO_DICT)
    if _HAS_NEW_ARUCO_API:
        p = cv2.aruco.DetectorParameters()
    else:
        p = cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshConstant = 7
    if _HAS_NEW_ARUCO_API:
        return cv2.aruco.ArucoDetector(d, p)
    # Legacy: return a tuple (dict, params). detect_tags branches on type.
    return (d, p)


def detect_tags(img, detector):
    """Retourne {id: {'corners': (4,2), 'center': (2,)}}."""
    if isinstance(detector, tuple):
        d, p = detector
        corners, ids, _ = cv2.aruco.detectMarkers(img, d, parameters=p)
    else:
        corners, ids, _ = detector.detectMarkers(img)
    out = {}
    if ids is None:
        return out
    for i, idarr in enumerate(ids):
        c4 = corners[i][0]
        out[int(idarr[0])] = {'corners': c4, 'center': c4.mean(0)}
    return out


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def compute_homography(tags):
    """H : pixel -> monde mm si les 4 tags fixes sont visibles, sinon None."""
    if not all(t in tags for t in TAGS_FIXED):
        return None
    src = np.array([tags[t]['center'] for t in TAGS_FIXED], dtype=np.float32)
    dst = np.array([TAGS_FIXED[t]      for t in TAGS_FIXED], dtype=np.float32)
    return cv2.getPerspectiveTransform(src, dst)


def apply_h(H, pts):
    pts = np.atleast_2d(np.asarray(pts, dtype=np.float64))
    homo = np.column_stack([pts, np.ones(len(pts))])
    out = (H @ homo.T).T
    return out[:, :2] / out[:, 2:3]


def estimate_camera_position_solvepnp(tags, image_size, fx_estimate=None):
    """
    Estime (Xc, Yc, Zc) en mm via solvePnP avec K approximatif.
    Avec un fx mal calibré (caméra fisheye, distortion non-modélisée),
    le résultat est *approximatif*. Pour de la précision, utiliser
    `solve_camera_xy_from_known_position`.
    Retourne (cam_xyz, fx_used) ou (None, None).
    """
    if not all(t in tags for t in TAGS_FIXED):
        return None, None
    w, h = image_size
    fx = float(fx_estimate) if fx_estimate is not None else w * 0.7
    K = np.array([[fx, 0, w/2.],
                  [0, fx, h/2.],
                  [0, 0, 1]], dtype=np.float64)
    obj = np.array([list(TAGS_FIXED[t]) + [0.0] for t in TAGS_FIXED], dtype=np.float32)
    img = np.array([tags[t]['center']           for t in TAGS_FIXED], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, np.zeros(5),
                                   flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None, None
    R, _ = cv2.Rodrigues(rvec)
    cam = -R.T @ tvec
    return cam.ravel(), fx


def solve_camera_xy_from_known_position(naive_xy, real_xy, Zc, z_object):
    """
    Inversion de la formule de parallaxe.
    Étant donné qu'un tag (à hauteur z_object) projeté donne 'naive_xy' au sol
    et que sa vraie position est 'real_xy', avec une caméra à hauteur Zc,
    déduit la position caméra (Xc, Yc, Zc).

        real = camera_xy + (Zc-z_obj)/Zc * (naive - camera_xy)
        => camera_xy = (real - factor*naive) / (1 - factor)   où factor = (Zc-z_obj)/Zc
    """
    factor = (Zc - z_object) / Zc
    if abs(1 - factor) < 1e-9:
        return None
    cam_xy = (np.asarray(real_xy) - factor * np.asarray(naive_xy)) / (1 - factor)
    return np.array([cam_xy[0], cam_xy[1], Zc])


def correct_parallax(pos_naive, cam_xyz, z_object):
    """
    pos_naive : (X_obs, Y_obs) projection sur Z=0 d'un tag à hauteur z_object,
                vue depuis la caméra située en cam_xyz.
    Retourne (X_real, Y_real) la position réelle au sol Z=0 sous le tag.
    """
    Xc, Yc, Zc = cam_xyz
    factor = (Zc - z_object) / Zc
    return np.array([Xc, Yc]) + factor * (np.asarray(pos_naive) - np.array([Xc, Yc]))


# ---------------------------------------------------------------------------
# Etat tracker (calibration cachée + lissage)
# ---------------------------------------------------------------------------
class TrackerState:
    """
    Pipeline de tracking par frame.

    Si camera_override = (Xc, Yc, Zc) est fourni, la position caméra est forcée
    à cette valeur (et la correction de parallaxe l'utilise). Sinon, on estime
    via décomposition de l'homographie.

    z_object : hauteur des tags robots (mm). Modifiable à tout moment.
    """
    def __init__(self, image_size,
                 z_object=Z_ROBOT_MM,
                 fx_estimate=None,
                 camera_override=None,
                 smooth_calib=True,
                 trail_len=80):
        self.image_size = image_size
        self.fx_estimate = fx_estimate
        self.z_object = z_object
        self.camera_override = camera_override
        self.smooth_calib = smooth_calib
        self.detector = make_detector()
        self.trails = {tid: deque(maxlen=trail_len) for tid in TAGS_ROBOT}
        self.last_positions = {}
        self.last_naive = {}
        self.tags_seen = set()
        self.H_pix2world = None
        self.cam_pos_auto = None       # estimée depuis homographie
        self.cam_pos = None            # active (auto ou override)
        self.fx_auto = None
        self.calib_source = "none"
        self.calib_age = 0
        self._calib_history = deque(maxlen=30)

    def reset_trails(self):
        for d in self.trails.values():
            d.clear()

    def _update_calibration(self, tags):
        H_fresh = compute_homography(tags)
        if H_fresh is not None:
            self._calib_history.append(H_fresh.copy())
            self.calib_age = 0
            self.calib_source = "fresh"
            if self.smooth_calib and len(self._calib_history) >= 5:
                self.H_pix2world = np.stack(list(self._calib_history)).mean(0)
            else:
                self.H_pix2world = H_fresh
        else:
            self.calib_age += 1
            if self.H_pix2world is not None:
                self.calib_source = "cached"
            else:
                self.calib_source = "none"

        # estimation auto position caméra (via solvePnP, approximative)
        cam_auto = None; fx_used = None
        if H_fresh is not None or len(self._calib_history) > 0:
            try:
                cam_auto, fx_used = estimate_camera_position_solvepnp(
                    tags, self.image_size, self.fx_estimate)
            except Exception:
                cam_auto = None
        self.cam_pos_auto = cam_auto
        self.fx_auto = fx_used

        # cam_pos = override si fourni, sinon auto
        if self.camera_override is not None:
            self.cam_pos = np.asarray(self.camera_override, dtype=np.float64)
        else:
            self.cam_pos = cam_auto

    def process(self, frame):
        tags = detect_tags(frame, self.detector)
        self.tags_seen = set(tags.keys())
        self._update_calibration(tags)

        self.last_positions = {}
        self.last_naive = {}
        if self.H_pix2world is not None:
            for tid in TAGS_ROBOT:
                if tid in tags:
                    pix = tags[tid]['center']
                    naive = apply_h(self.H_pix2world, pix)[0]
                    if self.cam_pos is not None and self.z_object > 0:
                        pos = correct_parallax(naive, self.cam_pos, self.z_object)
                    else:
                        pos = naive
                    self.last_positions[tid] = pos
                    self.last_naive[tid] = naive
                    self.trails[tid].append(tuple(pos))
        return tags


# ---------------------------------------------------------------------------
# Préprocesseur image
# ---------------------------------------------------------------------------
class Preprocessor:
    """Pipeline d'opérations à activer/désactiver. Toutes en BGR uint8."""

    def __init__(self):
        self.opts = {
            'grayscale':       {'on': False},
            'clahe':           {'on': False, 'clip': 3.0,  'tile': 8},
            'gaussian_blur':   {'on': False, 'sigma': 1.5},
            'bilateral':       {'on': False, 'd': 9, 'sigma_color': 75, 'sigma_space': 75},
            'unsharp':         {'on': False, 'amount': 1.0, 'radius': 1.5},
            'brightness':      {'on': False, 'alpha': 1.0, 'beta': 0.0},
            'gamma':           {'on': False, 'value': 1.0},
            'adaptive_thresh': {'on': False, 'block': 31, 'C': 5},
        }

    def apply(self, img):
        out = img.copy()
        o = self.opts

        if o['gaussian_blur']['on']:
            sig = max(0.1, float(o['gaussian_blur']['sigma']))
            ksize = int(2 * round(3*sig) + 1)
            out = cv2.GaussianBlur(out, (ksize, ksize), sig)

        if o['bilateral']['on']:
            out = cv2.bilateralFilter(
                out, int(o['bilateral']['d']),
                float(o['bilateral']['sigma_color']),
                float(o['bilateral']['sigma_space']))

        if o['brightness']['on']:
            out = cv2.convertScaleAbs(out,
                                      alpha=float(o['brightness']['alpha']),
                                      beta=float(o['brightness']['beta']))

        if o['gamma']['on']:
            g = max(0.05, float(o['gamma']['value']))
            lut = np.array([((i / 255.0) ** (1.0/g)) * 255
                            for i in range(256)]).astype(np.uint8)
            out = cv2.LUT(out, lut)

        if o['clahe']['on']:
            yuv = cv2.cvtColor(out, cv2.COLOR_BGR2YUV)
            clahe = cv2.createCLAHE(
                clipLimit=float(o['clahe']['clip']),
                tileGridSize=(int(o['clahe']['tile']), int(o['clahe']['tile'])))
            yuv[:, :, 0] = clahe.apply(yuv[:, :, 0])
            out = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

        if o['unsharp']['on']:
            blurred = cv2.GaussianBlur(out, (0, 0), float(o['unsharp']['radius']))
            out = cv2.addWeighted(out, 1.0 + float(o['unsharp']['amount']),
                                  blurred, -float(o['unsharp']['amount']), 0)

        if o['adaptive_thresh']['on']:
            gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
            block = int(o['adaptive_thresh']['block']) | 1  # impair
            block = max(3, block)
            th = cv2.adaptiveThreshold(gray, 255,
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY,
                                       block, int(o['adaptive_thresh']['C']))
            out = cv2.cvtColor(th, cv2.COLOR_GRAY2BGR)

        if o['grayscale']['on']:
            gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
            out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        return out


# ---------------------------------------------------------------------------
# Rendu
# ---------------------------------------------------------------------------
def draw_annotated(frame, tags, state):
    vis = frame.copy()
    for tid, info in tags.items():
        c4 = info['corners'].astype(int)
        cx, cy = info['center'].astype(int)
        if tid in TAGS_FIXED:
            color = (200, 200, 200); label = f"FIX {tid}"
        elif tid in TAGS_ROBOT:
            color = TAGS_ROBOT[tid][1]; label = f"{TAGS_ROBOT[tid][0]} ({tid})"
        else:
            color = (100, 100, 100); label = f"id {tid}"
        cv2.polylines(vis, [c4.reshape(-1, 1, 2)], True, color, 2)
        cv2.circle(vis, (cx, cy), 4, color, -1)
        cv2.putText(vis, label, (cx+8, cy-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    # périmètre table reprojeté
    if state.H_pix2world is not None:
        try:
            H_w2p = np.linalg.inv(state.H_pix2world)
            per_w = []
            for x in np.linspace(0, TABLE_WIDTH_MM, 30): per_w.append([x, 0])
            for y in np.linspace(0, TABLE_DEPTH_MM, 20): per_w.append([TABLE_WIDTH_MM, y])
            for x in np.linspace(TABLE_WIDTH_MM, 0, 30): per_w.append([x, TABLE_DEPTH_MM])
            for y in np.linspace(TABLE_DEPTH_MM, 0, 20): per_w.append([0, y])
            per_w = np.array(per_w)
            per_h = np.column_stack([per_w, np.ones(len(per_w))])
            per_p = (H_w2p @ per_h.T).T
            per_p = (per_p[:, :2] / per_p[:, 2:3]).astype(np.int32)
            cv2.polylines(vis, [per_p], True, (0, 200, 0), 2)
        except Exception:
            pass
    return vis


def draw_bev(state, extended_y=True):
    """BEV de la table 3x2m. Si extended_y=True, on étend Y à -1500..+3500
    pour voir les robots stationnés hors-table."""
    bw = int(TABLE_WIDTH_MM * BEV_PX_PER_MM)
    if extended_y:
        y_min, y_max = -1500.0, TABLE_DEPTH_MM + 1500.0
    else:
        y_min, y_max = 0.0, TABLE_DEPTH_MM
    bh = int((y_max - y_min) * BEV_PX_PER_MM)
    canvas = np.full((bh, bw, 3), 25, np.uint8)

    def w2bev(xy):
        return (int(xy[0] * BEV_PX_PER_MM),
                int((y_max - xy[1]) * BEV_PX_PER_MM))

    # Quadrillage 100mm/500mm
    for x_mm in range(0, int(TABLE_WIDTH_MM)+1, 100):
        x = int(x_mm * BEV_PX_PER_MM)
        col = (50, 50, 50) if x_mm % 500 else (80, 80, 80)
        cv2.line(canvas, (x, 0), (x, bh), col, 1)
    for y_mm in range(int(y_min), int(y_max)+1, 100):
        y = int((y_max - y_mm) * BEV_PX_PER_MM)
        col = (50, 50, 50) if y_mm % 500 else (80, 80, 80)
        cv2.line(canvas, (0, y), (bw, y), col, 1)

    # Périmètre table
    p1 = w2bev((0, 0)); p2 = w2bev((TABLE_WIDTH_MM, TABLE_DEPTH_MM))
    cv2.rectangle(canvas, p1, p2, (0, 200, 0), 2)

    # Tags fixes
    for tid, (X, Y) in TAGS_FIXED.items():
        p = w2bev((X, Y))
        cv2.drawMarker(canvas, p, (200, 200, 200), cv2.MARKER_SQUARE, 12, 2)
        cv2.putText(canvas, f"{tid}", (p[0]+8, p[1]+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # Trails
    for tid, (label, color) in TAGS_ROBOT.items():
        trail = list(state.trails.get(tid, []))
        for i in range(1, len(trail)):
            a = w2bev(trail[i-1]); b = w2bev(trail[i])
            alpha = i / len(trail)
            col_a = tuple(int(c*alpha) for c in color)
            cv2.line(canvas, a, b, col_a, 2)

    # Position robots
    for tid, (label, color) in TAGS_ROBOT.items():
        pos = state.last_positions.get(tid)
        if pos is None: continue
        p = w2bev(pos)
        cv2.circle(canvas, p, 6, color, -1)
        cv2.circle(canvas, p, 12, color, 2)
        cv2.putText(canvas, f"{label} ({pos[0]:.0f},{pos[1]:.0f})",
                    (p[0]+15, p[1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    # Caméra projetée
    if state.cam_pos is not None:
        Xc, Yc, Zc = state.cam_pos
        p = w2bev((Xc, Yc))
        # Toujours dessiner même si hors zone visible : clamp
        if 0 <= p[0] < bw and 0 <= p[1] < bh:
            cv2.drawMarker(canvas, p, (0, 255, 255),
                           cv2.MARKER_TILTED_CROSS, 16, 2)
            cv2.putText(canvas, f"CAM(Z={Zc:.0f})",
                        (p[0]+10, p[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        else:
            cv2.putText(canvas, f"CAM at ({Xc:.0f},{Yc:.0f},{Zc:.0f}) [hors-vue]",
                        (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    cv2.putText(canvas, "BEV (mm)  | Y croissant vers le haut",
                (5, bh-8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    return canvas
