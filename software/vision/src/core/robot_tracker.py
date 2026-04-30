"""
RobotTracker — suivi de tags ArUco "robots" en coordonnées table (mm) avec
correction de parallaxe pour les tags sur-élevés.

Convention table TwinVision (cohérente avec le reste du code) :
    Origine = coin haut-gauche
    X horizontal : 0 → 3000 mm (largeur)
    Y vertical   : 0 → 2000 mm (profondeur)
    Z = 0        : plan du sol

Les robots peuvent stationner hors-table (Y > 2000 ou Y < 0).

Dépendances :
    - core.aruco_detector.DetectionResult : la détection est faite ailleurs.
    - core.table_rectifier.TableRectifier : fournit l'homographie pixel→mm.

Le tracker prend en entrée le résultat de la détection + l'état de la
rectification, et produit pour chaque tag-robot configuré :
    - sa position « naïve » au sol (projection directe via H)
    - sa position « corrigée » de la parallaxe si la position caméra est connue
    - une trace (deque) des dernières positions corrigées

La position caméra peut être :
    - estimée automatiquement par solvePnP si une matrice K est fournie
      (typiquement issue d'une CalibrationData ChArUco)
    - forcée manuellement par l'utilisateur (override)
    - déduite par inversion de parallaxe à partir de la position connue d'un
      robot visible (« calibrate-from-known-position »)

Porté/adapté depuis experiments/robot_tracker.py.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from core.aruco_detector import DetectionResult
from core.table_rectifier import TableRectifier, TABLE_W_MM, TABLE_H_MM

# ---------------------------------------------------------------------------
# Kalman 2D constant-velocity tracker (per robot)
# ---------------------------------------------------------------------------
# State    : [x, y, vx, vy]      (mm, mm, mm/s, mm/s)
# Measure  : [x, y]               (mm, mm)
#
# Used to smooth detections under motion blur and to predict the position
# when the tag is not detected on the current frame. A future hook accepts
# embedded odometry as an extra observation (see set_odometry()).
# ---------------------------------------------------------------------------

class Kalman2D:
    """Wrapper around cv2.KalmanFilter for a 2D constant-velocity model."""

    def __init__(self,
                 process_noise: float = 50.0,    # mm of model uncertainty per frame
                 meas_noise:    float = 10.0):    # mm of measurement noise
        self._kf = cv2.KalmanFilter(4, 2)
        self._kf.transitionMatrix = np.eye(4, dtype=np.float32)  # F (set in predict)
        self._kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)
        self._kf.processNoiseCov = np.eye(4, dtype=np.float32) * (process_noise ** 2)
        self._kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * (meas_noise ** 2)
        self._kf.errorCovPost = np.eye(4, dtype=np.float32) * 1e3
        self._kf.statePost = np.zeros((4, 1), dtype=np.float32)
        self._initialized = False
        # Future hook : odometry observation [vx, vy] in mm/s. Adds a 2nd
        # measurement matrix when fused.
        self._odom_v: Optional[np.ndarray] = None
        self._odom_noise: float = 50.0   # mm/s

    @property
    def initialized(self) -> bool:
        return self._initialized

    def predict(self, dt: float = 1.0 / 30) -> np.ndarray:
        """Advance the state by dt seconds. Returns predicted [x, y] (mm)."""
        F = np.eye(4, dtype=np.float32)
        F[0, 2] = dt
        F[1, 3] = dt
        self._kf.transitionMatrix = F
        s = self._kf.predict()
        return np.array([s[0, 0], s[1, 0]], dtype=np.float64)

    def update(self, z_xy: np.ndarray) -> np.ndarray:
        """Correct with measurement [x, y] (mm). Returns the corrected [x, y]."""
        z = np.array([[float(z_xy[0])], [float(z_xy[1])]], dtype=np.float32)
        if not self._initialized:
            self._kf.statePost = np.array(
                [[z[0, 0]], [z[1, 0]], [0.0], [0.0]], dtype=np.float32,
            )
            self._initialized = True
            return np.array([z[0, 0], z[1, 0]], dtype=np.float64)
        s = self._kf.correct(z)
        return np.array([s[0, 0], s[1, 0]], dtype=np.float64)

    def set_odometry(self, vx: Optional[float], vy: Optional[float],
                     noise_mm_per_s: Optional[float] = None):
        """Provide embedded odometry velocity as an extra observation. Pass
        None to disable. (Used in the next correct/predict step.)"""
        if vx is None or vy is None:
            self._odom_v = None
        else:
            self._odom_v = np.array([float(vx), float(vy)], dtype=np.float32)
        if noise_mm_per_s is not None:
            self._odom_noise = float(noise_mm_per_s)

    def reset(self):
        self._initialized = False
        self._kf.statePost = np.zeros((4, 1), dtype=np.float32)
        self._kf.errorCovPost = np.eye(4, dtype=np.float32) * 1e3

    def state(self) -> np.ndarray:
        return self._kf.statePost.flatten().astype(np.float64)



# ──────────────────────────────────────────────────────────────────────────────
# Configuration robots
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class RobotConfig:
    """Configuration d'un tag-robot suivi."""
    tag_id: int
    label: str = "ROBOT"
    color_bgr: tuple[int, int, int] = (0, 255, 0)
    z_mm: float = 530.0   # hauteur du tag au-dessus du sol (Z = 0)

    def to_dict(self) -> dict:
        return {
            "tag_id":   self.tag_id,
            "label":    self.label,
            "color_bgr": list(self.color_bgr),
            "z_mm":     self.z_mm,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "RobotConfig":
        c = d.get("color_bgr", [0, 255, 0])
        return cls(
            tag_id=int(d["tag_id"]),
            label=str(d.get("label", "ROBOT")),
            color_bgr=(int(c[0]), int(c[1]), int(c[2])),
            z_mm=float(d.get("z_mm", 530.0)),
        )


# ──────────────────────────────────────────────────────────────────────────────
# Géométrie : parallaxe
# ──────────────────────────────────────────────────────────────────────────────

def correct_parallax(
    pos_naive: np.ndarray,
    cam_xyz: np.ndarray,
    z_object: float,
) -> np.ndarray:
    """
    Étant donné la position « naïve » d'un tag projeté sur Z=0 (homographie
    sol→sol), retourne la position réelle au sol sous le tag, en compensant
    la parallaxe due à sa hauteur z_object > 0.

        real = camera_xy + factor * (naive - camera_xy)
        avec factor = (Zc - z_object) / Zc
    """
    Xc, Yc, Zc = cam_xyz
    if Zc <= z_object + 1e-3:
        return np.asarray(pos_naive, dtype=np.float64).copy()
    factor = (Zc - z_object) / Zc
    cam_xy = np.array([Xc, Yc], dtype=np.float64)
    return cam_xy + factor * (np.asarray(pos_naive, dtype=np.float64) - cam_xy)


def solve_camera_xy_from_known_position(
    naive_xy: np.ndarray,
    real_xy: tuple[float, float] | np.ndarray,
    Zc: float,
    z_object: float,
) -> Optional[np.ndarray]:
    """
    Inversion de la parallaxe : si on connaît la VRAIE position d'un robot
    `real_xy` et qu'on observe sa projection naïve `naive_xy` au sol depuis
    une caméra à hauteur Zc, on déduit (Xc, Yc, Zc) de la caméra.

        camera_xy = (real - factor*naive) / (1 - factor)
        avec factor = (Zc - z_object) / Zc

    Retourne None si Zc ≈ z_object (parallaxe non-résoluble).
    """
    if Zc <= z_object + 1e-3:
        return None
    factor = (Zc - z_object) / Zc
    if abs(1 - factor) < 1e-9:
        return None
    real = np.asarray(real_xy, dtype=np.float64)
    naive = np.asarray(naive_xy, dtype=np.float64)
    cam_xy = (real - factor * naive) / (1 - factor)
    return np.array([cam_xy[0], cam_xy[1], Zc], dtype=np.float64)


def estimate_camera_position_pnp(
    fixed_centers_pix: np.ndarray,
    fixed_world_mm: np.ndarray,
    image_size: tuple[int, int],
    K: Optional[np.ndarray] = None,
    dist: Optional[np.ndarray] = None,
    fx_estimate: Optional[float] = None,
) -> tuple[Optional[np.ndarray], Optional[float], str]:
    """
    Estime la position caméra (Xc, Yc, Zc) en mm via solvePnP.

    Args:
        fixed_centers_pix : (N, 2) centres pixel des tags fixes
        fixed_world_mm    : (N, 2) coords table mm des tags fixes
        image_size        : (W, H)
        K                 : matrice intrinsèque 3x3 si dispo
        dist              : coefficients de distortion si dispo
        fx_estimate       : fallback fx en pixels si K non fourni

    Returns:
        (cam_xyz, fx_used, source)
        source ∈ {"K", "fx_estimate", "fail"}
    """
    if len(fixed_centers_pix) < 4 or len(fixed_world_mm) < 4:
        return None, None, "fail"

    obj = np.array(
        [[p[0], p[1], 0.0] for p in fixed_world_mm],
        dtype=np.float32,
    )
    img = np.asarray(fixed_centers_pix, dtype=np.float32)
    w, h = image_size

    if K is not None:
        K_used = K.astype(np.float64)
        D_used = dist if dist is not None else np.zeros(5)
        source = "K"
        fx_used = float(K_used[0, 0])
    else:
        fx = float(fx_estimate) if fx_estimate is not None else w * 0.7
        K_used = np.array([
            [fx, 0,  w / 2.0],
            [0,  fx, h / 2.0],
            [0,  0,  1],
        ], dtype=np.float64)
        D_used = np.zeros(5)
        source = "fx_estimate"
        fx_used = fx

    try:
        ok, rvec, tvec = cv2.solvePnP(
            obj, img, K_used, D_used,
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error:
        return None, None, "fail"

    if not ok:
        return None, None, "fail"

    R, _ = cv2.Rodrigues(rvec)
    cam = (-R.T @ tvec).ravel()
    return cam.astype(np.float64), fx_used, source


# ──────────────────────────────────────────────────────────────────────────────
# Tracker principal
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class RobotState:
    """État courant d'un robot pour un frame."""
    tag_id: int
    label: str
    color_bgr: tuple[int, int, int]
    pos_mm: np.ndarray             # position corrigée parallaxe (X, Y) en mm
    naive_mm: np.ndarray           # position « naïve » (X, Y) en mm
    pixel_center: tuple[float, float]
    # Orientation du robot dans le repère table (mm). Convention
    # trigonometrique : 0 rad = direction +X, sens trigo positif.
    # NaN si la marker n'est pas detectee a ce frame (eg. Kalman predict only).
    theta_rad: float = float("nan")


class RobotTracker:
    """
    Pipeline de tracking robots. Doit être appelé une fois par frame, après la
    détection ArUco et la mise à jour du TableRectifier.

    Mode caméra :
        cam_mode = "auto"     → estimation par solvePnP (utilise K si dispo)
        cam_mode = "manual"   → utilisateur fixe (cam_override)

    En mode "auto" sans K caméra, l'estimation est très approximative (utilise
    fx_estimate comme heuristique). Pour de la précision, utiliser le mode
    manuel ou la calibration-from-known-position.
    """

    def __init__(
        self,
        robots: Optional[list[RobotConfig]] = None,
        cam_mode: str = "manual",
        cam_override: Optional[tuple[float, float, float]] = None,
        fx_estimate: Optional[float] = None,
        smooth_calib: bool = True,
        trail_len: int = 80,
    ):
        self._robots: list[RobotConfig] = list(robots) if robots else []
        self._cam_mode = cam_mode
        self._cam_override = (
            np.array(cam_override, dtype=np.float64) if cam_override else None
        )
        self.fx_estimate = fx_estimate
        self.smooth_calib = smooth_calib
        self._trail_len = trail_len

        self._trails: dict[int, deque] = {
            r.tag_id: deque(maxlen=trail_len) for r in self._robots
        }
        self._last_states: dict[int, RobotState] = {}
        # Per-robot Kalman 2D (constant velocity) for smoothing + prediction
        # under motion blur / occlusion.
        self._kalmans: dict[int, Kalman2D] = {
            r.tag_id: Kalman2D() for r in self._robots
        }
        self._use_kalman: bool = True
        self._dt_s: float = 1.0 / 30.0   # default frame interval

        # Position caméra
        self._cam_pos_auto: Optional[np.ndarray] = None
        self._cam_pos: Optional[np.ndarray] = None
        self._cam_source: str = "none"   # "none" | "manual" | "auto_K" | "auto_fx"

        # Cache pour le solvePnP : matrice K/dist optionnelle
        self._K: Optional[np.ndarray] = None
        self._dist: Optional[np.ndarray] = None

    # ── Configuration ─────────────────────────────────────────────────────────

    @property
    def robots(self) -> list[RobotConfig]:
        return self._robots

    def set_robots(self, robots: list[RobotConfig]):
        self._robots = list(robots)
        # Re-create trails / kalmans, keep existing ones for IDs still present
        new_trails: dict[int, deque] = {}
        new_kalmans: dict[int, Kalman2D] = {}
        for r in self._robots:
            new_trails[r.tag_id] = self._trails.get(
                r.tag_id, deque(maxlen=self._trail_len)
            )
            new_trails[r.tag_id] = deque(new_trails[r.tag_id], maxlen=self._trail_len)
            new_kalmans[r.tag_id] = self._kalmans.get(r.tag_id, Kalman2D())
        self._trails = new_trails
        self._kalmans = new_kalmans

    @property
    def use_kalman(self) -> bool:
        return self._use_kalman

    def set_use_kalman(self, on: bool):
        self._use_kalman = bool(on)
        if not on:
            for k in self._kalmans.values():
                k.reset()

    def set_dt(self, dt_s: float):
        """Set the inter-frame interval in seconds for Kalman prediction."""
        self._dt_s = max(1e-3, float(dt_s))

    def get_kalman(self, tag_id: int) -> Optional[Kalman2D]:
        """Access the underlying Kalman filter for a robot. Use to inject
        embedded odometry via set_odometry()."""
        return self._kalmans.get(tag_id)

    @property
    def cam_mode(self) -> str:
        return self._cam_mode

    def set_cam_mode(self, mode: str):
        if mode in ("auto", "manual"):
            self._cam_mode = mode

    @property
    def cam_override(self) -> Optional[np.ndarray]:
        return self._cam_override

    def set_cam_override(self, xyz: Optional[tuple[float, float, float]]):
        self._cam_override = (
            np.array(xyz, dtype=np.float64) if xyz is not None else None
        )

    @property
    def cam_pos(self) -> Optional[np.ndarray]:
        return self._cam_pos

    @property
    def cam_pos_auto(self) -> Optional[np.ndarray]:
        return self._cam_pos_auto

    @property
    def cam_source(self) -> str:
        return self._cam_source

    def set_camera_intrinsics(
        self, K: Optional[np.ndarray], dist: Optional[np.ndarray] = None,
    ):
        """Fournir la matrice K (et optionnellement les coefficients de distortion)
        pour améliorer l'estimation auto solvePnP."""
        self._K = K.copy() if K is not None else None
        self._dist = dist.copy() if dist is not None else None

    # ── Trails ────────────────────────────────────────────────────────────────

    def trails(self, tag_id: int) -> list[tuple[float, float]]:
        d = self._trails.get(tag_id)
        return list(d) if d else []

    def reset_trails(self):
        for d in self._trails.values():
            d.clear()
        # Also reset Kalman filters so they re-init from next detection
        for k in self._kalmans.values():
            k.reset()

    @property
    def last_states(self) -> dict[int, RobotState]:
        return self._last_states

    def get_robot_config(self, tag_id: int) -> Optional[RobotConfig]:
        for r in self._robots:
            if r.tag_id == tag_id:
                return r
        return None

    # ── Process ───────────────────────────────────────────────────────────────

    def update_camera(
        self,
        result: DetectionResult,
        rectifier: TableRectifier,
        image_size: tuple[int, int],
    ):
        """Met à jour `cam_pos` selon le mode courant + résultat solvePnP."""
        # 1. Estimation auto via solvePnP sur les 4 tags d'ancrage du rectifier
        anchors = rectifier.config.anchors()
        centers_pix: list[list[float]] = []
        world_mm: list[list[float]] = []
        for a in anchors:
            c = result.get_center_for_id(a.tag_id)
            if c is None:
                self._cam_pos_auto = None
                break
            centers_pix.append(list(c))
            world_mm.append([a.x_mm, a.y_mm])
        else:
            cam_xyz, _fx, src = estimate_camera_position_pnp(
                np.asarray(centers_pix),
                np.asarray(world_mm),
                image_size,
                K=self._K, dist=self._dist,
                fx_estimate=self.fx_estimate,
            )
            self._cam_pos_auto = cam_xyz

        # 2. Sélection du cam_pos actif
        if self._cam_mode == "manual" and self._cam_override is not None:
            self._cam_pos = self._cam_override.copy()
            self._cam_source = "manual"
        elif self._cam_pos_auto is not None:
            self._cam_pos = self._cam_pos_auto.copy()
            self._cam_source = "auto_K" if self._K is not None else "auto_fx"
        else:
            # Fallback : si pas d'auto et override dispo, on l'utilise quand même
            if self._cam_override is not None:
                self._cam_pos = self._cam_override.copy()
                self._cam_source = "manual"
            else:
                self._cam_pos = None
                self._cam_source = "none"

    def process(
        self,
        result: DetectionResult,
        rectifier: TableRectifier,
        image_size: tuple[int, int],
    ) -> dict[int, RobotState]:
        """
        Met à jour les positions des robots configurés à partir du résultat
        de détection et du rectifier (qui fournit pixel→mm via H).

        Doit être appelé APRÈS rectifier.update(result).
        """
        self.update_camera(result, rectifier, image_size)
        self._last_states.clear()

        if not rectifier.has_homography:
            return self._last_states

        for cfg in self._robots:
            kf = self._kalmans.get(cfg.tag_id)
            center = result.get_center_for_id(cfg.tag_id)

            # Step 1 : Kalman predict (advances state regardless of detection)
            if self._use_kalman and kf is not None:
                kf_pred = kf.predict(self._dt_s)
            else:
                kf_pred = None

            if center is None:
                # No detection : if Kalman is initialized, use prediction
                if (self._use_kalman and kf is not None and kf.initialized
                        and kf_pred is not None):
                    pos = kf_pred
                    state = RobotState(
                        tag_id=cfg.tag_id,
                        label=cfg.label,
                        color_bgr=cfg.color_bgr,
                        pos_mm=pos,
                        naive_mm=pos.copy(),
                        pixel_center=(float("nan"), float("nan")),
                    )
                    self._last_states[cfg.tag_id] = state
                    self._trails[cfg.tag_id].append((float(pos[0]), float(pos[1])))
                continue

            naive_mm = rectifier.raw_pixel_to_table_mm(
                int(round(center[0])), int(round(center[1]))
            )
            if naive_mm is None:
                continue
            naive_arr = np.array(naive_mm, dtype=np.float64)

            if self._cam_pos is not None and cfg.z_mm > 0.0:
                pos_meas = correct_parallax(naive_arr, self._cam_pos, cfg.z_mm)
            else:
                pos_meas = naive_arr.copy()

            # Step 2 : Kalman update with the measurement
            if self._use_kalman and kf is not None:
                pos = kf.update(pos_meas)
            else:
                pos = pos_meas

            # Orientation : direction du vecteur corner[0] -> corner[1]
            # exprime dans le repere table (mm). Le marker etant plat et a
            # hauteur constante, la correction de parallaxe est la meme pour
            # ses 4 coins -> elle s'annule dans la difference, donc on peut
            # calculer theta directement depuis raw_pixel_to_table_mm.
            theta = float("nan")
            corners = result.get_corners_for_id(cfg.tag_id)
            if corners is not None:
                pts = corners.reshape(4, 2)
                p0 = rectifier.raw_pixel_to_table_mm(
                    int(round(pts[0, 0])), int(round(pts[0, 1])),
                )
                p1 = rectifier.raw_pixel_to_table_mm(
                    int(round(pts[1, 0])), int(round(pts[1, 1])),
                )
                if p0 is not None and p1 is not None:
                    import math
                    dx = float(p1[0] - p0[0])
                    dy = float(p1[1] - p0[1])
                    if dx != 0.0 or dy != 0.0:
                        theta = math.atan2(dy, dx)
            state = RobotState(
                tag_id=cfg.tag_id,
                label=cfg.label,
                color_bgr=cfg.color_bgr,
                pos_mm=pos,
                naive_mm=naive_arr,
                pixel_center=(float(center[0]), float(center[1])),
                theta_rad=theta,
            )
            self._last_states[cfg.tag_id] = state
            self._trails[cfg.tag_id].append((float(pos[0]), float(pos[1])))

        return self._last_states

    # === Serialisation =======================================================

    def to_dict(self) -> dict:
        return {
            "robots": [r.to_dict() for r in self._robots],
            "cam_mode": self._cam_mode,
            "cam_override": (
                self._cam_override.tolist() if self._cam_override is not None else None
            ),
            "fx_estimate": self.fx_estimate,
        }

    def load_dict(self, d: dict):
        if "robots" in d and isinstance(d["robots"], list):
            self.set_robots([RobotConfig.from_dict(r) for r in d["robots"]])
        self._cam_mode = d.get("cam_mode", self._cam_mode)
        if "cam_override" in d:
            v = d["cam_override"]
            self._cam_override = (
                np.array(v, dtype=np.float64) if v is not None else None
            )
        if "fx_estimate" in d:
            self.fx_estimate = d["fx_estimate"]
