"""
Calibration table polynomiale (depuis image + mask rouge).

Approche :
  Une photo de la table couverte d'un masque rouge plein permet d'extraire le
  contour de la table dans l'image. À partir de ce contour et des 4 coins, on
  ajuste un modèle polynomial 2D (degré 3) qui mappe pixel → coordonnées table.

  RMS typique : ~10 mm (vs ~150 mm pour une homographie 4-coins sur fisheye).

Convention de coordonnées TwinVision :
    Origine = coin haut-gauche
    X horizontal : 0 → 3000 mm (largeur)
    Y vertical   : 0 → 2000 mm (profondeur)
    Z = 0        : plan du sol

(Note : le code original `experiments/table_calib.py` utilise une origine au
centre et travaille en mètres. Les fonctions de ce module convertissent
systématiquement vers la convention TwinVision (top-left, mm).)

Porté/adapté depuis experiments/table_calib.py.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


# Géométrie table (alignée avec core.table_rectifier)
TABLE_W_MM = 3000.0
TABLE_H_MM = 2000.0
DEFAULT_POLY_DEG = 3
DEFAULT_PX_PER_MM = 0.20    # BEV : 1 px = 5 mm


# ──────────────────────────────────────────────────────────────────────────────
# Extraction du contour de la table depuis le mask rouge
# ──────────────────────────────────────────────────────────────────────────────

def extract_red_mask_contour(
    mask_bgr: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Renvoie (mask binaire 0/255, contour Nx2 float64)."""
    B, G, R = cv2.split(mask_bgr)
    m = ((R > 150) & (G < 80) & (B < 80)).astype(np.uint8) * 255

    # Garder la plus grande composante connexe
    n, lbl, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if n > 1:
        idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        m = (lbl == idx).astype(np.uint8) * 255

    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise RuntimeError("Aucun contour rouge détecté dans le mask.")
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    return m, contour


def find_corners(
    contour: np.ndarray,
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    """
    Trouve les 4 coins du contour (TL, TR, BR, BL) comme les 4 maxima de
    distance au centroïde, puis classifie par angle.
    """
    from scipy.signal import find_peaks
    from scipy.ndimage import gaussian_filter1d

    cx, cy = contour.mean(0)
    ang = np.arctan2(contour[:, 1] - cy, contour[:, 0] - cx)
    order = np.argsort(ang)
    contour_s = contour[order]
    ang_s = ang[order]
    ds = gaussian_filter1d(
        np.hypot(contour_s[:, 0] - cx, contour_s[:, 1] - cy),
        sigma=8, mode="wrap",
    )
    ext = np.concatenate([ds, ds, ds])
    peaks, _ = find_peaks(ext, distance=len(ds) // 6)
    peaks = peaks[(peaks >= len(ds)) & (peaks < 2 * len(ds))] - len(ds)
    peaks = peaks[np.argsort(-ds[peaks])[:4]]
    peaks.sort()

    corners: dict[str, np.ndarray] = {}
    for p in peaks:
        a = np.degrees(ang_s[p])
        pt = contour_s[p]
        if   -180 <= a < -90: corners["TL"] = pt
        elif  -90 <= a < 0:   corners["TR"] = pt
        elif    0 <= a <  90: corners["BR"] = pt
        else:                 corners["BL"] = pt

    if len(corners) != 4:
        raise RuntimeError(f"Détection coins incomplète : {sorted(corners)}")
    return corners, contour_s


def _split_contour(
    contour_s: np.ndarray, corners: dict[str, np.ndarray],
) -> dict[str, np.ndarray]:
    """Sépare le contour ordonné en 4 arcs (un par côté)."""
    def fi(pt: np.ndarray) -> int:
        return int(np.argmin(np.linalg.norm(contour_s - pt, axis=1)))

    def arc(i1: int, i2: int) -> np.ndarray:
        if i1 <= i2:
            return contour_s[i1:i2 + 1]
        return np.vstack([contour_s[i1:], contour_s[:i2 + 1]])

    iTL = fi(corners["TL"]); iTR = fi(corners["TR"])
    iBR = fi(corners["BR"]); iBL = fi(corners["BL"])
    return {
        "TOP":    arc(iTL, iTR),
        "RIGHT":  arc(iTR, iBR),
        "BOTTOM": arc(iBR, iBL),
        "LEFT":   arc(iBL, iTL),
    }


def _build_correspondences(
    corners: dict[str, np.ndarray],
    sides: dict[str, np.ndarray],
    step: int = 4,
    corner_weight: int = 20,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Construit les correspondances pixel ↔ monde (mm) en paramétrant
    linéairement chaque côté.

    Convention TwinVision : TL = (0,0), TR = (TABLE_W,0),
                            BR = (TABLE_W,TABLE_H), BL = (0,TABLE_H)
    """
    TL = np.array([0.0,         0.0])
    TR = np.array([TABLE_W_MM,  0.0])
    BR = np.array([TABLE_W_MM,  TABLE_H_MM])
    BL = np.array([0.0,         TABLE_H_MM])

    pairs = [
        (sides["TOP"],    corners["TL"], corners["TR"], TL, TR),
        (sides["RIGHT"],  corners["TR"], corners["BR"], TR, BR),
        (sides["BOTTOM"], corners["BR"], corners["BL"], BR, BL),
        (sides["LEFT"],   corners["BL"], corners["TL"], BL, TL),
    ]

    img_pts: list[np.ndarray] = []
    world_pts: list[np.ndarray] = []
    for arr, c1p, c2p, c1m, c2m in pairs:
        sub = arr[::step]
        v = c2p - c1p
        if v @ v < 1e-9:
            continue
        t = np.clip(((sub - c1p) @ v) / (v @ v), 0, 1)
        for i, ti in enumerate(t):
            img_pts.append(sub[i])
            world_pts.append(c1m + ti * (c2m - c1m))

    # Coins répétés pour leur donner un poids fort
    for _ in range(corner_weight):
        img_pts.extend([corners["TL"], corners["TR"],
                        corners["BR"], corners["BL"]])
        world_pts.extend([TL, TR, BR, BL])

    return (
        np.array(img_pts, dtype=np.float64),
        np.array(world_pts, dtype=np.float64),
    )


# ──────────────────────────────────────────────────────────────────────────────
# Modèles : homographie + polynôme 2D
# ──────────────────────────────────────────────────────────────────────────────

def fit_homography_corners(corners: dict[str, np.ndarray]) -> np.ndarray:
    """Homographie 4-coins pixel → table (mm), convention TwinVision."""
    src = np.array([corners["TL"], corners["TR"],
                    corners["BR"], corners["BL"]], dtype=np.float32)
    dst = np.array([
        [0.0,         0.0],
        [TABLE_W_MM,  0.0],
        [TABLE_W_MM,  TABLE_H_MM],
        [0.0,         TABLE_H_MM],
    ], dtype=np.float32)
    return cv2.getPerspectiveTransform(src, dst)


def _poly_design(u: np.ndarray, v: np.ndarray, deg: int) -> np.ndarray:
    """Matrice de design polynomiale 2D (jusqu'au degré `deg`)."""
    return np.column_stack([
        (u ** i) * (v ** j)
        for i in range(deg + 1)
        for j in range(deg + 1 - i)
    ])


@dataclass
class PolyCalibration:
    """Modèle polynomial pixel → table (mm)."""
    deg: int
    coef_x: np.ndarray            # coefs pour X (mm)
    coef_y: np.ndarray            # coefs pour Y (mm)
    u_mean: float
    v_mean: float
    u_std: float
    v_std: float
    image_size: tuple[int, int]   # (W, H)
    rms_mm: float = 0.0
    max_mm: float = 0.0
    median_mm: float = 0.0
    homography: Optional[np.ndarray] = None      # H pixel→mm
    homography_rms_mm: float = 0.0
    homography_max_mm: float = 0.0
    corners_pixel: dict[str, list[float]] = field(default_factory=dict)
    calibrated_at: str = ""
    note: str = ""
    # Camera intrinsics extracted from the polynomial -- optional.
    # When present, downstream code can use solvePnP per frame from 4 ArUco
    # anchors instead of the polynomial. Useful when only 4 anchors are
    # available live (no red mask).
    K: Optional[np.ndarray] = None              # 3x3
    dist: Optional[np.ndarray] = None           # (1, n) coefficients
    intrinsics_rms_px: float = 0.0

    # ── Application ───────────────────────────────────────────────────────────

    # ---- Live correction (small homography mm_pred -> mm_true) ----------
    # Refitted from 4 detected anchor tags each frame to absorb camera drift
    # without redoing the full polynomial calibration.

    def __post_init__(self):
        self._correction_H = None       # type: Optional[np.ndarray]
        self._correction_H_inv = None   # type: Optional[np.ndarray]
        self._correction_residuals_mm = []  # type: list

    def update_live_correction(self, mm_pred, mm_true):
        """Fit an mm-space homography from mm_predicted (poly raw output)
        to mm_true (known anchor positions). Pass arrays of shape (4, 2).
        Returns True if fit succeeded."""
        mm_pred = np.asarray(mm_pred, dtype=np.float32)
        mm_true = np.asarray(mm_true, dtype=np.float32)
        if mm_pred.shape != (4, 2) or mm_true.shape != (4, 2):
            return False
        try:
            H = cv2.getPerspectiveTransform(mm_pred, mm_true)
        except cv2.error:
            return False
        if H is None:
            return False
        try:
            H_inv = np.linalg.inv(H.astype(np.float64))
        except np.linalg.LinAlgError:
            return False
        self._correction_H = H.astype(np.float64)
        self._correction_H_inv = H_inv
        # Residuals after correction (should be near zero at the 4 fit points)
        h_pts = np.column_stack([mm_pred.astype(np.float64), np.ones(4)])
        out = (self._correction_H @ h_pts.T).T
        corr_pred = out[:, :2] / out[:, 2:3]
        self._correction_residuals_mm = list(np.linalg.norm(
            corr_pred - mm_true.astype(np.float64), axis=1
        ))
        # IMPORTANT : ne PAS invalider _remap_cache.
        # La rectification graphique reste sur le polynome statique (rapide,
        # cache permanent). La correction live n'agit que sur les conversions
        # pixel<->mm ponctuelles (tracking, hover) -- cheap homography.
        return True

    def clear_live_correction(self):
        self._correction_H = None
        self._correction_H_inv = None
        self._correction_residuals_mm = []
        self._remap_cache = None

    def refine_with_anchors(
        self,
        anchor_pixels: np.ndarray,
        anchor_mm: np.ndarray,
        weight: int = 200,
        sample_step: int = 8,
    ):
        """Refine the polynomial to ALSO fit a set of detected ArUco anchors
        whose mm positions are known. The current polynomial is preserved
        away from the anchors (via dense self-sampling) and pulled to fit the
        anchors exactly (via high replication weight).

        Practical use case: the mask-only calibration fits the table BORDERS
        perfectly but is under-constrained inside. Detecting the 4 anchor
        ArUco at known mm positions provides interior constraints. This
        refit combines both : borders stay good, anchors get pulled to their
        known positions.

        Args:
            anchor_pixels : (N, 2) detected pixel centers of the anchors
            anchor_mm     : (N, 2) corresponding known world positions in mm
            weight        : how many times each anchor is replicated in the
                            least-squares system (200 by default = strong)
            sample_step   : pixel step to sample the existing polynomial

        Returns:
            (rms_mm_at_anchors, max_mm_at_anchors) : residuals at the 4 anchors
            after the refit. Should be near zero with weight=200.
        """
        anchor_pixels = np.asarray(anchor_pixels, dtype=np.float64)
        anchor_mm = np.asarray(anchor_mm, dtype=np.float64)
        if anchor_pixels.shape != anchor_mm.shape:
            raise ValueError("anchor_pixels and anchor_mm must match shape")
        if anchor_pixels.ndim != 2 or anchor_pixels.shape[1] != 2:
            raise ValueError("anchor arrays must be (N, 2)")

        w, h = self.image_size
        if w <= 0 or h <= 0:
            raise RuntimeError("image_size missing.")

        # 1. Build the "background" correspondences from the EXISTING polynomial.
        # These preserve the border fit obtained from the red mask.
        yy, xx = np.mgrid[0:h:sample_step, 0:w:sample_step]
        bg_pix = np.column_stack([xx.ravel(), yy.ravel()]).astype(np.float64)
        bg_mm = self.pixel_to_mm_raw(bg_pix)

        margin_w = TABLE_W_MM * 0.30
        margin_h = TABLE_H_MM * 0.30
        keep = (
            (bg_mm[:, 0] > -margin_w) & (bg_mm[:, 0] < TABLE_W_MM + margin_w) &
            (bg_mm[:, 1] > -margin_h) & (bg_mm[:, 1] < TABLE_H_MM + margin_h)
        )
        bg_pix = bg_pix[keep]
        bg_mm = bg_mm[keep]

        # 2. Add the anchor correspondences with replication for high weight.
        all_pix = np.vstack([bg_pix] + [anchor_pixels] * weight)
        all_mm  = np.vstack([bg_mm]  + [anchor_mm]     * weight)

        # 3. Refit the polynomial.
        u = (all_pix[:, 0] - self.u_mean) / self.u_std
        v = (all_pix[:, 1] - self.v_mean) / self.v_std
        F = _poly_design(u, v, self.deg)
        coef_x, _, _, _ = np.linalg.lstsq(F, all_mm[:, 0], rcond=None)
        coef_y, _, _, _ = np.linalg.lstsq(F, all_mm[:, 1], rcond=None)

        self.coef_x = coef_x
        self.coef_y = coef_y

        # Invalidate caches (poly changed)
        self.invalidate_caches()

        # Compute residuals at the anchor points after the refit
        anchor_pred = self.pixel_to_mm_raw(anchor_pixels)
        diff = anchor_pred - anchor_mm
        rms = float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))
        mx = float(np.max(np.linalg.norm(diff, axis=1)))
        return rms, mx

    def bake_correction(self):
        """Absorb the current live correction (affine part) into the polynomial
        coefficients. After calling this, the polynomial outputs mm_TRUE
        directly and the live correction is reset to identity.

        Useful to fix orientation/translation mismatches once and for all,
        without paying the runtime cost of applying the correction every
        frame in build_rectif_maps. The rectified view is then permanently
        in user-world space.

        Note: only the affine part (rotation + translation) of the homography
        is baked. The perspective component (rare with table-plane anchors)
        is dropped. After baking, residuals will be near zero for true affine
        corrections (rotation + translation, no perspective) -- as is the
        case for a 180-degree rotation between coordinate systems."""
        if self._correction_H is None:
            return False
        H = self._correction_H
        if abs(H[2, 2]) < 1e-9:
            return False
        # Extract 2x3 affine approximation : new = A * (X, Y, 1)
        A = H[:2, :3] / H[2, 2]
        a, b, tx = float(A[0, 0]), float(A[0, 1]), float(A[0, 2])
        c, d, ty = float(A[1, 0]), float(A[1, 1]), float(A[1, 2])
        # Apply to polynomial coefficients :
        #   new_X(u,v) = a * X(u,v) + b * Y(u,v) + tx
        #   new_Y(u,v) = c * X(u,v) + d * Y(u,v) + ty
        new_cx = a * self.coef_x + b * self.coef_y
        new_cx[0] += tx           # constant term (poly_design index 0)
        new_cy = c * self.coef_x + d * self.coef_y
        new_cy[0] += ty
        self.coef_x = new_cx
        self.coef_y = new_cy
        # Clear correction + caches
        self.clear_live_correction()
        self.invalidate_caches()
        return True

    @property
    def has_correction(self):
        return self._correction_H is not None

    @property
    def correction_residuals_mm(self):
        return list(self._correction_residuals_mm)

    @staticmethod
    def _apply_h2d(H, pts):
        pts = np.atleast_2d(np.asarray(pts, dtype=np.float64))
        h = np.column_stack([pts, np.ones(len(pts))])
        out = (H @ h.T).T
        return out[:, :2] / out[:, 2:3]

    # ---- Forward / inverse with optional live correction ----------------

    def pixel_to_mm_raw(self, uv):
        """Forward poly without live correction."""
        uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
        u = (uv[:, 0] - self.u_mean) / self.u_std
        v = (uv[:, 1] - self.v_mean) / self.v_std
        F = _poly_design(u, v, self.deg)
        return np.column_stack([F @ self.coef_x, F @ self.coef_y])

    def pixel_to_mm(self, uv):
        """uv (N,2) en pixels -> (N,2) mm (table TwinVision).
        Applique la correction live si presente."""
        mm = self.pixel_to_mm_raw(uv)
        if self._correction_H is not None:
            mm = self._apply_h2d(self._correction_H, mm)
        return mm

    # ── Inverse mm → pixel (polynôme inverse, fitté à la demande) ────────────

    _INV_X_MEAN: float = TABLE_W_MM / 2.0
    _INV_X_STD:  float = TABLE_W_MM / 2.0
    _INV_Y_MEAN: float = TABLE_H_MM / 2.0
    _INV_Y_STD:  float = TABLE_H_MM / 2.0

    def _ensure_inverse(self):
        """Fit le polynôme inverse (mm → pixel) la première fois qu'on en a besoin."""
        if getattr(self, "_inv_coef_u", None) is not None:
            return
        w, h = self.image_size
        if w <= 0 or h <= 0:
            raise RuntimeError("image_size non défini, polynôme inverse impossible.")

        # Sample dense de l'image, conserver les points qui retombent dans la table
        step = max(1, min(w, h) // 240)   # ≈ 240 échantillons par axe min
        yy, xx = np.mgrid[0:h:step, 0:w:step]
        img = np.column_stack([xx.ravel(), yy.ravel()]).astype(np.float64)
        mm = self.pixel_to_mm(img)
        margin = 200.0
        keep = (
            (mm[:, 0] > -margin) & (mm[:, 0] < TABLE_W_MM + margin) &
            (mm[:, 1] > -margin) & (mm[:, 1] < TABLE_H_MM + margin)
        )
        if keep.sum() < 64:
            raise RuntimeError("Pas assez d'échantillons pour fitter l'inverse.")
        mm = mm[keep]; img = img[keep]

        xn = (mm[:, 0] - self._INV_X_MEAN) / self._INV_X_STD
        yn = (mm[:, 1] - self._INV_Y_MEAN) / self._INV_Y_STD
        F = _poly_design(xn, yn, self.deg)
        u, _, _, _ = np.linalg.lstsq(F, img[:, 0], rcond=None)
        v, _, _, _ = np.linalg.lstsq(F, img[:, 1], rcond=None)
        self._inv_coef_u = u
        self._inv_coef_v = v

    def mm_to_pixel_raw(self, mm):
        """Inverse poly without live correction."""
        self._ensure_inverse()
        mm = np.atleast_2d(np.asarray(mm, dtype=np.float64))
        xn = (mm[:, 0] - self._INV_X_MEAN) / self._INV_X_STD
        yn = (mm[:, 1] - self._INV_Y_MEAN) / self._INV_Y_STD
        F = _poly_design(xn, yn, self.deg)
        return np.column_stack([F @ self._inv_coef_u, F @ self._inv_coef_v])

    def mm_to_pixel(self, mm):
        """(N,2) mm -> (N,2) pixel. Applique la correction inverse si presente."""
        if self._correction_H_inv is not None:
            mm = self._apply_h2d(self._correction_H_inv, mm)
        return self.mm_to_pixel_raw(mm)

    # ── Maps de rectification (cv2.remap) ─────────────────────────────────────

    def build_rectif_maps(
        self, out_w: int, out_h: int, mm_per_px: float, sample_step: int = 4,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Build remap maps for cv2.remap producing a top-down BEV in mm_PRED
        coordinates (image-aligned, the polynomial's natural orientation).
        Output pixel (px, py) corresponds to mm_PRED (px*mm_per_px, py*mm_per_px).

        The live correction (mm_pred -> mm_user) is NOT applied to the rendered
        image. It is applied only to point-wise lookups (hover, robot drawing,
        anchor crosses) so that those reflect user-world labels even though the
        image is image-aligned. This keeps the cache permanent (no rebuild) and
        the rectified view right-side-up matching the camera's view direction.
        """
        from scipy.interpolate import griddata

        cache_key = (out_w, out_h, float(mm_per_px), self.image_size)
        cache = getattr(self, "_remap_cache", None)
        if cache is not None and cache[0] == cache_key:
            return cache[1], cache[2]

        w, h = self.image_size
        yy, xx = np.mgrid[0:h:sample_step, 0:w:sample_step]
        img_grid = np.column_stack([xx.ravel(), yy.ravel()]).astype(np.float64)
        # world_grid : sample image pixels mapped to mm_PRED via raw poly
        world_grid = self.pixel_to_mm_raw(img_grid)

        margin_w = TABLE_W_MM * 0.20
        margin_h = TABLE_H_MM * 0.20
        keep = (
            (world_grid[:, 0] > -margin_w) & (world_grid[:, 0] < TABLE_W_MM + margin_w) &
            (world_grid[:, 1] > -margin_h) & (world_grid[:, 1] < TABLE_H_MM + margin_h)
        )
        world_grid = world_grid[keep]
        img_grid = img_grid[keep]

        # Target grid in mm_PRED (image-aligned). No correction applied.
        bev_uu, bev_vv = np.meshgrid(np.arange(out_w), np.arange(out_h))
        target = np.column_stack([
            (bev_uu * mm_per_px).ravel(),
            (bev_vv * mm_per_px).ravel(),
        ])

        img_uv = griddata(world_grid, img_grid, target, method="linear")
        img_uv = img_uv.reshape(out_h, out_w, 2)
        mx = img_uv[..., 0].astype(np.float32)
        my = img_uv[..., 1].astype(np.float32)
        nan_mask = np.isnan(mx) | np.isnan(my)
        mx[nan_mask] = -1
        my[nan_mask] = -1

        self._remap_cache = (cache_key, mx, my)
        return mx, my

    def invalidate_caches(self):
        self._remap_cache = None
        self._inv_coef_u = None
        self._inv_coef_v = None
        self._perimeter_cache = None
        self._world_grid_cache = None

    # ---- Extract camera intrinsics (K, dist) from the polynomial ----------
    #
    # Use case: during competition the red mask is not available, but the
    # camera distortion is fixed. We fit a pinhole+distortion model that
    # approximates the polynomial (~14 px RMS achievable with a single
    # planar view -- worse than the polynomial itself but good enough for
    # solvePnP-based per-frame pose estimation).

    def fit_intrinsics(self, sample_step: int = 15):
        """Fit pinhole + distortion intrinsics that approximate the
        polynomial. Single-view planar fit; uses the OpenCV TILTED + RATIONAL
        + THIN_PRISM model for max flexibility on fisheye lenses.
        Stores K, dist on self and returns (K, dist, rms_px)."""
        w, h = self.image_size
        if w <= 0 or h <= 0:
            raise RuntimeError("image_size missing.")

        yy, xx = np.mgrid[10:h - 10:sample_step, 10:w - 10:sample_step]
        img_pts = np.column_stack([xx.ravel(), yy.ravel()]).astype(np.float64)
        mm_pts = self.pixel_to_mm_raw(img_pts)
        keep = (
            (mm_pts[:, 0] > -50) & (mm_pts[:, 0] < TABLE_W_MM + 50) &
            (mm_pts[:, 1] > -50) & (mm_pts[:, 1] < TABLE_H_MM + 50)
        )
        img_pts = img_pts[keep].astype(np.float32)
        mm_pts = mm_pts[keep].astype(np.float32)
        if len(img_pts) < 100:
            raise RuntimeError("Not enough sample points to fit intrinsics.")
        obj_pts = np.column_stack(
            [mm_pts, np.zeros(len(mm_pts))]
        ).astype(np.float32)

        K_init = np.array([
            [w * 0.7, 0,       w / 2.0],
            [0,       w * 0.7, h / 2.0],
            [0,       0,       1.0    ],
        ], dtype=np.float64)
        flags = (
            cv2.CALIB_USE_INTRINSIC_GUESS
            | cv2.CALIB_FIX_PRINCIPAL_POINT
            | cv2.CALIB_RATIONAL_MODEL
            | cv2.CALIB_THIN_PRISM_MODEL
            | cv2.CALIB_TILTED_MODEL
        )
        rms, K, dist, _, _ = cv2.calibrateCamera(
            [obj_pts], [img_pts], (w, h), K_init, None, flags=flags,
        )

        self.K = K.astype(np.float64)
        self.dist = dist.astype(np.float64)
        self.intrinsics_rms_px = float(rms)
        return self.K, self.dist, float(rms)

    def compute_world_grid_cached(self, image_shape):
        """Cache the (X_mm, Y_mm) grid for the calibration's image size.
        These are constant -- depend only on the static polynomial."""
        cache = getattr(self, "_world_grid_cache", None)
        if cache is not None and cache[0] == tuple(image_shape[:2]):
            return cache[1], cache[2]
        X, Y = self.world_grid(image_shape)
        self._world_grid_cache = (tuple(image_shape[:2]), X, Y)
        return X, Y

    def compute_perimeter_pixels(self, image_shape, thickness_mm: float = 15.0):
        """Return a boolean mask of pixels lying on the table perimeter (in
        world mm), as the wizard's '05_overlay_grid' does : rasterized via
        the forward poly. Follows the fisheye curve exactly.
        Cached on (image_shape, thickness_mm)."""
        key = (tuple(image_shape[:2]), float(thickness_mm))
        cache = getattr(self, "_perimeter_cache", None)
        if cache is not None and cache[0] == key:
            return cache[1]
        X, Y = self.compute_world_grid_cached(image_shape)
        # Inside : keep pixels whose mm is inside the table (with a small slack)
        slack = thickness_mm * 1.5
        inside = (
            (X > -slack) & (X < TABLE_W_MM + slack) &
            (Y > -slack) & (Y < TABLE_H_MM + slack)
        )
        # Perimeter : pixels whose X is near 0 or TABLE_W_MM, or Y near 0 or H
        perim = (
            (np.abs(X - 0.0)        < thickness_mm) |
            (np.abs(X - TABLE_W_MM) < thickness_mm) |
            (np.abs(Y - 0.0)        < thickness_mm) |
            (np.abs(Y - TABLE_H_MM) < thickness_mm)
        ) & inside
        self._perimeter_cache = (key, perim)
        return perim

    # ── Anciennes API (compat) ────────────────────────────────────────────────

    def pixel_to_mm_homography(self, uv: np.ndarray) -> Optional[np.ndarray]:
        if self.homography is None:
            return None
        uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
        h_pts = np.column_stack([uv, np.ones(len(uv))])
        out = (self.homography @ h_pts.T).T
        return out[:, :2] / out[:, 2:3]

    def world_grid(self, image_shape: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
        """X(u,v), Y(u,v) en mm sur tous les pixels de l'image."""
        h_, w_ = image_shape[:2]
        yy, xx = np.mgrid[0:h_, 0:w_]
        u = (xx - self.u_mean) / self.u_std
        v = (yy - self.v_mean) / self.v_std
        X = np.zeros_like(u, dtype=np.float64)
        Y = np.zeros_like(u, dtype=np.float64)
        k = 0
        for i in range(self.deg + 1):
            for j in range(self.deg + 1 - i):
                term = (u ** i) * (v ** j)
                X += self.coef_x[k] * term
                Y += self.coef_y[k] * term
                k += 1
        return X, Y

    # ── Sérialisation ─────────────────────────────────────────────────────────

    def to_dict(self) -> dict:
        return {
            "type": "poly_table_calibration",
            "convention": "twinvision_topleft_mm",
            "table_size_mm": [TABLE_W_MM, TABLE_H_MM],
            "image_size": list(self.image_size),
            "calibrated_at": self.calibrated_at,
            "note": self.note,
            "poly_model": {
                "deg": self.deg,
                "coef_x": self.coef_x.tolist(),
                "coef_y": self.coef_y.tolist(),
                "normalization": {
                    "u_mean": self.u_mean, "v_mean": self.v_mean,
                    "u_std":  self.u_std,  "v_std":  self.v_std,
                },
            },
            "homography_pixel_to_mm": (
                self.homography.tolist() if self.homography is not None else None
            ),
            "corners_pixel": self.corners_pixel,
            "residuals_mm": {
                "poly_rms":       self.rms_mm,
                "poly_max":       self.max_mm,
                "poly_median":    self.median_mm,
                "homography_rms": self.homography_rms_mm,
                "homography_max": self.homography_max_mm,
            },
            "intrinsics": {
                "K":    self.K.tolist()    if self.K    is not None else None,
                "dist": self.dist.tolist() if self.dist is not None else None,
                "rms_px": self.intrinsics_rms_px,
            } if (self.K is not None) else None,
        }

    def save(self, path: str | Path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def from_dict(cls, d: dict) -> "PolyCalibration":
        if d.get("type") == "poly_table_calibration" \
           or d.get("convention") == "twinvision_topleft_mm":
            # Format TwinVision natif (mm)
            pm = d["poly_model"]
            n = pm["normalization"]
            H = d.get("homography_pixel_to_mm")
            r = d.get("residuals_mm", {})
            obj = cls(
                deg=int(pm["deg"]),
                coef_x=np.array(pm["coef_x"], dtype=np.float64),
                coef_y=np.array(pm["coef_y"], dtype=np.float64),
                u_mean=float(n["u_mean"]), v_mean=float(n["v_mean"]),
                u_std=float(n["u_std"]),   v_std=float(n["v_std"]),
                image_size=tuple(d["image_size"]),
                rms_mm=float(r.get("poly_rms", 0.0)),
                max_mm=float(r.get("poly_max", 0.0)),
                median_mm=float(r.get("poly_median", 0.0)),
                homography=(np.array(H, dtype=np.float64) if H else None),
                homography_rms_mm=float(r.get("homography_rms", 0.0)),
                homography_max_mm=float(r.get("homography_max", 0.0)),
                corners_pixel=d.get("corners_pixel", {}),
                calibrated_at=d.get("calibrated_at", ""),
                note=d.get("note", ""),
            )
            intr = d.get("intrinsics")
            if intr and intr.get("K") is not None:
                obj.K = np.array(intr["K"], dtype=np.float64)
                if intr.get("dist") is not None:
                    obj.dist = np.array(intr["dist"], dtype=np.float64)
                obj.intrinsics_rms_px = float(intr.get("rms_px", 0.0))
            return obj

        # Compat : ancien format experiments/table_calib.py (mètres, origine centre)
        # → convertir en mm + top-left.
        return cls._from_legacy_dict(d)

    @classmethod
    def _from_legacy_dict(cls, d: dict) -> "PolyCalibration":
        """
        Convertit l'ancien format (origine centre, mètres) vers la convention
        TwinVision (origine TL, mm).

        Ancien : pixel → (X_legacy, Y_legacy) en mètres avec
                 X_legacy ∈ [-1.5, +1.5], Y_legacy ∈ [-1, +1] (sens y inversé)
        Nouveau : pixel → (X_mm, Y_mm) avec
                 X_mm = (X_legacy + 1.5) * 1000
                 Y_mm = (1.0 - Y_legacy) * 1000
        Le polynôme est linéaire en ses coefs, donc on peut mettre à l'échelle
        et translater coefficient par coefficient (translation : seul le terme
        constant change ; mise à l'échelle : tous les coefs).
        """
        pm = d["poly_model"]
        n = pm["normalization"]
        deg = int(pm["deg"])
        coef_x_m = np.array(pm["coef_x"], dtype=np.float64)
        coef_y_m = np.array(pm["coef_y"], dtype=np.float64)

        # mm = m * 1000 (X), mm = (1.0 - m) * 1000 = 1000 - m*1000 (Y)
        # Trouver l'index du coefficient constant (i=0, j=0)
        # _poly_design ordonne : pour i=0 → j=0..deg ; pour i=1 → j=0..deg-1 ; ...
        # Donc le constant (i=0,j=0) est l'index 0.
        coef_x_mm = coef_x_m * 1000.0
        coef_x_mm[0] += 1500.0  # +1.5 m → +1500 mm

        coef_y_mm = -coef_y_m * 1000.0
        coef_y_mm[0] += 1000.0  # 1.0 m → 1000 mm

        # Homographie : reconstruit ou recalculé depuis les coins
        H_legacy = d.get("homography_pixel_to_world")
        H_mm: Optional[np.ndarray] = None
        if H_legacy is not None:
            # H_legacy : pixel → m (centre)
            # On post-multiplie par S (m → mm top-left) :
            #     S = [[1000, 0, 1500], [0, -1000, 1000], [0, 0, 1]]
            S = np.array([
                [1000.0,    0.0, 1500.0],
                [   0.0, -1000.0, 1000.0],
                [   0.0,    0.0,    1.0],
            ], dtype=np.float64)
            H_mm = S @ np.array(H_legacy, dtype=np.float64)

        corners_pix = d.get("corners_pixel", {})
        # Note : dans l'ancien format, TL est en haut-gauche dans l'IMAGE pixel
        # (mêmes coins que TwinVision), donc on peut garder tel quel.

        r = d.get("residuals_mm", {})

        return cls(
            deg=deg,
            coef_x=coef_x_mm, coef_y=coef_y_mm,
            u_mean=float(n["u_mean"]), v_mean=float(n["v_mean"]),
            u_std=float(n["u_std"]),   v_std=float(n["v_std"]),
            image_size=tuple(d.get("image_size", [0, 0])),
            rms_mm=float(r.get("poly_rms", 0.0)),
            max_mm=float(r.get("poly_max", 0.0)),
            median_mm=float(r.get("poly_median", 0.0)),
            homography=H_mm,
            homography_rms_mm=float(r.get("homography_rms", 0.0)),
            homography_max_mm=float(r.get("homography_max", 0.0)),
            corners_pixel=corners_pix,
            calibrated_at=d.get("calibrated_at", ""),
            note=d.get("note", "") + " [converti depuis legacy]",
        )

    @classmethod
    def load(cls, path: str | Path) -> "PolyCalibration":
        with open(path, "r") as f:
            return cls.from_dict(json.load(f))


def calibrate_from_mask(image, mask_bgr, deg: int = DEFAULT_POLY_DEG, note: str = ""):
    """Run the full polynomial calibration from a table image + red mask."""
    if mask_bgr.shape[:2] != image.shape[:2]:
        mask_bgr = cv2.resize(mask_bgr, (image.shape[1], image.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
    h, w = image.shape[:2]
    import time
    mask, contour = extract_red_mask_contour(mask_bgr)
    corners, contour_s = find_corners(contour)
    sides = _split_contour(contour_s, corners)
    img_pts, world_pts = _build_correspondences(corners, sides)
    H = fit_homography_corners(corners)
    u_mean, v_mean = w / 2.0, h / 2.0
    u_std,  v_std  = w / 2.0, h / 2.0
    u = (img_pts[:, 0] - u_mean) / u_std
    v = (img_pts[:, 1] - v_mean) / v_std
    F = _poly_design(u, v, deg)
    coef_x, _, _, _ = np.linalg.lstsq(F, world_pts[:, 0], rcond=None)
    coef_y, _, _, _ = np.linalg.lstsq(F, world_pts[:, 1], rcond=None)
    pred_x = F @ coef_x; pred_y = F @ coef_y
    err_p = np.linalg.norm(np.column_stack([pred_x, pred_y]) - world_pts, axis=1)
    h_pts = np.column_stack([img_pts, np.ones(len(img_pts))])
    out_h = (H @ h_pts.T).T
    pred_h = out_h[:, :2] / out_h[:, 2:3]
    err_h = np.linalg.norm(pred_h - world_pts, axis=1)
    cal = PolyCalibration(
        deg=deg, coef_x=coef_x, coef_y=coef_y,
        u_mean=u_mean, v_mean=v_mean, u_std=u_std, v_std=v_std,
        image_size=(w, h),
        rms_mm=float(np.sqrt(np.mean(err_p ** 2))),
        max_mm=float(err_p.max()),
        median_mm=float(np.median(err_p)),
        homography=H,
        homography_rms_mm=float(np.sqrt(np.mean(err_h ** 2))),
        homography_max_mm=float(err_h.max()),
        corners_pixel={k: [float(v_[0]), float(v_[1])] for k, v_ in corners.items()},
        calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
        note=note or "Calibration polynomiale TwinVision (mask rouge)",
    )
    vis_c = image.copy()
    cv2.drawContours(vis_c, [contour.astype(np.int32).reshape(-1, 1, 2)],
                     -1, (0, 255, 255), 2)
    for k_, p_ in corners.items():
        cv2.circle(vis_c, (int(p_[0]), int(p_[1])), 14, (0, 0, 255), 3)
        cv2.putText(vis_c, k_, (int(p_[0]) + 18, int(p_[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return cal, {"corners_vis": vis_c, "mask": mask, "corners": corners}


def make_bev(image, cal, px_per_mm: float = DEFAULT_PX_PER_MM, sample_step: int = 4):
    bev_w = int(TABLE_W_MM * px_per_mm)
    bev_h = int(TABLE_H_MM * px_per_mm)
    mm_per_px = 1.0 / px_per_mm
    mx, my = cal.build_rectif_maps(bev_w, bev_h, mm_per_px, sample_step)
    return cv2.remap(image, mx, my, cv2.INTER_LINEAR, borderValue=(0, 0, 0))
