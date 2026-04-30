"""
Gestion de la calibration caméra.

Deux modèles supportés :
  - "standard"  : modèle polynomial OpenCV classique (k1,k2,p1,p2,k3)
                  adapté aux objectifs normaux / légèrement grand-angle.
  - "fisheye"   : modèle équidistant cv2.fisheye (k1,k2,k3,k4)
                  adapté aux objectifs grand-angle / fisheye (FOV > 120°).

Deux méthodes de calibration :
  - ChessboardCalibrator : échiquier classique.
  - CharucoCalibrator    : échiquier + marqueurs ArUco imbriqués (meilleur
                           pour grands angles car robuste aux vues partielles).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


# ──────────────────────────────────────────────────────────────────────────────
# CalibrationData
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class CalibrationData:
    camera_matrix: np.ndarray       # (3, 3)
    dist_coeffs: np.ndarray         # standard: (1,5) | fisheye: (4,1)
    rms_error: float = 0.0
    image_size: tuple[int, int] = (0, 0)   # (width, height)
    calibrated_at: str = ""
    source_name: str = ""
    model: str = "standard"        # "standard" | "fisheye"

    # Maps de remap (calculées à la demande, non sérialisées)
    _map1: Optional[np.ndarray] = field(default=None, repr=False, compare=False)
    _map2: Optional[np.ndarray] = field(default=None, repr=False, compare=False)
    _map_size: tuple[int, int] = field(default=(0, 0), repr=False, compare=False)

    def _ensure_maps(self, w: int, h: int):
        if self._map1 is not None and self._map_size == (w, h):
            return

        if self.model == "fisheye":
            K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self.camera_matrix, self.dist_coeffs, (w, h),
                R=np.eye(3), balance=0.0,
            )
            self._map1, self._map2 = cv2.fisheye.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs,
                R=np.eye(3), P=K_new, size=(w, h), m1type=cv2.CV_16SC2,
            )
        else:
            K_new, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), alpha=0.0,
            )
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs,
                R=None, newCameraMatrix=K_new, size=(w, h), m1type=cv2.CV_16SC2,
            )

        self._map_size = (w, h)

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        self._ensure_maps(w, h)
        return cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)

    @property
    def model_label(self) -> str:
        return "Fisheye" if self.model == "fisheye" else "Standard"

    def to_dict(self) -> dict:
        return {
            "model": self.model,
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "rms_error": self.rms_error,
            "image_size": list(self.image_size),
            "calibrated_at": self.calibrated_at,
            "source_name": self.source_name,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CalibrationData":
        return cls(
            model=d.get("model", "standard"),
            camera_matrix=np.array(d["camera_matrix"], dtype=np.float64),
            dist_coeffs=np.array(d["dist_coeffs"], dtype=np.float64),
            rms_error=d.get("rms_error", 0.0),
            image_size=tuple(d.get("image_size", [0, 0])),
            calibrated_at=d.get("calibrated_at", ""),
            source_name=d.get("source_name", ""),
        )

    def save(self, path: str | Path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path: str | Path) -> "CalibrationData":
        with open(path, "r") as f:
            return cls.from_dict(json.load(f))


# ──────────────────────────────────────────────────────────────────────────────
# Calibrateurs
# ──────────────────────────────────────────────────────────────────────────────

class _BaseCalibrator:
    """Interface commune."""

    @property
    def sample_count(self) -> int:
        raise NotImplementedError

    def detect(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        """Détecte le pattern dans frame, retourne (trouvé, frame_annoté).
        Ne stocke PAS les points (utile pour le preview live)."""
        raise NotImplementedError

    def add_frame(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        """Comme detect() mais stocke les points si trouvés."""
        raise NotImplementedError

    def calibrate(self) -> Optional[CalibrationData]:
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError


# ── Échiquier ─────────────────────────────────────────────────────────────────

class ChessboardCalibrator(_BaseCalibrator):
    """
    Calibration par échiquier classique.
    Supporte les modèles standard et fisheye.
    """

    def __init__(
        self,
        cols: int = 9,
        rows: int = 6,
        square_mm: float = 25.0,
        model: str = "standard",
    ):
        self.cols = cols
        self.rows = rows
        self.square_mm = square_mm
        self.model = model

        self._obj_pts = np.zeros((cols * rows, 3), np.float32)
        self._obj_pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm

        self._image_points: list[np.ndarray] = []
        self._image_size: tuple[int, int] = (0, 0)

    @property
    def sample_count(self) -> int:
        return len(self._image_points)

    def _find_corners(self, frame: np.ndarray) -> tuple[bool, Optional[np.ndarray]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        self._image_size = (w, h)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), flags)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return found, corners

    def detect(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        found, corners = self._find_corners(frame)
        preview = frame.copy()
        cv2.drawChessboardCorners(preview, (self.cols, self.rows), corners, found)
        return found, preview

    def add_frame(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        found, corners = self._find_corners(frame)
        preview = frame.copy()
        cv2.drawChessboardCorners(preview, (self.cols, self.rows), corners, found)
        if found:
            self._image_points.append(corners)
        return found, preview

    def calibrate(self) -> Optional[CalibrationData]:
        n = len(self._image_points)
        if n < 5:
            return None

        obj_points = [self._obj_pts] * n

        if self.model == "fisheye":
            return self._calibrate_fisheye(obj_points)
        else:
            return self._calibrate_standard(obj_points)

    def _calibrate_standard(self, obj_points) -> Optional[CalibrationData]:
        rms, K, D, _, _ = cv2.calibrateCamera(
            obj_points, self._image_points, self._image_size, None, None,
        )
        return CalibrationData(
            camera_matrix=K, dist_coeffs=D,
            rms_error=float(rms), image_size=self._image_size,
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            model="standard",
        )

    def _calibrate_fisheye(self, obj_points) -> Optional[CalibrationData]:
        # Le modèle fisheye exige des tableaux (N,1,3) et (N,1,2)
        obj3d = [p.reshape(-1, 1, 3).astype(np.float64) for p in obj_points]
        img2d = [p.reshape(-1, 1, 2).astype(np.float64) for p in self._image_points]
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
                 cv2.fisheye.CALIB_FIX_SKEW)
        try:
            rms, K, D, _, _ = cv2.fisheye.calibrate(
                obj3d, img2d, self._image_size, K, D,
                flags=flags,
            )
        except cv2.error as e:
            raise RuntimeError(f"Calibration fisheye échouée : {e}") from e

        return CalibrationData(
            camera_matrix=K, dist_coeffs=D,
            rms_error=float(rms), image_size=self._image_size,
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            model="fisheye",
        )

    def reset(self):
        self._image_points.clear()


# ── ChArUco ───────────────────────────────────────────────────────────────────

class CharucoCalibrator(_BaseCalibrator):
    """
    Calibration par board ChArUco (échiquier + marqueurs ArUco imbriqués).

    Avantages vs échiquier classique :
      - Robuste aux vues partielles (utile pour fisheye).
      - Sous-pixel accuracy via les coins d'échiquier + robustesse des tags ArUco.
      - Fonctionne même si une partie du board est hors-champ.

    Paramètres :
      cols, rows   : nombre de CASES (pas de coins) du board.
      square_mm    : taille d'une case en mm.
      marker_mm    : taille du marqueur ArUco dans chaque case (< square_mm).
                     Typiquement 60-75% de square_mm.
      dict_name    : dictionnaire ArUco à utiliser.
      model        : "standard" ou "fisheye".
    """

    DICTS = {
        "4x4_100":  cv2.aruco.DICT_4X4_100,
        "4x4_250":  cv2.aruco.DICT_4X4_250,
        "5x5_100":  cv2.aruco.DICT_5X5_100,
        "6x6_250":  cv2.aruco.DICT_6X6_250,
    }

    def __init__(
        self,
        cols: int = 10,
        rows: int = 7,
        square_mm: float = 30.0,
        marker_mm: float = 22.0,
        dict_name: str = "5x5_100",
        model: str = "standard",
    ):
        self.cols = cols
        self.rows = rows
        self.square_mm = square_mm
        self.marker_mm = marker_mm
        self.dict_name = dict_name
        self.model = model

        self._build_board()

        self._all_corners: list[np.ndarray] = []
        self._all_ids:     list[np.ndarray] = []
        self._image_size:  tuple[int, int] = (0, 0)

    def _build_board(self):
        dict_id = self.DICTS.get(self.dict_name, cv2.aruco.DICT_5X5_100)
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._board = cv2.aruco.CharucoBoard(
            (self.cols, self.rows),
            self.square_mm,
            self.marker_mm,
            self._aruco_dict,
        )
        self._detector = cv2.aruco.CharucoDetector(self._board)

    def reconfigure(
        self,
        cols: int, rows: int,
        square_mm: float, marker_mm: float,
        dict_name: str, model: str,
    ):
        self.cols = cols; self.rows = rows
        self.square_mm = square_mm; self.marker_mm = marker_mm
        self.dict_name = dict_name; self.model = model
        self._build_board()

    @property
    def sample_count(self) -> int:
        return len(self._all_corners)

    def _find(self, frame: np.ndarray) -> tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        self._image_size = (w, h)
        charuco_corners, charuco_ids, _, _ = self._detector.detectBoard(gray)
        found = (charuco_ids is not None and len(charuco_ids) >= 4)
        return found, charuco_corners, charuco_ids

    def detect(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        found, corners, ids = self._find(frame)
        return found, self._draw(frame, corners, ids, found)

    def add_frame(self, frame: np.ndarray) -> tuple[bool, np.ndarray]:
        found, corners, ids = self._find(frame)
        if found:
            self._all_corners.append(corners)
            self._all_ids.append(ids)
        return found, self._draw(frame, corners, ids, found)

    def _draw(self, frame, corners, ids, found) -> np.ndarray:
        out = frame.copy()
        if found and corners is not None:
            cv2.aruco.drawDetectedCornersCharuco(out, corners, ids)
        return out

    def generate_board_image(self, px_per_mm: float = 10.0) -> np.ndarray:
        """Génère une image du board pour impression."""
        w = int(self.cols * self.square_mm * px_per_mm)
        h = int(self.rows * self.square_mm * px_per_mm)
        img = self._board.generateImage((w, h), marginSize=20)
        return img

    def calibrate(self) -> Optional[CalibrationData]:
        n = len(self._all_corners)
        if n < 4:
            return None

        if self.model == "fisheye":
            return self._calibrate_fisheye()
        else:
            return self._calibrate_standard()

    def _calibrate_standard(self) -> Optional[CalibrationData]:
        try:
            rms, K, D, _, _ = cv2.aruco.calibrateCameraCharuco(
                self._all_corners, self._all_ids,
                self._board, self._image_size,
                None, None,
            )
        except cv2.error as e:
            raise RuntimeError(f"Calibration ChArUco standard échouée : {e}") from e

        return CalibrationData(
            camera_matrix=K, dist_coeffs=D,
            rms_error=float(rms), image_size=self._image_size,
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            model="standard",
        )

    def _calibrate_fisheye(self) -> Optional[CalibrationData]:
        """
        Pour le modèle fisheye, on extrait les objPts / imgPts à partir des
        détections ChArUco, puis on appelle cv2.fisheye.calibrate.
        """
        obj_pts_list = []
        img_pts_list = []

        for corners, ids in zip(self._all_corners, self._all_ids):
            obj_pts, img_pts = self._board.matchImagePoints(corners, ids)
            if obj_pts is None or len(obj_pts) < 4:
                continue
            # Fisheye attend (N,1,3) et (N,1,2)
            obj_pts_list.append(obj_pts.reshape(-1, 1, 3).astype(np.float64))
            img_pts_list.append(img_pts.reshape(-1, 1, 2).astype(np.float64))

        if len(obj_pts_list) < 4:
            raise RuntimeError("Pas assez de frames valides pour calibration fisheye.")

        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
                 cv2.fisheye.CALIB_FIX_SKEW)
        try:
            rms, K, D, _, _ = cv2.fisheye.calibrate(
                obj_pts_list, img_pts_list,
                self._image_size, K, D,
                flags=flags,
            )
        except cv2.error as e:
            raise RuntimeError(f"Calibration fisheye échouée : {e}") from e

        return CalibrationData(
            camera_matrix=K, dist_coeffs=D,
            rms_error=float(rms), image_size=self._image_size,
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            model="fisheye",
        )

    def reset(self):
        self._all_corners.clear()
        self._all_ids.clear()
