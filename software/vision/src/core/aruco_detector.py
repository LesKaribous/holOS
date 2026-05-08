"""
Détection de marqueurs ArUco.
Encapsule l'API cv2.aruco (OpenCV 4.7+).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np


# OpenCV ≥ 4.7 introduces cv2.aruco.ArucoDetector + cv2.aruco.DetectorParameters().
# Older builds (incl. the NVIDIA Jetson cv2 4.5.x) only expose the legacy
# functional API: cv2.aruco.detectMarkers + DetectorParameters_create().
_HAS_NEW_ARUCO_API = hasattr(cv2.aruco, 'ArucoDetector')


# Dictionnaires disponibles pour le sélecteur UI
ARUCO_DICTS: dict[str, int] = {
    "4x4_50":    cv2.aruco.DICT_4X4_50,
    "4x4_100":   cv2.aruco.DICT_4X4_100,
    "4x4_250":   cv2.aruco.DICT_4X4_250,
    "4x4_1000":  cv2.aruco.DICT_4X4_1000,
    "5x5_50":    cv2.aruco.DICT_5X5_50,
    "5x5_100":   cv2.aruco.DICT_5X5_100,
    "5x5_250":   cv2.aruco.DICT_5X5_250,
    "6x6_50":    cv2.aruco.DICT_6X6_50,
    "6x6_100":   cv2.aruco.DICT_6X6_100,
    "APRILTAG_16h5":  cv2.aruco.DICT_APRILTAG_16h5,
    "APRILTAG_25h9":  cv2.aruco.DICT_APRILTAG_25h9,
    "APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


@dataclass
class DetectionResult:
    corners: list[np.ndarray]   # liste de (4, 1, 2) float32
    ids: list[int]              # liste d'IDs correspondants
    rejected: list[np.ndarray]  # candidats rejetés (debug)

    @property
    def count(self) -> int:
        return len(self.ids)

    def get_corners_for_id(self, marker_id: int) -> Optional[np.ndarray]:
        """Retourne les 4 coins (shape (4,2)) d'un tag par son ID, ou None."""
        for c, mid in zip(self.corners, self.ids):
            if mid == marker_id:
                return c.reshape(4, 2)
        return None

    def get_center_for_id(self, marker_id: int) -> Optional[tuple[float, float]]:
        pts = self.get_corners_for_id(marker_id)
        if pts is None:
            return None
        cx, cy = pts.mean(axis=0)
        return float(cx), float(cy)


COLORS = {
    "id_text":   (255, 255, 255),
    "border":    (0, 255, 0),
    "center":    (0, 0, 255),
    "rejected":  (128, 128, 128),
}


class ArucoDetector:
    def __init__(self, dict_name: str = "4x4_50", refine: str = "subpix"):
        """refine in {"none", "subpix", "contour", "apriltag"} -- default subpix
        is a good tradeoff for accuracy on blurred markers."""
        self._dict_name = dict_name
        self._refine = refine
        self._detector: Optional[cv2.aruco.ArucoDetector] = None
        self._build_detector(dict_name)

    @property
    def refine(self) -> str:
        return self._refine

    def set_refine(self, refine: str):
        if refine != self._refine:
            self._refine = refine
            self._build_detector(self._dict_name)

    def _build_detector(self, dict_name: str):
        dict_id = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
        # Dictionary getter — getPredefinedDictionary exists on most builds;
        # the very old API (pre-3.4) only had Dictionary_get.
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        else:
            aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        # Parameters constructor differs between APIs.
        if _HAS_NEW_ARUCO_API:
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()
        # Sub-pixel corner refinement: better accuracy, useful for blurred markers
        # SUBPIX is fast; CONTOUR is most accurate; APRILTAG_REFINE is best for
        # APRILTAG dictionaries.
        if self._refine == "subpix":
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        elif self._refine == "contour":
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        elif self._refine == "apriltag":
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        else:
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        # Slightly more permissive thresholds for low-contrast / blurred frames
        params.adaptiveThreshConstant = 7
        params.minMarkerPerimeterRate = 0.02
        params.polygonalApproxAccuracyRate = 0.05
        # Stash dict + params on the legacy path so detect() can call the
        # module-level cv2.aruco.detectMarkers function.
        self._aruco_dict = aruco_dict
        self._aruco_params = params
        if _HAS_NEW_ARUCO_API:
            self._detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        else:
            self._detector = None
        self._dict_name = dict_name

    def set_dictionary(self, dict_name: str):
        if dict_name != self._dict_name:
            self._build_detector(dict_name)

    @property
    def dictionary_name(self) -> str:
        return self._dict_name

    def detect(self, frame: np.ndarray) -> DetectionResult:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, rejected = self._detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params)

        flat_ids: list[int] = []
        flat_corners: list[np.ndarray] = []
        if ids is not None:
            for c, mid in zip(corners, ids.flatten()):
                flat_corners.append(c)
                flat_ids.append(int(mid))

        return DetectionResult(
            corners=flat_corners,
            ids=flat_ids,
            rejected=list(rejected) if rejected else [],
        )

    def draw(
        self,
        frame: np.ndarray,
        result: DetectionResult,
        draw_ids: bool = True,
        draw_rejected: bool = False,
        draw_axes: bool = False,
        highlight_ids: Optional[list[int]] = None,
    ) -> np.ndarray:
        out = frame.copy()

        # Tags rejetés (debug)
        if draw_rejected and result.rejected:
            for rc in result.rejected:
                pts = rc.reshape(4, 2).astype(int)
                for i in range(4):
                    cv2.line(out, tuple(pts[i]), tuple(pts[(i + 1) % 4]),
                             COLORS["rejected"], 1)

        # Tags détectés
        for corners_raw, marker_id in zip(result.corners, result.ids):
            pts = corners_raw.reshape(4, 2).astype(int)

            # Couleur spéciale si l'ID est mis en évidence
            color = (0, 200, 255) if (highlight_ids and marker_id in highlight_ids) else COLORS["border"]
            thickness = 3 if (highlight_ids and marker_id in highlight_ids) else 2

            for i in range(4):
                cv2.line(out, tuple(pts[i]), tuple(pts[(i + 1) % 4]), color, thickness)

            # Centre
            cx, cy = pts.mean(axis=0).astype(int)
            cv2.circle(out, (cx, cy), 4, COLORS["center"], -1)

            # ID
            if draw_ids:
                cv2.putText(
                    out, str(marker_id),
                    (pts[0][0], pts[0][1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLORS["id_text"], 2, cv2.LINE_AA
                )

        return out
