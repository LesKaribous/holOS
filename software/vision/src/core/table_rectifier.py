"""
Rectification de la table via homographie ou polynome.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

import cv2
import numpy as np

from core.aruco_detector import DetectionResult

if TYPE_CHECKING:
    from core.poly_calibration import PolyCalibration

TABLE_W_MM = 3000
TABLE_H_MM = 2000

OUTPUT_SCALE = 0.25

# Extra margin around the table in the rectified BEV (mm). The rectified
# view spans (-MARGIN, -MARGIN) to (TABLE_W+MARGIN, TABLE_H+MARGIN) so the
# user sees a small border around the table -- useful to visually confirm
# the rectif is well centered and to keep the anchors away from the edges.
MARGIN_MM = 100.0


@dataclass
class TagAnchor:
    """ArUco tag id paired with its known position on the table (mm)."""
    tag_id: int
    x_mm: float
    y_mm: float

    def to_dict(self) -> dict:
        return {"tag_id": self.tag_id, "x_mm": self.x_mm, "y_mm": self.y_mm}

    @classmethod
    def from_dict(cls, d: dict) -> "TagAnchor":
        return cls(
            tag_id=int(d["tag_id"]),
            x_mm=float(d["x_mm"]),
            y_mm=float(d["y_mm"]),
        )


@dataclass
class CornerConfig:
    # 4 ArUco anchors precisely placed at 600 mm from each corner of the
    # 3000 x 2000 mm table -- forming a 1800 x 800 inner rectangle.
    top_left:     TagAnchor = field(default_factory=lambda: TagAnchor(23,  600,  600))
    top_right:    TagAnchor = field(default_factory=lambda: TagAnchor(22, 2400,  600))
    bottom_right: TagAnchor = field(default_factory=lambda: TagAnchor(20, 2400, 1400))
    bottom_left:  TagAnchor = field(default_factory=lambda: TagAnchor(21,  600, 1400))

    def anchors(self) -> list[TagAnchor]:
        return [self.top_left, self.top_right, self.bottom_right, self.bottom_left]

    def to_dict(self) -> dict:
        return {
            "top_left":     self.top_left.to_dict(),
            "top_right":    self.top_right.to_dict(),
            "bottom_right": self.bottom_right.to_dict(),
            "bottom_left":  self.bottom_left.to_dict(),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CornerConfig":
        return cls(
            top_left=     TagAnchor.from_dict(d["top_left"]),
            top_right=    TagAnchor.from_dict(d["top_right"]),
            bottom_right= TagAnchor.from_dict(d["bottom_right"]),
            bottom_left=  TagAnchor.from_dict(d["bottom_left"]),
        )


class TableRectifier:
    def __init__(self, config: Optional[CornerConfig] = None, scale: float = OUTPUT_SCALE,
                 margin_mm: float = MARGIN_MM):
        self._config = config or CornerConfig()
        self._scale = scale
        self._margin_mm = float(margin_mm)
        self._H: Optional[np.ndarray] = None
        self._H_is_fresh: bool = False
        # Output spans table + 2*margin so the rectif shows a small border
        # around the table edges.
        self._out_w = int((TABLE_W_MM + 2 * self._margin_mm) * scale)
        self._out_h = int((TABLE_H_MM + 2 * self._margin_mm) * scale)
        self._poly: Optional["PolyCalibration"] = None
        # Camera intrinsics for pose-based mode (the clean robotics workflow:
        # K + dist from ChArUco calibration + per-frame solvePnP from 4 anchors).
        # When set, takes priority over poly for rectification.
        self._intrinsics = None  # type: Optional[CameraIntrinsics]
        # Per-frame H_table_to_pixel from solvePnP. Updated by update_pose().
        self._pose_H: Optional[np.ndarray] = None
        # Per-frame rvec/tvec stored so cv2.projectPoints can be used to draw
        # accurate overlays on the (distorted) raw view.
        self._pose_rvec: Optional[np.ndarray] = None
        self._pose_tvec: Optional[np.ndarray] = None
        # Lazily allocated by set_intrinsics().
        self._undist_mx = self._undist_my = self._undist_K = None
        # Cache of last detected anchor pixel positions {tag_id: (x, y)}.
        # The anchors are physically fixed so we keep the last seen pixel
        # forever -- this lets us survive frames where one anchor is
        # occluded (eg by a robot or a hand passing in front of it).
        self._anchor_pixels_cache: dict[int, tuple[float, float]] = {}
        # True when the last computed pose used at least one cached anchor
        # (signals the UI that the pose is not 100% live).
        self._pose_used_cache: bool = False

    @property
    def poly(self) -> Optional["PolyCalibration"]:
        return self._poly

    def set_poly(self, poly: Optional["PolyCalibration"]):
        self._poly = poly

    @property
    def has_poly(self) -> bool:
        return self._poly is not None

    # ---- Pose-based mode (K + dist + 4 anchors -> homography per frame) -----

    @property
    def intrinsics(self):
        return self._intrinsics

    def set_intrinsics(self, intr):
        """Set CameraIntrinsics for pose-based rectification. Pass None to clear."""
        self._intrinsics = intr
        self._pose_H = None
        # Pre-compute undistort maps for fast frame undistortion
        if intr is not None:
            try:
                self._undist_mx, self._undist_my, self._undist_K = intr.build_undistort_maps()
            except Exception:
                self._undist_mx = self._undist_my = self._undist_K = None
        else:
            self._undist_mx = self._undist_my = self._undist_K = None

    @property
    def has_intrinsics(self) -> bool:
        return self._intrinsics is not None

    def _observe_anchors(self, result: DetectionResult):
        """Refresh the anchor pixel cache with newly detected centers."""
        for anchor in self._config.anchors():
            c = result.get_center_for_id(anchor.tag_id)
            if c is not None:
                self._anchor_pixels_cache[anchor.tag_id] = (
                    float(c[0]), float(c[1]),
                )

    def clear_anchor_cache(self):
        """Drop all cached anchor positions (e.g. when the camera moves)."""
        self._anchor_pixels_cache.clear()

    def update_pose(self, result: DetectionResult) -> bool:
        """When intrinsics are set, detect 4 anchor centers and run solvePnP
        to get a per-frame homography H mapping (X_mm, Y_mm, 1) -> UNDISTORTED
        pixel (the pose is computed using K + dist on the distorted detected
        centers, but the resulting H is built with K_new because the frame is
        undistorted before the warp). Returns True if pose computed."""
        if self._intrinsics is None:
            return False
        # Refresh cache with whatever is detected this frame.
        self._observe_anchors(result)
        anchor_pixels = []
        anchor_mm = []
        used_cache = False
        for anchor in self._config.anchors():
            c = result.get_center_for_id(anchor.tag_id)
            if c is None:
                # Fall back to the last known pixel for this anchor.
                cached = self._anchor_pixels_cache.get(anchor.tag_id)
                if cached is None:
                    # Never seen -> we cannot solve.
                    self._pose_H = None
                    self._pose_rvec = None
                    self._pose_tvec = None
                    return False
                c = cached
                used_cache = True
            anchor_pixels.append([float(c[0]), float(c[1])])
            anchor_mm.append([anchor.x_mm, anchor.y_mm])
        self._pose_used_cache = used_cache
        try:
            R, T, rvec = self._intrinsics.estimate_table_pose(
                np.array(anchor_pixels, dtype=np.float32),
                np.array(anchor_mm, dtype=np.float32),
                z_mm=0.0,
            )
        except Exception:
            self._pose_H = None
            self._pose_rvec = None
            self._pose_tvec = None
            return False
        # Build H using the UNDISTORTED camera matrix K_new (after undistortion
        # the camera is a pure pinhole with intrinsics K_new and zero dist).
        # If K_new isn't available (undistort maps not built), fall back to
        # the original K -- still gives a usable pose-based BEV but the
        # remaining lens curvature stays in the warp.
        K_use = self._undist_K if self._undist_K is not None else self._intrinsics.K
        H = K_use @ np.column_stack([R[:, 0], R[:, 1], T.ravel()])
        self._pose_H = H
        self._pose_rvec = rvec
        self._pose_tvec = T
        return True

    @property
    def config(self) -> CornerConfig:
        return self._config

    @config.setter
    def config(self, c: CornerConfig):
        self._config = c
        self._H = None
        self._H_is_fresh = False

    @property
    def output_size(self) -> tuple[int, int]:
        return self._out_w, self._out_h

    @property
    def has_homography(self) -> bool:
        return (
            self._pose_H is not None
            or self._H is not None
            or self._poly is not None
        )

    @property
    def homography_is_fresh(self) -> bool:
        # Pose-based : fresh only if the 4 anchors were all detected LIVE.
        if self._pose_H is not None:
            return not self._pose_used_cache
        if self._poly is not None:
            return True
        return self._H is not None and self._H_is_fresh

    @property
    def homography_is_cached(self) -> bool:
        # Pose-based : cached if at least one anchor came from the cache.
        if self._pose_H is not None:
            return self._pose_used_cache
        if self._poly is not None:
            return False
        return self._H is not None and not self._H_is_fresh

    def update(self, result: DetectionResult) -> bool:
        src_pts: list = []
        dst_pts: list = []
        m = self._margin_mm
        # Refresh cache with whatever is detected this frame.
        self._observe_anchors(result)
        all_live = True
        for anchor in self._config.anchors():
            center = result.get_center_for_id(anchor.tag_id)
            if center is None:
                cached = self._anchor_pixels_cache.get(anchor.tag_id)
                if cached is None:
                    self._H_is_fresh = False
                    return self._H is not None
                center = cached
                all_live = False
            src_pts.append(list(center))
            # Output pixel coords include the margin offset so the rectif
            # spans (-m, -m) to (TABLE_W+m, TABLE_H+m) in mm.
            dst_pts.append([
                (anchor.x_mm + m) * self._scale,
                (anchor.y_mm + m) * self._scale,
            ])
        src_arr = np.array(src_pts, dtype=np.float32)
        dst_arr = np.array(dst_pts, dtype=np.float32)
        H, _ = cv2.findHomography(src_arr, dst_arr, cv2.RANSAC, 5.0)
        if H is None:
            self._H_is_fresh = False
            return self._H is not None
        self._H = H
        # Mark fresh only if this frame had ALL 4 anchors detected live ;
        # if we needed cached values, the homography is "stale" (still usable
        # but the user should know).
        self._H_is_fresh = all_live
        return True

    def rectify(self, frame: np.ndarray) -> Optional[np.ndarray]:
        # Priority 1 : pose-based (intrinsics + per-frame solvePnP from 4 anchors).
        # Mathematically clean, requires K+dist + visible anchors. Camera-aligned BEV.
        if self._intrinsics is not None and self._pose_H is not None:
            # 1) Undistort the frame so the lens curvature is removed BEFORE
            #    the homography warp. Without this step the floor stays bombe
            #    (the homography is a pinhole model, it does not absorb radial
            #    distortion).
            if self._undist_mx is not None and self._undist_my is not None:
                frame_und = cv2.remap(
                    frame, self._undist_mx, self._undist_my,
                    cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
                )
            else:
                frame_und = frame
            # 2) Build output_pixel -> undistorted source_pixel homography.
            #    output_pixel (px, py) -> table_mm (px/scale - margin, py/scale - margin)
            #    table_mm -> undistorted pixel via self._pose_H (uses K_new).
            m = self._margin_mm
            S = np.array([
                [1.0 / self._scale, 0.0,                -m],
                [0.0,                1.0 / self._scale, -m],
                [0.0,                0.0,                1.0],
            ], dtype=np.float64)
            H_total = self._pose_H @ S
            return cv2.warpPerspective(
                frame_und, H_total, (self._out_w, self._out_h),
                flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
            )

        # Priority 2 : polynomial calibration (red-mask based).
        if self._poly is not None:
            mm_per_px = 1.0 / self._scale
            try:
                mx, my = self._poly.build_rectif_maps(self._out_w, self._out_h, mm_per_px)
            except Exception:
                if self._H is None:
                    return None
                return cv2.warpPerspective(frame, self._H, (self._out_w, self._out_h))
            return cv2.remap(frame, mx, my, cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        # Priority 3 : 4-tag homography fallback.
        if self._H is None:
            return None
        return cv2.warpPerspective(frame, self._H, (self._out_w, self._out_h))

    def draw_grid(self, image: np.ndarray, step_mm: float = 200.0,
                  color: tuple = (0, 200, 80), thickness: int = 1,
                  draw_labels: bool = True) -> np.ndarray:
        """Draw a mm grid + anchor crosses on the rectified image.
        The grid lines and labels are drawn in USER-WORLD coordinates : we
        sample user-world mm steps and convert each to rectified pixel via
        table_mm_to_rectified (which applies inverse correction). This way
        the grid lines coincide with the actual table grid in user-world
        regardless of the camera orientation."""
        out = image.copy()
        w, h = self._out_w, self._out_h
        # Draw a thick red rectangle for the actual table edges so the user
        # can see exactly where the playable area is vs the margin.
        tl = self.table_mm_to_rectified(0.0, 0.0)
        br = self.table_mm_to_rectified(float(TABLE_W_MM), float(TABLE_H_MM))
        cv2.rectangle(out, tl, br, (0, 0, 220), 2)
        # Sample user-world X gridlines
        for x_user in np.arange(0, TABLE_W_MM + 1, step_mm):
            # 2 endpoints in user-world : (x_user, 0) and (x_user, TABLE_H_MM)
            px0, py0 = self.table_mm_to_rectified(float(x_user), 0.0)
            px1, py1 = self.table_mm_to_rectified(float(x_user), float(TABLE_H_MM))
            cv2.line(out, (px0, py0), (px1, py1), color, thickness)
            if draw_labels and 0 < x_user < TABLE_W_MM:
                # Place label near the top of the line
                cv2.putText(out, f"{int(x_user)}",
                            (px0 + 2, py0 + 14 if py0 < h - 20 else py0 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        # Sample user-world Y gridlines
        for y_user in np.arange(0, TABLE_H_MM + 1, step_mm):
            px0, py0 = self.table_mm_to_rectified(0.0, float(y_user))
            px1, py1 = self.table_mm_to_rectified(float(TABLE_W_MM), float(y_user))
            cv2.line(out, (px0, py0), (px1, py1), color, thickness)
            if draw_labels and 0 < y_user < TABLE_H_MM:
                cv2.putText(out, f"{int(y_user)}",
                            (px0 + 2 if px0 < w - 30 else px0 - 30, py0 - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        # Anchor crosses at the user-world position (label = tag id)
        for anchor in self._config.anchors():
            px, py = self.table_mm_to_rectified(anchor.x_mm, anchor.y_mm)
            cv2.drawMarker(out, (px, py), (0, 120, 255), cv2.MARKER_CROSS, 12, 2)
            cv2.putText(out, f"#{anchor.tag_id}", (px + 6, py - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 120, 255), 1, cv2.LINE_AA)
        return out

    def draw_overlay_on_raw(self, frame: np.ndarray,
                            result: Optional[DetectionResult] = None,
                            color: tuple = (0, 200, 80)) -> np.ndarray:
        """Draw the table contour + anchor crosses on the raw image.

        Projection strategy (in order of accuracy) :
          1. Pose-based (intrinsics + R, T from solvePnP) : cv2.projectPoints
             with K + dist projects 3D world points to distorted raw pixels.
             Mathematically exact, follows the lens curvature perfectly.
          2. Polynomial calibration : forward-rasterize the perimeter via the
             stored polynomial coefficients.
          3. 4-tag homography fallback : back-project the output rectangle
             through H_inv (linear approx, only valid in pinhole geometry).
        Anchor crosses always use the DETECTED pixel position when visible.
        """
        out = frame.copy()
        # Priority 1 : pose-based projection (exact, includes distortion)
        if (self._intrinsics is not None
                and self._pose_rvec is not None
                and self._pose_tvec is not None):
            try:
                # Build a dense table contour in 3D (Z=0) and project it.
                n = 60
                edge_3d = []
                for x in np.linspace(0, TABLE_W_MM, n):
                    edge_3d.append([x, 0, 0])
                for y in np.linspace(0, TABLE_H_MM, n):
                    edge_3d.append([TABLE_W_MM, y, 0])
                for x in np.linspace(TABLE_W_MM, 0, n):
                    edge_3d.append([x, TABLE_H_MM, 0])
                for y in np.linspace(TABLE_H_MM, 0, n):
                    edge_3d.append([0, y, 0])
                edge_3d_arr = np.array(edge_3d, dtype=np.float32).reshape(-1, 1, 3)
                proj, _ = cv2.projectPoints(
                    edge_3d_arr, self._pose_rvec, self._pose_tvec,
                    self._intrinsics.K, self._intrinsics.dist,
                )
                edge_px = proj.reshape(-1, 2).astype(np.int32)
                cv2.polylines(out, [edge_px.reshape(-1, 1, 2)], True, color, 2)
                # Anchor crosses
                for anchor in self._config.anchors():
                    detected = (
                        result.get_center_for_id(anchor.tag_id)
                        if result is not None else None
                    )
                    if detected is not None:
                        px, py = int(round(detected[0])), int(round(detected[1]))
                        cross_color = (0, 220, 0)
                    else:
                        # Project the predicted anchor mm -> pixel
                        pt3d = np.array([[[anchor.x_mm, anchor.y_mm, 0.0]]],
                                         dtype=np.float32)
                        pp, _ = cv2.projectPoints(
                            pt3d, self._pose_rvec, self._pose_tvec,
                            self._intrinsics.K, self._intrinsics.dist,
                        )
                        px, py = int(pp[0, 0, 0]), int(pp[0, 0, 1])
                        cross_color = (0, 120, 255)
                    cv2.drawMarker(out, (px, py), cross_color,
                                   cv2.MARKER_CROSS, 14, 2)
                    cv2.putText(out, f"#{anchor.tag_id}", (px + 8, py - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                cross_color, 1, cv2.LINE_AA)
            except Exception:
                pass
            return out
        if self._poly is not None:
            try:
                # Forward-rasterized contour : same approach as the calibration
                # wizard's '05_overlay_grid' -- evaluate the polynomial at every
                # pixel and color those whose world mm sits on the table edge.
                # Cached on the polynomial (constant), so cost is one-shot.
                # Follows the fisheye curvature exactly.
                if frame.shape[:2] == (self._poly.image_size[1], self._poly.image_size[0]):
                    perim = self._poly.compute_perimeter_pixels(
                        frame.shape, thickness_mm=15.0,
                    )
                    out[perim] = color
                else:
                    # Frame size differs from calibration -- fall back to the
                    # 4-corner homography (linear approximation).
                    cp = getattr(self._poly, "corners_pixel", None) or {}
                    if all(k in cp and cp[k] for k in ("TL", "TR", "BR", "BL")):
                        mm_corners = np.array([
                            [0, 0], [TABLE_W_MM, 0],
                            [TABLE_W_MM, TABLE_H_MM], [0, TABLE_H_MM],
                        ], dtype=np.float32)
                        px_corners = np.array([
                            cp["TL"], cp["TR"], cp["BR"], cp["BL"],
                        ], dtype=np.float32)
                        H_mm_to_px = cv2.getPerspectiveTransform(
                            mm_corners, px_corners,
                        )
                        edge_mm = []
                        n = 40
                        for x in np.linspace(0, TABLE_W_MM, n):
                            edge_mm.append([x, 0])
                        for y in np.linspace(0, TABLE_H_MM, n):
                            edge_mm.append([TABLE_W_MM, y])
                        for x in np.linspace(TABLE_W_MM, 0, n):
                            edge_mm.append([x, TABLE_H_MM])
                        for y in np.linspace(TABLE_H_MM, 0, n):
                            edge_mm.append([0, y])
                        edge_mm_arr = np.array(edge_mm, dtype=np.float32)
                        edge_h = np.column_stack([
                            edge_mm_arr,
                            np.ones(len(edge_mm_arr), dtype=np.float32),
                        ])
                        proj = (H_mm_to_px @ edge_h.T).T
                        edge_px = (proj[:, :2] / proj[:, 2:3]).astype(np.int32)
                        cv2.polylines(out, [edge_px.reshape(-1, 1, 2)],
                                      True, color, 2)

                # Compute the 4-corner homography (used for predicted anchors only)
                cp = getattr(self._poly, "corners_pixel", None) or {}
                H_mm_to_px = None
                if all(k in cp and cp[k] for k in ("TL", "TR", "BR", "BL")):
                    mm_corners = np.array([
                        [0, 0], [TABLE_W_MM, 0],
                        [TABLE_W_MM, TABLE_H_MM], [0, TABLE_H_MM],
                    ], dtype=np.float32)
                    px_corners = np.array([
                        cp["TL"], cp["TR"], cp["BR"], cp["BL"],
                    ], dtype=np.float32)
                    H_mm_to_px = cv2.getPerspectiveTransform(
                        mm_corners, px_corners,
                    )
                # Anchor crosses : prefer DETECTED pixels when visible
                for anchor in self._config.anchors():
                    detected = None
                    if result is not None:
                        detected = result.get_center_for_id(anchor.tag_id)
                    if detected is not None:
                        px, py = int(round(detected[0])), int(round(detected[1]))
                        cross_color = (0, 220, 0)
                    else:
                        if H_mm_to_px is not None:
                            pt = np.array([anchor.x_mm, anchor.y_mm, 1.0],
                                          dtype=np.float32)
                            p = H_mm_to_px @ pt
                            px, py = int(p[0] / p[2]), int(p[1] / p[2])
                        else:
                            p = self._poly.mm_to_pixel(
                                np.array([[anchor.x_mm, anchor.y_mm]])
                            )[0]
                            px, py = int(p[0]), int(p[1])
                        cross_color = (0, 120, 255)
                    cv2.drawMarker(out, (px, py), cross_color,
                                   cv2.MARKER_CROSS, 14, 2)
                    cv2.putText(out, f"#{anchor.tag_id}",
                                (px + 8, py - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, cross_color, 1,
                                cv2.LINE_AA)
            except Exception:
                pass
            return out
        # Priority 3 : 4-tag homography fallback. Back-project the BEV
        # rectangle (which spans the full TABLE+margin in mm) into raw
        # pixels via H_inv. Linear approximation only.
        if self._H is None:
            return out
        m = self._margin_mm
        table_corners_mm = np.array([
            [0, 0], [TABLE_W_MM, 0],
            [TABLE_W_MM, TABLE_H_MM], [0, TABLE_H_MM],
        ], dtype=np.float32)
        out_px_corners = np.array([
            [(c[0] + m) * self._scale, (c[1] + m) * self._scale]
            for c in table_corners_mm
        ], dtype=np.float32).reshape(-1, 1, 2)
        try:
            H_inv = np.linalg.inv(self._H)
        except np.linalg.LinAlgError:
            return out
        corners_src = cv2.perspectiveTransform(out_px_corners, H_inv)
        pts = corners_src.reshape(4, 2).astype(int)
        for i in range(4):
            cv2.line(out, tuple(pts[i]), tuple(pts[(i + 1) % 4]), color, 2)
        for anchor in self._config.anchors():
            pt_dst = np.array([[[
                (anchor.x_mm + m) * self._scale,
                (anchor.y_mm + m) * self._scale,
            ]]], dtype=np.float32)
            pt_src = cv2.perspectiveTransform(pt_dst, H_inv)
            px, py = int(pt_src[0, 0, 0]), int(pt_src[0, 0, 1])
            cv2.drawMarker(out, (px, py), (0, 120, 255), cv2.MARKER_CROSS, 10, 2)
        return out

    def image_to_table_mm(self, x: int, y: int):
        """Rectified pixel -> user-world mm."""
        if self._scale == 0:
            return None
        m = self._margin_mm
        mm_pred = (x / self._scale - m, y / self._scale - m)
        if self._intrinsics is not None and self._pose_H is not None:
            return mm_pred
        if self._poly is not None and self._poly.has_correction:
            mm_user = self._poly._apply_h2d(
                self._poly._correction_H, np.array([list(mm_pred)])
            )[0]
            return float(mm_user[0]), float(mm_user[1])
        return mm_pred

    def table_mm_to_rectified(self, x_mm: float, y_mm: float):
        """User-world mm -> rectified pixel."""
        m = self._margin_mm
        if self._intrinsics is not None and self._pose_H is not None:
            return int((x_mm + m) * self._scale), int((y_mm + m) * self._scale)
        if self._poly is not None and self._poly.has_correction:
            mm_pred = self._poly._apply_h2d(
                self._poly._correction_H_inv, np.array([[x_mm, y_mm]])
            )[0]
            x_mm, y_mm = float(mm_pred[0]), float(mm_pred[1])
        return int((x_mm + m) * self._scale), int((y_mm + m) * self._scale)

    def raw_pixel_to_table_mm(self, x: int, y: int):
        """Raw image pixel -> world mm."""
        if self._intrinsics is not None and self._pose_H is not None:
            try:
                P = self._undist_K if self._undist_K is not None else self._intrinsics.K
                pt = np.array([[[float(x), float(y)]]], dtype=np.float32)
                und = cv2.undistortPoints(
                    pt, self._intrinsics.K, self._intrinsics.dist, P=P,
                )
                ux, uy = float(und[0, 0, 0]), float(und[0, 0, 1])
                H_inv = np.linalg.inv(self._pose_H)
            except (np.linalg.LinAlgError, cv2.error):
                return None
            h = np.array([ux, uy, 1.0])
            out = H_inv @ h
            mm = out[:2] / out[2]
            return float(mm[0]), float(mm[1])
        if self._poly is not None:
            mm = self._poly.pixel_to_mm(np.array([[float(x), float(y)]]))[0]
            return float(mm[0]), float(mm[1])
        if self._H is None:
            return None
        pt = np.array([[[float(x), float(y)]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, self._H)
        rx, ry = result[0, 0, 0], result[0, 0, 1]
        return rx / self._scale - self._margin_mm, ry / self._scale - self._margin_mm
