"""
Rectification de la table via homographie ou polynome.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

import cv2
import numpy as np

from core.aruco_detector import DetectionResult

# CUDA offload — Jetson Orin Nano builds of OpenCV ship with CUDA support.
# When available, warpPerspective + remap are both ~3-5× faster than the CPU
# path on 1080p frames. The check is one-shot at import; failures fall back
# silently to the CPU path so a non-CUDA build (or a runtime API mismatch)
# never breaks rectification.
try:
    _HAS_CUDA = (cv2.cuda.getCudaEnabledDeviceCount() > 0)
except Exception:
    _HAS_CUDA = False


def _warp_perspective(frame, H, dsize, flags=cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)):
    """warpPerspective with CUDA fast path + CPU fallback. Same call sig as
    cv2.warpPerspective so callers don't care which path runs."""
    if _HAS_CUDA:
        try:
            g = cv2.cuda_GpuMat()
            g.upload(frame)
            out = cv2.cuda.warpPerspective(
                g, H, dsize, flags=flags,
                borderMode=borderMode, borderValue=borderValue)
            return out.download()
        except Exception:
            pass   # silent fallback — CPU path below
    return cv2.warpPerspective(
        frame, H, dsize, flags=flags,
        borderMode=borderMode, borderValue=borderValue)


def _remap(frame, mx, my, interpolation=cv2.INTER_LINEAR,
           borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)):
    """remap with CUDA fast path + CPU fallback."""
    if _HAS_CUDA:
        try:
            g = cv2.cuda_GpuMat();   g.upload(frame)
            gx = cv2.cuda_GpuMat();  gx.upload(mx)
            gy = cv2.cuda_GpuMat();  gy.upload(my)
            out = cv2.cuda.remap(
                g, gx, gy, interpolation,
                borderMode=borderMode, borderValue=borderValue)
            return out.download()
        except Exception:
            pass
    return cv2.remap(frame, mx, my, interpolation,
                     borderMode=borderMode, borderValue=borderValue)

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
    """ArUco tag id paired with its known position on the table (mm) and
    its in-plane orientation. yaw_deg = rotation of the tag's own axes
    relative to the BEV-native frame (TwinVision: X→right, Y→down).
    Positive = clockwise as seen from the camera. Used by the 2-tag
    perspective rectification (update_2pt_corners) so the 4 detected
    corners line up with the right BEV-native mm positions even when
    tags are placed at non-zero yaw."""
    tag_id: int
    x_mm: float
    y_mm: float
    yaw_deg: float = 0.0

    def to_dict(self) -> dict:
        return {
            "tag_id":  self.tag_id,
            "x_mm":    self.x_mm,
            "y_mm":    self.y_mm,
            "yaw_deg": self.yaw_deg,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "TagAnchor":
        return cls(
            tag_id=int(d["tag_id"]),
            x_mm=float(d["x_mm"]),
            y_mm=float(d["y_mm"]),
            yaw_deg=float(d.get("yaw_deg", 0.0)),
        )


@dataclass
class CornerConfig:
    # 4 ArUco anchors precisely placed at 600 mm from each corner of the
    # 3000 x 2000 mm table -- forming a 1800 x 800 inner rectangle.
    top_left:     TagAnchor = field(default_factory=lambda: TagAnchor(23,  600,  600))
    top_right:    TagAnchor = field(default_factory=lambda: TagAnchor(22, 2400,  600))
    bottom_right: TagAnchor = field(default_factory=lambda: TagAnchor(20, 2400, 1400))
    bottom_left:  TagAnchor = field(default_factory=lambda: TagAnchor(21,  600, 1400))
    # Optional additional anchors not bound to the 4 corner slots (e.g. a
    # center tag added physically to give the corner-based homography a
    # 3rd non-collinear point). They participate in update_corners_homography
    # the same way as the 4 corner anchors but are NOT used by the legacy
    # `update()` (4-anchor RANSAC) — that one is hard-wired to the 4 slots.
    extras: list = field(default_factory=list)

    def anchors(self) -> list[TagAnchor]:
        return [self.top_left, self.top_right, self.bottom_right, self.bottom_left] + list(self.extras)

    def to_dict(self) -> dict:
        out = {
            "top_left":     self.top_left.to_dict(),
            "top_right":    self.top_right.to_dict(),
            "bottom_right": self.bottom_right.to_dict(),
            "bottom_left":  self.bottom_left.to_dict(),
        }
        if self.extras:
            out["extras"] = [a.to_dict() for a in self.extras]
        return out

    @classmethod
    def from_dict(cls, d: dict) -> "CornerConfig":
        return cls(
            top_left=     TagAnchor.from_dict(d["top_left"]),
            top_right=    TagAnchor.from_dict(d["top_right"]),
            bottom_right= TagAnchor.from_dict(d["bottom_right"]),
            bottom_left=  TagAnchor.from_dict(d["bottom_left"]),
            extras=[TagAnchor.from_dict(x) for x in (d.get("extras") or [])],
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

    def update_corners_homography(self, result: DetectionResult,
                                  tag_ids: list,
                                  tag_size_mm: float) -> bool:
        """Multi-tag perspective homography from the 4 corners of each
        detected anchor tag. Uses whichever ids in `tag_ids` are visible
        this tick (need at least 2 for an 8-point findHomography fit;
        more is better — 3+ non-collinear tags makes the H well-conditioned
        and immune to the colinearity trap that 2 colinear tags hit).

        Each anchor must already exist in the rectifier's CornerConfig
        (either as one of the 4 corner slots or as an `extras` entry) so
        we know its world-mm position + per-tag yaw.

        tag_size_mm: edge length of the printed marker (black border to
        black border) in mm. Same value for all tags.
        """
        anchors_by_id = {a.tag_id: a for a in self._config.anchors()}
        s = float(tag_size_mm)
        if s <= 0:
            self._H_is_fresh = False
            return self._H is not None
        m = self._margin_mm
        scale = self._scale
        import math as _m

        src_pts: list = []
        dst_pts: list = []
        used_ids: list = []
        for raw_tid in (tag_ids or []):
            tid = int(raw_tid)
            anc = anchors_by_id.get(tid)
            if anc is None:
                continue
            c = result.get_corners_for_id(tid)   # (4, 2) pixels
            if c is None:
                continue
            for p in c:
                src_pts.append([float(p[0]), float(p[1])])
            cx, cy = float(anc.x_mm), float(anc.y_mm)
            half = s / 2.0
            offsets = [(-half, -half),
                       ( half, -half),
                       ( half,  half),
                       (-half,  half)]
            yaw_rad = _m.radians(float(getattr(anc, 'yaw_deg', 0.0)))
            cos_y, sin_y = _m.cos(yaw_rad), _m.sin(yaw_rad)
            for ox, oy in offsets:
                rx = cos_y * ox - sin_y * oy
                ry = sin_y * ox + cos_y * oy
                x_mm, y_mm = cx + rx, cy + ry
                dst_pts.append([(x_mm + m) * scale, (y_mm + m) * scale])
            used_ids.append(tid)

        # findHomography needs ≥ 4 src/dst pairs. With 1 tag (4 corners)
        # the fit is degenerate (a single planar marker carries no
        # perspective info beyond a similarity); require ≥ 2 tags.
        if len(used_ids) < 2:
            self._H_is_fresh = False
            return self._H is not None
        src_arr = np.array(src_pts, dtype=np.float32)
        dst_arr = np.array(dst_pts, dtype=np.float32)
        H, _ = cv2.findHomography(src_arr, dst_arr, cv2.RANSAC, 5.0)
        if H is None:
            self._H_is_fresh = False
            return self._H is not None
        self._H = H
        self._H_is_fresh = True
        return True

    def update_2pt_corners(self, result: DetectionResult,
                           tag_a_id: int, tag_b_id: int,
                           tag_size_mm: float) -> bool:
        """Back-compat wrapper. Delegates to the generic N-tag method."""
        return self.update_corners_homography(
            result, [tag_a_id, tag_b_id], tag_size_mm)

    def update_2pt_similarity(self, result: DetectionResult,
                              tag_a_id: int, tag_b_id: int) -> bool:
        """Reduced-DOF rectification when only 2 anchor tags are physically
        on the table. Solves a similarity (rotation + uniform scale +
        translation = 4 DOF) from the 2 detected centers — exact fit, no
        perspective correction. Assumes the camera is roughly orthogonal
        to the table; otherwise residual perspective distortion remains.

        The 2 tag IDs must be present in the configured anchors so we know
        their world-mm coordinates. Detected pixel centers fall back to the
        cache (same convention as update()) when one is missing this tick.

        Returns True on success and writes self._H. Marks fresh iff both
        anchors were detected live this frame.
        """
        anchors_by_id = {a.tag_id: a for a in self._config.anchors()}
        anchor_a = anchors_by_id.get(int(tag_a_id))
        anchor_b = anchors_by_id.get(int(tag_b_id))
        if anchor_a is None or anchor_b is None:
            self._H_is_fresh = False
            return self._H is not None
        self._observe_anchors(result)

        def _pix(tag_id: int):
            c = result.get_center_for_id(tag_id)
            if c is not None:
                return c, True
            cached = self._anchor_pixels_cache.get(tag_id)
            return (cached, False) if cached is not None else (None, False)

        pa, pa_live = _pix(anchor_a.tag_id)
        pb, pb_live = _pix(anchor_b.tag_id)
        if pa is None or pb is None:
            self._H_is_fresh = False
            return self._H is not None

        pdx, pdy = float(pb[0] - pa[0]), float(pb[1] - pa[1])
        mdx, mdy = float(anchor_b.x_mm - anchor_a.x_mm), float(anchor_b.y_mm - anchor_a.y_mm)
        p_len = (pdx * pdx + pdy * pdy) ** 0.5
        m_len = (mdx * mdx + mdy * mdy) ** 0.5
        if p_len < 1e-3 or m_len < 1e-3:
            self._H_is_fresh = False
            return self._H is not None

        # Similarity pixel→mm: rotate the pixel-space vector onto the mm
        # vector, scale by m_len/p_len, translate so anchor_a → its mm pos.
        import math
        scale_mm_per_px = m_len / p_len
        rot = math.atan2(mdy, mdx) - math.atan2(pdy, pdx)
        cos_r, sin_r = math.cos(rot), math.sin(rot)
        a = scale_mm_per_px * cos_r
        b = -scale_mm_per_px * sin_r
        c = anchor_a.x_mm - a * pa[0] - b * pa[1]
        d = scale_mm_per_px * sin_r
        e =  scale_mm_per_px * cos_r
        f = anchor_a.y_mm - d * pa[0] - e * pa[1]
        H_pix_to_mm = np.array([[a, b, c],
                                [d, e, f],
                                [0, 0, 1]], dtype=np.float64)
        # Chain with mm → BEV pixel (same margin/scale convention as update()).
        s = self._scale
        margin = self._margin_mm
        H_mm_to_bev = np.array([[s, 0, s * margin],
                                [0, s, s * margin],
                                [0, 0, 1]], dtype=np.float64)
        self._H = H_mm_to_bev @ H_pix_to_mm
        self._H_is_fresh = bool(pa_live and pb_live)
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
                frame_und = _remap(
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
            return _warp_perspective(
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
                return _warp_perspective(frame, self._H, (self._out_w, self._out_h))
            return _remap(frame, mx, my, cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        # Priority 3 : 4-tag homography fallback.
        if self._H is None:
            return None
        return _warp_perspective(frame, self._H, (self._out_w, self._out_h))

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
