"""
Shared helpers for nodes that consume `coord_state` from rectify.

`coord_state` schema (emitted by RectifyNode):
    {
      flip_x: bool,         flip_y: bool,
      flip_theta: bool,
      table_w_mm: float,    table_h_mm: float,
      locked: bool,
      source: str,          world_corner: str,
      ref_tag_id: int,
    }
"""

from __future__ import annotations

import math
from typing import Optional

try:
    import cv2
    _CV2_OK = True
except Exception:
    _CV2_OK = False


def coord_state_or_none(value) -> Optional[dict]:
    """Validate that an input is a usable coord_state dict."""
    if not isinstance(value, dict):
        return None
    if 'flip_x' not in value or 'flip_y' not in value:
        return None
    return value


def flip_mm(x, y, cs: dict):
    """Mirror an (x, y) mm pair using cs's flip flags + table dimensions."""
    if x is None or y is None:
        return x, y
    try:
        xf = float(x); yf = float(y)
    except (TypeError, ValueError):
        return x, y
    if cs.get('flip_x'):
        xf = float(cs.get('table_w_mm', 3000.0)) - xf
    if cs.get('flip_y'):
        yf = float(cs.get('table_h_mm', 2000.0)) - yf
    return xf, yf


def flip_theta(theta_rad, cs: dict):
    """Mirror a heading angle to match the flipped frame."""
    if theta_rad is None or not cs.get('flip_theta', True):
        return theta_rad
    try:
        v = float(theta_rad)
    except (TypeError, ValueError):
        return theta_rad
    if cs.get('flip_x'): v = math.pi - v
    if cs.get('flip_y'): v = -v
    # wrap to (-π, π]
    return (v + math.pi) % (2 * math.pi) - math.pi


def flip_pose_list(poses, cs: dict):
    """Apply coord-state flip to every dict in a pose_list. Returns new list."""
    if not poses or not cs:
        return poses
    out = []
    for q in poses:
        if not isinstance(q, dict):
            continue
        nq = dict(q)
        nq['x_mm'], nq['y_mm'] = flip_mm(q.get('x_mm'), q.get('y_mm'), cs)
        if 'naive_x_mm' in q or 'naive_y_mm' in q:
            nq['naive_x_mm'], nq['naive_y_mm'] = flip_mm(
                q.get('naive_x_mm'), q.get('naive_y_mm'), cs)
        if cs.get('flip_theta', True) and q.get('theta_rad') is not None:
            nq['theta_rad'] = flip_theta(q['theta_rad'], cs)
        out.append(nq)
    return out


def flip_cam_xyz(cam, cs: dict):
    """Apply coord-state flip to a cam_xyz dict (x_mm, y_mm only — z is up)."""
    if not isinstance(cam, dict) or not cs:
        return cam
    ncam = dict(cam)
    ncam['x_mm'], ncam['y_mm'] = flip_mm(cam.get('x_mm'), cam.get('y_mm'), cs)
    return ncam


def world_to_bev_mm(x_world, y_world, cs: Optional[dict]):
    """Convert a world-frame mm pair back into BEV-native mm.

    The BEV image is in its native frame (origin top-left). When a node
    has user-frame data and wants to draw it on the BEV, it must invert
    the flip first. The flip is its own inverse (mirror around the
    midline) so this is identical to `flip_mm` — but having a clearly
    named function in callsites makes the intent explicit.
    """
    if cs is None:
        return x_world, y_world
    return flip_mm(x_world, y_world, cs)


def world_mm_to_bev_pixel(x_world, y_world, cs: Optional[dict],
                           margin_mm: float, scale: float):
    """Convenience: world mm → BEV pixel via the BEV-native projection."""
    if x_world is None or y_world is None:
        return None
    try:
        xb, yb = world_to_bev_mm(x_world, y_world, cs)
        if xb is None or yb is None:
            return None
        return (int(round((float(xb) + margin_mm) * scale)),
                int(round((float(yb) + margin_mm) * scale)))
    except (TypeError, ValueError):
        return None
