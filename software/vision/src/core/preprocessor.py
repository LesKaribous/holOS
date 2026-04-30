"""
Preprocessor -- pipeline d'operations image (a activer/desactiver) appliquees
en amont de la detection ArUco.

Toutes les operations travaillent en BGR uint8.
Chaque option = {'on': bool, ...parametres}. Ordre d'application dans apply().

Etapes disponibles (ordre fixe) :
    median_blur       -> impulse noise (sel et poivre, hot pixels)
    gaussian_blur     -> flou gaussien
    bilateral         -> debruitage bord-preservant
    nlm_denoise       -> Non-Local Means (denoise tres puissant, plus lent)
    motion_deblur     -> Wiener avec PSF lineaire (length, angle, snr)
    brightness        -> alpha (contraste) + beta (luminosite)
    gamma             -> correction gamma
    clahe             -> egalisation locale du contraste
    unsharp           -> masque flou inverse (sharpening)
    edge_enhance      -> boost des aretes par Sobel
    adaptive_thresh   -> seuil adaptatif (image binaire, pour ArUco)
    grayscale         -> conversion grayscale finale (3 canaux pour homogeneite)
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# PSF helper for motion deblur
# ---------------------------------------------------------------------------

def _motion_psf(length: float, angle_deg: float, size: int = 31) -> np.ndarray:
    """Return a 2D point spread function corresponding to a linear motion blur
    of given pixel length and angle (degrees). Normalized to sum=1."""
    size = max(3, int(size) | 1)   # odd
    psf = np.zeros((size, size), dtype=np.float32)
    cx, cy = size // 2, size // 2
    rad = np.deg2rad(angle_deg)
    dx, dy = np.cos(rad), np.sin(rad)
    n = max(1, int(round(length)))
    for i in range(-n // 2, n // 2 + 1):
        x = int(round(cx + i * dx))
        y = int(round(cy + i * dy))
        if 0 <= x < size and 0 <= y < size:
            psf[y, x] = 1.0
    s = psf.sum()
    if s <= 0:
        psf[cy, cx] = 1.0
        return psf
    return psf / s


def _wiener_deblur(channel: np.ndarray, psf: np.ndarray, snr_db: float) -> np.ndarray:
    """Wiener deconvolution of a single 2D channel with a known PSF.
    snr_db : signal-to-noise ratio in dB. Higher = sharper (and more noise).
    Returns a uint8 image."""
    img = channel.astype(np.float32) / 255.0
    H, W = img.shape
    # FFT-based deconvolution. Pad PSF to image size, center it.
    psf_pad = np.zeros_like(img)
    pH, pW = psf.shape
    psf_pad[:pH, :pW] = psf
    psf_pad = np.roll(psf_pad, -pH // 2, axis=0)
    psf_pad = np.roll(psf_pad, -pW // 2, axis=1)

    PSF_F = np.fft.rfft2(psf_pad)
    IMG_F = np.fft.rfft2(img)
    snr_linear = 10.0 ** (snr_db / 10.0)
    H_conj = np.conj(PSF_F)
    H_abs2 = (PSF_F.real ** 2 + PSF_F.imag ** 2)
    inv = H_conj / (H_abs2 + 1.0 / max(snr_linear, 1e-6))
    out = np.fft.irfft2(IMG_F * inv, s=img.shape)
    out = np.clip(out, 0.0, 1.0)
    return (out * 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Preprocessor
# ---------------------------------------------------------------------------

class Preprocessor:
    """Pipeline d'operations image. Chaque etape activable/parametree."""

    DEFAULT_OPTS: dict[str, dict[str, Any]] = {
        "median_blur":     {"on": False, "ksize": 3},
        "gaussian_blur":   {"on": False, "sigma": 1.5},
        "bilateral":       {"on": False, "d": 9, "sigma_color": 75, "sigma_space": 75},
        "nlm_denoise":     {"on": False, "h": 7, "template": 7, "search": 21},
        "motion_deblur":   {"on": False, "length": 12.0, "angle": 0.0, "snr_db": 25.0},
        "brightness":      {"on": False, "alpha": 1.0, "beta": 0.0},
        "gamma":           {"on": False, "value": 1.0},
        "clahe":           {"on": False, "clip": 3.0, "tile": 8},
        "unsharp":         {"on": False, "amount": 1.0, "radius": 1.5},
        "edge_enhance":    {"on": False, "amount": 0.5},
        "adaptive_thresh": {"on": False, "block": 31, "C": 5},
        "grayscale":       {"on": False},
    }

    def __init__(self):
        self.opts: dict[str, dict[str, Any]] = {
            k: dict(v) for k, v in self.DEFAULT_OPTS.items()
        }

    # ---- Serialisation ----------------------------------------------------

    def to_dict(self):
        return {k: dict(v) for k, v in self.opts.items()}

    def load_dict(self, d):
        if not d:
            return
        for k, v in d.items():
            if k in self.opts and isinstance(v, dict):
                self.opts[k].update(v)

    @property
    def any_active(self) -> bool:
        return any(v.get("on") for v in self.opts.values())

    def reset(self):
        self.opts = {k: dict(v) for k, v in self.DEFAULT_OPTS.items()}

    # ---- Apply ------------------------------------------------------------

    def apply(self, img: np.ndarray) -> np.ndarray:
        out = img.copy()
        o = self.opts

        if o["median_blur"]["on"]:
            k = max(3, int(o["median_blur"]["ksize"])) | 1
            out = cv2.medianBlur(out, k)

        if o["gaussian_blur"]["on"]:
            sig = max(0.1, float(o["gaussian_blur"]["sigma"]))
            ksize = int(2 * round(3 * sig) + 1)
            out = cv2.GaussianBlur(out, (ksize, ksize), sig)

        if o["bilateral"]["on"]:
            out = cv2.bilateralFilter(
                out, int(o["bilateral"]["d"]),
                float(o["bilateral"]["sigma_color"]),
                float(o["bilateral"]["sigma_space"]),
            )

        if o["nlm_denoise"]["on"]:
            out = cv2.fastNlMeansDenoisingColored(
                out, None,
                float(o["nlm_denoise"]["h"]),
                float(o["nlm_denoise"]["h"]),
                int(o["nlm_denoise"]["template"]) | 1,
                int(o["nlm_denoise"]["search"]) | 1,
            )

        if o["motion_deblur"]["on"]:
            psf = _motion_psf(
                length=float(o["motion_deblur"]["length"]),
                angle_deg=float(o["motion_deblur"]["angle"]),
                size=max(15, int(o["motion_deblur"]["length"]) | 1),
            )
            snr = float(o["motion_deblur"]["snr_db"])
            chans = cv2.split(out)
            chans = [_wiener_deblur(c, psf, snr) for c in chans]
            out = cv2.merge(chans)

        if o["brightness"]["on"]:
            out = cv2.convertScaleAbs(
                out,
                alpha=float(o["brightness"]["alpha"]),
                beta=float(o["brightness"]["beta"]),
            )

        if o["gamma"]["on"]:
            g = max(0.05, float(o["gamma"]["value"]))
            lut = np.array(
                [((i / 255.0) ** (1.0 / g)) * 255 for i in range(256)]
            ).astype(np.uint8)
            out = cv2.LUT(out, lut)

        if o["clahe"]["on"]:
            yuv = cv2.cvtColor(out, cv2.COLOR_BGR2YUV)
            clahe = cv2.createCLAHE(
                clipLimit=float(o["clahe"]["clip"]),
                tileGridSize=(int(o["clahe"]["tile"]), int(o["clahe"]["tile"])),
            )
            yuv[:, :, 0] = clahe.apply(yuv[:, :, 0])
            out = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

        if o["unsharp"]["on"]:
            blurred = cv2.GaussianBlur(out, (0, 0), float(o["unsharp"]["radius"]))
            out = cv2.addWeighted(
                out, 1.0 + float(o["unsharp"]["amount"]),
                blurred, -float(o["unsharp"]["amount"]),
                0,
            )

        if o["edge_enhance"]["on"]:
            gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
            sx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
            sy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
            mag = np.hypot(sx, sy)
            mag = np.clip(mag, 0, 255).astype(np.uint8)
            edges = cv2.cvtColor(mag, cv2.COLOR_GRAY2BGR)
            out = cv2.addWeighted(out, 1.0, edges, float(o["edge_enhance"]["amount"]), 0)

        if o["adaptive_thresh"]["on"]:
            gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
            block = int(o["adaptive_thresh"]["block"]) | 1
            block = max(3, block)
            th = cv2.adaptiveThreshold(
                gray, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
                block, int(o["adaptive_thresh"]["C"]),
            )
            out = cv2.cvtColor(th, cv2.COLOR_GRAY2BGR)

        if o["grayscale"]["on"]:
            gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
            out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        return out
