"""
GPU-accel helpers for the vision pipeline.

Detects cv2.cuda capability at module load. Stock pip `opencv-contrib-
python` doesn't ship with CUDA — you need a CUDA-built OpenCV (Jetson's
default OpenCV from L4T BSP qualifies, as do custom builds compiled
with -DWITH_CUDA=ON).

When CUDA isn't available, every helper here falls back to the CPU
implementation transparently — so callers don't have to branch.
"""

from __future__ import annotations

CUDA_AVAILABLE = False
CUDA_DEVICE_NAME = None

try:
    import cv2
    if hasattr(cv2, 'cuda') and cv2.cuda.getCudaEnabledDeviceCount() > 0:
        CUDA_AVAILABLE = True
        try:
            CUDA_DEVICE_NAME = cv2.cuda.printCudaDeviceInfo(0) or '(unknown)'
        except Exception:
            CUDA_DEVICE_NAME = '(unknown)'
        # Non-fatal print so the user sees the path was chosen at startup.
        print(f'[vision] CUDA enabled — '
              f'{cv2.cuda.getCudaEnabledDeviceCount()} device(s)')
    else:
        print('[vision] CUDA not available — running OpenCV on CPU')
except Exception as _e:
    # cv2 itself missing or older API surface — same outcome (no CUDA).
    print(f'[vision] CUDA detection failed: {_e}')


def upload_to_gpu(frame):
    """Upload a numpy ndarray to a GpuMat. Returns the GpuMat, or None
    if CUDA isn't available / cv2 unavailable."""
    if not CUDA_AVAILABLE:
        return None
    try:
        g = cv2.cuda_GpuMat()
        g.upload(frame)
        return g
    except Exception:
        return None


def gpu_remap(frame, mx_cpu, my_cpu, mx_gpu, my_gpu):
    """GPU equivalent of cv2.remap. Maps `frame` (numpy ndarray) through
    the supplied remap maps and returns a numpy ndarray.

    `mx_gpu` and `my_gpu` should be pre-uploaded `cv2.cuda_GpuMat`
    (allocated once at start()) — uploading the maps every frame would
    swamp the bus.

    Falls back to CPU `cv2.remap(frame, mx_cpu, my_cpu, …)` when CUDA
    isn't available.
    """
    if not CUDA_AVAILABLE or mx_gpu is None or my_gpu is None:
        return cv2.remap(
            frame, mx_cpu, my_cpu, cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
        )
    try:
        g_in = cv2.cuda_GpuMat()
        g_in.upload(frame)
        g_out = cv2.cuda.remap(
            g_in, mx_gpu, my_gpu, cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
        )
        return g_out.download()
    except Exception:
        # Any GPU error: fall back transparently to CPU for this frame.
        return cv2.remap(
            frame, mx_cpu, my_cpu, cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
        )


def gpu_warp_perspective(frame, H, dsize, flags=None):
    """GPU equivalent of cv2.warpPerspective with CPU fallback.

    `H` and `dsize` are the same as the CPU API. `flags` defaults to
    INTER_LINEAR | WARP_INVERSE_MAP when None.
    """
    import cv2
    if flags is None:
        flags = cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP
    if not CUDA_AVAILABLE:
        return cv2.warpPerspective(
            frame, H, dsize, flags=flags,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
        )
    try:
        g_in = cv2.cuda_GpuMat()
        g_in.upload(frame)
        g_out = cv2.cuda.warpPerspective(
            g_in, H, dsize, flags=flags,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
        )
        return g_out.download()
    except Exception:
        return cv2.warpPerspective(
            frame, H, dsize, flags=flags,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
        )
