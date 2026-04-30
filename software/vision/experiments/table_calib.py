"""
Calibration de la caméra (table 2 m × 3 m) à partir d'une image fisheye + mask manuel
=====================================================================================

Entrées :
  - capture.jpg : photo de la table par la caméra fisheye
  - red.mask.png : même résolution, table couverte d'un masque rouge plein

Sortie : mapping pixel ↔ coordonnées sol (3 m × 2 m), via deux modèles :
  - homographie 4 coins (rapide, ≈ 16 cm RMS — peu précis à cause de la distorsion)
  - polynôme 2D deg 3 (≈ 11 mm RMS — recommandé)

Convention monde :
  - origine au centre de la table
  - X horizontal, plage [-1.5, +1.5] m (3 m)
  - Y profondeur (positif = loin), plage [-1.0, +1.0] m (2 m)
  - Z = 0 = plan du sol

Note : pour une calibration intrinsèque (fx, fy, k1..k4) précise, utiliser
cv2.fisheye.calibrate() avec >=10 vues d'un échiquier — un seul rectangle ne
suffit pas pour résoudre les paramètres intrinsèques de manière stable.
"""

import cv2
import numpy as np
import json
import os
from pathlib import Path
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import griddata

# Géométrie de la table
TABLE_WIDTH_M  = 3.0   # X total
TABLE_DEPTH_M  = 2.0   # Y total
LX = TABLE_WIDTH_M / 2
LY = TABLE_DEPTH_M / 2

POLY_DEG = 3   # 3 = compromis précision/stabilité ; 4-5 oscillent à l'intérieur
PX_PER_M = 200 # résolution BEV (200 px/m = 5 mm/px)


# -------------------------------------------------------------------------
# 1. Extraire le contour de la table depuis le mask rouge
# -------------------------------------------------------------------------
def extract_red_mask_contour(mask_img):
    B, G, R = cv2.split(mask_img)
    m = ((R > 150) & (G < 80) & (B < 80)).astype(np.uint8) * 255
    n, lbl, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n > 1:
        m = (lbl == 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])).astype(np.uint8) * 255
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    return m, contour


def find_corners(contour):
    """4 coins = 4 maxima de distance au centroïde du contour."""
    cx, cy = contour.mean(0)
    ang = np.arctan2(contour[:, 1] - cy, contour[:, 0] - cx)
    order = np.argsort(ang)
    contour_s = contour[order]
    ang_s = ang[order]
    ds = gaussian_filter1d(np.hypot(contour_s[:, 0]-cx, contour_s[:, 1]-cy),
                           sigma=8, mode='wrap')
    ext = np.concatenate([ds, ds, ds])
    peaks, _ = find_peaks(ext, distance=len(ds)//6)
    peaks = peaks[(peaks >= len(ds)) & (peaks < 2*len(ds))] - len(ds)
    peaks = peaks[np.argsort(-ds[peaks])[:4]]
    peaks.sort()

    corners = {}
    for p in peaks:
        a = np.degrees(ang_s[p])
        pt = contour_s[p]
        if   -180 <= a < -90: corners['TL'] = pt
        elif  -90 <= a < 0:   corners['TR'] = pt
        elif    0 <= a < 90:  corners['BR'] = pt
        else:                 corners['BL'] = pt
    return corners, contour_s


def split_contour(contour_s, corners):
    """Sépare le contour ordonné en 4 arcs (un par côté)."""
    def fi(pt):
        return int(np.argmin(np.linalg.norm(contour_s - pt, axis=1)))
    def arc(i1, i2):
        return contour_s[i1:i2+1] if i1 <= i2 \
               else np.vstack([contour_s[i1:], contour_s[:i2+1]])

    iTL, iTR = fi(corners['TL']), fi(corners['TR'])
    iBR, iBL = fi(corners['BR']), fi(corners['BL'])
    return {
        'TOP':    arc(iTL, iTR),
        'RIGHT':  arc(iTR, iBR),
        'BOTTOM': arc(iBR, iBL),
        'LEFT':   arc(iBL, iTL),
    }


# -------------------------------------------------------------------------
# 2. Construire les correspondances pixel ↔ monde
# -------------------------------------------------------------------------
def build_correspondences(corners, sides, step=4, corner_weight=20):
    """
    Pour chaque point du contour, on infère sa position monde par
    paramétrisation linéaire le long de la corde du côté correspondant
    (approximation suffisante car le poly absorbe la non-linéarité).
    """
    TL3, TR3, BR3, BL3 = (
        np.array([-LX, +LY]), np.array([+LX, +LY]),
        np.array([+LX, -LY]), np.array([-LX, -LY]),
    )
    pairs = [
        (sides['TOP'],    corners['TL'], corners['TR'], TL3, TR3),
        (sides['RIGHT'],  corners['TR'], corners['BR'], TR3, BR3),
        (sides['BOTTOM'], corners['BR'], corners['BL'], BR3, BL3),
        (sides['LEFT'],   corners['BL'], corners['TL'], BL3, TL3),
    ]

    img_pts, world_pts = [], []
    for arr, c1p, c2p, c1m, c2m in pairs:
        sub = arr[::step]
        v = c2p - c1p
        t = np.clip(((sub - c1p) @ v) / (v @ v), 0, 1)
        for i, ti in enumerate(t):
            img_pts.append(sub[i])
            world_pts.append(c1m + ti * (c2m - c1m))

    # Coins répétés pour leur donner un poids fort
    for _ in range(corner_weight):
        img_pts.extend([corners['TL'], corners['TR'],
                        corners['BR'], corners['BL']])
        world_pts.extend([TL3, TR3, BR3, BL3])
    return (np.array(img_pts, dtype=np.float64),
            np.array(world_pts, dtype=np.float64))


# -------------------------------------------------------------------------
# 3. Modèles : homographie + polynôme 2D
# -------------------------------------------------------------------------
def fit_homography(corners):
    src = np.array([corners['TL'], corners['TR'],
                    corners['BR'], corners['BL']], dtype=np.float32)
    dst = np.array([[-LX, +LY], [+LX, +LY],
                    [+LX, -LY], [-LX, -LY]], dtype=np.float32)
    return cv2.getPerspectiveTransform(src, dst)


def _poly_design(u, v, deg):
    return np.column_stack([(u**i) * (v**j)
                            for i in range(deg+1)
                            for j in range(deg+1-i)])


def fit_poly(img_pts, world_pts, image_size, deg=POLY_DEG):
    w_, h_ = image_size
    m = {'u_mean': w_/2., 'v_mean': h_/2.,
         'u_std':  w_/2., 'v_std':  h_/2., 'deg': deg}
    u = (img_pts[:, 0] - m['u_mean']) / m['u_std']
    v = (img_pts[:, 1] - m['v_mean']) / m['v_std']
    F = _poly_design(u, v, deg)
    m['coef_x'], _, _, _ = np.linalg.lstsq(F, world_pts[:, 0], rcond=None)
    m['coef_y'], _, _, _ = np.linalg.lstsq(F, world_pts[:, 1], rcond=None)
    return m


def pixel_to_world_poly(uv, m):
    """uv: (N,2) ou (2,) → (N,2) en mètres."""
    uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
    u = (uv[:, 0] - m['u_mean']) / m['u_std']
    v = (uv[:, 1] - m['v_mean']) / m['v_std']
    F = _poly_design(u, v, m['deg'])
    return np.column_stack([F @ m['coef_x'], F @ m['coef_y']])


def pixel_to_world_homo(uv, H):
    uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
    h_pts = np.column_stack([uv, np.ones(len(uv))])
    out = (H @ h_pts.T).T
    return out[:, :2] / out[:, 2:3]


# -------------------------------------------------------------------------
# 4. Bird's-eye view (interpolation forward)
# -------------------------------------------------------------------------
def make_bev(img, poly_model, px_per_m=PX_PER_M, sample_step=4):
    h_, w_ = img.shape[:2]
    bev_w = int(TABLE_WIDTH_M * px_per_m)
    bev_h = int(TABLE_DEPTH_M * px_per_m)

    yy, xx = np.mgrid[0:h_:sample_step, 0:w_:sample_step]
    img_grid = np.column_stack([xx.ravel(), yy.ravel()]).astype(np.float64)
    world_grid = pixel_to_world_poly(img_grid, poly_model)
    keep = (np.abs(world_grid[:, 0]) < LX*1.4) & (np.abs(world_grid[:, 1]) < LY*1.4)
    world_grid = world_grid[keep]
    img_grid = img_grid[keep]

    bev_uu, bev_vv = np.meshgrid(np.arange(bev_w), np.arange(bev_h))
    target = np.column_stack([
        (-LX + bev_uu / px_per_m).ravel(),
        (+LY - bev_vv / px_per_m).ravel(),
    ])
    img_uv = griddata(world_grid, img_grid, target,
                      method='linear').reshape(bev_h, bev_w, 2)
    mx = img_uv[..., 0].astype(np.float32)
    my = img_uv[..., 1].astype(np.float32)
    mx[np.isnan(mx)] = -1
    my[np.isnan(my)] = -1
    return cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderValue=(0, 0, 0))


# -------------------------------------------------------------------------
# 5. Visualisations forward (rasterisation, sans inversion fragile)
# -------------------------------------------------------------------------
def _world_grid_per_pixel(img_shape, poly_model):
    """X(u,v), Y(u,v) sur tous les pixels par évaluation directe du poly."""
    h_, w_ = img_shape[:2]
    yy, xx = np.mgrid[0:h_, 0:w_]
    u = (xx - poly_model['u_mean']) / poly_model['u_std']
    v = (yy - poly_model['v_mean']) / poly_model['v_std']
    X = np.zeros_like(u, dtype=np.float64)
    Y = np.zeros_like(u, dtype=np.float64)
    k = 0
    for i in range(poly_model['deg']+1):
        for j in range(poly_model['deg']+1-i):
            term = (u**i) * (v**j)
            X += poly_model['coef_x'][k] * term
            Y += poly_model['coef_y'][k] * term
            k += 1
    return X, Y


def overlay_grid(img, poly_model, mask_inside,
                 step_fine=0.10, step_coarse=0.50):
    """Rasterise les lignes monde (X=cst, Y=cst) sur l'image."""
    X, Y = _world_grid_per_pixel(img.shape, poly_model)
    valid = mask_inside

    out = img.copy()
    # Fine grid (gris)
    for x_t in np.arange(-LX, LX+0.001, step_fine):
        out[(np.abs(X - x_t) < 0.005) & valid] = (200, 200, 200)
    for y_t in np.arange(-LY, LY+0.001, step_fine):
        out[(np.abs(Y - y_t) < 0.005) & valid] = (200, 200, 200)
    # Coarse grid (jaune, axes spéciaux)
    for x_t in np.arange(-LX, LX+0.001, step_coarse):
        near = (np.abs(X - x_t) < 0.012) & valid
        out[near] = (0, 200, 255) if abs(x_t) < 0.01 else (0, 255, 255)
    for y_t in np.arange(-LY, LY+0.001, step_coarse):
        near = (np.abs(Y - y_t) < 0.012) & valid
        out[near] = (0, 255, 200) if abs(y_t) < 0.01 else (0, 255, 255)
    # Périmètre rouge
    for x_t in (-LX, LX):
        out[(np.abs(X - x_t) < 0.020) & (np.abs(Y) < LY*1.05) & valid] = (0, 0, 255)
    for y_t in (-LY, LY):
        out[(np.abs(Y - y_t) < 0.020) & (np.abs(X) < LX*1.05) & valid] = (0, 0, 255)

    # Origine
    dist = X**2 + Y**2
    oy_, ox_ = np.unravel_index(np.argmin(dist), dist.shape)
    cv2.drawMarker(out, (int(ox_), int(oy_)), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
    cv2.putText(out, "(0,0)", (int(ox_)+15, int(oy_)+5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return out


# -------------------------------------------------------------------------
# Pipeline complet
# -------------------------------------------------------------------------
def calibrate(image_path, mask_path, out_dir):
    img = cv2.imread(str(image_path))
    mask_img = cv2.imread(str(mask_path))
    assert img is not None, f"Image introuvable: {image_path}"
    assert mask_img is not None, f"Mask introuvable: {mask_path}"
    if mask_img.shape[:2] != img.shape[:2]:
        mask_img = cv2.resize(mask_img, (img.shape[1], img.shape[0]))
    h, w = img.shape[:2]
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    mask, contour = extract_red_mask_contour(mask_img)
    corners, contour_s = find_corners(contour)
    sides = split_contour(contour_s, corners)
    img_pts, world_pts = build_correspondences(corners, sides)

    H = fit_homography(corners)
    pm = fit_poly(img_pts, world_pts, (w, h), deg=POLY_DEG)

    # Erreurs
    err_p = np.linalg.norm(pixel_to_world_poly(img_pts, pm) - world_pts, axis=1)
    err_h = np.linalg.norm(pixel_to_world_homo(img_pts, H) - world_pts, axis=1)

    # Sauvegardes
    # 02_corners
    vis_c = img.copy()
    cv2.drawContours(vis_c, [contour.astype(np.int32).reshape(-1, 1, 2)],
                     -1, (0, 255, 255), 2)
    for k_, p_ in corners.items():
        cv2.circle(vis_c, (int(p_[0]), int(p_[1])), 14, (0, 0, 255), 3)
        cv2.putText(vis_c, k_, (int(p_[0])+18, int(p_[1])-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.imwrite(str(Path(out_dir)/'02_corners.png'), vis_c)

    # 03 BEV propre + 04 BEV grid
    bev = make_bev(img, pm)
    cv2.imwrite(str(Path(out_dir)/'03_bev_clean.png'), bev)
    bev_g = bev.copy()
    bw, bh = bev_g.shape[1], bev_g.shape[0]
    for x_m in np.arange(-LX, LX+0.001, 0.10):
        px = int((x_m+LX)*PX_PER_M); cv2.line(bev_g, (px, 0), (px, bh), (0, 255, 255), 1)
    for y_m in np.arange(-LY, LY+0.001, 0.10):
        py = int((LY-y_m)*PX_PER_M); cv2.line(bev_g, (0, py), (bw, py), (0, 255, 255), 1)
    for x_m in np.arange(-LX, LX+0.001, 0.50):
        px = int((x_m+LX)*PX_PER_M); cv2.line(bev_g, (px, 0), (px, bh), (0, 255, 0), 2)
    for y_m in np.arange(-LY, LY+0.001, 0.50):
        py = int((LY-y_m)*PX_PER_M); cv2.line(bev_g, (0, py), (bw, py), (0, 255, 0), 2)
    cx_b, cy_b = int(LX*PX_PER_M), int(LY*PX_PER_M)
    cv2.drawMarker(bev_g, (cx_b, cy_b), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
    cv2.arrowedLine(bev_g, (cx_b, cy_b), (cx_b+100, cy_b), (0, 0, 255), 2, tipLength=0.3)
    cv2.putText(bev_g, "X", (cx_b+105, cy_b+5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.arrowedLine(bev_g, (cx_b, cy_b), (cx_b, cy_b-100), (0, 0, 255), 2, tipLength=0.3)
    cv2.putText(bev_g, "Y", (cx_b+5, cy_b-105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.rectangle(bev_g, (0, 0), (bw-1, bh-1), (0, 255, 0), 3)
    cv2.imwrite(str(Path(out_dir)/'04_bev_grid.png'), bev_g)

    # 05 overlay grille sur image originale
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    inside = cv2.dilate(mask, kernel) > 0
    over = overlay_grid(img, pm, inside)
    cv2.putText(over, f"Grille 10cm/50cm + perimetre 3x2m | poly deg={POLY_DEG} RMS {np.sqrt(np.mean(err_p**2))*1000:.1f}mm",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(str(Path(out_dir)/'05_overlay_grid.png'), over)

    # 06 cercles
    X, Y = _world_grid_per_pixel(img.shape, pm)
    R = np.sqrt(X**2 + Y**2)
    vis2 = img.copy()
    for radius in [0.25, 0.5, 0.75, 1.0]:
        vis2[(np.abs(R - radius) < 0.010) & inside] = (0, 255, 255)
    dist0 = X**2 + Y**2
    oy_, ox_ = np.unravel_index(np.argmin(dist0), dist0.shape)
    cv2.drawMarker(vis2, (int(ox_), int(oy_)), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
    cv2.putText(vis2, "Cercles 25/50/75/100 cm autour de (0,0)",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imwrite(str(Path(out_dir)/'06_test_cercles.png'), vis2)

    # JSON
    calib_data = {
        'image_size': [w, h],
        'rectangle_size_m': [TABLE_WIDTH_M, TABLE_DEPTH_M],
        'corners_pixel': {k: [float(v[0]), float(v[1])] for k, v in corners.items()},
        'corners_world_m': {
            'TL': [-LX, +LY], 'TR': [+LX, +LY],
            'BR': [+LX, -LY], 'BL': [-LX, -LY],
        },
        'homography_pixel_to_world': H.tolist(),
        'poly_model': {
            'deg': pm['deg'],
            'coef_x': pm['coef_x'].tolist(),
            'coef_y': pm['coef_y'].tolist(),
            'normalization': {
                'u_mean': pm['u_mean'], 'v_mean': pm['v_mean'],
                'u_std': pm['u_std'], 'v_std': pm['v_std'],
            },
        },
        'residuals_mm': {
            'poly_rms':       float(np.sqrt(np.mean(err_p**2))*1000),
            'poly_max':       float(err_p.max()*1000),
            'poly_median':    float(np.median(err_p)*1000),
            'homography_rms': float(np.sqrt(np.mean(err_h**2))*1000),
            'homography_max': float(err_h.max()*1000),
        },
        'note': "Calibration depuis red.mask.png (mask manuel précis). "
                "Origine au centre de la table, X horizontal (3 m), "
                "Y profondeur (2 m), Z=0 plan du sol.",
    }
    with open(Path(out_dir)/'calibration.json', 'w') as f:
        json.dump(calib_data, f, indent=2)
    return calib_data


# -------------------------------------------------------------------------
# CLI
# -------------------------------------------------------------------------
if __name__ == '__main__':
    import sys
    img_path  = sys.argv[1] if len(sys.argv) > 1 else 'capture.jpg'
    mask_path = sys.argv[2] if len(sys.argv) > 2 else 'red.mask.png'
    out_dir   = sys.argv[3] if len(sys.argv) > 3 else 'calib_output'
    c = calibrate(img_path, mask_path, out_dir)
    print(f"Erreurs (mm): poly RMS={c['residuals_mm']['poly_rms']:.2f}, "
          f"max={c['residuals_mm']['poly_max']:.2f} | "
          f"homography RMS={c['residuals_mm']['homography_rms']:.2f}")
    print(f"Coins pixel:")
    for k, v in c['corners_pixel'].items():
        print(f"  {k}: ({v[0]:.0f}, {v[1]:.0f})")
    print(f"Sortie : {out_dir}/")
