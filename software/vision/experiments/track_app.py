"""
track_app.py - App Tkinter pour le tracking ArUco
==================================================

Lancement :
    python track_app.py video.mkv

Fonctionnalites :
  - Lecture video avec play/pause
  - Frame-by-frame fiable (gere les keyframes MPEG-4)
  - Panneau de preprocessing toggle-able (CLAHE, blur, sharpen, gamma, etc.)
    -> le tracking s'execute sur l'image preprocessee
  - Panneau camera : override manuel de (Xc, Yc, Zc)
  - Calibration "from known position" : entre la vraie position d'un robot
    visible, deduit (Xc, Yc) automatiquement
  - Bird's-eye view en temps reel (etendu de Y=-1500 a Y=3500 pour voir
    les robots stationnes hors-table)
  - Sauvegarde de frame composite (PNG)

Raccourcis clavier :
    Espace        play/pause
    Left/Right    +/-1 frame
    Shift+L/R     +/-10 frames
    Ctrl+L/R      +/-100 frames
    Home/End      first / last frame
    s             save snapshot
    d             toggle debug
    r             reset trails
    q / Esc       quitter
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import cv2
import numpy as np
from PIL import Image, ImageTk
from pathlib import Path
import argparse
import sys
import time

from robot_tracker import (
    TrackerState, Preprocessor,
    draw_annotated, draw_bev,
    solve_camera_xy_from_known_position,
    TAGS_FIXED, TAGS_ROBOT, Z_ROBOT_MM,
    TABLE_WIDTH_MM, TABLE_DEPTH_MM,
)


# ---------------------------------------------------------------------------
# Wrapper VideoCapture avec seek frame-exact
# ---------------------------------------------------------------------------
class FrameAccurateVideo:
    """
    Encapsule cv2.VideoCapture pour garantir un positionnement exact
    par frame, en compensant les imprecisions des codecs avec keyframes.

    cap.set(POS_FRAMES) snap souvent a la keyframe precedente. On lit
    ensuite vers l'avant pour atteindre la frame demandee.
    """
    def __init__(self, path):
        self.path = path
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise IOError(f"Impossible d'ouvrir {path}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.n_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.idx = -1
        self._last_frame = None
        self._last_known_frame = self.n_frames - 1

    def read_next(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None
        self.idx = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1
        self._last_frame = frame
        return self.idx, frame

    def goto(self, target):
        target = max(0, min(self._last_known_frame, target))
        if target == self.idx and self._last_frame is not None:
            return self.idx, self._last_frame
        if target == self.idx + 1:
            return self.read_next()

        seek_back = 0
        last_frame = None
        last_idx = -1
        for attempt in range(4):
            seek_to = max(0, target - seek_back)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, seek_to)
            last_frame = None
            last_idx = -1
            budget = max(400, (target - seek_to) + 100)
            for _ in range(budget):
                ret, frame = self.cap.read()
                if not ret:
                    break
                pos = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
                last_idx = pos - 1
                last_frame = frame
                if last_idx >= target:
                    break
            if last_idx == target and last_frame is not None:
                self.idx = last_idx
                self._last_frame = last_frame
                return self.idx, last_frame
            if last_idx > target:
                seek_back += max(60, (last_idx - target) * 2 + 30)
                continue
            if last_idx < 0 or last_frame is None:
                seek_back += 300
                continue
            self._last_known_frame = min(self._last_known_frame, last_idx)
            self.idx = last_idx
            self._last_frame = last_frame
            return self.idx, last_frame
        if last_idx >= 0 and last_frame is not None:
            self.idx = last_idx
            self._last_frame = last_frame
            return self.idx, last_frame
        return None, None

    def release(self):
        self.cap.release()


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
class TrackingApp:
    def __init__(self, video_path):
        self.video = FrameAccurateVideo(video_path)
        self.tracker = TrackerState((self.video.w, self.video.h))
        self.preproc = Preprocessor()

        self.playing = False
        self._after_id = None
        self._show_debug = True

        self._photo_main = None
        self._photo_bev = None
        self._photo_pre = None
        self._save_dir = Path('snapshots')
        self._raw = None
        self._processed = None
        self._tags = {}

        self.root = tk.Tk()
        self.root.title(f"Robot Tracker - {Path(video_path).name}")
        self.root.geometry("1700x950")
        try:
            self.root.state('zoomed')
        except Exception:
            pass

        self._build_ui()
        self._bind_keys()
        self.root.after(50, lambda: self.goto_frame(0))

    def _build_ui(self):
        self.root.columnconfigure(1, weight=3)
        self.root.rowconfigure(0, weight=1)

        left = ttk.Frame(self.root, padding=4)
        left.grid(row=0, column=0, sticky="ns", rowspan=2)
        self._build_settings_panel(left)

        center = ttk.Frame(self.root)
        center.grid(row=0, column=1, sticky="nsew")
        center.rowconfigure(0, weight=1)
        center.columnconfigure(0, weight=1)

        self.canvas_main = tk.Canvas(center, bg='black', highlightthickness=0)
        self.canvas_main.grid(row=0, column=0, sticky="nsew")
        self.canvas_main.bind("<Configure>", lambda e: self._render())

        ctrl = ttk.Frame(center, padding=4)
        ctrl.grid(row=1, column=0, sticky="ew")
        self._build_controls(ctrl)

        right = ttk.Frame(self.root, padding=4)
        right.grid(row=0, column=2, sticky="ns", rowspan=2)
        self._build_right_panel(right)

    def _build_settings_panel(self, parent):
        ttk.Label(parent, text="Pre-processing", font=("Helvetica", 11, "bold")).pack(anchor='w', pady=(0,4))

        self.var_grayscale = tk.BooleanVar(value=False)
        ttk.Checkbutton(parent, text="Grayscale", variable=self.var_grayscale,
                        command=self._on_settings_changed).pack(anchor='w')

        self.var_clahe = tk.BooleanVar(value=False)
        f = ttk.LabelFrame(parent, text="CLAHE", padding=4); f.pack(fill='x', pady=2)
        ttk.Checkbutton(f, text="On", variable=self.var_clahe,
                        command=self._on_settings_changed).pack(anchor='w')
        self.var_clahe_clip = tk.DoubleVar(value=3.0)
        ttk.Label(f, text="clip:").pack(anchor='w')
        ttk.Scale(f, from_=1, to=10, variable=self.var_clahe_clip,
                  orient='horizontal', command=self._on_settings_changed).pack(fill='x')
        self.var_clahe_tile = tk.IntVar(value=8)
        ttk.Label(f, text="tile:").pack(anchor='w')
        ttk.Scale(f, from_=2, to=32, variable=self.var_clahe_tile,
                  orient='horizontal', command=self._on_settings_changed).pack(fill='x')

        self.var_blur = tk.BooleanVar(value=False)
        f = ttk.LabelFrame(parent, text="Gaussian blur", padding=4); f.pack(fill='x', pady=2)
        ttk.Checkbutton(f, text="On", variable=self.var_blur,
                        command=self._on_settings_changed).pack(anchor='w')
        self.var_blur_sigma = tk.DoubleVar(value=1.5)
        ttk.Label(f, text="sigma:").pack(anchor='w')
        ttk.Scale(f, from_=0.3, to=10, variable=self.var_blur_sigma,
                  orient='horizontal', command=self._on_settings_changed).pack(fill='x')

        self.var_bilat = tk.BooleanVar(value=False)
        f = ttk.LabelFrame(parent, text="Bilateral (denoise)", padding=4); f.pack(fill='x', pady=2)
        ttk.Checkbutton(f, text="On", variable=self.var_bilat,
                        command=self._on_settings_changed).pack(anchor='w')
        self.var_bilat_d = tk.IntVar(value=9)
        ttk.Label(f, text="d:").pack(anchor='w')
        ttk.Scale(f, from_=3, to=21, variable=self.var_bilat_d,
                  orient='horizontal', command=self._on_settings_changed).pack(fill='x')

        self.var_unsharp = tk.BooleanVar(value=False)
        f = ttk.LabelFrame(parent, text="Unsharp mask", padding=4); f.pack(fill='x', pady=2)
        ttk.Checkbutton(f, text="On", variable=self.var_unsharp,
                        command=self._on_settings_changed).pack(anchor='w')
        self.var_unsharp_amount = tk.DoubleVar(value=1.0)
        ttk.Label(f, text="amount:").pack(anchor='w')
        ttk.Scale(f, from_=0, to=3, variable=self.var_unsharp_amount,
                  orient='horizontal', command=self._on_settings_changed).pack(fill='x')

        self.var_bright = tk.BooleanVar(value=False)
        f = ttk.LabelFrame(parent, text="Brightness / contrast", padding=4); f.pack(fill='x', pady=2)
        ttk.Checkbutton(f, text="On", variable=self.var_bright,
                        command=self._on_settings_changed).pack(anchor='w')
        self.var_alpha = tk.DoubleVar(value=1.0)
        ttk.Label(f, text="alpha (contrast):").pack(anchor='w')
        ttk.Scale(f, from_=0.3, to=3.0, variable=self.var_alpha,
                  orient='horizontal', command=self._on_settings_changed).pack(fill='x')
        self.var_beta = tk.DoubleVar(value=0.0)
        ttk.Label(f, text="beta (brightness):").pack(anchor='w')
        ttk.Scale(f, from_=-100, to=100, variable=self.var_beta,
                  orient='horizontal', command=self._on_settings_changed).pack(fill='x')

        self.var_gamma = tk.BooleanVar(value=False)
        f = ttk.LabelFrame(parent, text="Gamma", padding=4); f.pack(fill='x', pady=2)
        ttk.Checkbutton(f, text="On", variable=self.var_gamma,
                        command=self._on_settings_changed).pack(anchor='w')
        self.var_gamma_val = tk.DoubleVar(value=1.0)
        ttk.Scale(f, from_=0.3, to=3, variable=self.var_gamma_val,
                  orient='horizontal', command=self._on_settings_changed).pack(fill='x')

        ttk.Separator(parent).pack(fill='x', pady=8)

        ttk.Label(parent, text="Camera position", font=("Helvetica", 11, "bold")).pack(anchor='w', pady=(0,4))
        self.var_cam_mode = tk.StringVar(value="auto")
        ttk.Radiobutton(parent, text="Auto (solvePnP)", variable=self.var_cam_mode,
                        value="auto", command=self._on_cam_changed).pack(anchor='w')
        ttk.Radiobutton(parent, text="Manual override", variable=self.var_cam_mode,
                        value="manual", command=self._on_cam_changed).pack(anchor='w')

        f = ttk.LabelFrame(parent, text="Manual (mm)", padding=4); f.pack(fill='x', pady=2)
        self.var_cam_x = tk.DoubleVar(value=1500.0)
        self.var_cam_y = tk.DoubleVar(value=4000.0)
        self.var_cam_z = tk.DoubleVar(value=1000.0)
        for label, var, vmin, vmax in [
            ("Xc", self.var_cam_x, -3000, 6000),
            ("Yc", self.var_cam_y, -3000, 7000),
            ("Zc", self.var_cam_z, 100, 5000),
        ]:
            row = ttk.Frame(f); row.pack(fill='x', pady=1)
            ttk.Label(row, text=f"{label}:", width=4).pack(side='left')
            entry = ttk.Entry(row, textvariable=var, width=8)
            entry.pack(side='left')
            entry.bind('<Return>', lambda e: self._on_cam_changed())
            ttk.Scale(row, from_=vmin, to=vmax, variable=var, orient='horizontal',
                      command=lambda e: self._on_cam_changed()).pack(side='left', fill='x', expand=True)

        ttk.Separator(parent).pack(fill='x', pady=8)
        ttk.Label(parent, text="Tag height (mm)", font=("Helvetica", 10, "bold")).pack(anchor='w')
        self.var_z_obj = tk.DoubleVar(value=Z_ROBOT_MM)
        row = ttk.Frame(parent); row.pack(fill='x')
        e = ttk.Entry(row, textvariable=self.var_z_obj, width=8)
        e.pack(side='left')
        e.bind('<Return>', lambda ev: self._on_cam_changed())
        ttk.Scale(row, from_=0, to=1500, variable=self.var_z_obj, orient='horizontal',
                  command=lambda e: self._on_cam_changed()).pack(side='left', fill='x', expand=True)

        ttk.Separator(parent).pack(fill='x', pady=8)

        f = ttk.LabelFrame(parent, text="Calibrate cam from known robot pos", padding=4)
        f.pack(fill='x', pady=2)
        ttk.Label(f, text="Tag id:").grid(row=0, column=0, sticky='w')
        self.var_calib_tagid = tk.IntVar(value=2)
        ttk.Combobox(f, textvariable=self.var_calib_tagid,
                     values=[2, 7], width=4).grid(row=0, column=1, sticky='w')
        ttk.Label(f, text="Real X (mm):").grid(row=1, column=0, sticky='w')
        self.var_calib_X = tk.DoubleVar(value=150.0)
        ttk.Entry(f, textvariable=self.var_calib_X, width=10).grid(row=1, column=1)
        ttk.Label(f, text="Real Y (mm):").grid(row=2, column=0, sticky='w')
        self.var_calib_Y = tk.DoubleVar(value=2850.0)
        ttk.Entry(f, textvariable=self.var_calib_Y, width=10).grid(row=2, column=1)
        ttk.Button(f, text="Solve cam (Xc, Yc) from this position",
                   command=self.calibrate_from_known).grid(row=3, column=0, columnspan=2,
                                                            sticky='ew', pady=2)
        ttk.Label(f, text="(Uses Zc from slider above)",
                  foreground='#888').grid(row=4, column=0, columnspan=2)

        ttk.Separator(parent).pack(fill='x', pady=8)
        ttk.Button(parent, text="Reset trails (r)", command=self.reset_trails).pack(fill='x')
        ttk.Button(parent, text="Save snapshot (s)", command=self.save_snapshot).pack(fill='x', pady=2)

    def _build_right_panel(self, parent):
        f = ttk.LabelFrame(parent, text="Debug", padding=4)
        f.pack(fill='both', expand=False)
        self.txt_debug = tk.Text(f, width=42, height=20,
                                 bg='#101010', fg='#E0E0E0',
                                 font=("Consolas", 9), wrap='none')
        self.txt_debug.pack(fill='both', expand=True)
        self.txt_debug.tag_configure("ok",     foreground="#4FE36C")
        self.txt_debug.tag_configure("warn",   foreground="#FFA040")
        self.txt_debug.tag_configure("err",    foreground="#FF5050")
        self.txt_debug.tag_configure("title",  foreground="#FFFFFF", font=("Consolas", 10, "bold"))
        self.txt_debug.tag_configure("user",   foreground="#4FE36C")
        self.txt_debug.tag_configure("opp",    foreground="#FF5050")
        self.txt_debug.tag_configure("dim",    foreground="#888888")

        f2 = ttk.LabelFrame(parent, text="Bird's-eye view", padding=2)
        f2.pack(fill='both', expand=True, pady=(6, 0))
        self.canvas_bev = tk.Canvas(f2, bg='black', width=620, height=620,
                                    highlightthickness=0)
        self.canvas_bev.pack(fill='both', expand=True)

        f3 = ttk.LabelFrame(parent, text="Pre-processed (live)", padding=2)
        f3.pack(fill='x', pady=(6, 0))
        self.canvas_pre = tk.Canvas(f3, bg='black', width=320, height=180,
                                    highlightthickness=0)
        self.canvas_pre.pack()

    def _build_controls(self, parent):
        ttk.Button(parent, text="|<<", width=4, command=lambda: self.step(-100)).pack(side='left', padx=2)
        ttk.Button(parent, text="<<",  width=4, command=lambda: self.step(-10)).pack(side='left', padx=2)
        ttk.Button(parent, text="<",   width=4, command=lambda: self.step(-1)).pack(side='left', padx=2)
        self.btn_play = ttk.Button(parent, text="Play", width=8, command=self.toggle_play)
        self.btn_play.pack(side='left', padx=2)
        ttk.Button(parent, text=">",   width=4, command=lambda: self.step(1)).pack(side='left', padx=2)
        ttk.Button(parent, text=">>",  width=4, command=lambda: self.step(10)).pack(side='left', padx=2)
        ttk.Button(parent, text=">>|", width=4, command=lambda: self.step(100)).pack(side='left', padx=2)

        self.var_slider = tk.IntVar(value=0)
        self.scale = ttk.Scale(parent, from_=0, to=self.video.n_frames-1,
                               variable=self.var_slider, orient='horizontal',
                               command=self._on_slider_drag)
        self.scale.pack(side='left', fill='x', expand=True, padx=8)
        self.lbl_frame = ttk.Label(parent, text=f"0 / {self.video.n_frames-1}", width=20)
        self.lbl_frame.pack(side='left')

    def _bind_keys(self):
        r = self.root
        r.bind("<space>",        lambda e: self.toggle_play())
        r.bind("<Left>",         lambda e: self.step(-1))
        r.bind("<Right>",        lambda e: self.step(1))
        r.bind("<Shift-Left>",   lambda e: self.step(-10))
        r.bind("<Shift-Right>",  lambda e: self.step(10))
        r.bind("<Control-Left>", lambda e: self.step(-100))
        r.bind("<Control-Right>",lambda e: self.step(100))
        r.bind("<Home>",         lambda e: self.goto_frame(0))
        r.bind("<End>",          lambda e: self.goto_frame(self.video.n_frames - 1))
        r.bind("s",              lambda e: self.save_snapshot())
        r.bind("d",              lambda e: self._toggle_debug())
        r.bind("r",              lambda e: self.reset_trails())
        r.bind("q",              lambda e: self._quit())
        r.bind("<Escape>",       lambda e: self._quit())
        r.protocol("WM_DELETE_WINDOW", self._quit)

    def _sync_preproc(self):
        o = self.preproc.opts
        o['grayscale']['on']     = self.var_grayscale.get()
        o['clahe']['on']         = self.var_clahe.get()
        o['clahe']['clip']       = float(self.var_clahe_clip.get())
        o['clahe']['tile']       = int(self.var_clahe_tile.get())
        o['gaussian_blur']['on'] = self.var_blur.get()
        o['gaussian_blur']['sigma'] = float(self.var_blur_sigma.get())
        o['bilateral']['on']     = self.var_bilat.get()
        o['bilateral']['d']      = int(self.var_bilat_d.get())
        o['unsharp']['on']       = self.var_unsharp.get()
        o['unsharp']['amount']   = float(self.var_unsharp_amount.get())
        o['brightness']['on']    = self.var_bright.get()
        o['brightness']['alpha'] = float(self.var_alpha.get())
        o['brightness']['beta']  = float(self.var_beta.get())
        o['gamma']['on']         = self.var_gamma.get()
        o['gamma']['value']      = float(self.var_gamma_val.get())

    def _sync_camera(self):
        if self.var_cam_mode.get() == "manual":
            self.tracker.camera_override = np.array([
                float(self.var_cam_x.get()),
                float(self.var_cam_y.get()),
                float(self.var_cam_z.get()),
            ])
        else:
            self.tracker.camera_override = None
        self.tracker.z_object = float(self.var_z_obj.get())

    def _on_settings_changed(self, *args):
        self._sync_preproc()
        self._reprocess_current()

    def _on_cam_changed(self, *args):
        self._sync_camera()
        self._reprocess_current()

    def _toggle_debug(self):
        self._show_debug = not self._show_debug
        self._render()

    def reset_trails(self):
        self.tracker.reset_trails()
        self._render()

    def calibrate_from_known(self):
        tid = int(self.var_calib_tagid.get())
        if tid not in self.tracker.last_naive:
            messagebox.showwarning("Calibration",
                f"Tag {tid} non detecte sur cette frame.\n"
                "Aller a une frame ou il est visible.")
            return
        naive = self.tracker.last_naive[tid]
        real = (float(self.var_calib_X.get()), float(self.var_calib_Y.get()))
        Zc = float(self.var_cam_z.get())
        z_obj = float(self.var_z_obj.get())
        cam = solve_camera_xy_from_known_position(naive, real, Zc, z_obj)
        if cam is None:
            messagebox.showerror("Calibration", "Echec (Zc trop proche de Z_obj?)")
            return
        self.var_cam_x.set(round(float(cam[0]), 1))
        self.var_cam_y.set(round(float(cam[1]), 1))
        self.var_cam_z.set(round(float(cam[2]), 1))
        self.var_cam_mode.set("manual")
        self._on_cam_changed()
        messagebox.showinfo("Calibration",
            f"Camera calibree:\n"
            f"  Xc = {cam[0]:.0f} mm\n"
            f"  Yc = {cam[1]:.0f} mm\n"
            f"  Zc = {cam[2]:.0f} mm\n"
            "(Mode 'manual override' active)")

    def goto_frame(self, idx):
        idx, frame = self.video.goto(idx)
        if frame is None:
            return
        self._raw = frame
        self._reprocess_current()

    def step(self, delta):
        self._stop_play()
        self.goto_frame(self.video.idx + delta)

    def _on_slider_drag(self, _val):
        target = int(float(_val))
        if target != self.video.idx:
            self._stop_play()
            self.goto_frame(target)

    def toggle_play(self):
        if self.playing:
            self._stop_play()
        else:
            self.playing = True
            self.btn_play.config(text="Pause")
            self._play_loop()

    def _stop_play(self):
        self.playing = False
        self.btn_play.config(text="Play")
        if self._after_id is not None:
            try: self.root.after_cancel(self._after_id)
            except Exception: pass
            self._after_id = None

    def _play_loop(self):
        if not self.playing:
            return
        t0 = time.time()
        idx, frame = self.video.read_next()
        if frame is None:
            self._stop_play()
            return
        self._raw = frame
        self._reprocess_current()
        elapsed_ms = (time.time() - t0) * 1000
        delay = max(1, int(1000.0/self.video.fps - elapsed_ms))
        self._after_id = self.root.after(delay, self._play_loop)

    def _reprocess_current(self):
        if self._raw is None:
            return
        self._sync_preproc()
        self._sync_camera()
        self._processed = self.preproc.apply(self._raw)
        self._tags = self.tracker.process(self._processed)
        self._render()

    def _render(self):
        if self._processed is None:
            return

        ann = draw_annotated(self._processed, self._tags, self.tracker)
        status = (f"frame {self.video.idx} / {self.video.n_frames-1} "
                  f"({self.video.idx/self.video.fps:.2f}s) | "
                  f"{'PLAY' if self.playing else 'PAUSE'} | "
                  f"calib={self.tracker.calib_source}")
        cv2.putText(ann, status, (10, ann.shape[0]-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        cw = max(self.canvas_main.winfo_width(), 320)
        ch = max(self.canvas_main.winfo_height(), 240)
        scale = min(cw / ann.shape[1], ch / ann.shape[0])
        new_w = int(ann.shape[1] * scale)
        new_h = int(ann.shape[0] * scale)
        if new_w > 0 and new_h > 0:
            disp = cv2.resize(ann, (new_w, new_h))
            self._photo_main = self._cv_to_photo(disp)
            self.canvas_main.delete("all")
            self.canvas_main.create_image(cw//2, ch//2, image=self._photo_main, anchor='center')

        bev = draw_bev(self.tracker, extended_y=True)
        self._photo_bev = self._cv_to_photo(bev)
        self.canvas_bev.delete("all")
        bw = self.canvas_bev.winfo_width()
        bh = self.canvas_bev.winfo_height()
        self.canvas_bev.create_image(bw//2, bh//2, image=self._photo_bev, anchor='center')

        thumb = cv2.resize(self._processed, (320, 180))
        self._photo_pre = self._cv_to_photo(thumb)
        self.canvas_pre.delete("all")
        self.canvas_pre.create_image(160, 90, image=self._photo_pre, anchor='center')

        try:
            self.var_slider.set(self.video.idx)
        except Exception: pass
        self.lbl_frame.config(text=f"{self.video.idx} / {self.video.n_frames-1}")

        if self._show_debug:
            self._update_debug_text()

    def _update_debug_text(self):
        t = self.txt_debug
        t.delete(1.0, tk.END)
        st = self.tracker
        ts = self.video.idx / self.video.fps
        t.insert(tk.END, f"FRAME {self.video.idx} ({ts:.2f}s)\n", "title")
        t.insert(tk.END, f"FPS: {self.video.fps:.2f}  Total: {self.video.n_frames}\n\n", "dim")

        seen = sorted(st.tags_seen)
        t.insert(tk.END, f"Tags seen ({len(seen)}): {seen}\n")
        fixed_seen = sorted(t_ for t_ in seen if t_ in TAGS_FIXED)
        if len(fixed_seen) == 4:
            t.insert(tk.END, f"  fixed (4/4): {fixed_seen}\n", "ok")
        else:
            t.insert(tk.END, f"  fixed ({len(fixed_seen)}/4): {fixed_seen}\n", "warn")

        if st.calib_source == "fresh":
            t.insert(tk.END, f"  calib: FRESH\n", "ok")
        elif st.calib_source == "cached":
            t.insert(tk.END, f"  calib: CACHED (age {st.calib_age})\n", "warn")
        else:
            t.insert(tk.END, f"  calib: NONE\n", "err")

        if st.cam_pos is not None:
            Xc, Yc, Zc = st.cam_pos
            mode = "AUTO" if st.camera_override is None else "OVERRIDE"
            t.insert(tk.END, f"\nCamera ({mode}):\n", "title")
            t.insert(tk.END, f"  X={Xc:.0f}  Y={Yc:.0f}  Z={Zc:.0f} mm\n")
            if st.fx_auto:
                t.insert(tk.END, f"  fx_auto={st.fx_auto:.0f} px\n", "dim")
        else:
            t.insert(tk.END, "\nCamera: ---\n", "err")

        t.insert(tk.END, f"\nTag Z height: {st.z_object:.0f} mm\n\n", "dim")
        for tid, (label, _) in TAGS_ROBOT.items():
            tag = "user" if tid == 2 else "opp"
            pos = st.last_positions.get(tid)
            naive = st.last_naive.get(tid)
            if pos is not None:
                t.insert(tk.END, f"{label} (id={tid})\n", tag)
                t.insert(tk.END, f"  pos:    ({pos[0]:7.0f}, {pos[1]:7.0f}) mm\n")
                t.insert(tk.END, f"  naive:  ({naive[0]:7.0f}, {naive[1]:7.0f}) mm\n", "dim")
                dx = pos[0] - naive[0]; dy = pos[1] - naive[1]
                t.insert(tk.END, f"  d_par.: ({dx:+7.0f}, {dy:+7.0f}) mm\n", "dim")
            else:
                t.insert(tk.END, f"{label} (id={tid}): not seen\n", "dim")

        t.insert(tk.END, "\nPreproc actifs:\n", "title")
        active = [k for k, v in self.preproc.opts.items() if v.get('on')]
        if active:
            for k in active:
                t.insert(tk.END, f"  - {k}\n")
        else:
            t.insert(tk.END, "  (aucun)\n", "dim")

    @staticmethod
    def _cv_to_photo(img_bgr):
        rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        return ImageTk.PhotoImage(Image.fromarray(rgb))

    def save_snapshot(self):
        if self._processed is None: return
        self._save_dir.mkdir(exist_ok=True)
        ann = draw_annotated(self._processed, self._tags, self.tracker)
        bev = draw_bev(self.tracker, extended_y=True)
        h = max(ann.shape[0], bev.shape[0])
        ann_p = cv2.copyMakeBorder(ann, 0, h-ann.shape[0], 0, 0,
                                    cv2.BORDER_CONSTANT, value=(0,0,0))
        bev_p = cv2.copyMakeBorder(bev, 0, h-bev.shape[0], 0, 0,
                                    cv2.BORDER_CONSTANT, value=(0,0,0))
        comp = np.hstack([ann_p, bev_p])
        out = self._save_dir / f"frame_{self.video.idx:06d}.png"
        cv2.imwrite(str(out), comp)
        print(f"[SAVED] {out}")
        try:
            messagebox.showinfo("Saved", f"Frame saved to {out}")
        except Exception:
            pass

    def _quit(self):
        self._stop_play()
        try: self.video.release()
        except Exception: pass
        try: self.root.destroy()
        except Exception: pass
        sys.exit(0)

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('video', nargs='?', help='chemin video (.mkv, .mp4)')
    args = parser.parse_args()

    video_path = args.video
    if not video_path:
        root = tk.Tk(); root.withdraw()
        video_path = filedialog.askopenfilename(
            title="Choisir une video",
            filetypes=[("Videos", "*.mkv *.mp4 *.avi *.mov"),
                       ("All", "*.*")])
        root.destroy()
        if not video_path:
            print("Aucune video selectionnee")
            return

    if not Path(video_path).exists():
        print(f"Fichier introuvable: {video_path}") 
        return

    app = TrackingApp(video_path)
    app.run()


if __name__ == '__main__':
    main()
    if not video_path:
        print("Aucune video selectionnee") 

    if not Path(video_path).exists():
        print(f"Fichier introuvable: {video_path}") 

    app = TrackingApp(video_path)
    app.run()