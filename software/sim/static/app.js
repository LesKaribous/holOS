/**
 * app.js — holOS Dashboard orchestrator
 * Shared globals, canvas, socket core, connectivity graph, view switching.
 * All feature-specific code lives in static/js/*.js, loaded after this file.
 */
'use strict';

// ── Field constants ────────────────────────────────────────────────────────────
const FIELD_W   = 3000;
const FIELD_H   = 2000;
const GRID_W    = 20;
const GRID_H    = 13;
const GRID_CELL = 150;
// Visual-only: rotate robot.png so face-A aligns with actual heading.
const ROBOT_IMG_OFFSET = Math.PI + Math.PI / 6;

const COLOR_HEX = {
  UNKNOWN:'#888888', NONE:'#cccccc', RED:'#e03232', GREEN:'#28c228',
  BLUE:'#2828e0', YELLOW:'#e0c000', WHITE:'#f0f0f0', BLACK:'#222222',
};
const ALL_COLORS = Object.keys(COLOR_HEX);

const MOTION_CLASS = {
  IDLE:'state-idle', RUNNING:'state-running', PAUSED:'state-paused',
  CANCELED:'state-canceled', COMPLETED:'state-completed',
};

// ── POI categories ─────────────────────────────────────────────────────────────
const POI_CAT = {
  pantry:      { color:'#1a8c3c', label: p => p.replace('pantry_','P'),        r:9  },
  Fridge:      { color:'#2980b9', label: p => 'F:'+p.replace(/Fridge\w+_/,''), r:10 },
  stockYellow: { color:'#b7770d', label: p => p.replace('stockYellow_','SY'),   r:9  },
  stockBlue:   { color:'#1d4ed8', label: p => p.replace('stockBlue_','SB'),     r:9  },
  stockNinja:  { color:'#7c3aed', label: p => p.replace('stockNinja','N'),      r:8  },
  start:       { color:'#1a5276', label: p => p.replace('start','⚑'),          r:11 },
};

function poiCategory(name) {
  if (name.startsWith('pantry'))      return POI_CAT.pantry;
  if (name.startsWith('Fridge'))      return POI_CAT.Fridge;
  if (name.startsWith('stockYellow')) return POI_CAT.stockYellow;
  if (name.startsWith('stockBlue'))   return POI_CAT.stockBlue;
  if (name.startsWith('stockNinja'))  return POI_CAT.stockNinja;
  if (name.startsWith('start'))       return POI_CAT.start;
  return { color:'#888', label: n => n, r:7 };
}

// ── Render state ───────────────────────────────────────────────────────────────
let lastState      = null;
let colorPanelOpen = true;
let activePoi      = null;
let scale = 1, offX = 0, offY = 0;
let showRobotImage = true;
let terrainImg = null, robotImg = null;
let poiData = [];

(function loadAssets() {
  const t = new Image(); t.onload = () => { terrainImg = t; if (lastState) render(lastState); }; t.src = '/static/terrain.png';
  const r = new Image(); r.onload = () => { robotImg   = r; if (lastState) render(lastState); }; r.src = '/static/robot.png';
})();

fetch('/api/poi').then(r => r.json()).then(d => { poiData = d; if (lastState) render(lastState); });

// ── Canvas ─────────────────────────────────────────────────────────────────────
const canvas     = document.getElementById('field-canvas');
const ctx        = canvas.getContext('2d');
const miniCanvas = document.getElementById('mini-canvas');
const mctx       = miniCanvas.getContext('2d');

function resizeCanvas() {
  const wrap = document.getElementById('canvas-wrap');
  if (!wrap) return;
  const rect = wrap.getBoundingClientRect();
  const W = Math.round(rect.width);
  const H = Math.round(rect.height);
  if (W < 10 || H < 10) { requestAnimationFrame(resizeCanvas); return; }
  if (canvas.width !== W || canvas.height !== H) { canvas.width = W; canvas.height = H; }
  const pad = 10;
  scale = Math.min((W - pad*2) / FIELD_W, (H - pad*2) / FIELD_H);
  offX  = (W - FIELD_W * scale) / 2;
  offY  = (H - FIELD_H * scale) / 2;
  ctx.fillStyle = '#e8ecf0';
  ctx.fillRect(0, 0, W, H);
  ctx.strokeStyle = 'rgba(26,82,118,.35)';
  ctx.lineWidth   = Math.max(1.5, scale * 4);
  ctx.strokeRect(wx(0), wy(0), wlen(FIELD_W), wlen(FIELD_H));
  if (lastState) render(lastState);
}

window.addEventListener('resize', () => { if (activeView === 'map') resizeCanvas(); });
if (window.ResizeObserver) {
  const wrap = document.getElementById('canvas-wrap');
  if (wrap) new ResizeObserver(() => { if (activeView === 'map') resizeCanvas(); }).observe(wrap);
}

// ── Coordinate helpers ─────────────────────────────────────────────────────────
function wx(mm)   { return offX + mm * scale; }
function wy(mm)   { return offY + mm * scale; }
function wlen(mm) { return mm * scale; }
function canvasToWorld(cx, cy) { return { x: (cx - offX) / scale, y: (cy - offY) / scale }; }

// ── Utility functions ──────────────────────────────────────────────────────────
function escHtml(s) { return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;'); }
function setText(id, v) { const el = document.getElementById(id); if (el) el.textContent = v; }

function showToast(msg) {
  const t = document.getElementById('reload-toast'); if (!t) return;
  t.textContent = msg; t.classList.remove('hidden'); t.style.opacity = '1';
  setTimeout(() => { t.style.opacity = '0'; setTimeout(() => t.classList.add('hidden'), 400); }, 2200);
}

// ── SocketIO ───────────────────────────────────────────────────────────────────
const socket = io();

socket.on('connect',    () => { cgSetWs(true);  if (typeof _ns !== 'undefined') _ns.ws = 'connected';    if (activeView === 'network' && typeof _ncDraw === 'function') _ncDraw(); });
socket.on('disconnect', () => { cgSetWs(false); if (typeof _ns !== 'undefined') _ns.ws = 'disconnected'; if (activeView === 'network' && typeof _ncDraw === 'function') _ncDraw(); });

socket.on('state', state => {
  lastState = state;

  // Sync match running state for late-joining clients (state broadcast from server)
  if (state.match_running !== undefined && typeof _matchRunning !== 'undefined') {
    _matchRunning = state.match_running;
    if (typeof _updateStartStopUI === 'function') _updateStartStopUI();
  }

  // Mirror server-side team onto the topbar Y/B buttons. In HW mode this
  // reflects the team_get poll of the robot's physical switch (so the
  // operator can sanity-check whether the robot is reporting the right
  // team without unlocking anything). In sim mode it just keeps the
  // .active class consistent across reloads.
  if (state.team === 'yellow' || state.team === 'blue') {
    document.getElementById('btn-yellow')?.classList.toggle('active', state.team === 'yellow');
    document.getElementById('btn-blue')  ?.classList.toggle('active', state.team === 'blue');
  }

  // holOS subtitle reflects where the server is running. The check is
  // server-side (reads /proc/device-tree/model) so the browser doesn't
  // have to guess from window.location.
  if (state.is_jetson !== undefined) {
    const sub = document.getElementById('cg-sub-server');
    if (sub) sub.textContent = state.is_jetson ? 'jetson' : 'local';
    if (typeof _ncSetClientLabel === 'function') _ncSetClientLabel(state.is_jetson);
  }

  if (activeView === 'map') render(state);
  renderMiniMap(state);
  updateMapSidebar(state);

  // Sync pathfinding checkbox from server state
  const pfCb = document.getElementById('feat-pathfinding');
  if (pfCb && state.features && state.features.pathfinding !== undefined) {
    pfCb.checked = !!state.features.pathfinding;
  }

  if (state.hw_connecting) {
    cgSetRobotConnecting();
    _serialSetConnectBusy(true);
  } else if (state.connection_mode !== undefined) {
    cgSetConnectionMode(state.connection_mode, state.hw_type, state.hw_serial_port);
    _serialSetConnectBusy(false);
  }

  updateNetworkFromState(state);
  if (activeView === 'modules') updateModulesView(state);
  // remote.js is loaded after this file and lives in its own script scope
  // (because of 'use strict' + `let`), so reach for the global it exposes
  // on `window` instead of bare-name lookup.
  if (window._remoteDrawerOpen && state.robot) {
    remoteUpdateTheta(state.robot.theta * 180 / Math.PI);
  }
});

socket.on('reload', d => showToast(d.msg));

// ── Connectivity graph ─────────────────────────────────────────────────────────
let _currentConnectionMode = 'idle';

function cgSetWs(connected) {
  const dot  = document.getElementById('cg-dot-ws');
  const link = document.getElementById('cg-lnk-ws');
  const sv   = document.getElementById('socket-status-val');
  if (dot)  dot.className  = 'cg-dot ' + (connected ? 'connected' : '');
  if (link) link.className = 'cg-link ' + (connected ? 'active' : '');
  if (sv)   sv.textContent = connected ? 'Connected ✓' : 'Disconnected';
}

function cgSetConnectionMode(mode, hwType, hwPort) {
  _currentConnectionMode = mode;
  // Team selection is only legal in sim mode — on real hardware the team
  // comes from a physical switch. Disable the topbar Y/B buttons in HW.
  const isHw = (mode === 'usb' || mode === 'xbee' || mode === 'wifi');
  const yBtn = document.getElementById('btn-yellow');
  const bBtn = document.getElementById('btn-blue');
  const teamWrap = document.querySelector('.team-toggle');
  if (yBtn) yBtn.disabled = isHw;
  if (bBtn) bBtn.disabled = isHw;
  if (teamWrap) {
    teamWrap.title = isHw
      ? 'Team is set by the robot’s physical switch in HW mode'
      : 'Team color (sim only)';
    teamWrap.classList.toggle('disabled', isHw);
  }

  // Both robot-panel-sim and robot-panel-hw stay visible: the user can
  // start the sim bridge OR connect HW from this single popup. Update
  // the sim section's status badge so it reflects whether the bridge is
  // currently running.
  const simStatusEl = document.getElementById('sim-bridge-status');
  if (simStatusEl) {
    if (mode === 'sim') {
      simStatusEl.textContent = 'running';
      simStatusEl.style.color = 'var(--green)';
    } else {
      simStatusEl.textContent = 'idle';
      simStatusEl.style.color = 'var(--text-dim)';
    }
  }
  // Disable HW connect controls while sim is running (and vice versa) so
  // the user can't accidentally have both at once.
  const hwBtn = document.getElementById('serial-connect-btn');
  if (hwBtn) hwBtn.disabled = (mode === 'sim');
  const simStartBtn = document.getElementById('sim-bridge-start-btn');
  if (simStartBtn) simStartBtn.disabled = isHw;
  const dot      = document.getElementById('cg-dot-robot');
  const link     = document.getElementById('cg-lnk-robot');
  const sub      = document.getElementById('cg-sub-robot');
  const lbl      = document.getElementById('cg-lbl-robot');
  const modeEl   = document.getElementById('conn-mode-val');

  // Right-side node label flips: "Simulator" in sim mode, "Robot" otherwise.
  if (lbl) lbl.textContent = (mode === 'sim') ? 'Simulator' : 'Robot';

  // Xiao node — only "lit" when the active transport is the wifi bridge.
  // For xbee/usb the Xiao isn't in the chain at all, so we show it idle.
  const xDot = document.getElementById('cg-dot-xiao');
  const xSub = document.getElementById('cg-sub-xiao');
  const xLnk = document.getElementById('cg-lnk-xiao');
  if (mode === 'wifi') {
    if (xDot) xDot.className = 'cg-dot connected';
    if (xSub) { xSub.className = 'cg-sub hw'; xSub.textContent = 'TCP'; }
    if (xLnk) xLnk.className = 'cg-link hw';
  } else {
    if (xDot) xDot.className = 'cg-dot';
    if (xSub) { xSub.className = 'cg-sub'; xSub.textContent = '—'; }
    if (xLnk) xLnk.className = 'cg-link';
  }

  if (mode === 'usb' || mode === 'xbee' || mode === 'wifi') {
    const label   = mode === 'xbee' ? 'XBee'
                  : mode === 'wifi' ? 'WiFi (Xiao)'
                  : 'USB Wired';
    const modeStr = `Hardware — ${label} (${hwPort || '?'})`;
    if (dot)       dot.className  = 'cg-dot hw-active';
    if (link)      link.className = 'cg-link hw';
    if (sub)       { sub.className = 'cg-sub hw'; sub.textContent = label; }
    if (modeEl)    modeEl.textContent = modeStr;
  } else if (mode === 'sim') {
    if (dot)       dot.className  = 'cg-dot sim-active';
    if (link)      link.className = 'cg-link sim';
    if (sub)       { sub.className = 'cg-sub sim'; sub.textContent = 'SIM'; }
    if (modeEl)    modeEl.textContent = 'Simulator';
  } else {
    if (dot)       dot.className  = 'cg-dot';
    if (link)      link.className = 'cg-link';
    if (sub)       { sub.className = 'cg-sub'; sub.textContent = 'idle'; }
    if (modeEl)    modeEl.textContent = 'Not connected';
  }
}

function cgSetRobot(hwConnected, hwType, hwPort) {
  // Pass 'wifi' through; collapse any other hw type to 'usb' (legacy callers
  // sometimes pass an empty string or 'hw').
  const m = !hwConnected ? 'idle'
          : (hwType === 'xbee' || hwType === 'wifi') ? hwType
          : 'usb';
  cgSetConnectionMode(m, hwType, hwPort);
}

function cgSetRobotConnecting() {
  const dot = document.getElementById('cg-dot-robot');
  const sub = document.getElementById('cg-sub-robot');
  if (dot) dot.className = 'cg-dot connecting';
  if (sub) { sub.className = 'cg-sub'; sub.textContent = '…'; }
}

// Populate server address from current URL
(function() {
  const addrEl = document.getElementById('conn-server-addr');
  if (addrEl) addrEl.textContent = window.location.host;
})();

// Bootstrap graph state
cgSetWs(false);
cgSetRobot(false);

// ── View switching ─────────────────────────────────────────────────────────────
let activeView = 'map';

function toggleNavGroup(name) {
  document.getElementById('nav-group-' + name)?.classList.toggle('expanded');
}

function switchView(name, btn) {
  const previousView = activeView;
  activeView = name;
  document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
  document.querySelectorAll('.nav-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('view-' + name).classList.add('active');
  btn.classList.add('active');

  document.querySelectorAll('.nav-group-hdr').forEach(h => h.classList.remove('has-active'));
  const group = btn.closest?.('.nav-group');
  if (group) {
    group.classList.add('expanded');
    group.querySelector('.nav-group-hdr')?.classList.add('has-active');
  }

  const miniWrap = document.getElementById('mini-map-wrap');
  if (miniWrap) miniWrap.style.display = (name === 'map' || _miniMapHidden) ? 'none' : 'block';

  if (name === 'map') {
    requestAnimationFrame(() => requestAnimationFrame(resizeCanvas));
  } else if (name === 'strategy') {
    refreshStrategyList();
  } else if (name === 'missions') {
    onMissionsViewActivated();
  } else if (name === 'macros') {
    renderMacroList();
  } else if (name === 'terminal') {
    serialRefreshPorts();
  } else if (name === 'network') {
    initNetwork();
  } else if (name === 'modules') {
    initModulesView();
    if (lastState) updateModulesView(lastState);
  } else if (name === 'calibration') {
    onCalibViewActivated();
  } else if (name === 'vision-dashboard') {
    onVisionViewActivated();
  } else if (name === 'vision-detection') {
    if (typeof onVisionDetectionActivated === 'function') onVisionDetectionActivated();
  } else if (name === 'vision-editor') {
    onVisionEditorActivated();
  } else if (name === 'vision-objects') {
    onVisionObjectsActivated();
  } else if (name === 'actuators') {
    actInit();
  }

  // Any of the three vision tabs counts as "vision tab active" for the
  // backend's stream-client gating. Only emit on TRANSITIONS into / out of
  // the vision group so the server-side counter doesn't drift when the
  // user switches between the three subtabs.
  const wasVision = previousView && previousView.startsWith('vision-');
  const isVision  = name && name.startsWith('vision-');
  if (wasVision !== isVision) {
    socket.emit('vision_view_active', { active: isVision });
  }
}

// Mini-map click → go to map view
document.getElementById('mini-map-wrap').addEventListener('click', () => {
  const mapBtn = document.querySelector('[data-view=map]');
  if (mapBtn) switchView('map', mapBtn);
});

// Toggle mini-map visibility
let _miniMapHidden = false;
function toggleMiniMap(show) {
  _miniMapHidden = !show;
  const wrap = document.getElementById('mini-map-wrap');
  const btn  = document.getElementById('btn-mini-map-show');
  if (show) {
    // Only show if not on map view
    const onMap = document.querySelector('.view.active')?.id === 'view-map';
    wrap.style.display = onMap ? 'none' : 'block';
    btn.classList.add('hidden');
  } else {
    wrap.style.display = 'none';
    btn.classList.remove('hidden');
  }
}

// Close panels when clicking elsewhere
document.addEventListener('click', e => {
  ['server-panel','robot-panel','color-popup'].forEach(id => {
    const el = document.getElementById(id);
    if (el && !el.classList.contains('hidden')) {
      if (!el.contains(e.target) && !e.target.closest('#conn-graph')) {
        el.classList.add('hidden');
      }
    }
  });
  activePoi = null;
});

function togglePanel(id) {
  const el = document.getElementById(id); if (!el) return;
  const wasHidden = el.classList.contains('hidden');
  ['wifi-panel','serial-panel','server-panel','robot-panel','cam-panel','net-panel','esp-panel','color-popup']
    .forEach(i => document.getElementById(i)?.classList.add('hidden'));
  if (wasHidden) {
    el.classList.remove('hidden');
    if (id === 'robot-panel' || id === 'serial-panel') serialRefreshPorts();
  }
}

// ── FrameSource status poll ────────────────────────────────────────────────
// Calls /api/vision_camera/status every 2 s and reflects the in-process
// FrameSource state on the "Cam" pill in the topbar + the click-through
// details panel. URL kept for back-compat — backend now serves the
// FrameSource's status() dict (no PID/port, has frame_age_s + has_frame).
// Cam dot semantics (per user request, May 2026):
//   red    — frame source failed
//   gray   — frame source idle
//   orange — source is delivering frames but vision isn't fully calibrated
//            yet (no homography lock OR no theta sync)
//   green  — source ok + homography locked + heading offset synced (i.e.
//            the full vision-based localisation pipeline is usable)
// _vd_calibState is filled in by pollVisionCalibration() below.
let _vd_calibState = { homography_locked: false, heading_synced: false };

function _camDotClass(s) {
  if (s.state === 'failed')                 return 'cg-dot disconnected';
  if (s.state !== 'running' || !s.has_frame) return 'cg-dot';
  const c = _vd_calibState;
  if (c.homography_locked && c.heading_synced) return 'cg-dot connected';
  return 'cg-dot partial';
}
function _camSubLabel(s) {
  if (s.state === 'failed') return 'fail';
  if (s.state === 'idle')   return 'idle';
  if (!s.has_frame)         return 'wait';
  if (s.playback === 'pause') return 'paused';
  const c = _vd_calibState;
  if (!c.homography_locked) return 'no homog';
  if (!c.heading_synced)    return 'no sync';
  return 'live';
}
async function pollVisionCalibration() {
  try {
    const r = await fetch('/api/vision/calibration', {cache: 'no-store'});
    if (!r.ok) return;
    const c = await r.json();
    _vd_calibState = {
      homography_locked: !!(c.rectify && c.rectify.capture_phase === 'locked'),
      heading_synced:    c.heading_offset_rad !== null
                         && c.heading_offset_rad !== undefined,
    };
  } catch (e) { /* network blip — try again next tick */ }
}
setInterval(pollVisionCalibration, 2000);

async function pollVisionCamera() {
  try {
    const r = await fetch('/api/vision_camera/status', {cache: 'no-store'});
    if (!r.ok) return;
    const s = await r.json();
    const dot = document.getElementById('cg-dot-cam');
    const sub = document.getElementById('cg-sub-cam');
    if (dot) dot.className = _camDotClass(s);
    if (sub) sub.textContent = _camSubLabel(s);
    const node = document.getElementById('cg-node-cam');
    if (node) {
      const tip = s.last_error
        ? `${s.state} — ${s.last_error}`
        : `${s.state} · ${s.source_kind || '?'} → ${s.source_path || '?'}`;
      node.title = tip;
    }
    // Panel content
    const setText = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
    setText('cam-state-val',  s.state || '—');
    setText('cam-source-val', s.source_kind && s.source_path
                              ? `${s.source_kind} → ${s.source_path}` : '—');
    setText('cam-frame-val',
      s.has_frame
        ? `${s.frame_shape ? s.frame_shape.slice(0, 2).reverse().join('×') : '?'}` +
          ` · age ${s.frame_age_s != null ? (s.frame_age_s * 1000).toFixed(0) + ' ms' : '—'}`
        : '— (no frame yet)');
    setText('cam-idx-val',
      s.frame_count > 0 ? `${s.frame_idx} / ${s.frame_count}`
                        : `${s.frame_idx}`);
    setText('cam-jetson-val', s.on_jetson ? 'yes (GPU decode)' : 'no');
    const errRow = document.getElementById('cam-error-row');
    const errVal = document.getElementById('cam-error-val');
    if (s.last_error) {
      if (errRow) errRow.style.display = '';
      if (errVal) errVal.textContent = s.last_error;
    } else {
      if (errRow) errRow.style.display = 'none';
    }
    const ov = s.runtime_override || {};
    const ovEl = document.getElementById('cam-override-status');
    if (ovEl) {
      if (ov.source_kind || ov.source_path) {
        ovEl.textContent = `override: ${ov.source_kind || '?'} → ${ov.source_path || '?'}`;
        ovEl.style.color = 'var(--warn)';
      } else {
        ovEl.textContent = 'disk config';
        ovEl.style.color = 'var(--text-dim)';
      }
    }
  } catch (e) { /* network blip — try again next tick */ }
}
setInterval(pollVisionCamera, 2000);

// ── ESP32-CAM liveness poll ────────────────────────────────────────────────
// Single source of truth: services/embed_cam.py EspProber.
//   - Background thread does a TCP-connect probe every 15 s.
//   - Probe is skipped when a /capture has happened recently (the
//     capture itself proves the ESP is alive), so during active use
//     we add zero ESP traffic.
//   - Online = (last /capture in last 30 s succeeded) OR (last probe ok).
function _espDotClass(s) {
  if (!s) return 'cg-dot';
  if (s.online) return 'cg-dot connected';
  // Got a result and it failed → red. No result yet → neutral.
  if (s.last && s.last.duration_ms >= 0) return 'cg-dot disconnected';
  return 'cg-dot';
}
function _espSubLabel(s) {
  if (!s) return '—';
  if (s.online) return 'online';
  if (s.last && s.last.error === 'timeout') return 'offline (to)';
  if (s.last && s.last.duration_ms >= 0)    return 'offline';
  return '—';
}
function _fmtHms(t_ms) {
  if (!t_ms) return '—';
  const d = new Date(t_ms);
  const pad = n => String(n).padStart(2, '0');
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}
function _renderProbeHistory(history) {
  const tbody = document.getElementById('esp-probe-history-body');
  if (!tbody) return;
  if (!Array.isArray(history) || history.length === 0) {
    tbody.innerHTML = '<tr><td colspan="4" style="color:var(--text-dim);padding:2px 0">no probes yet…</td></tr>';
    return;
  }
  const rows = history.slice().reverse().slice(0, 8).map(p => {
    const time = _fmtHms(p.t_ms);
    const ms   = (p.duration_ms != null && p.duration_ms >= 0) ? String(p.duration_ms) : '—';
    const okTxt = p.ok ? '✓' : '✗';
    const okColor = p.ok ? 'var(--ok, #4ec46e)' : 'var(--err, #e25555)';
    const note = (p.error || p.kind || '').replace(/</g, '&lt;');
    return `<tr>
      <td style="padding:1px 4px 1px 0;color:var(--text-dim)">${time}</td>
      <td style="padding:1px 4px">${ms}</td>
      <td style="padding:1px 4px;color:${okColor};font-weight:bold">${okTxt}</td>
      <td style="padding:1px 0;color:var(--text-dim);overflow:hidden;text-overflow:ellipsis;max-width:160px;white-space:nowrap" title="${note}">${note}</td>
    </tr>`;
  }).join('');
  tbody.innerHTML = rows;
}
async function pollEspHealth() {
  try {
    const r = await fetch('/api/embed_cam/health', {cache: 'no-store'});
    if (!r.ok) return;
    const j = await r.json();
    if (!j.ok) return;
    const s = j.status || {};
    const last = s.last || {};

    // Topbar pill
    const dot = document.getElementById('cg-dot-esp');
    const sub = document.getElementById('cg-sub-esp');
    if (dot) dot.className = _espDotClass(s);
    if (sub) sub.textContent = _espSubLabel(s);
    const node = document.getElementById('cg-node-esp');
    if (node) {
      const probeTxt = last.t_ms
        ? `probe ${last.ok ? 'ok' : 'fail'} · ${last.duration_ms >= 0 ? last.duration_ms+'ms' : '—'}` +
          (last.error ? ` · ${last.error}` : '')
        : 'no probe yet';
      const capTxt = (s.last_capture_age_s != null && s.last_capture_age_s >= 0)
        ? `last /capture ${s.last_capture_ok ? 'ok' : 'fail'} ${s.last_capture_age_s.toFixed(0)}s ago`
        : 'no /capture yet';
      node.title = `${s.online ? 'ONLINE' : 'OFFLINE'}\n${probeTxt}\n${capTxt}`;
    }

    // Panel
    const setText = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
    setText('esp-online-val', s.online ? 'online' : 'offline');
    if (last.t_ms) {
      const okTxt = last.ok ? 'ok' : 'fail';
      setText('esp-probe-last-val',
        `${_fmtHms(last.t_ms)} · ${okTxt} · ${last.duration_ms >= 0 ? last.duration_ms+' ms' : '—'}`);
    } else {
      setText('esp-probe-last-val', '—');
    }
    setText('esp-probe-host-val', last.host ? `${last.host}:${last.port}` : '—');
    if (s.last_capture_age_s != null && s.last_capture_age_s >= 0) {
      setText('esp-cap-age-val',
        `${s.last_capture_ok ? 'ok' : 'fail'} · ${s.last_capture_age_s.toFixed(1)} s ago`);
    } else {
      setText('esp-cap-age-val', '—');
    }
    const perr = document.getElementById('esp-probe-error-row');
    const perv = document.getElementById('esp-probe-error-val');
    if (last.error) {
      if (perr) perr.style.display = '';
      if (perv) perv.textContent = last.error;
    } else {
      if (perr) perr.style.display = 'none';
    }
    _renderProbeHistory(s.history);
  } catch (e) { /* network blip — try again next tick */ }
}
setInterval(pollEspHealth, 2000);

function espHealthAction(action) {
  fetch('/api/embed_cam/health', {
    method:  'POST',
    headers: {'Content-Type': 'application/json'},
    body:    JSON.stringify({action: action}),
  })
  .then(r => r.json())
  .then(() => pollEspHealth())
  .catch(() => {});
}
window.espHealthAction = espHealthAction;
window.addEventListener('load', pollVisionCamera);

async function camSetRuntimeSource() {
  const kind = document.getElementById('cam-src-kind').value;
  const path = document.getElementById('cam-src-path').value.trim();
  if (!path) {
    document.getElementById('cam-override-status').textContent = 'path required';
    return;
  }
  try {
    const r = await fetch('/api/vision_camera/source', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({source_kind: kind, source_path: path}),
    });
    const j = await r.json();
    if (!j.ok) {
      document.getElementById('cam-override-status').textContent = 'error: ' + (j.error || 'unknown');
    }
  } catch (e) {
    document.getElementById('cam-override-status').textContent = 'error: ' + e;
  }
  pollVisionCamera();
}

async function camClearRuntimeSource() {
  try {
    await fetch('/api/vision_camera/source/clear', {method: 'POST'});
  } catch (e) {
    document.getElementById('cam-override-status').textContent = 'error: ' + e;
  }
  pollVisionCamera();
}

// ── Network stats: ping every 10 s, derive RTT + throughput ─────────────────
// The browser sends a SocketIO 'net_ping' with a high-res timestamp. The
// server echoes back its cumulative SocketIO byte/event counters. RTT is
// computed locally; throughput + event rate are deltas between successive
// pongs. Useful when "the link feels slow" — a colleague hogging the
// LAN shows up here as a 200 ms+ RTT before anything else breaks.
let _netLastBytes  = null;
let _netLastEvents = null;
let _netLastT      = 0;
let _netRttMs      = null;
let _netKbps       = null;
let _netEps        = null;

function _netDotClass(rtt) {
  if (rtt == null)  return 'cg-dot';
  if (rtt < 50)     return 'cg-dot connected';
  if (rtt < 200)    return 'cg-dot connecting';
  return 'cg-dot disconnected';
}

function _renderNetStats() {
  const dot = document.getElementById('cg-dot-net');
  const sub = document.getElementById('cg-sub-net');
  if (dot) dot.className = _netDotClass(_netRttMs);
  if (sub) {
    sub.textContent = (_netRttMs == null) ? '—' : `${Math.round(_netRttMs)} ms`;
  }
  const setText = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
  setText('net-rtt-val',  _netRttMs  == null ? '—' : `${_netRttMs.toFixed(1)} ms`);
  setText('net-thp-val',  _netKbps   == null ? '—' :
    _netKbps > 1024 ? `${(_netKbps/1024).toFixed(2)} MB/s` : `${_netKbps.toFixed(1)} kB/s`);
  setText('net-eps-val',  _netEps    == null ? '—' : `${_netEps.toFixed(1)} /s`);
  setText('net-last-val', _netLastT  ? new Date().toLocaleTimeString() : '—');
}

socket.on('net_pong', (d) => {
  if (typeof d?.t_client_ms === 'number') {
    _netRttMs = performance.now() - d.t_client_ms;
  }
  const now = performance.now() / 1000;
  if (_netLastT > 0 && _netLastBytes != null) {
    const dt = now - _netLastT;
    if (dt > 0) {
      _netKbps = (d.bytes_emitted  - _netLastBytes ) / 1024 / dt;
      _netEps  = (d.events_emitted - _netLastEvents) / dt;
    }
  }
  _netLastBytes  = d.bytes_emitted;
  _netLastEvents = d.events_emitted;
  _netLastT      = now;
  _renderNetStats();
});

function netPing() {
  if (!socket || !socket.connected) {
    _netRttMs = null;
    _renderNetStats();
    return;
  }
  socket.emit('net_ping', { t_client_ms: performance.now() });
}
setInterval(netPing, 10000);
window.addEventListener('load', () => setTimeout(netPing, 500));

// ── Init ───────────────────────────────────────────────────────────────────────
window.addEventListener('load', () => {
  document.getElementById('mini-map-wrap').style.display = 'none';
  requestAnimationFrame(() => requestAnimationFrame(resizeCanvas));
  cgSetWs(false);
  cgSetRobot(false);

  const fr = document.getElementById('feedrate');
  if (fr) document.getElementById('feedrate-val').textContent = parseFloat(fr.value).toFixed(2) + '×';

  initCodeMirror();
  loadMacros();
  loadCppBlocks();
  serialRefreshPorts();
  initTests();
});
