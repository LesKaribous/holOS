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

  // Update "PC" label to "Jetson" when server reports it's running on Jetson
  if (state.is_jetson !== undefined) {
    const lbl = document.getElementById('cg-lbl-client');
    if (lbl) lbl.textContent = state.is_jetson ? 'Jetson' : 'PC';
    if (typeof _ncSetClientLabel === 'function') _ncSetClientLabel(state.is_jetson);
  }

  if (activeView === 'map') render(state);
  renderMiniMap(state);
  updateMapSidebar(state);
  if (activeView === 'monitor') updateMonitor(state);

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
  if (_remoteDrawerOpen && state.robot) {
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
  const dot      = document.getElementById('cg-dot-robot');
  const link     = document.getElementById('cg-lnk-robot');
  const sub      = document.getElementById('cg-sub-robot');
  const modeEl   = document.getElementById('conn-mode-val');
  const serverSub = document.getElementById('cg-sub-server');

  if (mode === 'usb' || mode === 'xbee') {
    const label   = mode === 'xbee' ? 'XBee' : 'USB Wired';
    const modeStr = mode === 'xbee'
      ? `Hardware — XBee (${hwPort || '?'})`
      : `Hardware — USB Wired (${hwPort || '?'})`;
    if (dot)       dot.className  = 'cg-dot hw-active';
    if (link)      link.className = 'cg-link hw';
    if (sub)       { sub.className = 'cg-sub hw'; sub.textContent = label; }
    if (modeEl)    modeEl.textContent = modeStr;
    if (serverSub) serverSub.textContent = mode === 'xbee' ? 'via Jetson' : 'Local';
  } else if (mode === 'sim') {
    if (dot)       dot.className  = 'cg-dot sim-active';
    if (link)      link.className = 'cg-link sim';
    if (sub)       { sub.className = 'cg-sub sim'; sub.textContent = 'SIM'; }
    if (modeEl)    modeEl.textContent = 'Simulator';
    if (serverSub) serverSub.textContent = 'Local';
  } else {
    if (dot)       dot.className  = 'cg-dot';
    if (link)      link.className = 'cg-link';
    if (sub)       { sub.className = 'cg-sub'; sub.textContent = 'idle'; }
    if (modeEl)    modeEl.textContent = 'Not connected';
    if (serverSub) serverSub.textContent = 'Local';
  }
}

function cgSetRobot(hwConnected, hwType, hwPort) {
  cgSetConnectionMode(hwConnected ? (hwType === 'xbee' ? 'xbee' : 'usb') : 'idle', hwType, hwPort);
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
  } else if (name === 'vision') {
    onVisionViewActivated();
  } else if (name === 'actuators') {
    actInit();
  }

  socket.emit('vision_view_active', { active: name === 'vision' });
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
  ['wifi-panel','serial-panel','server-panel','robot-panel','color-popup']
    .forEach(i => document.getElementById(i)?.classList.add('hidden'));
  if (wasHidden) {
    el.classList.remove('hidden');
    if (id === 'robot-panel' || id === 'serial-panel') serialRefreshPorts();
  }
}

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
