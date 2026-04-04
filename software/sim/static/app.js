/**
 * app.js — holOS Dashboard
 * Canvas rendering + Strategy/Macro management + SocketIO
 */
'use strict';

// ── Field constants ────────────────────────────────────────────────────────
const FIELD_W   = 3000;
const FIELD_H   = 2000;
const GRID_W    = 20;
const GRID_H    = 13;
const GRID_CELL = 150;
// Visual-only: rotate robot.png so face-A aligns with actual heading.
// Negate to −Math.PI/6 if correction goes the wrong way.
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

// ── POI categories ─────────────────────────────────────────────────────────
const POI_CAT = {
  pantry:      { color:'#1a8c3c', label: p => p.replace('pantry_','P'),       r:9  },
  Fridge:      { color:'#2980b9', label: p => 'F:'+p.replace(/Fridge\w+_/,''), r:10 },
  stockYellow: { color:'#b7770d', label: p => p.replace('stockYellow_','SY'),  r:9  },
  stockBlue:   { color:'#1d4ed8', label: p => p.replace('stockBlue_','SB'),    r:9  },
  stockNinja:  { color:'#7c3aed', label: p => p.replace('stockNinja','N'),     r:8  },
  start:       { color:'#1a5276', label: p => p.replace('start','⚑'),         r:11 },
};
function poiCategory(name) {
  if (name.startsWith('pantry'))       return POI_CAT.pantry;
  if (name.startsWith('Fridge'))       return POI_CAT.Fridge;
  if (name.startsWith('stockYellow'))  return POI_CAT.stockYellow;
  if (name.startsWith('stockBlue'))    return POI_CAT.stockBlue;
  if (name.startsWith('stockNinja'))   return POI_CAT.stockNinja;
  if (name.startsWith('start'))        return POI_CAT.start;
  return { color:'#888', label: n => n, r:7 };
}

// ── Render state ──────────────────────────────────────────────────────────
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

// ── Canvas (main + mini) ───────────────────────────────────────────────────
const canvas    = document.getElementById('field-canvas');
const ctx       = canvas.getContext('2d');
const miniCanvas = document.getElementById('mini-canvas');
const mctx       = miniCanvas.getContext('2d');

function resizeCanvas() {
  const wrap = document.getElementById('canvas-wrap');
  if (!wrap) return;

  // getBoundingClientRect() reads the actual rendered CSS dimensions —
  // the most reliable method regardless of flex / absolute / replaced-element quirks.
  const rect = wrap.getBoundingClientRect();
  const W = Math.round(rect.width);
  const H = Math.round(rect.height);

  if (W < 10 || H < 10) {
    requestAnimationFrame(resizeCanvas);
    return;
  }

  // Sync pixel buffer to rendered size (only when changed to avoid extra clear)
  if (canvas.width !== W || canvas.height !== H) {
    canvas.width  = W;
    canvas.height = H;
  }

  const pad = 10;
  scale = Math.min((W - pad*2) / FIELD_W, (H - pad*2) / FIELD_H);
  offX  = (W - FIELD_W * scale) / 2;
  offY  = (H - FIELD_H * scale) / 2;

  // Always draw something so the canvas is never invisibly blank
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

// ── Coordinate helpers ────────────────────────────────────────────────────
function wx(mm)   { return offX + mm * scale; }
function wy(mm)   { return offY + mm * scale; }
function wlen(mm) { return mm * scale; }
function canvasToWorld(cx, cy) {
  return { x: (cx - offX) / scale, y: (cy - offY) / scale };
}

// ── SocketIO ──────────────────────────────────────────────────────────────
const socket = io();
socket.on('connect',    () => { cgSetWs(true);  _ns.ws = 'connected';    if (activeView==='network') _ncDraw(); });
socket.on('disconnect', () => { cgSetWs(false); _ns.ws = 'disconnected'; if (activeView==='network') _ncDraw(); });
socket.on('state', state => {
  lastState = state;
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
  // Update remote dial whenever the drawer is open
  if (_remoteDrawerOpen && state.robot) {
    remoteUpdateTheta(state.robot.theta * 180 / Math.PI);
  }
});
socket.on('reload', d => showToast(d.msg));
socket.on('terminal_rx', d => appendTermLine(d.cmd, d.ok, d.res, d._fire || false, d.mode));
socket.on('seq_done', d => {
  document.querySelectorAll('.macro-step').forEach(el => el.classList.remove('running'));
  if (!d.stopped) showToast('Macro done ✓');
});

// ══════════════════════════════════════════════════════════════════════════
//  CONNECTIVITY GRAPH  (PC ── holOS ── Robot)
// ══════════════════════════════════════════════════════════════════════════

/**
 *  Node states:
 *    dot classes : connected | connecting | hw-active | sim-active  (+ none = gray)
 *    link classes: active | hw | sim  (+ none = gray)
 *    sub classes : hw | sim  (+ none = default white)
 *
 *  Three nodes:
 *    cg-dot-ws     / cg-lnk-ws    : PC ↔ holOS (WebSocket)
 *    cg-dot-server (always green)
 *    cg-lnk-robot  / cg-dot-robot : holOS ↔ Robot
 */

function cgSetWs(connected) {
  const dot  = document.getElementById('cg-dot-ws');
  const link = document.getElementById('cg-lnk-ws');
  const sv   = document.getElementById('socket-status-val');
  if (dot)  { dot.className  = 'cg-dot ' + (connected ? 'connected' : ''); }
  if (link) { link.className = 'cg-link ' + (connected ? 'active' : ''); }
  if (sv)   { sv.textContent  = connected ? 'Connected ✓' : 'Disconnected'; }
}

// connection_mode: 'idle' | 'sim' | 'usb' | 'xbee'
let _currentConnectionMode = 'idle';

function cgSetConnectionMode(mode, hwType, hwPort) {
  _currentConnectionMode = mode;
  const dot    = document.getElementById('cg-dot-robot');
  const link   = document.getElementById('cg-lnk-robot');
  const sub    = document.getElementById('cg-sub-robot');
  const modeEl = document.getElementById('conn-mode-val');
  const serverSub = document.getElementById('cg-sub-server');

  if (mode === 'usb' || mode === 'xbee') {
    const label = mode === 'xbee' ? 'XBee' : 'USB Wired';
    const modeStr = mode === 'xbee'
      ? `Hardware — XBee (${hwPort || '?'})`
      : `Hardware — USB Wired (${hwPort || '?'})`;
    if (dot)    { dot.className  = 'cg-dot hw-active'; }
    if (link)   { link.className = 'cg-link hw'; }
    if (sub)    { sub.className  = 'cg-sub hw'; sub.textContent = label; }
    if (modeEl) { modeEl.textContent = modeStr; }
    if (serverSub) serverSub.textContent = mode === 'xbee' ? 'via Jetson' : 'Local';
  } else if (mode === 'sim') {
    if (dot)    { dot.className  = 'cg-dot sim-active'; }
    if (link)   { link.className = 'cg-link sim'; }
    if (sub)    { sub.className  = 'cg-sub sim'; sub.textContent = 'SIM'; }
    if (modeEl) { modeEl.textContent = 'Simulator'; }
    if (serverSub) serverSub.textContent = 'Local';
  } else {
    // idle — nothing connected
    if (dot)    { dot.className  = 'cg-dot'; }
    if (link)   { link.className = 'cg-link'; }
    if (sub)    { sub.className  = 'cg-sub'; sub.textContent = 'idle'; }
    if (modeEl) { modeEl.textContent = 'Not connected'; }
    if (serverSub) serverSub.textContent = 'Local';
  }
}

// Legacy wrapper
function cgSetRobot(hwConnected, hwType, hwPort) {
  cgSetConnectionMode(hwConnected ? (hwType === 'xbee' ? 'xbee' : 'usb') : 'idle', hwType, hwPort);
}

function cgSetRobotConnecting() {
  const dot  = document.getElementById('cg-dot-robot');
  const sub  = document.getElementById('cg-sub-robot');
  if (dot)  dot.className = 'cg-dot connecting';
  if (sub)  { sub.className = 'cg-sub'; sub.textContent = '…'; }
}

// Populate server address from current URL
(function() {
  const addrEl = document.getElementById('conn-server-addr');
  if (addrEl) addrEl.textContent = window.location.host;
})();

// Bootstrap graph state
cgSetWs(false);
cgSetRobot(false);

// ── View switching ────────────────────────────────────────────────────────
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

  // If the button is a child inside a group, expand that group + highlight its header
  document.querySelectorAll('.nav-group-hdr').forEach(h => h.classList.remove('has-active'));
  const group = btn.closest?.('.nav-group');
  if (group) {
    group.classList.add('expanded');
    group.querySelector('.nav-group-hdr')?.classList.add('has-active');
  }

  const miniWrap = document.getElementById('mini-map-wrap');
  if (miniWrap) miniWrap.style.display = (name === 'map') ? 'none' : 'block';

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

  // Notify server about vision view focus (controls frame push rate)
  socket.emit('vision_view_active', { active: name === 'vision' });
}

// Mini-map click → go to map view
document.getElementById('mini-map-wrap').addEventListener('click', () => {
  const mapBtn = document.querySelector('[data-view=map]');
  if (mapBtn) switchView('map', mapBtn);
});

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
  const el = document.getElementById(id);
  if (!el) return;
  const wasHidden = el.classList.contains('hidden');
  // Close all panels
  ['wifi-panel','serial-panel','server-panel','robot-panel','wifi-panel','color-popup']
    .forEach(i => document.getElementById(i)?.classList.add('hidden'));
  if (wasHidden) {
    el.classList.remove('hidden');
    if (id === 'robot-panel' || id === 'serial-panel') serialRefreshPorts();
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  MAIN CANVAS RENDERING
// ══════════════════════════════════════════════════════════════════════════
function render(s) {
  ctx.fillStyle = '#e8ecf0';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawField(ctx, wx, wy, wlen, s);
  if (s.features.show_grid)  drawGrid(s);
  if (s.features.show_trail) drawTrail(s);
  if (s.features.show_path)  drawPath(s);
  drawOpponent(s);
  drawGameObjects(s);
  drawPOIs(ctx, wx, wy, wlen, poiData);
  drawMapTraj();
  drawMapPin();
  drawRobot(s);
}

// ── Mini-map renderer ──────────────────────────────────────────────────────
function renderMiniMap(s) {
  const MW = miniCanvas.width, MH = miniCanvas.height;
  const ms  = Math.min(MW / FIELD_W, MH / FIELD_H);
  const mox = (MW - FIELD_W * ms) / 2;
  const moy = (MH - FIELD_H * ms) / 2;
  const mwx = mm => mox + mm * ms;
  const mwy = mm => moy + (FIELD_H - mm) * ms;
  const mwl = mm => mm * ms;

  mctx.fillStyle = '#c8d8c8';
  mctx.fillRect(0, 0, MW, MH);
  // Field rect
  mctx.fillStyle = terrainImg ? 'transparent' : '#bdc8b4';
  if (terrainImg) mctx.drawImage(terrainImg, mwx(0), mwy(FIELD_H), mwl(FIELD_W), mwl(FIELD_H));
  mctx.strokeStyle = 'rgba(26,82,118,.5)';
  mctx.lineWidth = 1;
  mctx.strokeRect(mwx(0), mwy(FIELD_H), mwl(FIELD_W), mwl(FIELD_H));
  // POIs
  for (const p of poiData) {
    const cat = poiCategory(p.name);
    mctx.beginPath();
    mctx.arc(mwx(p.x), mwy(p.y), Math.max(1.5, mwl(6)), 0, Math.PI*2);
    mctx.fillStyle = cat.color + 'aa';
    mctx.fill();
  }
  // Path
  if (s.features.show_path && s.path && s.path.length > 1) {
    mctx.strokeStyle = 'rgba(41,128,185,.6)';
    mctx.lineWidth = 1;
    mctx.setLineDash([3,2]);
    mctx.beginPath();
    mctx.moveTo(mwx(s.path[0][0]), mwy(s.path[0][1]));
    for (let i = 1; i < s.path.length; i++) mctx.lineTo(mwx(s.path[i][0]), mwy(s.path[i][1]));
    mctx.stroke();
    mctx.setLineDash([]);
  }
  // Robot
  const r = s.robot;
  const rcx = mwx(r.x), rcy = mwy(r.y);
  mctx.beginPath();
  mctx.arc(rcx, rcy, Math.max(3, mwl(r.radius * 0.6)), 0, Math.PI*2);
  mctx.fillStyle = s.team === 'yellow' ? '#d4ac0d' : '#2980b9';
  mctx.fill();
  mctx.strokeStyle = 'rgba(255,255,255,.8)';
  mctx.lineWidth = 1;
  mctx.stroke();
  // Heading line
  const arr = mwl(r.radius * 1.0);
  mctx.strokeStyle = '#fff';
  mctx.lineWidth = 1.5;
  mctx.beginPath();
  mctx.moveTo(rcx, rcy);
  mctx.lineTo(rcx + arr * Math.cos(-r.theta), rcy + arr * Math.sin(-r.theta));
  mctx.stroke();
}

// ── Shared field drawing (reused by main + potentially prints) ─────────────
function drawField(c, wxf, wyf, wlf, s) {
  const fx = wxf(0), fy = wyf(0), fw = wlf(FIELD_W), fh = wlf(FIELD_H);
  if (terrainImg) {
    c.drawImage(terrainImg, fx, fy, fw, fh);
    c.fillStyle = 'rgba(0,0,0,0.15)';
    c.fillRect(fx, fy, fw, fh);
  } else {
    c.fillStyle = '#c8d4be';
    c.fillRect(fx, fy, fw, fh);
    c.fillStyle = 'rgba(180,140,0,.09)';
    c.fillRect(wxf(0), wyf(0), wlf(1500), wlf(FIELD_H));
    c.fillStyle = 'rgba(30,80,200,.06)';
    c.fillRect(wxf(1500), wyf(0), wlf(1500), wlf(FIELD_H));
  }
  c.strokeStyle = 'rgba(26,82,118,.5)';
  c.lineWidth = Math.max(1.5, wlf(4));
  c.strokeRect(fx, fy, fw, fh);
}

function drawGrid(s) {
  ctx.strokeStyle = 'rgba(16, 49, 70, 0.75)'; ctx.lineWidth = .5;
  for (let gx=0; gx<=GRID_W; gx++) {
    ctx.beginPath(); ctx.moveTo(wx(gx*GRID_CELL), wy(0)); ctx.lineTo(wx(gx*GRID_CELL), wy(FIELD_H)); ctx.stroke();
  }
  for (let gy=0; gy<=GRID_H; gy++) {
    ctx.beginPath(); ctx.moveTo(wx(0), wy(gy*GRID_CELL)); ctx.lineTo(wx(FIELD_W), wy(gy*GRID_CELL)); ctx.stroke();
  }
  for (const c of s.occupancy) {
    ctx.fillStyle = (c.layer === 'dynamic')
      ? 'rgba(255, 140, 0, 0.82)'    // dynamic obstacles — orange
      : 'rgba(41, 128, 185, 0.72)';  // static obstacles  — blue
    ctx.fillRect(wx(c.gx*GRID_CELL)+1, wy(c.gy*GRID_CELL)+1, wlen(GRID_CELL)-2, wlen(GRID_CELL)-2);
  }
}

function drawOpponent(s) {
  const opp = s.opponent;
  if (!opp || !opp.enabled) return;
  const cx = wx(opp.x), cy = wy(opp.y), r = wlen(120);
  // Body circle
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(231,76,60,.2)'; ctx.fill();
  ctx.strokeStyle = '#e74c3c'; ctx.lineWidth = 2; ctx.stroke();
  // Direction arrow
  const th = opp.theta || 0;
  const dx = Math.cos(th) * r, dy = -Math.sin(th) * r;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + dx, cy + dy);
  ctx.strokeStyle = '#e74c3c'; ctx.lineWidth = 2; ctx.stroke();
  // Label
  ctx.fillStyle = '#e74c3c'; ctx.font = '11px monospace'; ctx.textAlign = 'center';
  ctx.fillText('OPP', cx, cy - r - 4);
  ctx.textAlign = 'start';
}

function drawPath(s) {
  if (!s.path || s.path.length < 2) return;
  ctx.strokeStyle='rgba(41,128,185,.7)'; ctx.lineWidth=2; ctx.setLineDash([6,4]);
  ctx.beginPath(); ctx.moveTo(wx(s.path[0][0]), wy(s.path[0][1]));
  for (let i=1; i<s.path.length; i++) ctx.lineTo(wx(s.path[i][0]), wy(s.path[i][1]));
  ctx.stroke();
  for (let i=1; i<s.path.length-1; i++) {
    const px=wx(s.path[i][0]), py=wy(s.path[i][1]), d=wlen(7);
    ctx.fillStyle='rgba(41,128,185,.8)';
    ctx.beginPath(); ctx.moveTo(px,py-d); ctx.lineTo(px+d,py); ctx.lineTo(px,py+d); ctx.lineTo(px-d,py); ctx.closePath(); ctx.fill();
  }
  ctx.setLineDash([]);
}

function drawTrail(s) {
  const tr = s.robot.trail; if (!tr || tr.length < 2) return;
  for (let i=1; i<tr.length; i++) {
    const a = i/tr.length;
    ctx.strokeStyle=`rgba(41,128,185,${a*.3})`; ctx.lineWidth=1.5;
    ctx.beginPath(); ctx.moveTo(wx(tr[i-1][0]),wy(tr[i-1][1])); ctx.lineTo(wx(tr[i][0]),wy(tr[i][1])); ctx.stroke();
  }
}

function drawPOIs(c, wxf, wyf, wlf, pois) {
  for (const p of pois) {
    const px=wxf(p.x), py=wyf(p.y), cat=poiCategory(p.name), r=wlf(cat.r);
    c.beginPath(); c.arc(px,py,r,0,Math.PI*2);
    c.fillStyle=cat.color+'22'; c.fill();
    c.strokeStyle=cat.color+'cc'; c.lineWidth=1.5; c.stroke();
    if (wlf(1) > 0.09) {
      c.fillStyle=cat.color; c.font=`bold ${Math.max(9,Math.round(wlf(18)))}px monospace`;
      c.textAlign='center'; c.shadowColor='rgba(255,255,255,.8)'; c.shadowBlur=3;
      c.fillText(cat.label(p.name), px, py-r-2); c.shadowBlur=0;
    }
  }
}

function drawGameObjects(s) {
  for (const o of s.game_objs) {
    if (o.color==='UNKNOWN'||o.color==='NONE') continue;
    const hex=COLOR_HEX[o.color]||'#888', px=wx(o.x), py=wy(o.y), r=wlen(25);
    ctx.beginPath(); ctx.arc(px,py,r,0,Math.PI*2);
    ctx.fillStyle=hex+'cc'; ctx.fill();
    ctx.strokeStyle=hex; ctx.lineWidth=2; ctx.stroke();
  }
}

function drawRobot(s) {
  const r=s.robot, cx=wx(r.x), cy=wy(r.y), rad=wlen(r.radius), th=-r.theta;
  ctx.beginPath(); ctx.arc(cx,cy,rad,0,Math.PI*2);
  ctx.strokeStyle=r.collided?'rgba(192,57,43,.85)':'rgba(26,82,118,.28)';
  ctx.lineWidth=1.5; ctx.stroke();
  ctx.save(); ctx.translate(cx,cy); ctx.rotate(th);
  if (showRobotImage && robotImg) {
    ctx.rotate(ROBOT_IMG_OFFSET);
    if (r.collided) ctx.globalAlpha=0.55;
    const sz=rad*2.1; ctx.drawImage(robotImg, -sz/2,-sz/2, sz,sz);
    ctx.globalAlpha=1;
  } else {
    const rs=rad*.85;
    ctx.beginPath(); ctx.moveTo(rs,0); ctx.lineTo(-rs*.6,rs*.87); ctx.lineTo(-rs*.6,-rs*.87); ctx.closePath();
    ctx.fillStyle=r.collided?'#e74c3c':(s.team==='yellow'?'#d4ac0d':'#2980b9');
    ctx.fill(); ctx.strokeStyle='rgba(255,255,255,.6)'; ctx.lineWidth=1.5; ctx.stroke();
    ctx.strokeStyle='#fff'; ctx.lineWidth=2;
    ctx.beginPath(); ctx.moveTo(rs*.3,0); ctx.lineTo(rs*.95,0);
    const ar=rs*.15; ctx.lineTo(rs*.85,ar*.5); ctx.moveTo(rs*.95,0); ctx.lineTo(rs*.85,-ar*.5); ctx.stroke();
  }
  ctx.restore();
  if (s.safety.enabled && s.safety.detected) {
    ctx.beginPath(); ctx.arc(cx,cy,rad+wlen(15),0,Math.PI*2);
    ctx.strokeStyle='rgba(180,100,0,.7)'; ctx.lineWidth=3; ctx.setLineDash([5,4]); ctx.stroke(); ctx.setLineDash([]);
  }
}

// ── Map sidebar updates ────────────────────────────────────────────────────
let prevLog='[]', prevObjs='[]';
function updateMapSidebar(s) {
  const r=s.robot;
  setText('s-pos',   `(${r.x.toFixed(0)}, ${r.y.toFixed(0)})`);
  setText('s-theta', `${(r.theta*180/Math.PI).toFixed(1)}°`);
  setText('s-speed', `${Math.hypot(r.vx,r.vy).toFixed(0)} mm/s`);
  const me=document.getElementById('s-mstate');
  if (me) { me.textContent=s.motion.state; me.className='status-val '+(MOTION_CLASS[s.motion.state]||''); }
  const left=s.chrono.left, total=left+s.chrono.elapsed;
  setText('s-time',  `${left.toFixed(1)} s`);
  setText('s-score', `${s.score} pts`);
  const pct=total>0?left/total*100:100;
  const bar=document.getElementById('chrono-bar');
  if (bar) { bar.style.width=pct+'%'; bar.style.background=left<10?'#c0392b':left<30?'#b7770d':'#2980b9'; }
  // Log
  const lk=JSON.stringify(s.log.length);
  if (lk!==prevLog) { prevLog=lk;
    const logEl=document.getElementById('log-output');
    if (logEl) logEl.innerHTML=[...s.log].reverse().map(l=>{
      let c=''; if(l.includes('✓')||l.includes('SUCCESS'))c='ok'; else if(l.includes('✗')||l.includes('error'))c='err'; else if(l.includes('Stall')||l.includes('warn'))c='warn';
      return `<div class="log-line ${c}">${escHtml(l)}</div>`;
    }).join('');
  }
  updateColorPanel(s.game_objs);
}

function updateColorPanel(objs) {
  const key=JSON.stringify(objs.map(o=>o.color));
  if (key===prevObjs) return; prevObjs=key;
  const list=document.getElementById('color-list');
  if (!list) return;
  list.innerHTML=objs.map(o=>{
    const cat=poiCategory(o.name);
    return `<div class="color-item" onclick="openColorPopup(event,'${o.name}')">
      <div class="color-dot" style="background:${COLOR_HEX[o.color]||'#888'}"></div>
      <span class="color-name" style="color:${cat.color}">${cat.label(o.name)}</span></div>`;
  }).join('');
}

function openColorPopup(e, poi) {
  e.stopPropagation(); activePoi=poi;
  const pop=document.getElementById('color-popup');
  const ch =document.getElementById('color-choices');
  const ttl=document.getElementById('color-popup-title');
  if (!pop||!ch||!ttl) return;
  ttl.textContent='Color: '+poi;
  ch.innerHTML=ALL_COLORS.map(c=>`<div class="color-choice" onclick="applyColor('${c}')">
    <div class="color-dot" style="background:${COLOR_HEX[c]}"></div><span>${c}</span></div>`).join('');
  const rect=e.currentTarget.getBoundingClientRect();
  pop.style.left=`${rect.right+6}px`; pop.style.top=`${Math.min(rect.top,window.innerHeight-260)}px`;
  pop.classList.remove('hidden');
}
function applyColor(c) {
  if (!activePoi) return;
  socket.emit('set_color',{name:activePoi,color:c});
  document.getElementById('color-popup')?.classList.add('hidden'); activePoi=null;
}
function toggleColorPanel() {
  colorPanelOpen=!colorPanelOpen;
  const l=document.getElementById('color-list'); if (l) l.style.display=colorPanelOpen?'':'none';
}

// ── Canvas interaction ─────────────────────────────────────────────────────
// (mousemove handler is combined with brush drag below)
// ── Grid brush drag state ─────────────────────────────────────────────────
let _brushDown = false;
let _lastBrushCell = null;

function _paintGridCell(wx, wy) {
  const gx = Math.floor(wx / GRID_CELL);
  const gy = Math.floor(wy / GRID_CELL);
  if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H) return;
  const key = gx + ',' + gy;
  if (_lastBrushCell === key) return;          // avoid spamming same cell
  _lastBrushCell = key;
  const value = (_mapTool === 'obstacle');      // true = add, false = remove
  socket.emit('paint_grid', { gx, gy, value });
}

canvas.addEventListener('mousedown', e => {
  if (e.button !== 0) return;
  const rect = canvas.getBoundingClientRect(), w = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
  if (w.x < 0 || w.x > FIELD_W || w.y < 0 || w.y > FIELD_H) return;

  if (_mapTool === 'obstacle' || _mapTool === 'remove_obs') {
    _brushDown = true;
    _lastBrushCell = null;
    _paintGridCell(w.x, w.y);
    return;
  }
  if (_mapTool === 'place_opp') {
    _placeOpponentAt(w.x, w.y);
    return;
  }
});

canvas.addEventListener('mousemove', e => {
  const rect = canvas.getBoundingClientRect(), w = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
  const inF = w.x >= 0 && w.x <= FIELD_W && w.y >= 0 && w.y <= FIELD_H;
  const el = document.getElementById('coord-display');
  if (el) el.textContent = inF ? `x: ${w.x.toFixed(0)} mm   y: ${w.y.toFixed(0)} mm` : 'x:— y:—';
  // drag-painting
  if (_brushDown && inF) _paintGridCell(w.x, w.y);
});

canvas.addEventListener('mouseup', () => { _brushDown = false; _lastBrushCell = null; });
canvas.addEventListener('mouseleave', () => { _brushDown = false; _lastBrushCell = null; });

canvas.addEventListener('click', e => {
  if (_brushDown) return; // handled by mousedown/move
  const rect = canvas.getBoundingClientRect(), w = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
  if (w.x < 0 || w.x > FIELD_W || w.y < 0 || w.y > FIELD_H) return;

  // Grid tools handled in mousedown
  if (_mapTool === 'obstacle' || _mapTool === 'remove_obs' || _mapTool === 'place_opp') return;

  // Default: drop a pin and show popover
  // Theta: keep previous pin's theta for waypoint chaining; fall back to robot's current theta
  const _prevTheta = _mapPin != null
    ? _mapPin.theta
    : (lastState?.robot ? lastState.robot.theta * 180 / Math.PI : 0);
  _mapPin = { x: Math.round(w.x), y: Math.round(w.y), theta: _prevTheta };
  _showPinPopover(e.clientX - rect.left, e.clientY - rect.top);
  if (lastState) render(lastState);
});
canvas.addEventListener('contextmenu', e => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect(), w = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
  if (w.x < 0 || w.x > FIELD_W || w.y < 0 || w.y > FIELD_H) return;
  // Right-click = toggle cell
  const gx = Math.floor(w.x / GRID_CELL), gy = Math.floor(w.y / GRID_CELL);
  if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) {
    socket.emit('field_click', { x: w.x, y: w.y, button: 2 });
  }
});

// ── Controls ──────────────────────────────────────────────────────────────
function runStrategy()    { socket.emit('run_strategy'); }
function stopStrategy()   { socket.emit('stop_strategy'); }
function resetSim()       { socket.emit('reset'); }
function reloadStrategy() { socket.emit('reload_strategy'); }
function setTeam(team) {
  socket.emit('set_team',{team});
  document.getElementById('btn-yellow')?.classList.toggle('active',team==='yellow');
  document.getElementById('btn-blue')?.classList.toggle('active',team==='blue');
}
function setMode(mode)    { socket.emit('set_mode',{mode}); }

// ═══════════════════════════════════════════════════════════════════════════════
//  MAP PIN & TRAJECTORY SYSTEM
// ═══════════════════════════════════════════════════════════════════════════════

let _mapPin  = null;    // {x, y, theta} — current pin position (world mm)
let _mapTool = null;    // null | 'obstacle' | 'remove_obs'
let _mapTraj = [];      // [{x, y}, ...] — trajectory waypoints

// ── Tool toggle (obstacle modes) ────────────────────────────────────────────
function toggleMapTool(tool) {
  if (_mapTool === tool) {
    _mapTool = null;
  } else {
    _mapTool = tool;
  }
  document.getElementById('tool-obs-add')?.classList.toggle('active', _mapTool === 'obstacle');
  document.getElementById('tool-obs-del')?.classList.toggle('active', _mapTool === 'remove_obs');
  document.getElementById('tool-place-opp')?.classList.toggle('active', _mapTool === 'place_opp');
  // Set cursor style
  canvas.style.cursor = _mapTool ? 'crosshair' : '';
}

// ── Pin popover show/hide ───────────────────────────────────────────────────
function _showPinPopover(canvasPx, canvasPy) {
  const pop = document.getElementById('map-pin-popover');
  if (!pop || !_mapPin) return;
  pop.style.left = canvasPx + 'px';
  pop.style.top  = canvasPy + 'px';
  pop.classList.remove('hidden');
  document.getElementById('pin-x').value = _mapPin.x;
  document.getElementById('pin-y').value = _mapPin.y;
  document.getElementById('pin-theta').value = (_mapPin.theta ?? 0).toFixed(1);
}

function dismissPin() {
  _mapPin = null;
  document.getElementById('map-pin-popover')?.classList.add('hidden');
  if (lastState) render(lastState);
}

function pinCoordsEdited() {
  if (!_mapPin) return;
  _mapPin.x = +(document.getElementById('pin-x')?.value ?? 0);
  _mapPin.y = +(document.getElementById('pin-y')?.value ?? 0);
  _mapPin.theta = +(document.getElementById('pin-theta')?.value ?? 0);
  if (lastState) render(lastState);
}

// ── Pin actions ─────────────────────────────────────────────────────────────
function pinGoHere() {
  if (!_mapPin) return;
  fetch('/api/exec', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ cmd: `go(${_mapPin.x},${_mapPin.y})` })
  }).then(r => r.json()).then(d => {
    showToast(d.ok ? `Go → (${_mapPin.x}, ${_mapPin.y})` : `Error: ${d.res}`);
  }).catch(() => showToast('Request failed'));
}

function pinSetPos() {
  if (!_mapPin) return;
  socket.emit('set_robot_pos', { x: _mapPin.x, y: _mapPin.y, theta: _mapPin.theta || 0 });
  showToast(`Position set → (${_mapPin.x}, ${_mapPin.y}, ${_mapPin.theta || 0})`);
}

function pinCopy() {
  if (!_mapPin) return;
  const text = `(${_mapPin.x}, ${_mapPin.y}, ${_mapPin.theta || 0})`;
  navigator.clipboard.writeText(text).then(() => {
    showToast(`Copied ${text}`);
  }).catch(() => showToast('Copy failed'));
}

function pinAddWaypoint() {
  if (!_mapPin) return;
  _mapTraj.push({ x: _mapPin.x, y: _mapPin.y });
  _updateTrajBar();
  if (lastState) render(lastState);
  showToast(`Waypoint #${_mapTraj.length}: (${_mapPin.x}, ${_mapPin.y})`);
}

// ── Trajectory bar ──────────────────────────────────────────────────────────
function _updateTrajBar() {
  const bar = document.getElementById('map-traj-bar');
  if (!bar) return;
  bar.classList.toggle('hidden', _mapTraj.length === 0);
  document.getElementById('traj-count').textContent = `${_mapTraj.length} pts`;

  const list = document.getElementById('traj-list');
  if (list) {
    list.innerHTML = _mapTraj.map((p, i) =>
      `<span class="traj-wpt-chip">${i+1}: (${p.x}, ${p.y})<span class="traj-wpt-del" onclick="trajRemove(${i})">✕</span></span>`
    ).join('');
  }
}

function trajRemove(idx) {
  _mapTraj.splice(idx, 1);
  _updateTrajBar();
  if (lastState) render(lastState);
}

function trajClear() {
  _mapTraj = [];
  _updateTrajBar();
  if (lastState) render(lastState);
}

function trajCopy() {
  if (_mapTraj.length === 0) return;
  const text = '[' + _mapTraj.map(p => `(${p.x}, ${p.y})`).join(', ') + ']';
  navigator.clipboard.writeText(text).then(() => {
    showToast(`Trajectory copied (${_mapTraj.length} pts)`);
  }).catch(() => showToast('Copy failed'));
}

async function trajPaste() {
  try {
    const text = await navigator.clipboard.readText();
    // Parse formats: [(x,y), (x,y)] or (x,y)\n(x,y)
    const re = /\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)/g;
    let m, pts = [];
    while ((m = re.exec(text)) !== null) {
      pts.push({ x: Math.round(+m[1]), y: Math.round(+m[2]) });
    }
    if (pts.length === 0) { showToast('No valid points found in clipboard'); return; }
    _mapTraj = pts;
    _updateTrajBar();
    if (lastState) render(lastState);
    showToast(`Pasted ${pts.length} waypoints`);
  } catch (e) {
    showToast('Paste failed — clipboard access denied');
  }
}

async function trajExecute() {
  if (_mapTraj.length === 0) return;
  showToast(`Executing trajectory (${_mapTraj.length} pts)…`);
  for (let i = 0; i < _mapTraj.length; i++) {
    const p = _mapTraj[i];
    try {
      const res = await fetch('/api/exec', {
        method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({ cmd: `go(${p.x},${p.y})` })
      });
      const d = await res.json();
      if (!d.ok) { showToast(`Waypoint ${i+1} failed: ${d.res}`); return; }
    } catch (e) { showToast(`Waypoint ${i+1} request failed`); return; }
  }
  showToast(`Trajectory complete (${_mapTraj.length} pts)`);
}

// ── Canvas rendering for pin + trajectory ───────────────────────────────────
function drawMapPin() {
  if (!_mapPin) return;
  const px = wx(_mapPin.x), py = wy(_mapPin.y);
  // outer ring
  ctx.beginPath(); ctx.arc(px, py, 10, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(41,128,185,.15)'; ctx.fill();
  ctx.strokeStyle = '#2980b9'; ctx.lineWidth = 2; ctx.stroke();
  // crosshair
  ctx.beginPath();
  ctx.moveTo(px - 14, py); ctx.lineTo(px + 14, py);
  ctx.moveTo(px, py - 14); ctx.lineTo(px, py + 14);
  ctx.strokeStyle = 'rgba(41,128,185,.5)'; ctx.lineWidth = 1; ctx.stroke();
  // center dot
  ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2);
  ctx.fillStyle = '#2980b9'; ctx.fill();
  // heading arrow (canvas Y is flipped: world CCW → canvas CW, so negate theta)
  if (_mapPin.theta != null) {
    const a = -(_mapPin.theta * Math.PI / 180);  // world deg → canvas rad
    const arrowLen = 22, headLen = 7;
    const ex = px + Math.cos(a) * arrowLen;
    const ey = py + Math.sin(a) * arrowLen;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = '#2980b9'; ctx.lineWidth = 2; ctx.stroke();
    // arrowhead
    ctx.save(); ctx.translate(ex, ey); ctx.rotate(a);
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-headLen, -4); ctx.lineTo(-headLen, 4); ctx.closePath();
    ctx.fillStyle = '#2980b9'; ctx.fill();
    ctx.restore();
  }
}

function drawMapTraj() {
  if (_mapTraj.length === 0) return;
  // Connecting lines (dashed)
  ctx.beginPath();
  ctx.moveTo(wx(_mapTraj[0].x), wy(_mapTraj[0].y));
  for (let i = 1; i < _mapTraj.length; i++) {
    ctx.lineTo(wx(_mapTraj[i].x), wy(_mapTraj[i].y));
  }
  ctx.strokeStyle = '#e67e22'; ctx.lineWidth = 2;
  ctx.setLineDash([8, 4]); ctx.stroke(); ctx.setLineDash([]);

  // Direction arrows
  for (let i = 1; i < _mapTraj.length; i++) {
    const x0 = wx(_mapTraj[i-1].x), y0 = wy(_mapTraj[i-1].y);
    const x1 = wx(_mapTraj[i].x), y1 = wy(_mapTraj[i].y);
    const mx = (x0+x1)/2, my = (y0+y1)/2;
    const a = Math.atan2(y1-y0, x1-x0);
    ctx.save(); ctx.translate(mx, my); ctx.rotate(a);
    ctx.beginPath(); ctx.moveTo(6, 0); ctx.lineTo(-4, -4); ctx.lineTo(-4, 4); ctx.closePath();
    ctx.fillStyle = '#e67e22'; ctx.fill();
    ctx.restore();
  }

  // Waypoint circles with numbers
  for (let i = 0; i < _mapTraj.length; i++) {
    const px = wx(_mapTraj[i].x), py = wy(_mapTraj[i].y);
    ctx.beginPath(); ctx.arc(px, py, 11, 0, Math.PI * 2);
    ctx.fillStyle = '#e67e22'; ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.fillStyle = '#fff'; ctx.font = 'bold 10px monospace';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(i + 1, px, py + 0.5);
  }
}
// ═══════════════════════════════════════════════════════════════════════════════
//  FAKE OPPONENT SYSTEM
// ═══════════════════════════════════════════════════════════════════════════════

let _oppSeq = [];           // [{x, y, theta}, ...]
let _oppSeqPlaying = false;
let _oppSeqTimer = null;

function toggleOpponent(enabled) {
  socket.emit('set_opponent_enabled', { enabled });
}

function updateOpponentPos() {
  const x = parseFloat(document.getElementById('opp-x')?.value || 0);
  const y = parseFloat(document.getElementById('opp-y')?.value || 0);
  const theta = parseFloat(document.getElementById('opp-theta')?.value || 0) * Math.PI / 180;
  socket.emit('set_opponent_pos', { x, y, theta });
}

function _placeOpponentAt(wx, wy) {
  const elX = document.getElementById('opp-x');
  const elY = document.getElementById('opp-y');
  if (elX) elX.value = Math.round(wx);
  if (elY) elY.value = Math.round(wy);
  // Enable if not already
  const cb = document.getElementById('opp-enabled');
  if (cb && !cb.checked) { cb.checked = true; toggleOpponent(true); }
  updateOpponentPos();
  toggleMapTool('place_opp'); // deactivate tool after placing
}

function oppSeqAdd() {
  const x = parseFloat(document.getElementById('opp-x')?.value || 0);
  const y = parseFloat(document.getElementById('opp-y')?.value || 0);
  const theta = parseFloat(document.getElementById('opp-theta')?.value || 0);
  _oppSeq.push({ x, y, theta });
  _renderOppSeq();
}

function oppSeqClear() {
  _oppSeq = [];
  _renderOppSeq();
}

function oppSeqRemove(idx) {
  _oppSeq.splice(idx, 1);
  _renderOppSeq();
}

function _renderOppSeq() {
  const el = document.getElementById('opp-seq-list');
  if (!el) return;
  if (_oppSeq.length === 0) { el.innerHTML = '<span class="opp-empty">No waypoints</span>'; return; }
  el.innerHTML = _oppSeq.map((p, i) =>
    `<div class="opp-seq-item">
       <span class="opp-seq-idx">${i+1}</span>
       <span>(${p.x}, ${p.y})</span>
       <button class="btn-tiny" onclick="oppSeqRemove(${i})">✕</button>
     </div>`
  ).join('');
}

function oppSeqPlay() {
  if (_oppSeq.length < 2) { showToast('Need at least 2 waypoints'); return; }
  const speed = parseFloat(document.getElementById('opp-seq-speed')?.value || 200);
  _oppSeqPlaying = true;
  document.getElementById('opp-seq-play')?.classList.add('active');

  let segIdx = 0;
  let segProgress = 0;

  function tick() {
    if (!_oppSeqPlaying || segIdx >= _oppSeq.length - 1) {
      _oppSeqPlaying = false;
      document.getElementById('opp-seq-play')?.classList.remove('active');
      return;
    }
    const a = _oppSeq[segIdx], b = _oppSeq[segIdx + 1];
    const dist = Math.hypot(b.x - a.x, b.y - a.y);
    const dt = 1/30;  // 30 Hz tick
    const step = (speed * dt) / (dist || 1);
    segProgress += step;

    if (segProgress >= 1) {
      segProgress = 0;
      segIdx++;
      if (segIdx >= _oppSeq.length - 1) {
        // Loop back
        segIdx = 0;
      }
    }
    const p = _oppSeq[segIdx], q = _oppSeq[Math.min(segIdx + 1, _oppSeq.length - 1)];
    const t = Math.min(segProgress, 1);
    const cx = p.x + (q.x - p.x) * t;
    const cy = p.y + (q.y - p.y) * t;
    const th = Math.atan2(q.y - p.y, q.x - p.x);
    socket.emit('set_opponent_pos', { x: cx, y: cy, theta: th });

    _oppSeqTimer = setTimeout(tick, dt * 1000);
  }
  tick();
}

function oppSeqStop() {
  _oppSeqPlaying = false;
  if (_oppSeqTimer) { clearTimeout(_oppSeqTimer); _oppSeqTimer = null; }
  document.getElementById('opp-seq-play')?.classList.remove('active');
}

// Init opponent sequence list
_renderOppSeq();

function setFeature(k,v)  { socket.emit('set_feature',{feature:k,value:v}); }

// ── Occupancy static map ──────────────────────────────────────────────────
async function saveStaticMap() {
  const btn = document.getElementById('btn-occ-save');
  if (btn) { btn.disabled = true; btn.textContent = '…'; }
  try {
    const r = await fetch('/api/occupancy/static', { method: 'PUT' });
    const j = await r.json();
    if (btn) { btn.textContent = j.ok ? '✓ Saved' : '✗ Error'; }
  } catch(e) {
    if (btn) btn.textContent = '✗ Error';
  }
  setTimeout(() => { if (btn) { btn.disabled = false; btn.innerHTML = '&#128190; Save'; } }, 2000);
}

async function deployStaticMap() {
  const btn = document.getElementById('btn-occ-deploy');
  if (btn) { btn.disabled = true; btn.textContent = '…'; }
  try {
    const r = await fetch('/api/occupancy/deploy', { method: 'POST' });
    const j = await r.json();
    if (btn) { btn.textContent = j.ok ? '✓ Sent' : '✗ ' + (j.error || 'Error'); }
  } catch(e) {
    if (btn) btn.textContent = '✗ Error';
  }
  setTimeout(() => { if (btn) { btn.disabled = false; btn.innerHTML = '&#128225; Deploy'; } }, 2000);
}

function togglePathfinding(on) {
  fetch('/api/pathfinding', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: on }),
  });
}
function setFeedrate(v) {
  document.getElementById('feedrate-val').textContent=parseFloat(v).toFixed(2)+'×';
  socket.emit('set_feedrate',{value:parseFloat(v)});
}
function toggleRobotImage(c) { showRobotImage=c; if (lastState) render(lastState); }
function clearLog() { document.getElementById('log-output').innerHTML=''; }

// ══════════════════════════════════════════════════════════════════════════
//  MONITOR VIEW
// ══════════════════════════════════════════════════════════════════════════
function updateMonitor(s) {
  const r=s.robot;
  setText('mon-x',     r.x.toFixed(0));
  setText('mon-y',     r.y.toFixed(0));
  setText('mon-theta', (r.theta*180/Math.PI).toFixed(1)+'°');
  setText('mon-speed', Math.hypot(r.vx,r.vy).toFixed(0));
  setText('mon-vx',    r.vx.toFixed(0));
  setText('mon-vy',    r.vy.toFixed(0));
  setText('mon-omega', (r.omega ? (r.omega*180/Math.PI).toFixed(1) : '—'));
  const ms=document.getElementById('mon-mstate');
  if (ms) { ms.textContent=s.motion.state; ms.className='mon-val '+(MOTION_CLASS[s.motion.state]||''); }
  const left=s.chrono.left, total=left+s.chrono.elapsed;
  setText('mon-time',  left.toFixed(1)+' s');
  setText('mon-score', s.score+' pts');
  const pct=total>0?left/total*100:100;
  const cb=document.getElementById('mon-chrono-bar');
  if (cb) { cb.style.width=pct+'%'; cb.style.background=left<10?'#c0392b':left<30?'#b7770d':'#2980b9'; }
  const safe=s.safety.enabled&&s.safety.detected;
  const sf=document.getElementById('mon-safety');
  if (sf) { sf.textContent=safe?'⚠ OBSTACLE':'OK'; sf.style.color=safe?'#c0392b':'#1a8c3c'; }
  setText('mon-obs-detected', safe?'Yes':'No');
  setText('mon-collision',    r.collided?'Yes':'No');
  // Game objects
  const go=document.getElementById('mon-game-objs');
  if (go) go.innerHTML=s.game_objs.filter(o=>o.color!=='UNKNOWN'&&o.color!=='NONE').map(o=>
    `<span style="display:inline-block;width:12px;height:12px;border-radius:50%;background:${COLOR_HEX[o.color]||'#888'};border:1px solid rgba(0,0,0,.2);title='${o.name}'"></span>`
  ).join('');
}

// ══════════════════════════════════════════════════════════════════════════
//  STRATEGY FILE MANAGEMENT
// ══════════════════════════════════════════════════════════════════════════
let activeStrategyFile = 'match.py';
let strategyEditor = null;  // CodeMirror instance

function initCodeMirror() {
  if (typeof CodeMirror === 'undefined') return;
  const ta = document.getElementById('strategy-code');
  if (!ta || strategyEditor) return;
  strategyEditor = CodeMirror.fromTextArea(ta, {
    mode: 'python', theme: 'eclipse', lineNumbers: true,
    indentUnit: 4, indentWithTabs: false,
    extraKeys: { 'Ctrl-S': () => saveStrategy(), 'Ctrl-Enter': () => saveAndActivate() },
  });
  // Make the wrapper flex properly
  const scroller = strategyEditor.getScrollerElement();
  scroller.style.height = '100%';
  strategyEditor.refresh();
}

function refreshStrategyList() {
  fetch('/api/strategies').then(r=>r.json()).then(files => {
    const list=document.getElementById('strategy-file-list');
    if (!list) return;
    list.innerHTML=files.map(f=>`
      <div class="file-item${f===activeStrategyFile?' active':''}" onclick="loadStrategyFile('${f}')">
        <span class="file-item-name">📄 ${escHtml(f)}</span>
        <span class="file-item-actions">
          <button class="btn-tiny" onclick="event.stopPropagation();renameStrategyFile('${escHtml(f)}')">✏</button>
          ${f!=='match.py'?`<button class="btn-tiny red" onclick="event.stopPropagation();deleteStrategyFile('${escHtml(f)}')">🗑</button>`:''}
        </span>
      </div>`).join('');
  });
  // Also refresh macro refs
  renderMacroRefsInStrategy();
  // POI refs
  renderPoiRefs();
}

function loadStrategyFile(name) {
  activeStrategyFile = name;
  fetch(`/api/strategy/${encodeURIComponent(name)}`).then(r=>r.json()).then(d=>{
    if (strategyEditor) strategyEditor.setValue(d.content||'');
    else { const ta=document.getElementById('strategy-code'); if (ta) ta.value=d.content||''; }
    setText('strategy-filename', name);
    const delBtn=document.getElementById('del-strat-btn');
    if (delBtn) delBtn.disabled=(name==='match.py');
    refreshStrategyList();
    setStrategyStatus('');
  });
}

function saveStrategy() {
  const content=strategyEditor?strategyEditor.getValue():(document.getElementById('strategy-code')?.value||'');
  fetch(`/api/strategy/${encodeURIComponent(activeStrategyFile)}`,{
    method:'PUT', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({content})
  }).then(()=>setStrategyStatus('Saved ✓','ok'));
}

function saveAndActivate() {
  saveStrategy();
  socket.emit('activate_strategy',{name:activeStrategyFile});
  setStrategyStatus('Saved & activated ✓','ok');
}

function newStrategy() {
  const name=prompt('Strategy filename (without .py):');
  if (!name) return;
  fetch('/api/strategies',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({name})
  }).then(r=>r.json()).then(d=>{
    if (d.ok) { refreshStrategyList(); loadStrategyFile(d.name); }
  });
}

function renameStrategy() { renameStrategyFile(activeStrategyFile); }
function renameStrategyFile(name) {
  const newName=prompt(`Rename "${name}" to:`, name.replace('.py',''));
  if (!newName) return;
  fetch(`/api/strategy/${encodeURIComponent(name)}/rename`,{
    method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to:newName})
  }).then(r=>r.json()).then(d=>{
    if (d.ok) { if (activeStrategyFile===name) activeStrategyFile=d.name; refreshStrategyList(); }
  });
}

function deleteStrategy() { deleteStrategyFile(activeStrategyFile); }
function deleteStrategyFile(name) {
  if (!confirm(`Delete "${name}"?`)) return;
  fetch(`/api/strategy/${encodeURIComponent(name)}`,{method:'DELETE'}).then(()=>{
    if (activeStrategyFile===name) { activeStrategyFile='match.py'; loadStrategyFile('match.py'); }
    else refreshStrategyList();
  });
}

function setStrategyStatus(msg, cls='') {
  const el=document.getElementById('strategy-status');
  if (!el) return; el.textContent=msg; el.className='editor-status '+(cls||'');
  if (msg) setTimeout(()=>{ el.textContent=''; el.className='editor-status'; },3000);
}

function renderMacroRefsInStrategy() {
  const el=document.getElementById('macro-refs'); if (!el) return;
  if (!macros.length) { el.innerHTML='<span class="file-list-empty">No macros yet</span>'; return; }
  el.innerHTML=macros.map(m=>`
    <div class="file-item" onclick="insertMacroCall('${escHtml(m.name)}')">
      <span class="file-item-name">⚡ ${escHtml(m.name)}</span>
    </div>`).join('');
}

function insertMacroCall(name) {
  const call=`    run_macro_${name}(brain)  # TODO: define this macro\n`;
  if (strategyEditor) {
    const cur=strategyEditor.getCursor();
    strategyEditor.replaceRange(call, cur);
  } else {
    const ta=document.getElementById('strategy-code');
    if (ta) ta.value+=call;
  }
  setStrategyStatus('Macro call inserted','ok');
}

function renderPoiRefs() {
  const el=document.getElementById('poi-refs'); if (!el) return;
  el.innerHTML=poiData.map(p=>{
    const cat=poiCategory(p.name);
    return `<div class="poi-ref-item" onclick="insertPoiRef('${p.name}')">
      <span class="poi-ref-dot" style="background:${cat.color}"></span>
      <span>${p.name}</span>
      <span style="color:var(--text-dim);margin-left:auto;font-size:10px">(${Math.round(p.x)},${Math.round(p.y)})</span>
    </div>`;
  }).join('');
}

function insertPoiRef(name) {
  const ref=`POI.${name}`;
  if (strategyEditor) { const cur=strategyEditor.getCursor(); strategyEditor.replaceRange(ref, cur); }
  else { const ta=document.getElementById('strategy-code'); if (ta) { const pos=ta.selectionStart; ta.value=ta.value.slice(0,pos)+ref+ta.value.slice(pos); }}
}

// ══════════════════════════════════════════════════════════════════════════
//  MACRO MANAGEMENT
// ══════════════════════════════════════════════════════════════════════════
let macros = [];          // array of macro objects
let activeMacroIdx = -1;  // index in macros[]

const STEP_DEFS = [
  { type:'move_to',     icon:'🎯', label:'Move to',       fields:['x','y'] },
  { type:'face',        icon:'↻',  label:'Face angle',    fields:['angle_deg'] },
  { type:'actuator',    icon:'🔧', label:'Actuator cmd',  fields:['cmd','wait_ms'] },
  { type:'wait',        icon:'⏱',  label:'Wait',          fields:['ms'] },
  { type:'if_occupied', icon:'❓', label:'If zone occupied', fields:['poi','skip'] },
  { type:'call_macro',  icon:'▶',  label:'Call macro',    fields:['name'] },
  { type:'log',         icon:'📝', label:'Log message',   fields:['msg'] },
];

function loadMacros() {
  fetch('/api/macros').then(r=>r.json()).then(data=>{
    macros=Array.isArray(data)?data:[];
    renderMacroList();
    renderMacroRefsInStrategy();
  });
}

function saveMacros() {
  if (activeMacroIdx>=0) syncActiveMacroFromUI();
  fetch('/api/macros',{method:'PUT',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(macros)
  }).then(()=>showToast('Macros saved ✓'));
}

function renderMacroList() {
  const list=document.getElementById('macro-list'); if (!list) return;
  if (!macros.length) { list.innerHTML='<span class="file-list-empty">No macros yet</span>'; return; }
  list.innerHTML=macros.map((m,i)=>`
    <div class="file-item${i===activeMacroIdx?' active':''}" onclick="selectMacro(${i})">
      <span class="file-item-name">⚡ ${escHtml(m.name)}</span>
      <span class="file-item-actions">
        <button class="btn-tiny red" onclick="event.stopPropagation();confirmDeleteMacro(${i})">🗑</button>
      </span>
    </div>`).join('');
}

function newMacro() {
  const name=prompt('Macro name (snake_case):');
  if (!name) return;
  const m={ name:name.replace(/\s+/g,'_'), description:'', start_pos:null, steps:[] };
  macros.push(m);
  activeMacroIdx=macros.length-1;
  saveMacros();
  renderMacroList();
  showMacroEditor(activeMacroIdx);
}

function selectMacro(idx) {
  if (activeMacroIdx>=0) syncActiveMacroFromUI();
  activeMacroIdx=idx;
  renderMacroList();
  showMacroEditor(idx);
}

function showMacroEditor(idx) {
  const m=macros[idx];
  if (!m) return;
  document.getElementById('macro-empty').classList.add('hidden');
  document.getElementById('macro-editor').classList.remove('hidden');
  document.getElementById('macro-name').value=m.name;
  document.getElementById('macro-desc').value=m.description||'';
  document.getElementById('macro-sx').value=m.start_pos?m.start_pos.x:'';
  document.getElementById('macro-sy').value=m.start_pos?m.start_pos.y:'';
  renderMacroSteps(m.steps);
}

function syncActiveMacroFromUI() {
  const m=macros[activeMacroIdx]; if (!m) return;
  m.name        = document.getElementById('macro-name')?.value||m.name;
  m.description = document.getElementById('macro-desc')?.value||'';
  const sx=document.getElementById('macro-sx')?.value;
  const sy=document.getElementById('macro-sy')?.value;
  m.start_pos   = (sx&&sy)?{x:parseFloat(sx),y:parseFloat(sy)}:null;
}

function deleteMacro() {
  if (activeMacroIdx<0) return;
  confirmDeleteMacro(activeMacroIdx);
}
function confirmDeleteMacro(idx) {
  if (!confirm(`Delete macro "${macros[idx]?.name}"?`)) return;
  macros.splice(idx,1);
  activeMacroIdx=Math.min(activeMacroIdx, macros.length-1);
  saveMacros();
  renderMacroList();
  if (activeMacroIdx>=0) showMacroEditor(activeMacroIdx);
  else {
    document.getElementById('macro-empty').classList.remove('hidden');
    document.getElementById('macro-editor').classList.add('hidden');
  }
}

function renderMacroSteps(steps) {
  const container=document.getElementById('macro-steps-list'); if (!container) return;
  container.innerHTML=steps.map((step,i)=>buildStepHTML(step,i)).join('');
}

function buildStepHTML(step, idx) {
  const def=STEP_DEFS.find(d=>d.type===step.type)||STEP_DEFS[0];
  const typeOptions=STEP_DEFS.map(d=>
    `<option value="${d.type}"${d.type===step.type?' selected':''}>${d.icon} ${d.label}</option>`
  ).join('');
  const fields=buildFieldsHTML(step, idx);
  return `<div class="macro-step" id="ms-${idx}">
    <span class="step-icon">${def.icon}</span>
    <select class="step-type-sel" onchange="changeStepType(${idx},this.value)">${typeOptions}</select>
    <div class="step-fields">${fields}</div>
    <button class="btn-tiny" onclick="runStepOnRobot(${idx})" title="Exécuter cette étape sur le robot connecté">🤖</button>
    <button class="btn-tiny" onclick="moveStep(${idx},-1)" title="Move up">↑</button>
    <button class="btn-tiny" onclick="moveStep(${idx},+1)" title="Move down">↓</button>
    <button class="btn-tiny red" onclick="deleteStep(${idx})">✕</button>
  </div>`;
}

function buildFieldsHTML(step, idx) {
  const f=(key,placeholder,val,type='text',w='80px')=>
    `<label class="step-field-label">${key}</label>
     <input class="input-sm" type="${type}" placeholder="${placeholder}" value="${val||''}" style="width:${w}"
       oninput="updateStepField(${idx},'${key}',this.value)">`;
  switch (step.type) {
    case 'move_to':     return f('x','0',step.x,'number','60px')+f('y','0',step.y,'number','60px');
    case 'face':        return f('angle_deg','0°',step.angle_deg,'number','70px');
    case 'actuator':    return f('cmd','arm(180)',step.cmd,'text','120px')+f('wait_ms','0',step.wait_ms,'number','55px');
    case 'wait':        return f('ms','500',step.ms,'number','70px');
    case 'if_occupied': return buildPOISelect(step,idx)+f('skip','1',step.skip,'number','45px');
    case 'call_macro':  return buildMacroSelect(step,idx);
    case 'log':         return f('msg','message...',step.msg,'text','180px');
    default:            return '';
  }
}

function buildPOISelect(step, idx) {
  const opts=poiData.map(p=>`<option value="${p.name}"${p.name===step.poi?' selected':''}>${p.name}</option>`).join('');
  return `<label class="step-field-label">poi</label>
    <select class="input-sm" onchange="updateStepField(${idx},'poi',this.value)">
      <option value="">— choose —</option>${opts}</select>
    <label class="step-field-label">→ skip</label>`;
}

function buildMacroSelect(step, idx) {
  const opts=macros.map(m=>`<option value="${m.name}"${m.name===step.name?' selected':''}>${m.name}</option>`).join('');
  return `<label class="step-field-label">macro</label>
    <select class="input-sm" onchange="updateStepField(${idx},'name',this.value)">
      <option value="">— choose —</option>${opts}</select>`;
}

function changeStepType(idx, newType) {
  const m=macros[activeMacroIdx]; if (!m) return;
  m.steps[idx]={type:newType};
  renderMacroSteps(m.steps);
}

function updateStepField(idx, key, val) {
  const m=macros[activeMacroIdx]; if (!m||!m.steps[idx]) return;
  const numFields=['x','y','angle_deg','ms','wait_ms','skip'];
  m.steps[idx][key]=numFields.includes(key)?parseFloat(val)||0:val;
}

function addMacroStep() {
  const m=macros[activeMacroIdx]; if (!m) return;
  m.steps.push({type:'move_to',x:0,y:0});
  renderMacroSteps(m.steps);
}

function deleteStep(idx) {
  const m=macros[activeMacroIdx]; if (!m) return;
  m.steps.splice(idx,1); renderMacroSteps(m.steps);
}

function moveStep(idx, dir) {
  const m=macros[activeMacroIdx]; if (!m) return;
  const ni=idx+dir;
  if (ni<0||ni>=m.steps.length) return;
  [m.steps[idx],m.steps[ni]]=[m.steps[ni],m.steps[idx]];
  renderMacroSteps(m.steps);
}

function runMacroInSim() {
  syncActiveMacroFromUI();
  const m=macros[activeMacroIdx]; if (!m) return;
  // Add start_pos as first step if defined
  const steps=[...m.steps];
  if (m.start_pos) steps.unshift({type:'move_to',x:m.start_pos.x,y:m.start_pos.y});
  socket.emit('run_macro',{steps,macros});
  showToast(`Running macro: ${m.name}`);
}

function generateMacroPython() {
  syncActiveMacroFromUI();
  const m=macros[activeMacroIdx]; if (!m) return;
  const code=macroPythonCode(m);
  const pre=document.getElementById('macro-python-code');
  const wrap=document.getElementById('macro-python-preview');
  if (pre) pre.textContent=code;
  if (wrap) wrap.classList.remove('hidden');
}
function hidePythonPreview() {
  document.getElementById('macro-python-preview')?.classList.add('hidden');
}

function macroPythonCode(m) {
  const fn=m.name.replace(/[^a-z0-9_]/gi,'_').toLowerCase();
  let c=`def macro_${fn}(brain, transport):\n    """${m.description||m.name}"""\n`;
  if (m.start_pos) c+=`    # Navigate to start position\n    brain.motion._t._bridge._start_motion_xy(0, Vec2(${m.start_pos.x}, ${m.start_pos.y}), lambda u,o,r: None)\n    time.sleep(0.5)\n`;
  for (const s of m.steps) {
    switch (s.type) {
      case 'move_to':
        c+=`    brain.motion._t._bridge._start_motion_xy(0, Vec2(${s.x||0}, ${s.y||0}), lambda u,o,r: None)\n    time.sleep(0.3)\n`; break;
      case 'face':
        c+=`    # TODO: rotate to ${s.angle_deg||0}°\n`; break;
      case 'actuator':
        c+=`    transport.fire('${s.cmd||''}')\n`;
        if (s.wait_ms) c+=`    time.sleep(${(s.wait_ms/1000).toFixed(2)})\n`; break;
      case 'wait':
        c+=`    time.sleep(${((s.ms||0)/1000).toFixed(2)})\n`; break;
      case 'if_occupied':
        c+=`    _poi = POI.${s.poi||'pantry_01'}\n    _gx, _gy = int(_poi.x // 150), int(_poi.y // 150)\n    if not any(c['gx']==_gx and c['gy']==_gy for c in brain.occupancy.to_list()):\n        pass  # zone clear — skip next ${s.skip||1} step(s)\n    # else:\n`; break;
      case 'call_macro':
        c+=`    macro_${(s.name||'').replace(/[^a-z0-9_]/gi,'_').toLowerCase()}(brain, transport)\n`; break;
      case 'log':
        c+=`    brain.log('${s.msg||''}')\n`; break;
    }
  }
  return c;
}

// ── Macro: Run single step on robot ───────────────────────────────────────
function runStepOnRobot(idx) {
  const m = macros[activeMacroIdx]; if (!m) return;
  const step = m.steps[idx]; if (!step) return;
  fetch('/api/missions/run-step', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(step),
  })
  .then(r => r.json())
  .then(d => {
    const el = document.getElementById(`ms-${idx}`);
    const flash = d.ok ? '#22c55e44' : '#ef444444';
    if (el) { el.style.background = flash; setTimeout(() => el.style.background = '', 800); }
    showToast(d.ok ? `✓ ${d.cmd}` : `✗ ${d.error || d.response}`);
  })
  .catch(e => showToast('✗ ' + e));
}

// ── Macro: Run full macro on robot (sequential) ───────────────────────────
let _macroRobotRunning = false;
function runMacroOnRobot() {
  if (_macroRobotRunning) { showToast('Macro déjà en cours'); return; }
  syncActiveMacroFromUI();
  const m = macros[activeMacroIdx]; if (!m) return;
  const steps = [...m.steps];
  if (m.start_pos) steps.unshift({type:'move_to', x:m.start_pos.x, y:m.start_pos.y});
  _macroRobotRunning = true;
  const btn = document.getElementById('btn-macro-run-robot');
  if (btn) btn.textContent = '⏳ Running…';

  (async () => {
    for (let i = 0; i < steps.length; i++) {
      const step = steps[i];
      // highlight step
      document.querySelectorAll('.macro-step').forEach((el, idx) => {
        el.style.background = idx === i ? '#3b82f644' : '';
      });
      try {
        const res = await fetch('/api/missions/run-step', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(step),
        }).then(r => r.json());
        if (!res.ok) {
          showToast(`✗ Étape ${i+1} échouée: ${res.error || res.response}`);
          break;
        }
      } catch(e) {
        showToast('✗ Erreur réseau: ' + e); break;
      }
    }
    document.querySelectorAll('.macro-step').forEach(el => el.style.background = '');
    _macroRobotRunning = false;
    if (btn) btn.textContent = '🤖 Robot';
  })();
}

// ══════════════════════════════════════════════════════════════════════════
//  MISSIONS
// ══════════════════════════════════════════════════════════════════════════

let missions = [];
let activeMissionIdx = -1;
let _missionPickingApproach = false;

function loadMissions() {
  fetch('/api/missions').then(r => r.json()).then(data => {
    missions = Array.isArray(data) ? data : [];
    renderMissionList();
  });
}

function saveMissions() {
  if (activeMissionIdx >= 0) syncActiveMissionFromUI();
  fetch('/api/missions', {
    method: 'PUT',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(missions),
  }).then(() => showToast('Missions sauvegardées ✓'));
}

function renderMissionList() {
  const list = document.getElementById('mission-list'); if (!list) return;
  if (!missions.length) {
    list.innerHTML = '<span class="file-list-empty">Aucune mission</span>'; return;
  }
  const sorted = [...missions.entries()].sort(([,a],[,b]) => b.priority - a.priority);
  list.innerHTML = sorted.map(([i, m]) => `
    <div class="file-item${i === activeMissionIdx ? ' active' : ''}" onclick="selectMission(${i})">
      <span class="file-item-name${m.enabled === false ? ' text-dim' : ''}">
        ${m.enabled === false ? '○' : '●'} ${escHtml(m.name)}
        <span style="font-size:9px;color:var(--text-dim);margin-left:4px">#${m.priority || 0} • ${m.score || 0}pts</span>
      </span>
      <span class="file-item-actions">
        <button class="btn-tiny red" onclick="event.stopPropagation();confirmDeleteMission(${i})">🗑</button>
      </span>
    </div>`).join('');
}

function newMission() {
  const name = prompt('Nom de la mission (snake_case):');
  if (!name) return;
  const m = {
    id: 'm' + Date.now(),
    name: name.replace(/\s+/g, '_'),
    desc: '',
    approach: {x: null, y: null, angle: null},
    macro_id: '',
    score: 20,
    priority: 5,
    time_ms: 8000,
    enabled: true,
  };
  missions.push(m);
  activeMissionIdx = missions.length - 1;
  saveMissions();
  renderMissionList();
  showMissionEditor(activeMissionIdx);
}

function selectMission(idx) {
  if (activeMissionIdx >= 0) syncActiveMissionFromUI();
  activeMissionIdx = idx;
  renderMissionList();
  showMissionEditor(idx);
}

function showMissionEditor(idx) {
  const m = missions[idx]; if (!m) return;
  document.getElementById('mission-empty').classList.add('hidden');
  document.getElementById('mission-editor').classList.remove('hidden');
  document.getElementById('mission-name').value     = m.name || '';
  document.getElementById('mission-desc').value     = m.desc || '';
  document.getElementById('mission-score').value    = m.score ?? 20;
  document.getElementById('mission-priority').value = m.priority ?? 5;
  document.getElementById('mission-time-ms').value  = m.time_ms ?? 8000;
  const ap = m.approach || {};
  document.getElementById('mission-ax').value       = ap.x ?? '';
  document.getElementById('mission-ay').value       = ap.y ?? '';
  document.getElementById('mission-aa').value       = ap.angle ?? '';
  document.getElementById('mission-enabled').checked = m.enabled !== false;
  _refreshMacroSelector(m.macro_id || '');
  _refreshMissionMacroPreview(m.macro_id || '');
}

function _refreshMacroSelector(selectedId) {
  const sel = document.getElementById('mission-macro-sel'); if (!sel) return;
  sel.innerHTML = '<option value="">— aucune —</option>' +
    macros.map(m => `<option value="${escHtml(m.name)}"${m.name === selectedId ? ' selected' : ''}>${escHtml(m.name)}</option>`).join('');
}

function _refreshMissionMacroPreview(macroId) {
  const wrap = document.getElementById('mission-macro-preview');
  const container = document.getElementById('mission-steps-preview');
  if (!wrap || !container) return;
  const macro = macros.find(m => m.name === macroId);
  if (!macro || !macro.steps?.length) { wrap.classList.add('hidden'); return; }
  wrap.classList.remove('hidden');
  container.innerHTML = macro.steps.map((step, i) => {
    const def = STEP_DEFS.find(d => d.type === step.type) || STEP_DEFS[0];
    const summary = _stepSummary(step);
    return `<div class="mission-step-row">
      <span class="step-icon">${def.icon}</span>
      <span style="font-size:11px;flex:1">${def.label}: <code>${escHtml(summary)}</code></span>
      <button class="btn-tiny" onclick="runStepOnRobotDirect(${JSON.stringify(step).replace(/"/g,'&quot;')})" title="Exécuter sur robot">🤖</button>
    </div>`;
  }).join('');
}

function _stepSummary(step) {
  switch (step.type) {
    case 'move_to':     return `(${step.x||0}, ${step.y||0})`;
    case 'face':        return `${step.angle_deg||0}°`;
    case 'actuator':    return step.cmd || '';
    case 'wait':        return `${step.ms||0} ms`;
    case 'if_occupied': return `poi=${step.poi}`;
    case 'call_macro':  return `→ ${step.name}`;
    case 'log':         return step.msg || '';
    default:            return '';
  }
}

function runStepOnRobotDirect(step) {
  fetch('/api/missions/run-step', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(step),
  }).then(r => r.json()).then(d => showToast(d.ok ? `✓ ${d.cmd}` : `✗ ${d.error || d.response}`))
    .catch(e => showToast('✗ ' + e));
}

function syncActiveMissionFromUI() {
  const m = missions[activeMissionIdx]; if (!m) return;
  m.name      = document.getElementById('mission-name')?.value || m.name;
  m.desc      = document.getElementById('mission-desc')?.value || '';
  m.score     = parseInt(document.getElementById('mission-score')?.value) || 0;
  m.priority  = parseInt(document.getElementById('mission-priority')?.value) || 0;
  m.time_ms   = parseInt(document.getElementById('mission-time-ms')?.value) || 8000;
  m.enabled   = document.getElementById('mission-enabled')?.checked !== false;
  const ax = document.getElementById('mission-ax')?.value;
  const ay = document.getElementById('mission-ay')?.value;
  const aa = document.getElementById('mission-aa')?.value;
  m.approach  = {
    x: ax !== '' ? parseFloat(ax) : null,
    y: ay !== '' ? parseFloat(ay) : null,
    angle: aa !== '' ? parseFloat(aa) : null,
  };
  m.macro_id  = document.getElementById('mission-macro-sel')?.value || '';
}

function syncMissionApproach() {
  if (activeMissionIdx < 0) return;
  const m = missions[activeMissionIdx];
  const ax = document.getElementById('mission-ax')?.value;
  const ay = document.getElementById('mission-ay')?.value;
  if (m) m.approach = { ...(m.approach||{}), x: ax !== '' ? parseFloat(ax) : null, y: ay !== '' ? parseFloat(ay) : null };
}

function syncMissionMacro() {
  const sel = document.getElementById('mission-macro-sel');
  if (!sel || activeMissionIdx < 0) return;
  missions[activeMissionIdx].macro_id = sel.value;
  _refreshMissionMacroPreview(sel.value);
}

function syncMissionEnabled() {
  if (activeMissionIdx < 0) return;
  missions[activeMissionIdx].enabled = document.getElementById('mission-enabled')?.checked !== false;
  renderMissionList();
}

function deleteMission() { confirmDeleteMission(activeMissionIdx); }
function confirmDeleteMission(idx) {
  const m = missions[idx]; if (!m) return;
  if (!confirm(`Supprimer la mission "${m.name}" ?`)) return;
  missions.splice(idx, 1);
  activeMissionIdx = Math.min(activeMissionIdx, missions.length - 1);
  saveMissions();
  renderMissionList();
  if (activeMissionIdx >= 0) showMissionEditor(activeMissionIdx);
  else {
    document.getElementById('mission-empty')?.classList.remove('hidden');
    document.getElementById('mission-editor')?.classList.add('hidden');
  }
}

function missionGoToMacro() {
  const sel = document.getElementById('mission-macro-sel');
  if (!sel?.value) return;
  const idx = macros.findIndex(m => m.name === sel.value);
  if (idx < 0) return;
  const btn = document.querySelector('[data-view=macros]');
  if (btn) switchView('macros', btn);
  selectMacro(idx);
}

// ── Approach point picker (click on map to set approach) ─────────────────
function missionPickApproach() {
  _missionPickingApproach = true;
  showToast('Cliquez sur la carte pour définir le point d\'approche');
  // Switch to map view temporarily
  const btn = document.querySelector('[data-view=map]');
  if (btn) switchView('map', btn);
}

// Called by map click handler when approach-picking mode is active
function _onMapClickForMission(worldX, worldY) {
  if (!_missionPickingApproach) return false;
  _missionPickingApproach = false;
  document.getElementById('mission-ax').value = Math.round(worldX);
  document.getElementById('mission-ay').value = Math.round(worldY);
  syncMissionApproach();
  showToast(`Point d'approche: (${Math.round(worldX)}, ${Math.round(worldY)})`);
  // Return to missions view
  const btn = document.querySelector('[data-view=missions]');
  if (btn) switchView('missions', btn);
  return true;
}

function missionRunOnRobot() {
  if (activeMissionIdx < 0) return;
  syncActiveMissionFromUI();
  const m = missions[activeMissionIdx]; if (!m) return;
  const steps = [];

  // Approach move
  const ap = m.approach || {};
  if (ap.x !== null && ap.y !== null) {
    steps.push({type: 'move_to', x: ap.x, y: ap.y});
  }
  if (ap.angle !== null && ap.angle !== undefined) {
    steps.push({type: 'face', angle_deg: ap.angle});
  }

  // Macro steps
  const macro = macros.find(mc => mc.name === m.macro_id);
  if (macro) steps.push(...macro.steps);

  if (!steps.length) { showToast('Aucune étape à exécuter'); return; }

  // Reuse the macro robot runner
  const fakeM = {name: m.name, start_pos: null, steps};
  const savedIdx = activeMacroIdx;
  const savedMacros = macros;

  (async () => {
    const btn = document.getElementById('btn-mission-run');
    if (btn) btn.textContent = '⏳ Running…';
    for (const step of steps) {
      try {
        const res = await fetch('/api/missions/run-step', {
          method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(step),
        }).then(r => r.json());
        if (!res.ok) { showToast(`✗ ${res.error || res.response}`); break; }
      } catch(e) { showToast('✗ ' + e); break; }
    }
    if (btn) btn.textContent = '▶ Run on robot';
  })();
}

// ── SD Fallback preview / deploy ──────────────────────────────────────────
function missionPreviewFallback() {
  if (activeMissionIdx >= 0) syncActiveMissionFromUI();
  fetch('/api/missions/preview-fallback', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      const overlay = document.getElementById('mission-fallback-overlay');
      const pre     = document.getElementById('mission-fallback-text');
      if (!overlay || !pre) return;
      pre.textContent = d.cfg || '(empty)';
      overlay.style.display = 'flex';
    })
    .catch(e => showToast('✗ ' + e));
}

function missionCloseFallbackPreview() {
  const overlay = document.getElementById('mission-fallback-overlay');
  if (overlay) overlay.style.display = 'none';
}

function missionDeploySD() {
  if (activeMissionIdx >= 0) syncActiveMissionFromUI();
  const status = document.getElementById('mission-deploy-status');
  if (status) status.textContent = '⏳ Écriture…';
  const btn1 = document.getElementById('btn-deploy-sd');
  const btn2 = document.getElementById('btn-deploy-from-preview');
  [btn1, btn2].forEach(b => b && (b.disabled = true));

  fetch('/api/missions/deploy-sd', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      const msg = d.ok
        ? `✓ ${d.lines} lignes écrites sur SD`
        : `✗ ${d.error}`;
      if (status) status.textContent = msg;
      showToast(msg);
    })
    .catch(e => {
      const msg = '✗ ' + e;
      if (status) status.textContent = msg;
    })
    .finally(() => [btn1, btn2].forEach(b => b && (b.disabled = false)));
}

// ── Hook switchView for missions ──────────────────────────────────────────
function onMissionsViewActivated() {
  loadMissions();
  // Refresh macro selector in case macros changed
  if (activeMissionIdx >= 0) _refreshMacroSelector(missions[activeMissionIdx]?.macro_id || '');
}

// ══════════════════════════════════════════════════════════════════════════
//  TERMINAL
// ══════════════════════════════════════════════════════════════════════════
let _termHistory=[], _termHistIdx=-1;

function appendTermLine(cmd, ok, res, fire, mode) {
  const out=document.getElementById('terminal-output'); if (!out) return;
  const d=document.createElement('div'); d.className='term-line';
  const rc=fire?'term-fire':(ok?'term-ok':'term-fail');
  const modeHtml = mode
    ? `<span class="${mode==='[HW]'?'term-mode-hw':'term-mode-sim'}">${escHtml(mode)}</span>`
    : '';
  d.innerHTML=`${modeHtml}<span class="term-cmd">&gt; ${escHtml(cmd)}</span>`+
    (res?`<span class="${rc}">${escHtml(res)}</span>`:'');
  out.appendChild(d); out.scrollTop=out.scrollHeight;
}

function terminalSend(fire) {
  const inp=document.getElementById('terminal-input'); if (!inp) return;
  const cmd=inp.value.trim(); if (!cmd) return;
  _termHistory.unshift(cmd); if (_termHistory.length>60) _termHistory.pop();
  _termHistIdx=-1; inp.value='';
  if (fire) { socket.emit('actuator_fire',{cmd}); appendTermLine(cmd,true,'(fired)',true); }
  else socket.emit('terminal_cmd',{cmd,timeout_ms:5000});
}

function onTerminalKey(e) {
  if      (e.key==='Enter')    terminalSend(false);
  else if (e.key==='ArrowUp')  { e.preventDefault(); if (_termHistIdx<_termHistory.length-1) document.getElementById('terminal-input').value=_termHistory[++_termHistIdx]; }
  else if (e.key==='ArrowDown'){ e.preventDefault(); if (_termHistIdx>0) document.getElementById('terminal-input').value=_termHistory[--_termHistIdx]; else { _termHistIdx=-1; document.getElementById('terminal-input').value=''; } }
}

function clearTerminal() { const o=document.getElementById('terminal-output'); if (o) o.innerHTML=''; }

// ══════════════════════════════════════════════════════════════════════════
//  SERIAL CONNECTIVITY
// ══════════════════════════════════════════════════════════════════════════
function serialRefreshPorts() {
  fetch('/api/serial/ports').then(r=>r.json()).then(ports=>{
    ['serial-port-sel','t-port-sel'].forEach(id=>{
      const sel=document.getElementById(id); if (!sel) return;
      const cur=sel.value;
      sel.innerHTML='<option value="">— Select port —</option>'+
        ports.map(p=>`<option value="${escHtml(p.port)}"${p.port===cur?' selected':''}>${escHtml(p.port)} — ${escHtml(p.desc)}</option>`).join('');
    });
  });
}

function serialConnect() {
  // Accept port from robot-panel selector OR terminal selector
  const port = (document.getElementById('serial-port-sel') || document.getElementById('t-port-sel'))?.value;
  const mode = (document.getElementById('serial-mode-sel') || document.getElementById('t-mode-sel'))?.value || 'xbee';
  const baud = (mode === 'wired') ? 115200 : 31250;
  if (!port) { showToast('Select a serial port first'); return; }
  cgSetRobotConnecting();
  setSerialStatus('Connecting…', '');
  socket.emit('serial_connect', {port, baud});
}
function serialDisconnect() {
  socket.emit('serial_disconnect');
  setSerialStatus('Disconnected — idle', '');
  cgSetConnectionMode('idle');
}
function connectSim() {
  socket.emit('connect_sim');
  setSerialStatus('Simulator active', 'ok');
  cgSetConnectionMode('sim');
}
function setSerialStatus(msg, cls) {
  ['serial-status-val', 't-serial-status'].forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.textContent = msg;
      el.style.color = cls === 'ok'  ? 'var(--green)' :
                       cls === 'err' ? 'var(--red)'   : 'var(--text-dim)';
    }
  });
}
socket.on('serial_status', d => {
  setSerialStatus(d.msg, d.ok ? 'ok' : (d.connecting ? '' : 'err'));
  // Connecting = spinner state, already set by cgSetRobotConnecting()
  if (d.connecting) return;
  // For immediate feedback on failure, clear the connecting state right away.
  // Success / final state will follow via state.hw_mode on the next physics tick.
  if (!d.ok) {
    cgSetRobot(false);
    _serialSetConnectBusy(false);
  }
});

/** Enable/disable the Connect button(s) while an attempt is in progress. */
function _serialSetConnectBusy(busy) {
  ['serial-connect-btn'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.disabled = busy;
  });
}

// ══════════════════════════════════════════════════════════════════════════
//  UTILITIES
// ══════════════════════════════════════════════════════════════════════════
function escHtml(s) { return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;'); }
function setText(id,v) { const el=document.getElementById(id); if (el) el.textContent=v; }

function showToast(msg) {
  const t=document.getElementById('reload-toast'); if (!t) return;
  t.textContent=msg; t.classList.remove('hidden'); t.style.opacity='1';
  setTimeout(()=>{ t.style.opacity='0'; setTimeout(()=>t.classList.add('hidden'),400); },2200);
}

// ══════════════════════════════════════════════════════════════════════════
//  NETWORK VIEW  — Hardware topology canvas + side config panel
//
//  Two rows of hardware nodes:
//   Row A (online chain):  Browser ── holOS ── Teensy 4.1 ── Teensy 4.0
//   Row B (offline/alt):   Jetson                  ESP32-CAM
//
//  Each node card shows: icon / hardware name / role / SW+FW stack
//  Each link pill shows: protocol + baud rate
//  Click any node/link → side panel with config (IP, serial port, etc.)
// ══════════════════════════════════════════════════════════════════════════

// ── Runtime state ─────────────────────────────────────────────────────────
const _ns = {
  ws:        'disconnected',
  hw:        'idle',       // 'idle'|'sim'|'connecting'|'connected'|'disconnected'
  hwType:    'idle',       // 'idle'|'sim'|'usb'|'xbee'
  hwLabel:   'Not connected',
  hwPort:    null,
  t40:       'unknown',
  jetsonIp:  '',
  selected:  null,
  selType:   null,
};

// ── Hardware node definitions ─────────────────────────────────────────────
// xr/yr: relative positions (fraction of canvas W/H)
// sw: software stack lines shown inside the card
//
// Layout:
//   Row 1: PC (browser+holOS local) ─── T4.1 ─── T4.0
//   Row 2: Sim ── Jetson (holOS Server) ── ESP32-CAM
//   Links: PC↔T4.1 (USB), T4.1↔T4.0 (UART), Jetson↔T4.1 (XBee),
//          ESP32-CAM↔Jetson (WiFi), ESP32-CAM↔PC (WiFi), Sim↔PC (internal)
const _NN_W = 148, _NN_H = 106, _NN_R = 10;

const _nNodes = [
  { id:'pc',      label:'PC',              icon:'🖥',
    role:'Browser + holOS Local',
    sw:['holOS UI (HTML+JS)', 'Flask · SocketIO · Python'],
    xr:0.12, yr:0.25 },
  { id:'t41',     label:'Teensy 4.1',      icon:'🎛',
    role:'Main MCU — 600 MHz',
    sw:['TwinSystem firmware', 'PlatformIO · C++17'],
    xr:0.50, yr:0.25 },
  { id:'t40',     label:'Teensy 4.0',      icon:'🔧',
    role:'Secondary MCU',
    sw:['TwinActuator firmware', 'PlatformIO · C++17', 'Static map (RAM)'],
    xr:0.88, yr:0.25 },
  { id:'sim',     label:'Simulator',       icon:'🎮',
    role:'Virtual robot',
    sw:['SimBridge · Physics', 'VirtualTransport'],
    xr:0.12, yr:0.75 },
  { id:'jetson',  label:'NVIDIA Jetson',   icon:'🤖',
    role:'Edge computer',
    sw:['holOS Server', 'ROS2 Humble'],
    xr:0.50, yr:0.75, offline:true },
  { id:'espcam',  label:'ESP32-CAM',       icon:'📷',
    role:'Vision module',
    sw:['TwinVision firmware', 'MJPEG stream'],
    xr:0.88, yr:0.75, offline:true },
];

// ── Link definitions ──────────────────────────────────────────────────────
// proto: base protocol label
// configurable: shows connect panel when clicked
const _nLinks = [
  { id:'hw',      from:'pc',     to:'t41',     proto:'USB-CDC',               baud:115200,  configurable:true  },
  { id:'uart',    from:'t41',    to:'t40',     proto:'UART Intercom',         baud:31250,   configurable:false },
  { id:'sim',     from:'sim',    to:'pc',      proto:'VirtualTransport',      baud:null,    configurable:false },
  { id:'xbee',    from:'jetson', to:'t41',     proto:'XBee 868 MHz',          baud:31250,   configurable:false, offline:true },
  { id:'espcam_j',from:'espcam', to:'jetson',  proto:'WiFi · MJPEG',          baud:null,    configurable:false, offline:true },
  { id:'espcam_p',from:'espcam', to:'pc',      proto:'WiFi · MJPEG',          baud:null,    configurable:false, offline:true },
];

const _nsColors = {
  connected:'#1a8c3c', active:'#1a8c3c', sim:'#2980b9', idle:'#64748b',
  connecting:'#d97706', disconnected:'#94a3b8', unknown:'#94a3b8',
  offline:'#94a3b8', error:'#c0392b', ok:'#1a8c3c',
};
const _nsLabels = {
  connected:'Connected', active:'Active', sim:'Simulation', idle:'Idle',
  connecting:'Connecting…', disconnected:'Disconnected',
  unknown:'Unknown', offline:'Offline', ok:'OK', error:'Error',
};

let _netAnimFrame = null;

// ── Geometry helpers ──────────────────────────────────────────────────────
function _ncNodeById(id)   { return _nNodes.find(n=>n.id===id); }
function _ncCanvas()        { return document.getElementById('net-canvas'); }
function _ncCol(st)         { return _nsColors[st] || '#94a3b8'; }

function _ncBox(nd, W, H) {
  const cx = W * nd.xr, cy = H * nd.yr;
  return { cx, cy, x:cx-_NN_W/2, y:cy-_NN_H/2, w:_NN_W, h:_NN_H };
}

// Return point on box edge in direction of angle `ang` (from box center)
function _ncEdgePt(box, ang) {
  const hw = box.w/2, hh = box.h/2;
  const ta = Math.tan(ang);
  let x, y;
  if (Math.abs(ta) <= hh/hw) {
    x = (Math.cos(ang) >= 0) ? box.cx + hw : box.cx - hw;
    y = box.cy + (x - box.cx) * ta;
  } else {
    y = (Math.sin(ang) >= 0) ? box.cy + hh : box.cy - hh;
    x = box.cx + (ta !== 0 ? (y - box.cy) / ta : 0);
  }
  return { x, y };
}

// ── Node / link status ────────────────────────────────────────────────────
function _nsNodeStatus(nid) {
  if (nid === 'pc')      return _ns.ws === 'connected' ? 'connected' : 'disconnected';
  if (nid === 'sim')     return _ns.hwType === 'sim' ? 'active' : 'disconnected';
  if (nid === 't41') {
    if (_ns.hw === 'connected') return 'connected';
    if (_ns.hw === 'connecting') return 'connecting';
    return 'disconnected';
  }
  if (nid === 't40')     return _ns.t40 === 'ok' ? 'connected' : (_ns.t40 === 'unknown' ? 'unknown' : 'error');
  if (nid === 'jetson')  return _ns.hwType === 'xbee' ? 'connected' : 'offline';
  if (nid === 'espcam')  return 'offline';
  return 'unknown';
}
function _nsLinkStatus(lid) {
  if (lid==='hw')       return (_ns.hwType==='usb') ? 'connected' : ((_ns.hwType==='xbee') ? 'disconnected' : 'disconnected');
  if (lid==='uart')     return _ns.t40 === 'ok' ? 'connected' : (_ns.t40 === 'unknown' ? 'unknown' : 'disconnected');
  if (lid==='sim')      return _ns.hwType === 'sim' ? 'active' : 'disconnected';
  if (lid==='xbee')     return _ns.hwType === 'xbee' ? 'connected' : 'offline';
  if (lid==='espcam_j') return 'offline';
  if (lid==='espcam_p') return 'offline';
  return 'unknown';
}
function _nsLinkLabel(lid) {
  const lk = _nLinks.find(l=>l.id===lid); if (!lk) return '';
  let proto = lk.proto;
  if (lid==='hw') {
    proto = _ns.hwType==='xbee' ? 'XBee 868 MHz' : 'USB-CDC';
  }
  return lk.baud ? `${proto} · ${(lk.baud/1000).toFixed(lk.baud%1000?1:0)}k` : proto;
}

// ── Main draw ─────────────────────────────────────────────────────────────
function _ncDraw() {
  _netAnimFrame = null;
  const c = _ncCanvas();
  if (!c || activeView !== 'network') return;
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height;
  if (!W || !H) return;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#f0f4f8'; ctx.fillRect(0, 0, W, H);

  // Dot grid
  ctx.fillStyle = 'rgba(26,82,118,.05)';
  for (let x=30; x<W; x+=40)
    for (let y=30; y<H; y+=40) {
      ctx.beginPath(); ctx.arc(x,y,1.5,0,Math.PI*2); ctx.fill();
    }

  // Draw links first (under nodes)
  _nLinks.forEach(lk => _ncDrawLink(ctx, lk, W, H));
  // Draw nodes on top
  _nNodes.forEach(nd  => _ncDrawNode(ctx, nd, W, H));

  if (_ns.hw === 'connecting')
    _netAnimFrame = requestAnimationFrame(_ncDraw);
}

// ── Draw node card ────────────────────────────────────────────────────────
function _ncDrawNode(ctx, nd, W, H) {
  const b   = _ncBox(nd, W, H);
  const st  = _nsNodeStatus(nd.id);
  const col = _ncCol(st);
  const sel = _ns.selected===nd.id && _ns.selType==='node';
  const dim = !!(nd.offline);

  ctx.save();
  if (dim) ctx.globalAlpha = 0.55;

  // Card shadow
  ctx.shadowColor = `rgba(26,82,118,${sel?.18:.07})`; ctx.shadowBlur=sel?14:4; ctx.shadowOffsetY=2;
  ctx.fillStyle = '#fff';
  _ncRR(ctx, b.x, b.y, b.w, b.h, _NN_R); ctx.fill();
  ctx.shadowBlur = 0; ctx.shadowOffsetY = 0;

  // Card border
  ctx.strokeStyle = col + (sel?'dd':'44'); ctx.lineWidth = sel?2.5:1.5;
  _ncRR(ctx, b.x, b.y, b.w, b.h, _NN_R); ctx.stroke();

  // Top accent bar
  ctx.fillStyle = col + '22';
  ctx.beginPath();
  ctx.moveTo(b.x+_NN_R, b.y);
  ctx.lineTo(b.x+b.w-_NN_R, b.y);
  ctx.quadraticCurveTo(b.x+b.w, b.y, b.x+b.w, b.y+_NN_R);
  ctx.lineTo(b.x+b.w, b.y+22);
  ctx.lineTo(b.x, b.y+22);
  ctx.lineTo(b.x, b.y+_NN_R);
  ctx.quadraticCurveTo(b.x, b.y, b.x+_NN_R, b.y);
  ctx.closePath(); ctx.fill();

  // Status dot
  ctx.save();
  if (st==='connecting') ctx.globalAlpha=.5+.5*Math.sin(Date.now()/400);
  ctx.fillStyle=col; ctx.beginPath();
  ctx.arc(b.x+b.w-10, b.y+11, 4, 0, Math.PI*2); ctx.fill();
  ctx.restore();

  // Icon
  ctx.font='18px serif'; ctx.textAlign='center'; ctx.textBaseline='middle';
  ctx.fillText(nd.icon, b.x+18, b.y+11);

  // Hardware label
  ctx.font='bold 11.5px "Segoe UI",system-ui,sans-serif';
  ctx.fillStyle='#1a2332'; ctx.textAlign='center'; ctx.textBaseline='top';
  ctx.fillText(nd.label, b.cx, b.y+26);

  // Role
  ctx.font='10px "Segoe UI",system-ui,sans-serif';
  ctx.fillStyle='#6b7a8d';
  ctx.fillText(nd.role, b.cx, b.y+41);

  // SW stack (1-2 lines)
  ctx.font='9px "Segoe UI",system-ui,sans-serif';
  ctx.fillStyle='#94a3b8';
  (nd.sw||[]).forEach((line,i) => ctx.fillText(line, b.cx, b.y+56+i*13));

  ctx.restore();
}

// ── Draw link ─────────────────────────────────────────────────────────────
function _ncDrawLink(ctx, lk, W, H) {
  const n1 = _ncNodeById(lk.from);
  const n2 = _ncNodeById(lk.to);
  if (!n1 || !n2) return;
  const b1 = _ncBox(n1, W, H);
  const b2 = _ncBox(n2, W, H);
  const st  = _nsLinkStatus(lk.id);
  const col = _ncCol(st);
  const sel = _ns.selected===lk.id && _ns.selType==='link';

  const ang12 = Math.atan2(b2.cy-b1.cy, b2.cx-b1.cx);
  const ang21 = ang12 + Math.PI;
  const p1 = _ncEdgePt(b1, ang12);
  const p2 = _ncEdgePt(b2, ang21);

  ctx.save();
  ctx.strokeStyle = col;
  ctx.lineWidth   = sel ? 3 : 2;
  ctx.lineCap     = 'round';
  if (st==='offline'||st==='disconnected'||st==='unknown') {
    ctx.setLineDash([5,6]); ctx.globalAlpha=.55;
  } else if (st==='connecting') {
    ctx.setLineDash([5,6]);
    ctx.lineDashOffset = -((Date.now()/80)%11);
    ctx.globalAlpha    = .55+.45*Math.sin(Date.now()/250);
  }
  ctx.beginPath(); ctx.moveTo(p1.x,p1.y); ctx.lineTo(p2.x,p2.y); ctx.stroke();
  ctx.restore();

  // Arrowhead
  if (st!=='offline'&&st!=='disconnected'&&st!=='unknown') {
    const AW=7;
    ctx.fillStyle=col;
    ctx.beginPath();
    ctx.moveTo(p2.x,p2.y);
    ctx.lineTo(p2.x-AW*Math.cos(ang12-Math.PI/6), p2.y-AW*Math.sin(ang12-Math.PI/6));
    ctx.lineTo(p2.x-AW*Math.cos(ang12+Math.PI/6), p2.y-AW*Math.sin(ang12+Math.PI/6));
    ctx.closePath(); ctx.fill();
  }

  // Protocol label pill
  const mx=(p1.x+p2.x)/2, my=(p1.y+p2.y)/2 - (lk.offline?0:18);
  const lbl = _nsLinkLabel(lk.id);
  ctx.save();
  ctx.globalAlpha = (st==='offline'||st==='unknown') ? 0.5 : 1;
  ctx.font='9.5px "Segoe UI",system-ui,sans-serif';
  const PW=ctx.measureText(lbl).width+14, PH=17;
  ctx.fillStyle   = sel ? col : '#fff';
  ctx.strokeStyle = col+(st==='offline'?'66':'99');
  ctx.lineWidth   = 1;
  _ncRR(ctx, mx-PW/2, my-PH/2, PW, PH, 8); ctx.fill(); ctx.stroke();
  ctx.fillStyle = sel?'#fff':col; ctx.textAlign='center'; ctx.textBaseline='middle';
  ctx.fillText(lbl, mx, my);
  ctx.restore();

  // Store hit region
  lk._hit = { p1, p2 };
}

// ── Rounded rect ─────────────────────────────────────────────────────────
function _ncRR(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x+r,y); ctx.lineTo(x+w-r,y); ctx.quadraticCurveTo(x+w,y,x+w,y+r);
  ctx.lineTo(x+w,y+h-r); ctx.quadraticCurveTo(x+w,y+h,x+w-r,y+h);
  ctx.lineTo(x+r,y+h); ctx.quadraticCurveTo(x,y+h,x,y+h-r);
  ctx.lineTo(x,y+r); ctx.quadraticCurveTo(x,y,x+r,y); ctx.closePath();
}

// ── Hit test ─────────────────────────────────────────────────────────────
function _ncHit(mx, my) {
  const c=_ncCanvas(); if (!c) return null;
  const W=c.width, H=c.height;
  // Nodes first (higher priority)
  for (const nd of _nNodes) {
    const b=_ncBox(nd,W,H);
    if (mx>=b.x&&mx<=b.x+b.w&&my>=b.y&&my<=b.y+b.h) return {type:'node',id:nd.id};
  }
  // Links
  for (const lk of _nLinks) {
    if (!lk._hit) continue;
    const {p1,p2}=lk._hit;
    const dx=p2.x-p1.x, dy=p2.y-p1.y, lq=dx*dx+dy*dy;
    const t=lq?Math.max(0,Math.min(1,((mx-p1.x)*dx+(my-p1.y)*dy)/lq)):0;
    if (Math.hypot(mx-(p1.x+t*dx),my-(p1.y+t*dy))<=14) return {type:'link',id:lk.id};
  }
  return null;
}

// ── Resize & init ─────────────────────────────────────────────────────────
function _ncResize() {
  const c=_ncCanvas(), area=document.getElementById('net-canvas-area');
  if (!c||!area) return;
  c.width=area.clientWidth; c.height=area.clientHeight;
  _ncDraw();
}

function initNetwork() {
  const c=_ncCanvas(); if (!c) return;
  if (!c._netWired) {
    c._netWired=true;
    c.addEventListener('click', evt => {
      const r=c.getBoundingClientRect();
      const h=_ncHit(evt.clientX-r.left, evt.clientY-r.top);
      _ns.selected=h?h.id:null; _ns.selType=h?h.type:null;
      _ncUpdatePanel(h); _ncDraw();
    });
    c.addEventListener('mousemove', evt => {
      const r=c.getBoundingClientRect();
      c.style.cursor=_ncHit(evt.clientX-r.left,evt.clientY-r.top)?'pointer':'default';
    });
    new ResizeObserver(()=>{ if (activeView==='network') _ncResize(); })
      .observe(document.getElementById('net-canvas-area'));
  }
  _ncResize();
  _ncUpdatePanel(null);
}

// ── State update ──────────────────────────────────────────────────────────
function updateNetworkFromState(state) {
  const mode = state.connection_mode || 'idle';
  const newHw    = state.hw_connecting ? 'connecting' :
                   (mode==='usb'||mode==='xbee') ? 'connected' :
                   mode==='sim' ? 'sim' : 'idle';
  const newType  = mode;
  const newLabel = mode==='sim'       ? 'Virtual (SIM)' :
                   mode==='xbee'      ? 'XBee 868 MHz'  :
                   mode==='usb'       ? 'USB Wired'      : 'Not connected';
  const newPort  = state.hw_serial_port || null;
  const newT40   = state.hw_t40 || 'unknown';
  const newJetIp = state.jetson_ip || '';

  const changed  = _ns.hw!==newHw || _ns.hwType!==newType || _ns.t40!==newT40 || _ns.hwPort!==newPort;
  _ns.hw=newHw; _ns.hwType=newType; _ns.hwLabel=newLabel;
  _ns.hwPort=newPort; _ns.t40=newT40; _ns.jetsonIp=newJetIp;

  if (!changed) return;
  // Refresh panel live values
  const badge = document.getElementById('np-hw-badge');
  const modeEl = document.getElementById('np-hw-mode');
  const btn   = document.getElementById('np-connect-btn');
  if (badge) { badge.textContent=_nsLabels[_ns.hw]||_ns.hw; badge.className='np-badge np-'+_ns.hw; }
  if (modeEl) modeEl.textContent = _ns.hwPort ? `${_ns.hwLabel} (${_ns.hwPort})` : _ns.hwLabel;
  if (btn)   btn.disabled = _ns.hw==='connecting';
  if (activeView==='network') {
    _ncDraw();
    if (_ns.hw==='connecting' && !_netAnimFrame)
      _netAnimFrame=requestAnimationFrame(_ncDraw);
  }
}

// ── Side panel ────────────────────────────────────────────────────────────
function _ncUpdatePanel(hit) {
  const title=document.getElementById('net-panel-title');
  const body =document.getElementById('net-panel-body');
  if (!title||!body) return;
  if (!hit) { title.textContent='Network'; body.innerHTML=_ncPanelOverview(); return; }
  if (hit.type==='link') {
    const lk=_nLinks.find(l=>l.id===hit.id)||{};
    const titles={hw:'PC ↔ Teensy 4.1',uart:'T4.1 ↔ T4.0 Intercom',sim:'Simulator ↔ PC',
                  xbee:'Jetson ↔ T4.1 XBee',espcam_j:'ESP32-CAM ↔ Jetson',espcam_p:'ESP32-CAM ↔ PC'};
    title.textContent=titles[hit.id]||hit.id;
    if (hit.id==='hw')          { body.innerHTML=_ncPanelHW(); _ncPanelHWInit(); }
    else if (hit.id==='sim')    body.innerHTML=_ncPanelSimLink();
    else if (hit.id==='uart')   body.innerHTML=_ncPanelUART();
    else if (hit.id==='xbee')   body.innerHTML=_ncPanelXBeeLink();
    else if (hit.id==='espcam_j'||hit.id==='espcam_p') body.innerHTML=_ncPanelEspCamLink();
  } else {
    const nd=_ncNodeById(hit.id)||{id:hit.id,icon:'?',label:hit.id,sw:[]};
    title.textContent=nd.label;
    if (hit.id==='jetson') body.innerHTML=_ncPanelJetson();
    else                   body.innerHTML=_ncPanelNode(nd);
  }
}

function _ncPanelOverview() {
  const hwPortStr = _ns.hwPort ? ` (${_ns.hwPort})` : '';
  const simSt = _ns.hwType==='sim' ? 'active' : 'disconnected';
  const jetSt = _ns.hwType==='xbee' ? 'connected' : 'offline';
  const t40St = _ns.t40==='ok' ? 'connected' : _ns.t40;
  return `<div class="np-hint">Click a node or link to inspect and configure.</div>
    <div class="np-section">
      <div class="np-row"><span>Connection</span><span class="np-badge np-${_ns.hw}">${_ns.hwLabel}${hwPortStr}</span></div>
      <div class="np-row"><span>Simulator</span><span class="np-badge np-${simSt}">${simSt==='active'?'Active':'Off'}</span></div>
      <div class="np-row"><span>PC ↔ T4.1</span><span class="np-badge np-${_ns.hw}">${_nsLabels[_ns.hw]||_ns.hw}</span></div>
      <div class="np-row"><span>T4.1 ↔ T4.0</span><span class="np-badge np-${t40St}">${_nsLabels[t40St]||t40St}</span></div>
      <div class="np-row"><span>Jetson</span><span class="np-badge np-${jetSt}">${jetSt==='connected'?'Connected':'Offline'}</span></div>
      <div class="np-row"><span>ESP32-CAM</span><span class="np-badge np-offline">Offline</span></div>
    </div>
    <div class="np-section">
      <div class="np-row"><span>Server</span><span>${window.location.host}</span></div>
      ${_ns.jetsonIp ? `<div class="np-row"><span>Jetson IP</span><span>${_ns.jetsonIp}</span></div>` : ''}
    </div>`;
}

function _ncPanelWS() {
  return `<div class="np-hint">WebSocket link no longer shown — browser and holOS server are both on the PC node.</div>`;
}

function _ncPanelHW() {
  const portStr = _ns.hwPort ? ` (${_ns.hwPort})` : '';
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${_ns.hw}" id="np-hw-badge">${_nsLabels[_ns.hw]||_ns.hw}</span></div>
      <div class="np-row"><span>Mode</span><span id="np-hw-mode">${_ns.hwLabel}${portStr}</span></div>
    </div>
    <div class="np-section">
      <label class="np-label">Serial port</label>
      <div style="display:flex;gap:6px;margin-bottom:8px">
        <select id="np-port-sel" class="input-sm" style="flex:1;min-width:0"></select>
        <button class="btn-tiny" onclick="_ncRefreshPorts()" title="Refresh">↺</button>
      </div>
      <label class="np-label">Bridge mode</label>
      <select id="np-mode-sel" class="input-sm" style="width:100%;margin-bottom:10px">
        <option value="wired" selected>USB Wired — 115 200 bps</option>
        <option value="xbee">XBee / Jetson — 31 250 bps</option>
      </select>
      <div style="display:flex;gap:6px">
        <button id="np-connect-btn" class="btn green btn-sm" style="flex:1" onclick="_ncDoConnect()">Connect HW</button>
        <button class="btn red btn-sm" style="flex:1" onclick="serialDisconnect()">Disconnect</button>
      </div>
      <div style="margin-top:8px;padding-top:8px;border-top:1px solid rgba(255,255,255,.1)">
        <button class="btn btn-sm" style="width:100%;background:#6366f1;color:#fff" onclick="connectSim()">▶ Start Simulator</button>
      </div>
    </div>
    <div class="np-hint"><b>USB Wired:</b> T4.1 connects directly over USB-CDC @ 115 200 bps.<br>
      <b>XBee / Jetson:</b> holOS runs on the Jetson; T4.1 communicates via XBee radio @ 31 250 bps.<br>
      <b>Simulator:</b> Virtual robot with physics — no hardware needed.</div>`;
}

function _ncPanelHWInit() {
  _ncRefreshPorts();
  const mm=document.getElementById('serial-mode-sel'), nm=document.getElementById('np-mode-sel');
  if (mm&&nm) nm.value=mm.value;
  const btn=document.getElementById('np-connect-btn');
  if (btn) btn.disabled=_ns.hw==='connecting';
}

function _ncRefreshPorts() {
  const sel=document.getElementById('np-port-sel'); if (!sel) return;
  const cur=(document.getElementById('serial-port-sel')||{}).value||'';
  fetch('/api/serial/ports').then(r=>r.json()).then(ports=>{
    sel.innerHTML='<option value="">— port —</option>'+
      ports.map(p=>`<option value="${escHtml(p.port)}"${p.port===cur?' selected':''}>${escHtml(p.port)} — ${escHtml(p.desc)}</option>`).join('');
  });
}

function _ncDoConnect() {
  const ps=document.getElementById('np-port-sel'), ms=document.getElementById('np-mode-sel');
  if (!ps?.value) { showToast('Select a serial port first'); return; }
  const mp=document.getElementById('serial-port-sel'), mm=document.getElementById('serial-mode-sel');
  if (mp) mp.value=ps.value;
  if (mm) mm.value=ms?ms.value:'wired';
  serialConnect();
}

function _ncPanelUART() {
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${_ns.t40}">${_nsLabels[_ns.t40]||_ns.t40}</span></div>
      <div class="np-row"><span>Protocol</span><span>UART (Intercom)</span></div>
      <div class="np-row"><span>Baudrate</span><span>31 250 bps</span></div>
      <div class="np-row"><span>Direction</span><span>Bidirectional</span></div>
    </div>
    <div class="np-hint">T4.0 status is reported by T4.1 firmware via TEL:t40 telemetry when connected.</div>`;
}

function _ncPanelSimLink() {
  const active = _ns.hwType === 'sim';
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${active?'active':'disconnected'}">${active?'Active':'Inactive'}</span></div>
      <div class="np-row"><span>Type</span><span>In-process VirtualTransport</span></div>
      <div class="np-row"><span>Physics</span><span>60 Hz loop (SimBridge)</span></div>
    </div>
    <div class="np-section">
      <button class="btn btn-sm" style="width:100%;background:#6366f1;color:#fff" onclick="connectSim()">▶ Start Simulator</button>
    </div>
    <div class="np-hint">The simulator runs the full robot physics locally: motion, pathfinding, occupancy mapping, and game objects.</div>`;
}

function _ncPanelXBeeLink() {
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-offline">Offline</span></div>
      <div class="np-row"><span>Protocol</span><span>XBee 868 MHz UART</span></div>
      <div class="np-row"><span>Baudrate</span><span>31 250 bps</span></div>
    </div>
    <div class="np-hint">This path is used when holOS is deployed on the Jetson. The Jetson relays commands to T4.1 via XBee radio. Configure the Jetson node IP to enable this route.</div>`;
}

function _ncPanelEspCamLink() {
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-offline">Offline</span></div>
      <div class="np-row"><span>Protocol</span><span>Serial2 · WiFi MJPEG</span></div>
      <div class="np-row"><span>Baudrate</span><span>115 200 bps</span></div>
    </div>
    <div class="np-hint">The ESP32-CAM streams MJPEG video over WiFi and receives commands via T4.1 Serial2.</div>`;
}

function _ncPanelJetson() {
  const ip = _ns.jetsonIp || '';
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-offline">Offline</span></div>
      <div class="np-row"><span>Platform</span><span>NVIDIA Jetson Nano/NX</span></div>
      <div class="np-row"><span>Software</span><span>holOS Server · ROS2</span></div>
    </div>
    <div class="np-section">
      <label class="np-label">Jetson IP address</label>
      <div style="display:flex;gap:6px;margin-bottom:8px">
        <input id="np-jetson-ip" type="text" class="input-sm" style="flex:1"
               placeholder="192.168.x.x" value="${escHtml(ip)}">
        <button class="btn-tiny green" onclick="_ncSaveJetsonIp()">Save</button>
      </div>
      <label class="np-label">SSH user</label>
      <input id="np-jetson-user" type="text" class="input-sm" style="width:100%;margin-bottom:8px"
             placeholder="robot" value="robot">
    </div>
    <div class="np-hint">Set the Jetson IP to enable the XBee/Jetson bridge route. holOS will connect via SSH to launch the remote server.</div>`;
}

function _ncSaveJetsonIp() {
  const ip   = (document.getElementById('np-jetson-ip')   ||{}).value||'';
  const user = (document.getElementById('np-jetson-user') ||{}).value||'robot';
  fetch('/api/jetson', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ip, user})
  }).then(()=>{ _ns.jetsonIp=ip; showToast('Jetson config saved'); });
}

function _ncPanelNode(nd) {
  const st=_nsNodeStatus(nd.id);
  const swHtml = (nd.sw||[]).map(s=>`<div class="np-row"><span style="opacity:.6">SW</span><span>${escHtml(s)}</span></div>`).join('');
  return `<div class="np-section">
      <div class="np-title-row">${nd.icon} ${nd.label}</div>
      <div class="np-divider"></div>
      <div class="np-row"><span>Status</span><span class="np-badge np-${st}">${_nsLabels[st]||st}</span></div>
      <div class="np-row"><span>Role</span><span>${escHtml(nd.role||'')}</span></div>
      ${swHtml}
    </div>`;
}

// ══════════════════════════════════════════════════════════════════════════
//  INIT
// ══════════════════════════════════════════════════════════════════════════
window.addEventListener('load', () => {
  // Start on map view — hide mini-map, trigger canvas sizing
  document.getElementById('mini-map-wrap').style.display = 'none';

  // Use two rAFs so the browser has fully laid out flex children before we measure
  requestAnimationFrame(() => requestAnimationFrame(resizeCanvas));
  cgSetWs(false);
  cgSetRobot(false);

  // Feedrate display
  const fr=document.getElementById('feedrate');
  if (fr) document.getElementById('feedrate-val').textContent=parseFloat(fr.value).toFixed(2)+'×';

  // Init CodeMirror for strategy editor
  initCodeMirror();

  // Load data
  loadMacros();
  serialRefreshPorts();
  initTests();
});

// Re-init CodeMirror when switching to strategy view (lazy)
const _origSwitchView = window.switchView;


// ══════════════════════════════════════════════════════════════════════════════
//  TESTS VIEW
// ══════════════════════════════════════════════════════════════════════════════

// ── State ────────────────────────────────────────────────────────────────────
let _testCatalog   = {};         // {suiteId: {label, icon, tests: [...]}}
let _testStatus    = {};         // {testId: 'idle'|'running'|'passed'|'failed'}
let _testDuration  = {};         // {testId: ms}
let _testRunning   = false;
let _selectedSuite = null;       // suite highlighted in sidebar

const TEST_ICONS = {
  idle:     '○',
  running:  '⋯',
  passed:   '✓',
  failed:   '✗',
  stopped:  '—',
  skipped:  '⊖',
  prompt:   '⏸',
};

// ── Init ─────────────────────────────────────────────────────────────────────
function initTests() {
  // Fetch (or re-fetch) catalog from server — called on init and on catalog change.
  fetch('/api/tests/catalog')
    .then(r => r.json())
    .then(catalog => {
      _testCatalog = catalog;
      // Init status to idle for every test
      Object.values(catalog).forEach(suite => {
        suite.tests.forEach(t => {
          _testStatus[t.id]   = 'idle';
          _testDuration[t.id] = null;
        });
      });
      _buildCatalogUI();
    })
    .catch(() => {
      // Server may not be running yet — silently ignore
    });
}

// SocketIO listeners registered ONCE (not inside initTests to avoid duplicates)
socket.on('test_progress', data => {
  _setTestStatus(data.id, data.status);
  if (data.status === 'running') {
    _logEntry(data.id, 'running', '…', null);
  }
});

socket.on('test_result', data => {
  const status = data.passed ? 'passed' : 'failed';
  _setTestStatus(data.id, status, data.duration_ms);
  _logEntry(data.id, status, data.msg, data.duration_ms);
  _updateSummary();
});

socket.on('test_done', () => {
  _testRunning = false;
  _updateButtons();
  _updateSummary();
});

// Re-fetch catalog when connection mode changes (sim ↔ hardware ↔ idle)
socket.on('tests_catalog_changed', data => {
  _testStatus   = {};
  _testDuration = {};
  _testCatalog  = {};
  _testRunning  = false;
  _updateButtons();
  if (data && data.mode === 'idle') {
    clearTestLog();
    _logHeader('⚠ Connexion perdue — reconnectez le robot');
  }
  initTests();
});

// Interactive test prompt — show overlay, wait for user to click Continue
socket.on('test_prompt', data => {
  _showTestPrompt(data.msg || '');
});

// ── Catalog UI ───────────────────────────────────────────────────────────────
function _buildCatalogUI() {
  const container = document.getElementById('tests-catalog');
  if (!container) return;
  container.innerHTML = '';

  // Show "not connected" placeholder when catalog is empty
  if (Object.keys(_testCatalog).length === 0) {
    container.innerHTML = `
      <div style="
        padding:32px 16px;
        text-align:center;
        color:var(--text-dim);
        font-size:12px;
        line-height:1.7;
      ">
        <div style="font-size:2rem;margin-bottom:10px">🔌</div>
        <div style="font-weight:600;margin-bottom:6px">Robot non connecté</div>
        <div>Connectez-vous au robot (USB ou XBee)<br>pour accéder aux tests matériels.</div>
      </div>`;
    // Also disable toolbar buttons
    ['btn-run-all','btn-run-suite'].forEach(id => {
      const b = document.getElementById(id);
      if (b) b.disabled = true;
    });
    return;
  }

  Object.entries(_testCatalog).forEach(([suiteId, suite]) => {
    const suiteEl = document.createElement('div');
    suiteEl.className = 'test-suite';
    suiteEl.id = `test-suite-${suiteId}`;

    // Header
    const hdr = document.createElement('div');
    hdr.className = 'test-suite-header';
    hdr.innerHTML = `
      <span class="test-suite-icon">${suite.icon}</span>
      <span>${suite.label}</span>
      <button class="test-suite-run" title="Run ${suite.label}"
              onclick="testRunSuite('${suiteId}',event)">▶</button>`;
    hdr.addEventListener('click', e => {
      if (e.target.classList.contains('test-suite-run')) return;
      _selectSuite(suiteId);
    });
    suiteEl.appendChild(hdr);

    // Test items
    suite.tests.forEach(t => {
      const item = document.createElement('div');
      item.className = 'test-item idle';
      item.id = `test-item-${t.id}`;
      item.title = t.desc;
      item.innerHTML = `
        <span class="test-item-icon idle" id="test-icon-${t.id}">${TEST_ICONS.idle}</span>
        <span class="test-item-name">${t.name}</span>
        <span class="test-item-dur" id="test-dur-${t.id}"></span>`;
      item.addEventListener('click', () => testRunOne(t.id));
      suiteEl.appendChild(item);
    });

    container.appendChild(suiteEl);
  });
}

function _selectSuite(suiteId) {
  _selectedSuite = suiteId;
  // Highlight header
  document.querySelectorAll('.test-suite-header').forEach(h => h.classList.remove('selected'));
  const hdr = document.querySelector(`#test-suite-${suiteId} .test-suite-header`);
  if (hdr) hdr.classList.add('selected');
}

// ── Status updates ───────────────────────────────────────────────────────────
function _setTestStatus(id, status, durMs) {
  _testStatus[id] = status;
  if (durMs != null) _testDuration[id] = durMs;

  const item = document.getElementById(`test-item-${id}`);
  const icon = document.getElementById(`test-icon-${id}`);
  const dur  = document.getElementById(`test-dur-${id}`);

  if (item) {
    item.className = `test-item ${status}`;
  }
  if (icon) {
    icon.className = `test-item-icon ${status}`;
    icon.textContent = TEST_ICONS[status] || '?';
  }
  if (dur && durMs != null) {
    dur.textContent = durMs < 1000 ? `${durMs}ms` : `${(durMs/1000).toFixed(1)}s`;
  }
}

// ── Log ──────────────────────────────────────────────────────────────────────
function _logEntry(id, status, msg, durMs) {
  const log = document.getElementById('tests-log');
  if (!log) return;

  // Remove existing running entry for the same id
  const existing = document.getElementById(`tlog-${id}`);
  if (existing) existing.remove();

  const entry = document.createElement('div');
  entry.className = `tlog-entry ${status}`;
  entry.id = `tlog-${id}`;

  const icon = { running:'⋯', passed:'✓', failed:'✗', stopped:'—', skipped:'⊖' }[status] || '?';
  const durStr = durMs != null
    ? (durMs < 1000 ? `${durMs}ms` : `${(durMs/1000).toFixed(1)}s`)
    : '';
  // Get test name from catalog
  const meta = _findTest(id);
  const name = meta ? meta.name : id;

  entry.innerHTML = `
    <span class="tlog-icon">${icon}</span>
    <span class="tlog-id">${name}</span>
    <span class="tlog-msg">${_escHtml(msg)}</span>
    <span class="tlog-dur">${durStr}</span>`;

  log.appendChild(entry);
  log.scrollTop = log.scrollHeight;
}

function _logHeader(text) {
  const log = document.getElementById('tests-log');
  if (!log) return;
  const entry = document.createElement('div');
  entry.className = 'tlog-entry header';
  entry.textContent = text;
  log.appendChild(entry);
  log.scrollTop = log.scrollHeight;
}

function clearTestLog() {
  const log = document.getElementById('tests-log');
  if (log) log.innerHTML = '';
}

// ── Interactive test prompt overlay ──────────────────────────────────────────
function _showTestPrompt(msg) {
  // Reuse existing overlay or create one
  let overlay = document.getElementById('test-prompt-overlay');
  if (!overlay) {
    overlay = document.createElement('div');
    overlay.id = 'test-prompt-overlay';
    overlay.style.cssText = [
      'position:fixed', 'inset:0', 'z-index:9999',
      'display:flex', 'align-items:center', 'justify-content:center',
      'background:rgba(0,0,0,0.60)', 'backdrop-filter:blur(4px)',
    ].join(';');
    document.body.appendChild(overlay);
  }

  // Detect step indicators like "ÉTAPE 1/3 — Title\n\n..." and render them nicely
  let title = '';
  let body  = _escHtml(msg);
  const stepMatch = msg.match(/^(ÉTAPE\s+\d+\/\d+\s*[—\-–]\s*[^\n]+)\n+([\s\S]*)$/);
  if (stepMatch) {
    title = _escHtml(stepMatch[1]);
    body  = _escHtml(stepMatch[2].trim());
  }

  overlay.innerHTML = `
    <div style="
      background:#1a2235;
      border:1px solid #3a4a65;
      border-radius:12px;
      padding:28px 32px;
      max-width:520px;
      width:92%;
      box-shadow:0 12px 48px rgba(0,0,0,0.7);
      text-align:center;
    ">
      <div style="font-size:2rem;margin-bottom:10px">⏸</div>
      ${title ? `<div style="
        font-size:0.85rem;
        font-weight:700;
        letter-spacing:.06em;
        text-transform:uppercase;
        color:#7eb8e8;
        margin-bottom:14px;
      ">${title}</div>` : ''}
      <div style="
        font-size:0.93rem;
        color:#d8e4f2;
        margin-bottom:26px;
        line-height:1.65;
        white-space:pre-wrap;
        text-align:left;
      ">${body}</div>
      <button onclick="_testPromptAck()" style="
        background:#2980b9;
        color:#ffffff;
        border:none;
        border-radius:6px;
        padding:10px 32px;
        font-size:0.95rem;
        font-weight:600;
        cursor:pointer;
        letter-spacing:.03em;
      ">Continuer ▶</button>
    </div>`;

  overlay.style.display = 'flex';

  // Also add a log entry so the test log shows the pause
  _logHeader('⏸ ' + (title || msg).split('\n')[0]);
}

function _testPromptAck() {
  const overlay = document.getElementById('test-prompt-overlay');
  if (overlay) overlay.style.display = 'none';
  socket.emit('test_prompt_ack');
  _logHeader('▶ Continuer');
}

// ── Summary badge ────────────────────────────────────────────────────────────
function _updateSummary() {
  const badge = document.getElementById('test-summary');
  if (!badge) return;

  const total  = Object.keys(_testStatus).length;
  const passed = Object.values(_testStatus).filter(s => s === 'passed').length;
  const failed = Object.values(_testStatus).filter(s => s === 'failed').length;
  const ran    = passed + failed;

  if (ran === 0) {
    badge.textContent = '';
    badge.style.color = '';
  } else {
    badge.textContent = `${passed}/${ran}`;
    badge.style.color = failed > 0 ? 'var(--red)' : 'var(--green)';
  }
}

function _updateButtons() {
  const btnRunAll   = document.getElementById('btn-run-all');
  const btnRunSuite = document.getElementById('btn-run-suite');
  const btnStop     = document.getElementById('btn-stop-tests');
  if (btnRunAll)   btnRunAll.disabled   = _testRunning;
  if (btnRunSuite) btnRunSuite.disabled = _testRunning;
  if (btnStop)     btnStop.disabled     = !_testRunning;
}

// ── Public API ───────────────────────────────────────────────────────────────
function testRunAll() {
  if (_testRunning) return;
  _resetAllStatus();
  _logHeader('▶ Run All');
  _startRun();
  socket.emit('run_tests', {});
}

function testRunSuite(suiteId, evt) {
  if (evt) evt.stopPropagation();
  const sid = suiteId || _selectedSuite;
  if (!sid || _testRunning) return;
  _resetSuiteStatus(sid);
  const label = (_testCatalog[sid] || {}).label || sid;
  _logHeader(`▶ Suite: ${label}`);
  _startRun();
  socket.emit('run_tests', { suite: sid });
}

function testRunOne(testId) {
  if (_testRunning) return;
  _setTestStatus(testId, 'idle');
  const meta = _findTest(testId);
  _logHeader(`▶ ${meta ? meta.name : testId}`);
  _startRun();
  socket.emit('run_tests', { test: testId });
}

function testStop() {
  socket.emit('stop_tests');
}

// ── Helpers ──────────────────────────────────────────────────────────────────
function _startRun() {
  _testRunning = true;
  _updateButtons();
}

function _resetAllStatus() {
  Object.keys(_testStatus).forEach(id => {
    _testStatus[id] = 'idle';
    _setTestStatus(id, 'idle');
  });
  _updateSummary();
}

function _resetSuiteStatus(suiteId) {
  const suite = _testCatalog[suiteId];
  if (!suite) return;
  suite.tests.forEach(t => {
    _testStatus[t.id] = 'idle';
    _setTestStatus(t.id, 'idle');
  });
  _updateSummary();
}

function _findTest(id) {
  for (const suite of Object.values(_testCatalog)) {
    const t = suite.tests.find(t => t.id === id);
    if (t) return t;
  }
  return null;
}

function _escHtml(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;');
}


// ══════════════════════════════════════════════════════════════════════════
//  MODULES VIEW
//  Telemetry channel toggles + firmware service controls
// ══════════════════════════════════════════════════════════════════════════

const _TEL_CHANNELS = [
  { id: 'pos',    name: 'Position',   desc: 'X / Y / θ from the odometry estimator', freq: '10 Hz' },
  { id: 'motion', name: 'Motion',     desc: 'RUNNING / IDLE state + current target',  freq: '10 Hz' },
  { id: 'safety', name: 'Safety',     desc: 'Obstacle detection & collision flags',   freq: '10 Hz' },
  { id: 'chrono', name: 'Chrono',     desc: 'Match elapsed time',                     freq: '10 Hz' },
  { id: 'occ',    name: 'Occupancy',  desc: 'Compressed lidar occupancy grid',        freq: '2 Hz'  },
];

// Firmware services we can toggle via enable(SVC) / disable(SVC)
const _HW_SERVICES = [
  { id: 'SAFETY',  name: 'Safety',   desc: 'Lidar-based obstacle detection & emergency stop' },
  { id: 'LIDAR',   name: 'Lidar',    desc: 'Lidar scan processing & occupancy map update'     },
  { id: 'MOTION',  name: 'Motion',   desc: 'PID motion controller & trajectory planner'       },
  { id: 'CHRONO',  name: 'Chrono',   desc: 'Match timer (start / stop / reset)'               },
];

// Client-side override map: while a tel toggle command is in-flight we keep the
// optimistic state here so the 10-Hz server push doesn't flicker the toggle back.
const _telOverride = {};  // { channelId: { value: bool, ts: ms } }
const TEL_OVERRIDE_TTL = 2000;  // ms — give up after 2 s if server never confirms

let _modInited   = false;  // static card markup created
let _modUpdating = false;  // guard against re-entrant updates

// ── initModulesView ───────────────────────────────────────────────────────────
// Build the static card/row markup once. Called on first switchView('modules').
function initModulesView() {
  if (_modInited) return;
  _modInited = true;

  // ── Telemetry cards ─────────────────────────────────────────────────────────
  const grid = document.getElementById('tel-grid');
  if (grid) {
    grid.innerHTML = _TEL_CHANNELS.map(ch => `
      <div class="tel-card" id="tel-card-${ch.id}">
        <div class="tel-card-name">${ch.name}</div>
        <div class="tel-card-desc">${ch.desc}</div>
        <div class="tel-card-footer">
          <span class="tel-freq">${ch.freq}</span>
          <label class="toggle-sw" title="Toggle ${ch.name} telemetry">
            <input type="checkbox" id="tel-chk-${ch.id}"
                   onchange="modSetTelChannel('${ch.id}', this.checked)">
            <span class="toggle-slider"></span>
          </label>
        </div>
      </div>`).join('');
  }

  // ── Service rows ─────────────────────────────────────────────────────────────
  const list = document.getElementById('svc-list');
  if (list) {
    list.innerHTML = _HW_SERVICES.map(svc => `
      <div class="svc-row" id="svc-row-${svc.id}">
        <span class="svc-dot disabled" id="svc-dot-${svc.id}"></span>
        <span class="svc-name">${svc.name}</span>
        <span class="svc-desc">${svc.desc}</span>
        <span class="svc-btns">
          <button class="btn-svc enable"  onclick="modSvcCmd('enable','${svc.id}')"  title="Enable ${svc.name}">Enable</button>
          <button class="btn-svc disable" onclick="modSvcCmd('disable','${svc.id}')" title="Disable ${svc.name}">Disable</button>
        </span>
      </div>`).join('');
  }
}

// ── updateModulesView ─────────────────────────────────────────────────────────
// Called on every state push (10 Hz) and on view enter.
function updateModulesView(state) {
  if (!_modInited || _modUpdating) return;
  _modUpdating = true;

  const connected = !!(state && state.hw_mode);
  const telMask   = (state && state.hw_tel_mask) ? state.hw_tel_mask : {};

  // ── Banner ───────────────────────────────────────────────────────────────────
  const banner  = document.getElementById('mod-banner');
  const dot     = document.getElementById('mod-banner-dot');
  const bannerT = document.getElementById('mod-banner-text');
  if (banner && dot && bannerT) {
    if (connected) {
      const hwType = (state.hw_type || 'usb').toUpperCase();
      banner.classList.add('connected');
      dot.className    = 'mod-status-dot on';
      bannerT.textContent = `Connected — ${hwType} bridge active`;
    } else {
      banner.classList.remove('connected');
      dot.className    = 'mod-status-dot';
      bannerT.textContent = 'Not connected — connect to a robot to manage modules';
    }
  }

  // ── Telemetry toggles ────────────────────────────────────────────────────────
  const now = Date.now();
  _TEL_CHANNELS.forEach(ch => {
    const chk  = document.getElementById(`tel-chk-${ch.id}`);
    const card = document.getElementById(`tel-card-${ch.id}`);
    if (!chk) return;

    // Honour client-side override until TTL expires or server confirms new value
    const ov = _telOverride[ch.id];
    let enabled;
    if (ov && (now - ov.ts) < TEL_OVERRIDE_TTL) {
      enabled = ov.value;
      // If server has caught up, clear override
      if (telMask[ch.id] !== undefined && telMask[ch.id] === ov.value) {
        delete _telOverride[ch.id];
      }
    } else {
      delete _telOverride[ch.id];
      enabled = telMask[ch.id] !== undefined ? telMask[ch.id] : true;
    }

    chk.checked  = enabled;
    chk.disabled = !connected;
    if (card) {
      card.classList.toggle('active',   enabled);
      card.classList.toggle('inactive', !enabled);
    }
  });

  // ── Service rows (no server state yet — just reflect connected/disconnected) ──
  _HW_SERVICES.forEach(svc => {
    const dot = document.getElementById(`svc-dot-${svc.id}`);
    const btns = document.querySelectorAll(`#svc-row-${svc.id} .btn-svc`);
    if (dot) {
      dot.className = connected ? 'svc-dot enabled' : 'svc-dot disabled';
    }
    btns.forEach(b => { b.disabled = !connected; });
  });

  _modUpdating = false;
}

// ── modSetTelChannel ──────────────────────────────────────────────────────────
// Called when a telemetry toggle is clicked.
function modSetTelChannel(channelId, enabled) {
  // Optimistic update — prevent the 10-Hz state push from reverting the toggle
  _telOverride[channelId] = { value: enabled, ts: Date.now() };

  const cmd = `tel(${channelId},${enabled ? 1 : 0})`;
  socket.emit('hw_fire', { cmd });
}

// ── modSvcCmd ─────────────────────────────────────────────────────────────────
// Enable / disable a firmware service.
function modSvcCmd(action, serviceId) {
  const cmd = `${action}(${serviceId})`;
  socket.emit('hw_fire', { cmd });
  showToast(`${action}(${serviceId}) sent`);
}

// ═════════════════════════════════════════════════════════════════════════════
//  CALIBRATION VIEW
// ═════════════════════════════════════════════════════════════════════════════

// ── State ─────────────────────────────────────────────────────────────────────
let _calibDirty = {};   // which groups have unsaved local changes
let _calibData  = {     // local cache matching Python defaults
  cx: 1.089, cy: -1.089, cr: 0.831,
  ha: 1.0,   hb: 1.0,   hc: 1.0,
  ol: 0.9714, oa: 1.0,
};

// ── Init ──────────────────────────────────────────────────────────────────────
function calibInit() {
  fetch('/api/calibration')
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        _calibData = Object.assign(_calibData, d.calib);
        calibRefreshUI(d.connected);
      }
    })
    .catch(() => {});
}

// ── Socket event — server pushed a calibration update ─────────────────────────
socket.on('calib_updated', data => {
  if (data && data.calib) {
    _calibData = Object.assign(_calibData, data.calib);
    _calibDirty = {};
    calibRefreshInputs();
    _calibStatusMsg('✓ Calibration synchronisée depuis le robot');
  }
});

// ── Refresh UI from _calibData ─────────────────────────────────────────────────
function calibRefreshInputs() {
  const fields = ['cx','cy','cr','ha','hb','hc','ol','oa'];
  fields.forEach(k => {
    const el = document.getElementById(`calib-${k}`);
    if (el && document.activeElement !== el) {
      el.value = _calibData[k];
      el.classList.remove('dirty');
    }
  });
}

function calibRefreshUI(connected) {
  calibRefreshInputs();
  const dot  = document.getElementById('calib-banner-dot');
  const text = document.getElementById('calib-banner-text');
  if (!dot || !text) return;
  if (connected) {
    dot.style.background = '#22c55e';
    text.textContent = 'Robot connecté — les commandes sont envoyées en temps réel';
  } else {
    dot.style.background = '#6b7280';
    text.textContent = 'Non connecté — les valeurs seront appliquées à la connexion';
  }
}

// ── Mark a group as dirty (user edited an input) ──────────────────────────────
function calibMarkDirty(group) {
  _calibDirty[group] = true;
  // Highlight changed inputs
  const groups = {
    cart:   ['cx','cy','cr'],
    holo:   ['ha','hb','hc'],
    otos_l: ['ol'],
    otos_a: ['oa'],
  };
  (groups[group] || []).forEach(k => {
    const el = document.getElementById(`calib-${k}`);
    if (el) el.classList.add('dirty');
  });
}

// ── Apply helpers ─────────────────────────────────────────────────────────────
function _calibPost(payload) {
  return fetch('/api/calibration', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(payload),
  })
  .then(r => r.json())
  .then(d => {
    if (d.ok) {
      _calibData = Object.assign(_calibData, d.calib);
      _calibStatusMsg('✓ Appliqué');
    } else {
      _calibStatusMsg('✗ Erreur: ' + (d.error || '?'));
    }
    return d;
  });
}

function _calibGetFloat(id) {
  const el = document.getElementById(id);
  return el ? parseFloat(el.value) : NaN;
}

function calibApplyCart() {
  const cx = _calibGetFloat('calib-cx');
  const cy = _calibGetFloat('calib-cy');
  const cr = _calibGetFloat('calib-cr');
  if (isNaN(cx) || isNaN(cy) || isNaN(cr)) return;
  _calibPost({ cx, cy, cr }).then(() => {
    ['cx','cy','cr'].forEach(k => {
      const el = document.getElementById(`calib-${k}`);
      if (el) el.classList.remove('dirty');
    });
    delete _calibDirty.cart;
  });
}

function calibApplyHolo() {
  const ha = _calibGetFloat('calib-ha');
  const hb = _calibGetFloat('calib-hb');
  const hc = _calibGetFloat('calib-hc');
  if (isNaN(ha) || isNaN(hb) || isNaN(hc)) return;
  _calibPost({ ha, hb, hc }).then(() => {
    ['ha','hb','hc'].forEach(k => {
      const el = document.getElementById(`calib-${k}`);
      if (el) el.classList.remove('dirty');
    });
    delete _calibDirty.holo;
  });
}

function calibApplyOtosLinear() {
  const ol = _calibGetFloat('calib-ol');
  if (isNaN(ol)) return;
  if (ol < 0.872 || ol > 1.127) {
    _calibStatusMsg('⚠ Valeur hors plage OTOS (0.872–1.127)');
    return;
  }
  _calibPost({ ol }).then(() => {
    const el = document.getElementById('calib-ol');
    if (el) el.classList.remove('dirty');
    delete _calibDirty.otos_l;
  });
}

function calibApplyOtosAngular() {
  const oa = _calibGetFloat('calib-oa');
  if (isNaN(oa)) return;
  if (oa < 0.872 || oa > 1.127) {
    _calibStatusMsg('⚠ Valeur hors plage OTOS (0.872–1.127)');
    return;
  }
  _calibPost({ oa }).then(() => {
    const el = document.getElementById('calib-oa');
    if (el) el.classList.remove('dirty');
    delete _calibDirty.otos_a;
  });
}

function calibApplyAll() {
  const cx = _calibGetFloat('calib-cx'), cy = _calibGetFloat('calib-cy'), cr = _calibGetFloat('calib-cr');
  const ha = _calibGetFloat('calib-ha'), hb = _calibGetFloat('calib-hb'), hc = _calibGetFloat('calib-hc');
  const ol = _calibGetFloat('calib-ol'), oa = _calibGetFloat('calib-oa');
  _calibPost({ cx, cy, cr, ha, hb, hc, ol, oa }).then(() => {
    _calibDirty = {};
    document.querySelectorAll('.calib-input').forEach(el => el.classList.remove('dirty'));
  });
}

// ── SD card operations ────────────────────────────────────────────────────────
function calibSave() {
  fetch('/api/calibration/save', { method: 'POST' })
    .then(r => r.json())
    .then(d => _calibStatusMsg(d.ok ? '💾 Sauvegardé sur SD' : '✗ Erreur: ' + d.error));
}

function calibLoad() {
  fetch('/api/calibration/load', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        if (d.calib) {
          _calibData = Object.assign(_calibData, d.calib);
          _calibDirty = {};
          calibRefreshInputs();
        }
        _calibStatusMsg('📂 Chargé depuis SD');
      } else {
        _calibStatusMsg('✗ Erreur: ' + d.error);
      }
    });
}

function calibReset() {
  if (!confirm('Remettre tous les paramètres aux valeurs par défaut ?')) return;
  fetch('/api/calibration/reset', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.ok && d.calib) {
        _calibData = Object.assign(_calibData, d.calib);
        _calibDirty = {};
        calibRefreshInputs();
        _calibStatusMsg('↺ Réinitialisé aux défauts');
      }
    });
}

// ── Measurement tool ──────────────────────────────────────────────────────────
function calibMeasure() {
  const dist = parseFloat(document.getElementById('calib-measure-dist').value);
  if (isNaN(dist) || dist < 50 || dist > 3000) {
    _calibStatusMsg('⚠ Distance invalide (50–3000 mm)');
    return;
  }
  const btn = document.getElementById('btn-calib-measure');
  const res = document.getElementById('calib-measure-result');
  btn.disabled = true;
  btn.textContent = '⏳ En cours...';
  res.classList.add('hidden');
  res.textContent = '';

  fetch('/api/calibration/measure', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ dist_mm: dist }),
  })
  .then(r => r.json())
  .then(d => {
    btn.disabled = false;
    btn.textContent = '📏 Mesurer';
    if (d.ok) {
      res.textContent = d.result || '(pas de réponse)';
      res.classList.remove('hidden');
      _calibStatusMsg('✓ Mesure terminée');
    } else {
      res.textContent = 'Erreur: ' + (d.error || '?');
      res.classList.remove('hidden');
      _calibStatusMsg('✗ ' + (d.error || 'Erreur'));
    }
  })
  .catch(e => {
    btn.disabled = false;
    btn.textContent = '📏 Mesurer';
    _calibStatusMsg('✗ ' + e.message);
  });
}

// ── Status message ─────────────────────────────────────────────────────────────
let _calibStatusTimer = null;
function _calibStatusMsg(msg) {
  const el = document.getElementById('calib-status-msg');
  if (!el) return;
  el.textContent = msg;
  clearTimeout(_calibStatusTimer);
  _calibStatusTimer = setTimeout(() => { el.textContent = ''; }, 5000);
}

// ── View activation hook ──────────────────────────────────────────────────────
// Called by switchView() when the calibration tab is opened
function onCalibViewActivated() {
  calibInit();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  VISION VIEW
// ═══════════════════════════════════════════════════════════════════════════════

// ── State ──────────────────────────────────────────────────────────────────────
let _visionObjects     = [];
let _visionActiveObjId = null;
let _visionFrameInfo   = {};
let _visionEnabled     = false;

// ── SocketIO: receive vision frames ──────────────────────────────────────────
socket.on('vision_frame', data => {
  if (activeView !== 'vision') return;

  // Update raw feed
  const rawImg = document.getElementById('vision-raw-img');
  if (rawImg && data.raw) {
    rawImg.src = 'data:image/jpeg;base64,' + data.raw;
  }

  // Update rectified feed
  const rectImg   = document.getElementById('vision-rect-img');
  const rectPlch  = document.getElementById('vision-rect-placeholder');
  if (rectImg && rectPlch) {
    if (data.rect) {
      rectImg.src = 'data:image/jpeg;base64,' + data.rect;
      rectImg.style.display    = 'block';
      rectPlch.style.display   = 'none';
    } else {
      rectImg.style.display    = 'none';
      rectPlch.style.display   = 'flex';
    }
  }

  // Update processed feed
  const procWrap = document.getElementById('vision-proc-wrap');
  const procImg  = document.getElementById('vision-proc-img');
  const procMode = data.proc_mode || 'none';
  if (procWrap) procWrap.style.display = (procMode !== 'none' && data.proc) ? 'flex' : 'none';
  if (procImg && data.proc) procImg.src = 'data:image/jpeg;base64,' + data.proc;
  const procBadge = document.getElementById('vision-proc-mode-badge');
  if (procBadge) procBadge.textContent = procMode !== 'none' ? procMode : '—';

  // Homography badge
  const hBadge = document.getElementById('vision-h-badge');
  if (hBadge) {
    if (data.h_fresh)       { hBadge.textContent = 'LIVE';  hBadge.className = 'vision-h-badge h-live'; }
    else if (data.has_h)    { hBadge.textContent = 'CACHE'; hBadge.className = 'vision-h-badge h-cache'; }
    else                    { hBadge.textContent = 'NO H';  hBadge.className = 'vision-h-badge h-noh'; }
  }

  // Frame info + seek slider
  _visionFrameInfo = data;
  const fc   = data.frame_count ?? -1;
  const fi   = data.frame_idx  ?? 0;
  const pb   = document.getElementById('vision-playback');
  const sl   = document.getElementById('vision-seek-sl');
  const info = document.getElementById('vision-frame-info');
  if (pb) {
    pb.classList.toggle('hidden', fc <= 0);
    if (fc > 0) {
      if (sl) { sl.max = fc - 1; sl.value = fi; }
      if (info) info.textContent = `${fi} / ${fc - 1}`;
    }
  }

  // Detection list
  _renderDetections(data.detections || []);
});

function _renderDetections(dets) {
  const el = document.getElementById('vision-det-list');
  const ct = document.getElementById('vision-det-count');
  if (!el) return;
  if (ct) ct.textContent = dets.length;
  el.innerHTML = dets.map(d => {
    const pos = (d.x_mm !== undefined)
      ? ` → table (${d.x_mm.toFixed(0)}, ${d.y_mm.toFixed(0)}) mm`
      : '';
    return `<div class="vision-det-row">ID <b>${d.id}</b> px=(${d.px.toFixed(0)},${d.py.toFixed(0)})${pos}</div>`;
  }).join('');
}

// ── View activation ───────────────────────────────────────────────────────────
async function onVisionViewActivated() {
  try {
    const res  = await fetch('/api/vision/state');
    const data = await res.json();
    _visionEnabled = data.enabled;

    // Sync enable toggle
    const cb = document.getElementById('vision-enabled-cb');
    if (cb) cb.checked = _visionEnabled;
    _visionUpdateBadge(_visionEnabled);

    // Sync source selector
    if (data.source) _visionSyncSourceSelector(data.source);

    // Sync config checkboxes
    const cfg = data.config || {};
    _visionSyncCheckbox('v-undistort',  cfg.undistort);
    _visionSyncCheckbox('v-aruco',      cfg.show_aruco !== false);
    _visionSyncCheckbox('v-ids',        cfg.show_ids !== false);
    _visionSyncCheckbox('v-rej',        cfg.show_rejected);
    _visionSyncCheckbox('v-grid',       cfg.show_grid !== false);
    _visionSyncCheckbox('v-overlay',    cfg.show_table_overlay !== false);
    _visionSyncCheckbox('v-autocolor',  cfg.auto_color);
    const dictSel = document.getElementById('vision-dict-sel');
    if (dictSel && cfg.dict) dictSel.value = cfg.dict;
    const qsl = document.getElementById('vision-quality-sl');
    const qv  = document.getElementById('vision-quality-val');
    if (qsl && cfg.jpeg_quality) { qsl.value = cfg.jpeg_quality; if (qv) qv.textContent = cfg.jpeg_quality; }

    // Sync anchors
    if (cfg.anchors) _visionSyncAnchors(cfg.anchors);

    // Sync processing pipeline UI
    _visionSyncProcUI(cfg);

    // Load object registry
    await visionLoadObjects();
  } catch (e) {
    console.error('Vision state load error:', e);
  }
}

function _visionSyncProcUI(cfg) {
  const mode = cfg.proc_mode ?? 'none';
  const modeEl = document.getElementById('vp-mode');
  if (modeEl) modeEl.value = mode;

  // Binary
  const bm = document.getElementById('vp-bin-method');
  if (bm) bm.value = cfg.binary_method ?? 'global';
  _vSetSlider('vp-bin-thresh', 'vp-bin-thresh-val', cfg.binary_threshold ?? 128);
  _vSetSlider('vp-bin-block',  'vp-bin-block-val',  cfg.binary_block_size ?? 11);
  _visionSyncCheckbox('vp-bin-invert', cfg.binary_invert);

  // Color mask
  _vSetSlider('vp-h-lo',  'vp-h-lo-val',  cfg.color_lo_h  ?? 0);
  _vSetSlider('vp-h-hi',  'vp-h-hi-val',  cfg.color_hi_h  ?? 10);
  _vSetSlider('vp-h2-lo', 'vp-h2-lo-val', cfg.color_lo_h2 ?? 160);
  _vSetSlider('vp-h2-hi', 'vp-h2-hi-val', cfg.color_hi_h2 ?? -1);
  _vSetSlider('vp-s-lo',  'vp-s-lo-val',  cfg.color_lo_s  ?? 80);
  _vSetSlider('vp-s-hi',  'vp-s-hi-val',  cfg.color_hi_s  ?? 255);
  _vSetSlider('vp-v-lo',  'vp-v-lo-val',  cfg.color_lo_v  ?? 60);
  _vSetSlider('vp-v-hi',  'vp-v-hi-val',  cfg.color_hi_v  ?? 255);
  _visionSyncCheckbox('vp-color-masked', cfg.color_show_masked !== false);
  const cpEl = document.getElementById('vp-color-preset');
  if (cpEl && cfg.color_target) cpEl.value = cfg.color_target;

  // Canny
  _vSetSlider('vp-canny-lo', 'vp-canny-lo-val', cfg.canny_low  ?? 50);
  _vSetSlider('vp-canny-hi', 'vp-canny-hi-val', cfg.canny_high ?? 150);

  // Blur
  _vSetSlider('vp-blur-k', 'vp-blur-k-val', cfg.blur_kernel ?? 5);

  // Morph
  const morphOp = document.getElementById('vp-morph-op');
  if (morphOp) morphOp.value = cfg.morph_op ?? 'none';
  _vSetSlider('vp-morph-k', 'vp-morph-k-val', cfg.morph_kernel ?? 3);

  // Trigger visibility update
  visionProcModeChanged();
}

function _visionSyncCheckbox(id, val) {
  const el = document.getElementById(id);
  if (el) el.checked = !!val;
}

function _visionUpdateBadge(enabled) {
  const badge = document.getElementById('vision-status-badge');
  if (!badge) return;
  badge.textContent  = enabled ? 'ON' : 'OFF';
  badge.className    = 'vision-badge ' + (enabled ? 'vision-badge-on' : 'vision-badge-off');
}

// ── Enable / Disable ─────────────────────────────────────────────────────────
async function visionToggleEnabled(checked) {
  const url = checked ? '/api/vision/enable' : '/api/vision/disable';
  const res  = await fetch(url, { method: 'POST' });
  const data = await res.json();
  _visionEnabled = checked && data.ok;
  _visionUpdateBadge(_visionEnabled);
}

// ── Source selector ───────────────────────────────────────────────────────────

function visionSourceTypeChanged() {
  const type = document.querySelector('input[name="vsrc-type"]:checked')?.value ?? 'usb';
  ['usb','ip','video','image'].forEach(t => {
    const el = document.getElementById('vsrc-' + t);
    if (el) el.classList.toggle('hidden', t !== type);
  });
}

function _visionGetSourceString() {
  const type = document.querySelector('input[name="vsrc-type"]:checked')?.value ?? 'usb';
  switch (type) {
    case 'usb':   return document.getElementById('vsrc-usb-idx')?.value?.trim()   ?? '0';
    case 'ip':    return document.getElementById('vsrc-ip-url')?.value?.trim()    ?? '';
    case 'video': return document.getElementById('vsrc-video-path')?.value?.trim() ?? '';
    case 'image': return document.getElementById('vsrc-image-path')?.value?.trim() ?? '';
    default:      return '0';
  }
}

function _visionSyncSourceSelector(source) {
  // Try to guess type from source string
  if (!source) return;
  const s = String(source);
  let type = 'usb';
  if (/^rtsp:|^http:|^https:/.test(s)) {
    type = 'ip';
    const el = document.getElementById('vsrc-ip-url');
    if (el) el.value = s;
  } else if (/\.(mp4|mkv|avi|mov|webm)$/i.test(s)) {
    type = 'video';
    const el = document.getElementById('vsrc-video-path');
    if (el) el.value = s;
  } else if (/\.(jpg|jpeg|png|bmp|tiff?)$/i.test(s)) {
    type = 'image';
    const el = document.getElementById('vsrc-image-path');
    if (el) el.value = s;
  } else {
    // USB index
    const el = document.getElementById('vsrc-usb-idx');
    if (el) el.value = isNaN(+s) ? '0' : s;
  }
  const radio = document.querySelector(`input[name="vsrc-type"][value="${type}"]`);
  if (radio) { radio.checked = true; visionSourceTypeChanged(); }
}

async function visionApplySource() {
  const src = _visionGetSourceString();
  if (!src && src !== '0') return;
  await fetch('/api/vision/source', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({ source: src }),
  });
}

// ── Processing pipeline ───────────────────────────────────────────────────────

// Color presets matching _COLOR_RANGES_HSV in vision_backend.py
const _VP_COLOR_PRESETS = {
  red:    { lo_h: 0,  lo_s: 80,  lo_v: 60,  hi_h: 10,  hi_h2: 180, lo_h2: 160 },
  green:  { lo_h: 40, lo_s: 60,  lo_v: 50,  hi_h: 85,  hi_h2: -1,  lo_h2: 160 },
  blue:   { lo_h: 95, lo_s: 60,  lo_v: 50,  hi_h: 135, hi_h2: -1,  lo_h2: 160 },
  yellow: { lo_h: 20, lo_s: 80,  lo_v: 80,  hi_h: 38,  hi_h2: -1,  lo_h2: 160 },
  white:  { lo_h: 0,  lo_s: 0,   lo_v: 190, hi_h: 180, hi_h2: -1,  lo_h2: 160 },
  black:  { lo_h: 0,  lo_s: 0,   lo_v: 0,   hi_h: 180, hi_h2: -1,  lo_h2: 160 },
  brown:  { lo_h: 8,  lo_s: 50,  lo_v: 30,  hi_h: 22,  hi_h2: -1,  lo_h2: 160 },
};

function _vSetSlider(id, valId, val) {
  const sl = document.getElementById(id);
  const vl = document.getElementById(valId);
  if (sl) sl.value = val;
  if (vl) vl.textContent = (id === 'vp-h2-hi' && val < 0) ? 'off' : val;
}

function visionProcSlider(sliderId, valId) {
  const sl = document.getElementById(sliderId);
  const vl = document.getElementById(valId);
  if (sl && vl) {
    const v = +sl.value;
    vl.textContent = (sliderId === 'vp-h2-hi' && v < 0) ? 'off' : v;
  }
}

function visionProcModeChanged() {
  const mode = document.getElementById('vp-mode')?.value ?? 'none';
  // Show/hide sub-groups
  ['binarize','color','canny','blur'].forEach(g => {
    const el = document.getElementById('vp-' + g);
    if (el) el.classList.toggle('hidden', g !== {
      binarize: 'binarize', color_mask: 'color', canny: 'canny', blur: 'blur',
    }[mode]);
  });
  // Show morph row for all except none/blur
  const morphRow = document.getElementById('vp-morph-row');
  if (morphRow) morphRow.style.display = (mode === 'none' || mode === 'blur') ? 'none' : '';

  visionProcCfg();
}

function _visionBinMethodChanged() {
  const method = document.getElementById('vp-bin-method')?.value ?? 'global';
  const thRow  = document.getElementById('vp-bin-thresh-row');
  const blRow  = document.getElementById('vp-bin-block-row');
  if (thRow) thRow.classList.toggle('hidden', method === 'otsu');
  if (blRow) blRow.classList.toggle('hidden', !method.startsWith('adaptive'));
}

function visionColorPreset() {
  const preset = document.getElementById('vp-color-preset')?.value ?? 'custom';
  const p = _VP_COLOR_PRESETS[preset];
  if (!p) return;
  _vSetSlider('vp-h-lo',  'vp-h-lo-val',  p.lo_h);
  _vSetSlider('vp-h-hi',  'vp-h-hi-val',  p.hi_h);
  _vSetSlider('vp-h2-lo', 'vp-h2-lo-val', p.lo_h2 ?? 160);
  _vSetSlider('vp-h2-hi', 'vp-h2-hi-val', p.hi_h2 ?? -1);
  visionProcCfg();
}

async function visionProcCfg() {
  const mode = document.getElementById('vp-mode')?.value ?? 'none';
  const cfg = { proc_mode: mode };

  if (mode === 'binarize') {
    const method = document.getElementById('vp-bin-method')?.value ?? 'global';
    _visionBinMethodChanged();
    cfg.binary_method     = method;
    cfg.binary_threshold  = +(document.getElementById('vp-bin-thresh')?.value  ?? 128);
    cfg.binary_block_size = +(document.getElementById('vp-bin-block')?.value   ?? 11);
    cfg.binary_invert     = !!(document.getElementById('vp-bin-invert')?.checked);
  }
  if (mode === 'color_mask') {
    cfg.color_target      = document.getElementById('vp-color-preset')?.value ?? 'red';
    cfg.color_lo_h        = +(document.getElementById('vp-h-lo')?.value  ?? 0);
    cfg.color_hi_h        = +(document.getElementById('vp-h-hi')?.value  ?? 10);
    cfg.color_lo_h2       = +(document.getElementById('vp-h2-lo')?.value ?? 160);
    cfg.color_hi_h2       = +(document.getElementById('vp-h2-hi')?.value ?? -1);
    cfg.color_lo_s        = +(document.getElementById('vp-s-lo')?.value  ?? 80);
    cfg.color_hi_s        = +(document.getElementById('vp-s-hi')?.value  ?? 255);
    cfg.color_lo_v        = +(document.getElementById('vp-v-lo')?.value  ?? 60);
    cfg.color_hi_v        = +(document.getElementById('vp-v-hi')?.value  ?? 255);
    cfg.color_show_masked = !!(document.getElementById('vp-color-masked')?.checked);
  }
  if (mode === 'canny') {
    cfg.canny_low  = +(document.getElementById('vp-canny-lo')?.value ?? 50);
    cfg.canny_high = +(document.getElementById('vp-canny-hi')?.value ?? 150);
  }
  if (mode === 'blur') {
    cfg.blur_kernel = +(document.getElementById('vp-blur-k')?.value ?? 5);
  }
  if (mode !== 'none' && mode !== 'blur') {
    cfg.morph_op     = document.getElementById('vp-morph-op')?.value ?? 'none';
    cfg.morph_kernel = +(document.getElementById('vp-morph-k')?.value ?? 3);
  }

  await fetch('/api/vision/config', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify(cfg),
  });
}

// ── Config helpers ────────────────────────────────────────────────────────────
async function visionCfg(key, value) {
  // Special: update quality label
  if (key === 'jpeg_quality') {
    const qv = document.getElementById('vision-quality-val');
    if (qv) qv.textContent = value;
  }
  await fetch('/api/vision/config', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({ [key]: value }),
  });
}

// ── Anchors ────────────────────────────────────────────────────────────────────
function _visionSyncAnchors(anchors) {
  for (const [corner, vals] of Object.entries(anchors)) {
    document.querySelectorAll(`.va-id[data-corner="${corner}"]`).forEach(el => { el.value = vals.tag_id; });
    document.querySelectorAll(`.va-x[data-corner="${corner}"]`).forEach(el  => { el.value = vals.x_mm;  });
    document.querySelectorAll(`.va-y[data-corner="${corner}"]`).forEach(el  => { el.value = vals.y_mm;  });
  }
}

function _visionReadAnchors() {
  const corners = ['top_left', 'top_right', 'bottom_right', 'bottom_left'];
  const anchors = {};
  for (const c of corners) {
    const id = document.querySelector(`.va-id[data-corner="${c}"]`)?.value;
    const x  = document.querySelector(`.va-x[data-corner="${c}"]`)?.value;
    const y  = document.querySelector(`.va-y[data-corner="${c}"]`)?.value;
    anchors[c] = { tag_id: parseInt(id||0), x_mm: parseFloat(x||0), y_mm: parseFloat(y||0) };
  }
  return anchors;
}

async function visionAnchorChanged() {
  const anchors = _visionReadAnchors();
  await fetch('/api/vision/config', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({ anchors }),
  });
}

// ── Playback ───────────────────────────────────────────────────────────────────
async function visionPlayPause() {
  const btn = document.getElementById('vision-pp-btn');
  await fetch('/api/vision/playback', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({ action: 'play_pause' }),
  });
}

async function visionSeek(frame) {
  await fetch('/api/vision/playback', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({ action: 'seek', frame }),
  });
}

async function visionStep(delta) {
  await fetch('/api/vision/playback', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({ action: 'step', delta }),
  });
}

async function visionSetSpeed(speed) {
  await fetch('/api/vision/playback', {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify({ action: 'speed', speed }),
  });
}

// ── Object registry ───────────────────────────────────────────────────────────

async function visionLoadObjects() {
  const res  = await fetch('/api/vision/objects');
  _visionObjects = await res.json();
  _visionRenderObjects();
}

async function visionSaveObjects() {
  await fetch('/api/vision/objects', {
    method:  'PUT',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify(_visionObjects),
  });
}

function _visionRenderObjects() {
  const list = document.getElementById('vision-objects-list');
  if (!list) return;

  const COLOR_DOT = {
    unknown: '#888', red: '#e53', green: '#2a2', blue: '#36f',
    yellow: '#fb0', white: '#eee', black: '#555', brown: '#854',
  };

  list.innerHTML = _visionObjects.map(obj => {
    const dot   = COLOR_DOT[obj.color] || '#888';
    const live  = obj.last_seen_ms ? '●' : '○';
    const table = obj.on_table !== false ? '' : ' <span style="color:#f66;font-size:9px">gone</span>';
    const active = obj.id === _visionActiveObjId ? ' vision-obj-row-active' : '';
    return `<div class="vision-obj-row${active}" onclick="visionSelectObject('${obj.id}')">
      <span class="vision-obj-dot" style="background:${dot}"></span>
      <span class="vision-obj-name">${obj.name || obj.id}</span>
      <span style="font-size:9px;color:#888">${obj.type || ''}</span>
      ${obj.aruco_id !== null && obj.aruco_id !== undefined ? `<span class="vision-obj-tag">#${obj.aruco_id}</span>` : ''}
      ${table}
      <span style="margin-left:auto;color:${obj.last_seen_ms ? '#4a4' : '#666'};font-size:10px">${live}</span>
    </div>`;
  }).join('');
}

function visionSelectObject(objId) {
  _visionActiveObjId = objId;
  _visionRenderObjects();
  const obj = _visionObjects.find(o => o.id === objId);
  if (!obj) return;

  const ed = document.getElementById('vision-obj-editor');
  if (ed) ed.classList.remove('hidden');

  document.getElementById('voe-title').textContent   = obj.name || obj.id;
  document.getElementById('voe-name').value          = obj.name  || '';
  document.getElementById('voe-type').value          = obj.type  || 'other';
  document.getElementById('voe-aruco').value         = obj.aruco_id !== null && obj.aruco_id !== undefined ? obj.aruco_id : '';
  document.getElementById('voe-color').value         = obj.color || 'unknown';
  document.getElementById('voe-ix').value            = obj.initial_pos?.x ?? '';
  document.getElementById('voe-iy').value            = obj.initial_pos?.y ?? '';
  document.getElementById('voe-ontable').checked     = obj.on_table !== false;

  const ls = document.getElementById('voe-lastseen');
  if (ls) {
    if (obj.last_seen_ms) {
      const secs = ((Date.now() % 1e9) / 1000 - obj.last_seen_ms / 1000);
      ls.textContent = `Last seen ${Math.abs(secs).toFixed(1)}s ago  |  cur=(${obj.current_pos?.x?.toFixed(0)??'?'}, ${obj.current_pos?.y?.toFixed(0)??'?'}) mm`;
    } else {
      ls.textContent = 'Not yet detected';
    }
  }
}

function visionObjChanged() {
  const obj = _visionObjects.find(o => o.id === _visionActiveObjId);
  if (!obj) return;

  obj.name      = document.getElementById('voe-name').value;
  obj.type      = document.getElementById('voe-type').value;
  const arucoRaw = document.getElementById('voe-aruco').value;
  obj.aruco_id  = arucoRaw !== '' ? parseInt(arucoRaw) : null;
  obj.color     = document.getElementById('voe-color').value;
  obj.on_table  = document.getElementById('voe-ontable').checked;
  const ix      = parseFloat(document.getElementById('voe-ix').value);
  const iy      = parseFloat(document.getElementById('voe-iy').value);
  if (!isNaN(ix) && !isNaN(iy)) obj.initial_pos = { x: ix, y: iy };

  document.getElementById('voe-title').textContent = obj.name || obj.id;
  _visionRenderObjects();
  // Auto-save after short debounce
  clearTimeout(visionObjChanged._timer);
  visionObjChanged._timer = setTimeout(visionSaveObjects, 800);
}

function visionNewObject() {
  const id  = 'obj_' + Date.now();
  const obj = {
    id,
    name:        'New Object',
    type:        'cylinder',
    aruco_id:    null,
    color:       'unknown',
    initial_pos: { x: 1500, y: 1000 },
    current_pos: { x: 1500, y: 1000 },
    on_table:    true,
    last_seen_ms: null,
    notes:       '',
  };
  _visionObjects.push(obj);
  _visionRenderObjects();
  visionSelectObject(id);
}

async function visionDeleteObject() {
  if (!_visionActiveObjId) return;
  if (!confirm(`Delete object "${_visionActiveObjId}"?`)) return;
  _visionObjects = _visionObjects.filter(o => o.id !== _visionActiveObjId);
  _visionActiveObjId = null;
  document.getElementById('vision-obj-editor')?.classList.add('hidden');
  _visionRenderObjects();
  await visionSaveObjects();
}

// Periodically refresh object positions in the list (for live updates from detections)
setInterval(() => {
  if (activeView !== 'vision') return;
  fetch('/api/vision/objects').then(r => r.json()).then(objs => {
    // Merge only current_pos and last_seen_ms from server (don't overwrite edits)
    objs.forEach(serverObj => {
      const local = _visionObjects.find(o => o.id === serverObj.id);
      if (local) {
        local.current_pos  = serverObj.current_pos;
        local.last_seen_ms = serverObj.last_seen_ms;
        local.on_table     = serverObj.on_table;
      }
    });
    _visionRenderObjects();
    // Refresh editor last-seen if open
    if (_visionActiveObjId) {
      const obj = _visionObjects.find(o => o.id === _visionActiveObjId);
      if (obj) {
        const ls = document.getElementById('voe-lastseen');
        if (ls && obj.last_seen_ms) {
          ls.textContent = `cur=(${obj.current_pos?.x?.toFixed(0)??'?'}, ${obj.current_pos?.y?.toFixed(0)??'?'}) mm`;
        }
      }
    }
  }).catch(() => {});
}, 2000);

// ═══════════════════════════════════════════════════════════════════════════════
//  REMOTE CONTROL VIEW
// ═══════════════════════════════════════════════════════════════════════════════

let _remoteDrawerOpen = false;

function toggleRemoteDrawer(navBtn) {
  const drawer  = document.getElementById('remote-drawer');
  const navBtnEl = navBtn ?? document.getElementById('nav-remote');
  if (!drawer) return;

  _remoteDrawerOpen = !_remoteDrawerOpen;
  drawer.classList.toggle('closed', !_remoteDrawerOpen);
  navBtnEl?.classList.toggle('drawer-open', _remoteDrawerOpen);

  if (_remoteDrawerOpen) {
    // Make sure we are on the map view (drawer lives there)
    if (activeView !== 'map') {
      const mapBtn = document.querySelector('.nav-btn[data-view="map"]');
      switchView('map', mapBtn);
    }
    _remoteInitDial();
    if (lastState?.robot) remoteUpdateTheta(lastState.robot.theta * 180 / Math.PI);
    _remoteUpdateDial();
  }
}

let _remoteCurDeg  = 0;      // current heading (degrees, math: 0=East, CCW positive)
let _remoteTgtDeg  = 0;      // target heading (degrees, math: 0=East, CCW positive)
let _remoteSnapDeg = 5;      // snap interval for drag clamping
let _remoteDragging = false;
let _remoteMotionBusy = false;
let _remoteDialReady  = false;

const R_OUTER  = 145;  // outer ring radius (SVG units)
const R_TICK_M = 14;   // major tick height (every 30°)
const R_TICK_m = 7;    // minor tick height (every 10°)
const R_LABELS = 113;  // label radius
const R_ARC    = 100;  // delta arc radius
const R_CURIND = 110;  // current-heading indicator line end
const R_TGTIND = 128;  // target indicator line end & dot
const ROBOT_R  = 36;   // robot image half-size

// ── Polar geometry helpers ─────────────────────────────────────────────────────

function _deg2xy(r, deg) {
  // Math convention: 0=East (right), 90=North (top), CCW positive
  const rad = deg * Math.PI / 180;
  return { x: r * Math.cos(rad), y: -r * Math.sin(rad) };
}

function _normDeg(d) {
  return ((d % 360) + 360) % 360;
}

function _shortDelta(from, to) {
  let d = _normDeg(to) - _normDeg(from);
  if (d > 180)  d -= 360;
  if (d < -180) d += 360;
  return d;
}

// ── One-time dial initialisation ───────────────────────────────────────────────

function _remoteInitDial() {
  if (_remoteDialReady) return;
  _remoteDialReady = true;

  const svg  = document.getElementById('remote-dial');
  if (!svg) return;

  // ── Tick layer ─────────────────────────────────────────────────────────────
  const g = document.getElementById('dial-ticks-layer');

  // Outer ring
  const ring = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
  ring.setAttribute('cx', 0); ring.setAttribute('cy', 0);
  ring.setAttribute('r', R_OUTER);
  ring.setAttribute('fill', 'none');
  ring.setAttribute('stroke', 'var(--surface3)');
  ring.setAttribute('stroke-width', '1.5');
  g.appendChild(ring);

  // Inner ring
  const ring2 = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
  ring2.setAttribute('cx', 0); ring2.setAttribute('cy', 0);
  ring2.setAttribute('r', R_ARC + 12);
  ring2.setAttribute('fill', 'none');
  ring2.setAttribute('stroke', 'var(--surface3)');
  ring2.setAttribute('stroke-width', '0.8');
  ring2.setAttribute('stroke-dasharray', '2 4');
  g.appendChild(ring2);

  // Ticks and labels
  for (let deg = 0; deg < 360; deg += 10) {
    const isMajor = (deg % 30 === 0);
    const h  = isMajor ? R_TICK_M : R_TICK_m;
    const p1 = _deg2xy(R_OUTER, deg);
    const p2 = _deg2xy(R_OUTER - h, deg);

    const tick = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    tick.setAttribute('x1', p1.x.toFixed(2)); tick.setAttribute('y1', p1.y.toFixed(2));
    tick.setAttribute('x2', p2.x.toFixed(2)); tick.setAttribute('y2', p2.y.toFixed(2));
    tick.setAttribute('stroke', isMajor ? 'var(--text)' : 'var(--text-dim)');
    tick.setAttribute('stroke-width', isMajor ? '1.5' : '0.8');
    g.appendChild(tick);

    if (isMajor) {
      const lp = _deg2xy(R_LABELS, deg);
      const lbl = document.createElementNS('http://www.w3.org/2000/svg', 'text');
      lbl.setAttribute('x', lp.x.toFixed(2));
      lbl.setAttribute('y', lp.y.toFixed(2));
      lbl.setAttribute('text-anchor', 'middle');
      lbl.setAttribute('dominant-baseline', 'middle');
      // Math convention: 90=North (top), 0=East (right), 180=West, 270=South
      const CARD_LABELS = { 90: 'N', 0: 'E', 180: 'W', 270: 'S' };
      const cardLbl = CARD_LABELS[deg];
      const isNorth = deg === 90;
      const isCard  = cardLbl !== undefined;
      lbl.setAttribute('font-size', isCard ? '11' : '9');
      lbl.setAttribute('font-weight', isCard ? '700' : '400');
      lbl.setAttribute('fill', isNorth ? 'var(--brand)' : (isCard ? 'var(--text)' : 'var(--text-dim)'));
      lbl.setAttribute('font-family', 'monospace');
      lbl.textContent = cardLbl ?? deg + '°';
      g.appendChild(lbl);
    }
  }

  // Center dot
  const cdot = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
  cdot.setAttribute('cx', 0); cdot.setAttribute('cy', 0); cdot.setAttribute('r', 4);
  cdot.setAttribute('fill', 'var(--brand)');
  g.appendChild(cdot);

  // ── Apply brand colors to dynamic elements ────────────────────────────────
  const curLine = document.getElementById('dial-cur-line');
  if (curLine) curLine.setAttribute('stroke', 'var(--text-dim)');

  const tgtLine = document.getElementById('dial-tgt-line');
  if (tgtLine) tgtLine.setAttribute('stroke', 'var(--brand)');

  const tgtDot = document.getElementById('dial-tgt-dot');
  if (tgtDot) {
    tgtDot.setAttribute('stroke', 'var(--brand)');
    tgtDot.setAttribute('fill', 'var(--surface1)');
  }

  const tgtLabel = document.getElementById('dial-tgt-label');
  if (tgtLabel) {
    tgtLabel.setAttribute('fill', 'var(--brand)');
    tgtLabel.setAttribute('font-family', 'monospace');
  }

  // ── Drag events on the SVG ────────────────────────────────────────────────
  svg.addEventListener('mousedown',  _remoteDialMouseDown);
  svg.addEventListener('mousemove',  _remoteDialMouseMove);
  svg.addEventListener('mouseup',    _remoteDialMouseUp);
  svg.addEventListener('mouseleave', _remoteDialMouseUp);
  svg.addEventListener('touchstart', e => { e.preventDefault(); _remoteDialMouseDown(e.touches[0]); }, { passive: false });
  svg.addEventListener('touchmove',  e => { e.preventDefault(); _remoteDialMouseMove(e.touches[0]); }, { passive: false });
  svg.addEventListener('touchend',   e => { _remoteDialMouseUp(); }, { passive: false });

  // ── Generate goPolar direction buttons in a ring around the dial ────────
  // remote-dial-outer is 440×440, dial-wrap is 320×320 at top:60/left:60
  // Centre of outer div = (220, 220). Ring radius = 185px.
  const POLAR_CX = 220, POLAR_CY = 220, POLAR_R = 185;
  // Math convention: 0=East, 90=North, 180=West, 270=South
  const CARDINAL = { 90:'N', 0:'E', 180:'W', 270:'S' };
  const polarContainer = document.getElementById('remote-polar-btns');
  if (polarContainer) {
    polarContainer.innerHTML = '';
    for (let i = 0; i < 12; i++) {
      const deg = i * 30;
      const rad = deg * Math.PI / 180;
      // Math convention: x=cos, y=-sin (North at top)
      const cx  = POLAR_CX + POLAR_R * Math.cos(rad);
      const cy  = POLAR_CY - POLAR_R * Math.sin(rad);
      const label = CARDINAL[deg] ?? deg + '°';
      const isCard = CARDINAL[deg] !== undefined;
      const btn = document.createElement('button');
      btn.className = 'remote-polar-btn' + (isCard ? ' cardinal' : '');
      btn.style.left = cx.toFixed(1) + 'px';
      btn.style.top  = cy.toFixed(1) + 'px';
      btn.style.pointerEvents = 'auto';
      btn.textContent = label;
      btn.title = `goPolar ${deg}° (${label})`;
      btn.addEventListener('click', () => remoteGoPolar(deg));
      polarContainer.appendChild(btn);
    }
  }

  // Initial render
  _remoteUpdateDial();
}

// ── Dial update ────────────────────────────────────────────────────────────────

function _remoteUpdateDial() {
  const cur = _normDeg(_remoteCurDeg);
  const tgt = _normDeg(_remoteTgtDeg);
  const delta = _shortDelta(cur, tgt);

  // Robot image rotation — HTML img with CSS transform
  const robotImg = document.getElementById('dial-robot-img');
  const imgOffDeg = ROBOT_IMG_OFFSET * 180 / Math.PI;
  if (robotImg) robotImg.style.transform = `translate(-50%, -50%) rotate(${(-cur + imgOffDeg).toFixed(2)}deg)`;

  // Current heading line
  const curLine = document.getElementById('dial-cur-line');
  if (curLine) {
    const p = _deg2xy(R_CURIND, cur);
    curLine.setAttribute('x2', p.x.toFixed(2));
    curLine.setAttribute('y2', p.y.toFixed(2));
  }

  // Target indicator (line + dot + label)
  const tgtLine = document.getElementById('dial-tgt-line');
  const tgtDot  = document.getElementById('dial-tgt-dot');
  const tgtLbl  = document.getElementById('dial-tgt-label');
  const tp = _deg2xy(R_TGTIND, tgt);
  if (tgtLine) { tgtLine.setAttribute('x2', tp.x.toFixed(2)); tgtLine.setAttribute('y2', tp.y.toFixed(2)); }
  if (tgtDot)  { tgtDot.setAttribute('cx', tp.x.toFixed(2));  tgtDot.setAttribute('cy', tp.y.toFixed(2)); }
  if (tgtLbl) {
    const lp = _deg2xy(R_TGTIND + 18, tgt);
    tgtLbl.setAttribute('x', lp.x.toFixed(2));
    tgtLbl.setAttribute('y', lp.y.toFixed(2));
    tgtLbl.textContent = tgt.toFixed(0) + '°';
  }

  // Delta arc
  const arc = document.getElementById('dial-delta-arc');
  if (arc) {
    arc.setAttribute('stroke', delta >= 0 ? '#3b82f6' : '#f59e0b');
    if (Math.abs(delta) < 0.5) {
      arc.setAttribute('d', '');
    } else {
      const p1 = _deg2xy(R_ARC, cur);
      const p2 = _deg2xy(R_ARC, tgt);
      // Go from cur toward tgt the short way (math CCW = SVG sweep=0)
      const sweep = delta >= 0 ? 0 : 1;
      const large = Math.abs(delta) > 180 ? 1 : 0;
      arc.setAttribute('d',
        `M ${p1.x.toFixed(2)} ${p1.y.toFixed(2)} ` +
        `A ${R_ARC} ${R_ARC} 0 ${large} ${sweep} ${p2.x.toFixed(2)} ${p2.y.toFixed(2)}`);
    }
  }

  // Readout
  const sign = delta >= 0 ? '+' : '';
  document.getElementById('rr-cur')?.setAttribute('data-v',   cur.toFixed(1));
  document.getElementById('rr-cur'  )&&(document.getElementById('rr-cur').textContent   = cur.toFixed(1) + '°');
  document.getElementById('rr-tgt'  )&&(document.getElementById('rr-tgt').textContent   = tgt.toFixed(1) + '°');
  document.getElementById('rr-delta')&&(document.getElementById('rr-delta').textContent = sign + delta.toFixed(1) + '°');

  // Target input sync
  const tgtInput = document.getElementById('remote-tgt-input');
  if (tgtInput && document.activeElement !== tgtInput) tgtInput.value = Math.round(tgt);

  // Go button label
  const go = document.getElementById('remote-go-btn');
  if (go) {
    const sign2 = delta >= 0 ? '+' : '';
    go.textContent = `▶  TURN  ${sign2}${delta.toFixed(1)}°`;
    go.disabled = _remoteMotionBusy;
    go.className = 'remote-go-btn' + (_remoteMotionBusy ? ' busy' : '');
  }
}

// ── Drag logic ─────────────────────────────────────────────────────────────────

function _remoteAngleFromEvent(evt) {
  const svg  = document.getElementById('remote-dial');
  if (!svg) return 0;
  const rect = svg.getBoundingClientRect();
  const cx   = rect.left + rect.width  / 2;
  const cy   = rect.top  + rect.height / 2;
  const dx   = evt.clientX - cx;
  const dy   = evt.clientY - cy;
  // Math convention: 0=East, CCW positive (SVG y-axis is down, so negate dy)
  let angle  = Math.atan2(-dy, dx) * 180 / Math.PI;
  if (angle < 0) angle += 360;
  return angle;
}

function _remoteDialMouseDown(e) {
  _remoteDragging = true;
  const a = _remoteAngleFromEvent(e);
  remoteSetTarget(a);
}

function _remoteDialMouseMove(e) {
  if (!_remoteDragging) return;
  const a = _remoteAngleFromEvent(e);
  remoteSetTarget(a);
}

function _remoteDialMouseUp() {
  _remoteDragging = false;
}

// ── Public API ─────────────────────────────────────────────────────────────────

function remoteSetTarget(deg) {
  const snapped = Math.round(deg / _remoteSnapDeg) * _remoteSnapDeg;
  _remoteTgtDeg = _normDeg(snapped);
  _remoteUpdateDial();
}

function remoteDelta(deltaDeg) {
  if (deltaDeg === 0) {
    // Reset: target = current
    _remoteTgtDeg = _normDeg(_remoteCurDeg);
  } else {
    _remoteTgtDeg = _normDeg(_remoteTgtDeg + deltaDeg);
  }
  _remoteUpdateDial();
}

function remoteSnapChanged() {
  _remoteSnapDeg = +(document.getElementById('remote-snap-sel')?.value ?? 5);
  // Re-snap the current target
  remoteSetTarget(_remoteTgtDeg);
}

function remoteUpdateTheta(thetaDeg) {
  // Firmware theta is already math convention (0=East, CCW+) — use as-is
  _remoteCurDeg = thetaDeg;
  if (_remoteDrawerOpen) _remoteUpdateDial();
}

async function remoteSendTurn() {
  if (_remoteMotionBusy) return;
  const tgt   = _normDeg(_remoteTgtDeg);
  const delta = _shortDelta(_normDeg(_remoteCurDeg), tgt);
  if (Math.abs(delta) < 0.5) return;

  _remoteMotionBusy = true;
  _remoteUpdateDial();

  const status = document.getElementById('remote-status');
  if (status) { status.textContent = `Sending turn(${tgt.toFixed(1)}°)…`; status.className = 'remote-status-busy'; }

  try {
    // turn() takes an absolute angle in the table frame (math convention, same as _remoteTgtDeg)
    const res = await fetch('/api/exec', {
      method:  'POST',
      headers: { 'Content-Type': 'application/json' },
      body:    JSON.stringify({ cmd: `turn(${tgt.toFixed(2)})` }),
    });
    const data = await res.json();
    if (status) {
      if (data.ok) {
        const sign = delta >= 0 ? '+' : '';
        status.textContent = `✓ Done — turn → ${tgt.toFixed(1)}° (Δ${sign}${delta.toFixed(1)}°)`;
        status.className = 'remote-status-ok';
      } else {
        status.textContent = `✗ ${data.res ?? 'error'}`;
        status.className = 'remote-status-err';
      }
    }
  } catch (e) {
    if (status) { status.textContent = `✗ Request failed`; status.className = 'remote-status-err'; }
  }

  _remoteMotionBusy = false;
  _remoteUpdateDial();
}

async function remoteGoPolar(bearingDeg) {
  if (_remoteMotionBusy) return;
  const dist = parseInt(document.getElementById('remote-dist-sel')?.value ?? 200);
  // Both bearingDeg and _remoteCurDeg are in math convention (0=East, CCW+)
  // goPolar heading is relative: desired_direction - current_heading
  const relAngle = _shortDelta(_remoteCurDeg, _normDeg(bearingDeg));

  _remoteMotionBusy = true;
  _remoteUpdateDial();
  const status = document.getElementById('remote-status');
  const CARDINAL = { 90:'N', 0:'E', 180:'W', 270:'S' };
  const lbl = CARDINAL[bearingDeg] ?? bearingDeg + '°';
  if (status) { status.textContent = `Sending goPolar(${relAngle.toFixed(1)}°, ${dist}mm) [→${lbl}]…`; status.className = 'remote-status-busy'; }

  try {
    const res = await fetch('/api/exec', {
      method:  'POST',
      headers: { 'Content-Type': 'application/json' },
      body:    JSON.stringify({ cmd: `goPolar(${relAngle.toFixed(2)},${dist})` }),
    });
    const data = await res.json();
    if (status) {
      if (data.ok) {
        status.textContent = `✓ Done — moved ${dist}mm toward ${lbl}`;
        status.className = 'remote-status-ok';
      } else {
        status.textContent = `✗ ${data.res ?? 'error'}`;
        status.className = 'remote-status-err';
      }
    }
  } catch (e) {
    if (status) { status.textContent = `✗ Request failed`; status.className = 'remote-status-err'; }
  }

  _remoteMotionBusy = false;
  _remoteUpdateDial();
}

// (remote is now a drawer; activation handled by toggleRemoteDrawer)

// ═══════════════════════════════════════════════════════════════════════════════
//  ACTUATOR PAGE — Servo controls, Pose library, Sequence builder
// ═══════════════════════════════════════════════════════════════════════════════

// ── Servo definitions (matches firmware groups.h) ──────────────────────────
const ACT_GROUPS = {
  CA: {
    label: 'CA (Right Manip)',
    servos: [
      { id: 0, name: 'Grabber R',  min: 0, max: 180, def: 121, poses: { 0: 162, 1: 162, 2: 121 } },
      { id: 1, name: 'Elevator',   min: 0, max: 180, def: 50,  poses: { 0: 50, 1: 50, 2: 5 } },
      { id: 2, name: 'Grabber L',  min: 0, max: 180, def: 65,  poses: { 0: 20, 1: 20, 2: 65 } },
    ],
  },
  AB: {
    label: 'AB (Hugger)',
    servos: [
      { id: 3, name: 'Hug Elevator', min: 0, max: 180, def: 155, poses: { 0: 155, 1: 155, 2: 90 } },
      { id: 4, name: 'Hug Grab',     min: 0, max: 180, def: 90,  poses: { 0: 90, 1: 30, 2: 90 } },
    ],
  },
  BC: {
    label: 'BC (Banner)',
    servos: [],
  },
};

let _actGroup = 'CA';
let _actServoAngles = {};   // { "CA:0": 121, "CA:2": 65, ... }
let _actPoses = {};         // { poseName: { "CA:0": angle, ... } }
let _actSequences = {};     // { seqName: { steps: [...] } }
let _actSeqSteps = [];      // current editing steps
let _actSeqPlaying = false;
let _actSeqAbort = false;

// ── Init: load poses + sequences from backend ──────────────────────────────
async function actInit() {
  try {
    const [posesRes, seqsRes] = await Promise.all([
      fetch('/api/actuator/poses').then(r => r.json()),
      fetch('/api/actuator/sequences').then(r => r.json()),
    ]);
    _actPoses = posesRes || {};
    _actSequences = seqsRes || {};
  } catch (e) { console.warn('actInit failed:', e); }
  actRenderServos();
  actRenderPoses();
  actRenderSeqSelect();
  _actSeqSteps = [];
  actRenderSeqSteps();
}

// ── Group tab switch ───────────────────────────────────────────────────────
function actSelectGroup(group, btn) {
  _actGroup = group;
  document.querySelectorAll('.act-gtab').forEach(b => b.classList.remove('active'));
  btn?.classList.add('active');
  actRenderServos();
}

// ── Servo slider rendering ─────────────────────────────────────────────────
function actRenderServos() {
  const el = document.getElementById('act-servo-list');
  if (!el) return;
  const g = ACT_GROUPS[_actGroup];
  if (!g || g.servos.length === 0) {
    el.innerHTML = '<span class="act-empty">No servos in this group</span>';
    return;
  }
  el.innerHTML = g.servos.map(s => {
    const key = `${_actGroup}:${s.id}`;
    const val = _actServoAngles[key] ?? s.def;
    const poseNames = ['DROP', 'GRAB', 'STORE'];
    const poseBtns = Object.entries(s.poses).map(([pi, a]) =>
      `<button class="act-pose-btn" onclick="actServoMoveTo('${_actGroup}',${s.id},${a})" title="${poseNames[pi]||pi}: ${a}°">${poseNames[pi]||pi}</button>`
    ).join('');
    return `<div class="act-servo-row">
      <div class="act-servo-hdr">
        <span class="act-servo-name">${s.name}</span>
        <span class="act-servo-val" id="act-val-${key}">${val}°</span>
      </div>
      <div class="act-servo-slider-row">
        <input type="range" min="${s.min}" max="${s.max}" value="${val}" class="act-slider"
          id="act-slider-${key}"
          oninput="actSliderMove('${key}',this.value)"
          onchange="actSliderSend('${_actGroup}',${s.id},'${key}')">
        <button class="btn-tiny" onclick="actSliderSend('${_actGroup}',${s.id},'${key}')" title="Send to robot">Go</button>
      </div>
      <div class="act-servo-poses">${poseBtns}</div>
    </div>`;
  }).join('');
}

function actSliderMove(key, val) {
  _actServoAngles[key] = parseInt(val);
  const el = document.getElementById('act-val-' + key);
  if (el) el.textContent = val + '°';
}

async function actSliderSend(group, servoId, key) {
  const angle = _actServoAngles[key] ?? 90;
  await actCmd(`servo(${group},${servoId},${angle})`);
}

async function actServoMoveTo(group, servoId, angle) {
  const key = `${group}:${servoId}`;
  _actServoAngles[key] = angle;
  const slider = document.getElementById('act-slider-' + key);
  if (slider) slider.value = angle;
  const valEl = document.getElementById('act-val-' + key);
  if (valEl) valEl.textContent = angle + '°';
  await actCmd(`servo(${group},${servoId},${angle})`);
}

async function actCmd(cmd) {
  try {
    const res = await fetch('/api/exec', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ cmd, timeout_ms: 5000 })
    });
    const d = await res.json();
    const st = document.getElementById('act-seq-status');
    if (st) st.textContent = d.ok ? `✓ ${cmd}` : `✗ ${cmd}: ${d.res}`;
    return d;
  } catch (e) {
    const st = document.getElementById('act-seq-status');
    if (st) st.textContent = `✗ ${cmd}: request failed`;
    return { ok: false, res: 'error' };
  }
}

// ── Pose Library ───────────────────────────────────────────────────────────
function actSnapshotPose() {
  const name = prompt('Pose name:', `pose_${Object.keys(_actPoses).length}`);
  if (!name) return;
  // Capture current slider angles
  const snapshot = {};
  for (const [gname, g] of Object.entries(ACT_GROUPS)) {
    for (const s of g.servos) {
      const key = `${gname}:${s.id}`;
      snapshot[key] = _actServoAngles[key] ?? s.def;
    }
  }
  _actPoses[name] = snapshot;
  actRenderPoses();
  actSavePosesRemote();
}

function actRenderPoses() {
  const el = document.getElementById('act-pose-list');
  if (!el) return;
  const names = Object.keys(_actPoses);
  if (names.length === 0) {
    el.innerHTML = '<span class="act-empty">No poses saved. Use "+ Snapshot" to capture current positions.</span>';
    return;
  }
  el.innerHTML = names.map(name => {
    const p = _actPoses[name];
    const summary = Object.entries(p).map(([k, v]) => `${k}=${v}`).join(', ');
    return `<div class="act-pose-item">
      <span class="act-pose-name">${name}</span>
      <span class="act-pose-summary">${summary}</span>
      <div class="act-pose-btns">
        <button class="btn-tiny" onclick="actPoseApply('${name}')" title="Send all servos to these positions">Apply</button>
        <button class="btn-tiny" onclick="actPoseDelete('${name}')" title="Remove pose">✕</button>
      </div>
    </div>`;
  }).join('');
}

async function actPoseApply(name) {
  const p = _actPoses[name];
  if (!p) return;
  for (const [key, angle] of Object.entries(p)) {
    const [group, sid] = key.split(':');
    const servoId = parseInt(sid);
    _actServoAngles[key] = angle;
    const slider = document.getElementById('act-slider-' + key);
    if (slider) slider.value = angle;
    const valEl = document.getElementById('act-val-' + key);
    if (valEl) valEl.textContent = angle + '°';
    // Fire command (don't await each one — parallel send)
    actCmd(`servo(${group},${servoId},${angle})`);
  }
}

function actPoseDelete(name) {
  delete _actPoses[name];
  actRenderPoses();
  actSavePosesRemote();
}

async function actSavePosesRemote() {
  await fetch('/api/actuator/poses', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify(_actPoses)
  });
}

function actSavePoses() {
  const blob = new Blob([JSON.stringify(_actPoses, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'actuator_poses.json';
  a.click();
}

function actLoadPoses() {
  const input = document.createElement('input');
  input.type = 'file'; input.accept = '.json';
  input.onchange = async () => {
    const file = input.files[0]; if (!file) return;
    const text = await file.text();
    try {
      const data = JSON.parse(text);
      Object.assign(_actPoses, data);
      actRenderPoses();
      actSavePosesRemote();
      showToast(`Loaded ${Object.keys(data).length} poses`);
    } catch (e) { showToast('Invalid JSON'); }
  };
  input.click();
}

// ── Sequence Builder ───────────────────────────────────────────────────────
function actRenderSeqSelect() {
  const sel = document.getElementById('act-seq-select');
  if (!sel) return;
  const names = Object.keys(_actSequences);
  sel.innerHTML = '<option value="">— new —</option>' +
    names.map(n => `<option value="${n}">${n}</option>`).join('');
}

function actLoadSeq(name) {
  if (!name) { _actSeqSteps = []; actRenderSeqSteps(); return; }
  const seq = _actSequences[name];
  if (!seq) return;
  document.getElementById('act-seq-name').value = name;
  _actSeqSteps = JSON.parse(JSON.stringify(seq.steps || []));
  actRenderSeqSteps();
}

function actSeqAddPose() {
  const names = Object.keys(_actPoses);
  if (names.length === 0) { showToast('Create poses first'); return; }
  const name = prompt('Pose name to add:', names[0]);
  if (!name || !_actPoses[name]) { showToast('Unknown pose'); return; }
  _actSeqSteps.push({ type: 'pose', pose: name });
  actRenderSeqSteps();
}

function actSeqAddDelay() {
  const ms = prompt('Delay (ms):', '500');
  if (!ms) return;
  _actSeqSteps.push({ type: 'delay', ms: parseInt(ms) || 500 });
  actRenderSeqSteps();
}

function actSeqAddCmd() {
  const cmd = prompt('Raw command:', 'grab(CA)');
  if (!cmd) return;
  _actSeqSteps.push({ type: 'cmd', cmd });
  actRenderSeqSteps();
}

function actSeqRemoveStep(idx) {
  _actSeqSteps.splice(idx, 1);
  actRenderSeqSteps();
}

function actSeqMoveStep(idx, dir) {
  const ni = idx + dir;
  if (ni < 0 || ni >= _actSeqSteps.length) return;
  [_actSeqSteps[idx], _actSeqSteps[ni]] = [_actSeqSteps[ni], _actSeqSteps[idx]];
  actRenderSeqSteps();
}

function actRenderSeqSteps() {
  const el = document.getElementById('act-seq-steps');
  if (!el) return;
  if (_actSeqSteps.length === 0) {
    el.innerHTML = '<span class="act-empty">No steps. Add poses, delays, or commands.</span>';
    return;
  }
  el.innerHTML = _actSeqSteps.map((s, i) => {
    let icon, label;
    if (s.type === 'pose')  { icon = '🎯'; label = `Pose: <b>${s.pose}</b>`; }
    else if (s.type === 'delay') { icon = '⏱'; label = `Wait ${s.ms} ms`; }
    else if (s.type === 'cmd')   { icon = '⚡'; label = `<code>${s.cmd}</code>`; }
    else { icon = '?'; label = JSON.stringify(s); }
    return `<div class="act-seq-step" id="act-step-${i}">
      <span class="act-step-idx">${i+1}</span>
      <span class="act-step-icon">${icon}</span>
      <span class="act-step-label">${label}</span>
      <div class="act-step-btns">
        <button class="btn-tiny" onclick="actSeqMoveStep(${i},-1)" title="Move up">↑</button>
        <button class="btn-tiny" onclick="actSeqMoveStep(${i},1)" title="Move down">↓</button>
        <button class="btn-tiny" onclick="actSeqRemoveStep(${i})">✕</button>
      </div>
    </div>`;
  }).join('');
}

async function actSeqSave() {
  const name = (document.getElementById('act-seq-name')?.value || 'untitled').trim().replace(/\s+/g, '_');
  if (!name) return;
  const data = { steps: _actSeqSteps };
  await fetch(`/api/actuator/sequences/${encodeURIComponent(name)}`, {
    method: 'PUT', headers: {'Content-Type':'application/json'},
    body: JSON.stringify(data)
  });
  _actSequences[name] = data;
  actRenderSeqSelect();
  document.getElementById('act-seq-select').value = name;
  showToast(`Sequence "${name}" saved`);
}

async function actSeqDelete() {
  const name = document.getElementById('act-seq-select')?.value;
  if (!name) return;
  await fetch(`/api/actuator/sequences/${encodeURIComponent(name)}`, { method: 'DELETE' });
  delete _actSequences[name];
  actRenderSeqSelect();
  _actSeqSteps = [];
  actRenderSeqSteps();
  showToast(`Sequence "${name}" deleted`);
}

async function actSeqPlay() {
  if (_actSeqPlaying) return;
  if (_actSeqSteps.length === 0) { showToast('Empty sequence'); return; }
  _actSeqPlaying = true;
  _actSeqAbort = false;
  document.getElementById('act-seq-play-btn')?.classList.add('active');
  const st = document.getElementById('act-seq-status');

  for (let i = 0; i < _actSeqSteps.length; i++) {
    if (_actSeqAbort) break;
    const step = _actSeqSteps[i];
    // Highlight current step
    document.querySelectorAll('.act-seq-step').forEach(el => el.classList.remove('playing'));
    document.getElementById('act-step-' + i)?.classList.add('playing');

    if (step.type === 'pose') {
      if (st) st.textContent = `▶ Step ${i+1}: applying pose "${step.pose}"`;
      await actPoseApply(step.pose);
    } else if (step.type === 'delay') {
      if (st) st.textContent = `▶ Step ${i+1}: waiting ${step.ms}ms`;
      await new Promise(r => setTimeout(r, step.ms));
    } else if (step.type === 'cmd') {
      if (st) st.textContent = `▶ Step ${i+1}: ${step.cmd}`;
      await actCmd(step.cmd);
    }
  }

  document.querySelectorAll('.act-seq-step').forEach(el => el.classList.remove('playing'));
  if (st) st.textContent = _actSeqAbort ? '⏹ Stopped' : '✓ Sequence complete';
  _actSeqPlaying = false;
  document.getElementById('act-seq-play-btn')?.classList.remove('active');
}

function actSeqStop() {
  _actSeqAbort = true;
}

function actSeqExportClip() {
  // Generate Python-like code from the sequence
  const lines = _actSeqSteps.map(s => {
    if (s.type === 'pose') {
      const p = _actPoses[s.pose];
      if (!p) return `# Unknown pose: ${s.pose}`;
      return Object.entries(p).map(([key, angle]) => {
        const [group, sid] = key.split(':');
        return `brain.exec("servo(${group},${sid},${angle})")`;
      }).join('\n');
    }
    if (s.type === 'delay') return `time.sleep(${(s.ms/1000).toFixed(3)})`;
    if (s.type === 'cmd') return `brain.exec("${s.cmd}")`;
    return `# unknown step`;
  }).join('\n');

  navigator.clipboard.writeText(lines).then(() => showToast('Copied to clipboard'));
}

// Init actuator page when navigating to it
// (called from switchView or on page load if that's the active view)