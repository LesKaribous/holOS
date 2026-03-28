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
const ROBOT_IMG_OFFSET = 0;  // adjust if robot.png front ≠ East

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
  ctx.strokeRect(wx(0), wy(FIELD_H), wlen(FIELD_W), wlen(FIELD_H));

  if (lastState) render(lastState);
}

window.addEventListener('resize', () => { if (activeView === 'map') resizeCanvas(); });

if (window.ResizeObserver) {
  const wrap = document.getElementById('canvas-wrap');
  if (wrap) new ResizeObserver(() => { if (activeView === 'map') resizeCanvas(); }).observe(wrap);
}

// ── Coordinate helpers ────────────────────────────────────────────────────
function wx(mm)   { return offX + mm * scale; }
function wy(mm)   { return offY + (FIELD_H - mm) * scale; }
function wlen(mm) { return mm * scale; }
function canvasToWorld(cx, cy) {
  return { x: (cx - offX) / scale, y: FIELD_H - (cy - offY) / scale };
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
  if (state.hw_connecting) {
    cgSetRobotConnecting();
    _serialSetConnectBusy(true);
  } else if (state.hw_mode !== undefined) {
    cgSetRobot(state.hw_mode);
    _serialSetConnectBusy(false);
  }
  updateNetworkFromState(state);
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

function cgSetRobot(hwConnected) {
  const dot    = document.getElementById('cg-dot-robot');
  const link   = document.getElementById('cg-lnk-robot');
  const sub    = document.getElementById('cg-sub-robot');
  const modeEl = document.getElementById('conn-mode-val');
  if (hwConnected) {
    if (dot)    { dot.className  = 'cg-dot hw-active'; }
    if (link)   { link.className = 'cg-link hw'; }
    if (sub)    { sub.className  = 'cg-sub hw'; sub.textContent = 'XBee'; }
    if (modeEl) { modeEl.textContent = 'Hardware (XBee)'; }
  } else {
    if (dot)    { dot.className  = 'cg-dot sim-active'; }
    if (link)   { link.className = 'cg-link sim'; }
    if (sub)    { sub.className  = 'cg-sub sim'; sub.textContent = 'SIM'; }
    if (modeEl) { modeEl.textContent = 'Simulator'; }
  }
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
function switchView(name, btn) {
  activeView = name;
  document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
  document.querySelectorAll('.nav-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('view-' + name).classList.add('active');
  btn.classList.add('active');

  const miniWrap = document.getElementById('mini-map-wrap');
  if (miniWrap) miniWrap.style.display = (name === 'map') ? 'none' : 'block';

  if (name === 'map') {
    requestAnimationFrame(() => requestAnimationFrame(resizeCanvas));
  } else if (name === 'strategy') {
    refreshStrategyList();
  } else if (name === 'macros') {
    renderMacroList();
  } else if (name === 'terminal') {
    serialRefreshPorts();
  } else if (name === 'network') {
    initNetwork();
  }
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
  drawDynObs(s);
  drawGameObjects(s);
  drawPOIs(ctx, wx, wy, wlen, poiData);
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
  const fx = wxf(0), fy = wyf(FIELD_H), fw = wlf(FIELD_W), fh = wlf(FIELD_H);
  if (terrainImg) {
    c.drawImage(terrainImg, fx, fy, fw, fh);
    c.fillStyle = 'rgba(0,0,0,0.15)';
    c.fillRect(fx, fy, fw, fh);
  } else {
    c.fillStyle = '#c8d4be';
    c.fillRect(fx, fy, fw, fh);
    c.fillStyle = 'rgba(180,140,0,.09)';
    c.fillRect(wxf(0), wyf(FIELD_H), wlf(1500), wlf(FIELD_H));
    c.fillStyle = 'rgba(30,80,200,.06)';
    c.fillRect(wxf(1500), wyf(FIELD_H), wlf(1500), wlf(FIELD_H));
  }
  c.strokeStyle = 'rgba(26,82,118,.5)';
  c.lineWidth = Math.max(1.5, wlf(4));
  c.strokeRect(fx, fy, fw, fh);
}

function drawGrid(s) {
  ctx.strokeStyle = 'rgba(26,82,118,.18)'; ctx.lineWidth = .5;
  for (let gx=0; gx<=GRID_W; gx++) {
    ctx.beginPath(); ctx.moveTo(wx(gx*GRID_CELL), wy(0)); ctx.lineTo(wx(gx*GRID_CELL), wy(FIELD_H)); ctx.stroke();
  }
  for (let gy=0; gy<=GRID_H; gy++) {
    ctx.beginPath(); ctx.moveTo(wx(0), wy(gy*GRID_CELL)); ctx.lineTo(wx(FIELD_W), wy(gy*GRID_CELL)); ctx.stroke();
  }
  ctx.fillStyle = 'rgba(192,57,43,.35)';
  for (const c of s.occupancy) {
    ctx.fillRect(wx(c.gx*GRID_CELL)+1, wy((c.gy+1)*GRID_CELL)+1, wlen(GRID_CELL)-2, wlen(GRID_CELL)-2);
  }
}

function drawDynObs(s) {
  for (const o of s.dyn_obs) {
    ctx.beginPath(); ctx.arc(wx(o[0]), wy(o[1]), wlen(150), 0, Math.PI*2);
    ctx.fillStyle='rgba(192,57,43,.25)'; ctx.fill();
    ctx.strokeStyle='#c0392b'; ctx.lineWidth=1.5; ctx.stroke();
  }
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
canvas.addEventListener('mousemove', e => {
  const rect=canvas.getBoundingClientRect(), w=canvasToWorld(e.clientX-rect.left,e.clientY-rect.top);
  const inF=w.x>=0&&w.x<=FIELD_W&&w.y>=0&&w.y<=FIELD_H;
  const el=document.getElementById('coord-display');
  if (el) el.textContent=inF?`x: ${w.x.toFixed(0)} mm   y: ${w.y.toFixed(0)} mm`:'x:— y:—';
});
canvas.addEventListener('click', e => {
  const rect=canvas.getBoundingClientRect(), w=canvasToWorld(e.clientX-rect.left,e.clientY-rect.top);
  if (w.x<0||w.x>FIELD_W||w.y<0||w.y>FIELD_H) return;
  socket.emit('field_click',{x:w.x,y:w.y,button:0});
});
canvas.addEventListener('contextmenu', e => {
  e.preventDefault();
  const rect=canvas.getBoundingClientRect(), w=canvasToWorld(e.clientX-rect.left,e.clientY-rect.top);
  if (w.x<0||w.x>FIELD_W||w.y<0||w.y>FIELD_H) return;
  socket.emit('field_click',{x:w.x,y:w.y,button:2});
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
function setFeature(k,v)  { socket.emit('set_feature',{feature:k,value:v}); }
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
  const baud = (document.getElementById('serial-baud-sel') || document.getElementById('t-baud-sel'))?.value || '31250';
  if (!port) { showToast('Select a serial port first'); return; }
  cgSetRobotConnecting();
  setSerialStatus('Connecting…', '');
  socket.emit('serial_connect', {port, baud: parseInt(baud)});
}
function serialDisconnect() {
  socket.emit('serial_disconnect');
  setSerialStatus('Disconnected', '');
  cgSetRobot(false);
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
//  NETWORK VIEW  — canvas topology diagram + side config panel
// ══════════════════════════════════════════════════════════════════════════

// ── Runtime state ──────────────────────────────────────────────────────────
const _ns = {
  ws:       'disconnected',   // 'connected' | 'disconnected'
  hw:       'sim',            // 'sim' | 'connecting' | 'connected' | 'disconnected'
  hwType:   'sim',            // 'sim' | 'usb' | 'xbee'
  hwLabel:  'Virtual (SIM)',
  uart:     'unknown',        // 'connected' | 'disconnected' | 'unknown'
  selected: null,             // hovered / selected id
  selType:  null,             // 'link' | 'node'
};

// ── Node / link definitions ─────────────────────────────────────────────────
const _nNodes = [
  { id: 'browser', label: 'Browser',    sub: 'PC',          icon: '🖥', col: 0.10 },
  { id: 'holos',   label: 'holOS',      sub: 'Server',      icon: '⚙',  col: 0.36 },
  { id: 't41',     label: 'Teensy 4.1', sub: 'Main MCU',    icon: '🎛', col: 0.65 },
  { id: 't40',     label: 'Teensy 4.0', sub: 'Secondary',   icon: '🔧', col: 0.88 },
];
const _nLinks = [
  { id: 'ws',   fromIdx: 0, toIdx: 1, _hit: null },
  { id: 'hw',   fromIdx: 1, toIdx: 2, _hit: null },
  { id: 'uart', fromIdx: 2, toIdx: 3, _hit: null },
];

const _nsColors = {
  connected:    '#1a8c3c', active:       '#1a8c3c',
  sim:          '#2980b9', connecting:   '#d97706',
  disconnected: '#94a3b8', unknown:      '#94a3b8', error: '#c0392b',
};
const _nsLabels = {
  connected: 'Connected', active: 'Active', sim: 'Simulation',
  connecting: 'Connecting…', disconnected: 'Disconnected', unknown: 'Unknown',
};
const _NN_W = 120, _NN_H = 76, _NN_R = 10;
let _netAnimFrame = null;

// ── Status helpers ──────────────────────────────────────────────────────────
function _nsNodeStatus(nid) {
  if (nid === 'browser') return _ns.ws === 'connected' ? 'connected' : 'disconnected';
  if (nid === 'holos')   return 'active';
  if (nid === 't41')     return _ns.hw;
  if (nid === 't40')     return _ns.uart;
  return 'unknown';
}
function _nsLinkStatus(lid) {
  return lid==='ws' ? _ns.ws : lid==='hw' ? _ns.hw : lid==='uart' ? _ns.uart : 'unknown';
}
function _nsLinkLabel(lid) {
  return lid==='ws' ? 'WebSocket' : lid==='hw' ? _ns.hwLabel : lid==='uart' ? 'UART' : '';
}
function _nsCol(st) { return _nsColors[st] || '#94a3b8'; }

// ── Canvas geometry ─────────────────────────────────────────────────────────
function _ncBox(idx, W, H) {
  const cx = W * _nNodes[idx].col, cy = H * 0.42;
  return { cx, cy, x: cx-_NN_W/2, y: cy-_NN_H/2, w: _NN_W, h: _NN_H };
}
function _ncCanvas() { return document.getElementById('net-canvas'); }

// ── Main draw ───────────────────────────────────────────────────────────────
function _ncDraw() {
  _netAnimFrame = null;
  const c = _ncCanvas();
  if (!c || activeView !== 'network') return;
  const ctx = c.getContext('2d');
  const W = c.width, H = c.height;
  if (!W || !H) return;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#f0f4f8';
  ctx.fillRect(0, 0, W, H);

  // Dot grid
  ctx.fillStyle = 'rgba(26,82,118,.06)';
  for (let x = 30; x < W; x += 40)
    for (let y = 30; y < H; y += 40) {
      ctx.beginPath(); ctx.arc(x, y, 1.5, 0, Math.PI*2); ctx.fill();
    }

  _nLinks.forEach(lk => {
    _ncDrawLink(ctx, lk, _ncBox(lk.fromIdx, W, H), _ncBox(lk.toIdx, W, H));
  });
  _nNodes.forEach((nd, i) => _ncDrawNode(ctx, nd, i, W, H));

  if (_ns.hw === 'connecting')
    _netAnimFrame = requestAnimationFrame(_ncDraw);
}

// ── Draw node ───────────────────────────────────────────────────────────────
function _ncDrawNode(ctx, nd, idx, W, H) {
  const b   = _ncBox(idx, W, H);
  const st  = _nsNodeStatus(nd.id);
  const col = _nsCol(st);
  const sel = _ns.selected === nd.id && _ns.selType === 'node';

  ctx.save();
  ctx.shadowColor = 'rgba(26,82,118,' + (sel ? '.18' : '.09') + ')';
  ctx.shadowBlur  = sel ? 14 : 5;
  ctx.shadowOffsetY = 2;
  ctx.fillStyle   = '#fff';
  _ncRR(ctx, b.x, b.y, b.w, b.h, _NN_R); ctx.fill();
  ctx.restore();

  ctx.save();
  ctx.strokeStyle = col + (sel ? 'ff' : '55');
  ctx.lineWidth   = sel ? 2.5 : 1.5;
  _ncRR(ctx, b.x, b.y, b.w, b.h, _NN_R); ctx.stroke();
  ctx.restore();

  // Status dot
  ctx.save();
  if (st === 'connecting') ctx.globalAlpha = .5 + .5*Math.sin(Date.now()/400);
  ctx.fillStyle = col;
  ctx.beginPath(); ctx.arc(b.x+b.w-11, b.y+11, 4.5, 0, Math.PI*2); ctx.fill();
  ctx.restore();

  // Icon
  ctx.font = '22px serif';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(nd.icon, b.cx, b.y+25);

  // Labels
  ctx.font = 'bold 12px "Segoe UI",system-ui,sans-serif';
  ctx.fillStyle = '#1a2332';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(nd.label, b.cx, b.y+49);
  ctx.font = '10px "Segoe UI",system-ui,sans-serif';
  ctx.fillStyle = '#6b7a8d';
  ctx.fillText(nd.sub, b.cx, b.y+63);
}

// ── Draw link ───────────────────────────────────────────────────────────────
function _ncDrawLink(ctx, lk, b1, b2) {
  const st  = _nsLinkStatus(lk.id);
  const col = _nsCol(st);
  const sel = _ns.selected === lk.id && _ns.selType === 'link';

  const x1 = b1.cx+_NN_W/2, y1 = b1.cy;
  const x2 = b2.cx-_NN_W/2, y2 = b2.cy;

  ctx.save();
  ctx.strokeStyle = col;
  ctx.lineWidth   = sel ? 3 : 2;
  ctx.lineCap     = 'round';
  if (st === 'disconnected' || st === 'unknown') {
    ctx.setLineDash([5,6]);
  } else if (st === 'connecting') {
    ctx.setLineDash([5,6]);
    ctx.lineDashOffset = -((Date.now()/80) % 11);
    ctx.globalAlpha    = .55 + .45*Math.sin(Date.now()/250);
  }
  ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
  ctx.restore();

  // Arrowhead
  if (st !== 'disconnected' && st !== 'unknown') {
    const ang = Math.atan2(y2-y1, x2-x1), AW = 7;
    ctx.fillStyle = col;
    ctx.beginPath();
    ctx.moveTo(x2,y2);
    ctx.lineTo(x2-AW*Math.cos(ang-Math.PI/6), y2-AW*Math.sin(ang-Math.PI/6));
    ctx.lineTo(x2-AW*Math.cos(ang+Math.PI/6), y2-AW*Math.sin(ang+Math.PI/6));
    ctx.closePath(); ctx.fill();
  }

  // Label pill
  const mx = (x1+x2)/2, my = (y1+y2)/2 - 20;
  const lbl = _nsLinkLabel(lk.id);
  ctx.font = '10px "Segoe UI",system-ui,sans-serif';
  const PW = ctx.measureText(lbl).width + 14, PH = 18;
  ctx.save();
  ctx.fillStyle   = sel ? col : '#fff';
  ctx.strokeStyle = col;
  ctx.lineWidth   = 1;
  _ncRR(ctx, mx-PW/2, my-PH/2, PW, PH, 9); ctx.fill(); ctx.stroke();
  ctx.fillStyle    = sel ? '#fff' : col;
  ctx.textAlign    = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(lbl, mx, my);
  ctx.restore();

  lk._hit = { x1, y1, x2, y2 };
}

// ── Rounded rect path ────────────────────────────────────────────────────────
function _ncRR(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x+r,y); ctx.lineTo(x+w-r,y);
  ctx.quadraticCurveTo(x+w,y, x+w,y+r); ctx.lineTo(x+w,y+h-r);
  ctx.quadraticCurveTo(x+w,y+h, x+w-r,y+h); ctx.lineTo(x+r,y+h);
  ctx.quadraticCurveTo(x,y+h, x,y+h-r); ctx.lineTo(x,y+r);
  ctx.quadraticCurveTo(x,y, x+r,y); ctx.closePath();
}

// ── Hit test ─────────────────────────────────────────────────────────────────
function _ncHit(mx, my) {
  const c = _ncCanvas(); if (!c) return null;
  const W = c.width, H = c.height;
  for (let i = 0; i < _nNodes.length; i++) {
    const b = _ncBox(i, W, H);
    if (mx>=b.x && mx<=b.x+b.w && my>=b.y && my<=b.y+b.h) return {type:'node', id:_nNodes[i].id};
  }
  for (const lk of _nLinks) {
    if (!lk._hit) continue;
    const {x1,y1,x2,y2} = lk._hit;
    const dx=x2-x1,dy=y2-y1,lq=dx*dx+dy*dy;
    const t = lq ? Math.max(0,Math.min(1,((mx-x1)*dx+(my-y1)*dy)/lq)) : 0;
    if (Math.hypot(mx-(x1+t*dx), my-(y1+t*dy)) <= 16) return {type:'link', id:lk.id};
  }
  return null;
}

// ── Resize ───────────────────────────────────────────────────────────────────
function _ncResize() {
  const c = _ncCanvas(), area = document.getElementById('net-canvas-area');
  if (!c || !area) return;
  c.width  = area.clientWidth;
  c.height = area.clientHeight;
  _ncDraw();
}

// ── Init (called on switchView) ───────────────────────────────────────────────
function initNetwork() {
  const c = _ncCanvas(); if (!c) return;
  if (!c._netWired) {
    c._netWired = true;
    c.addEventListener('click', evt => {
      const r = c.getBoundingClientRect();
      const h = _ncHit(evt.clientX-r.left, evt.clientY-r.top);
      _ns.selected = h ? h.id   : null;
      _ns.selType  = h ? h.type : null;
      _ncUpdatePanel(h); _ncDraw();
    });
    c.addEventListener('mousemove', evt => {
      const r = c.getBoundingClientRect();
      c.style.cursor = _ncHit(evt.clientX-r.left, evt.clientY-r.top) ? 'pointer' : 'default';
    });
    new ResizeObserver(() => { if (activeView==='network') _ncResize(); })
      .observe(document.getElementById('net-canvas-area'));
  }
  _ncResize();
  _ncUpdatePanel(null);
}

// ── State update from socket ─────────────────────────────────────────────────
function updateNetworkFromState(state) {
  const newHw = state.hw_connecting ? 'connecting' :
                state.hw_mode       ? 'connected'  : 'sim';
  const newHwType  = state.hw_type  || (state.hw_mode ? 'usb' : 'sim');
  const newHwLabel = newHw==='sim'         ? 'Virtual (SIM)'  :
                     newHwType==='xbee'    ? 'XBee Radio'     : 'USB Serial';

  const changed = _ns.hw!==newHw || _ns.hwType!==newHwType;
  _ns.hw = newHw; _ns.hwType = newHwType; _ns.hwLabel = newHwLabel;

  if (!changed) return;
  const badge = document.getElementById('np-hw-badge');
  const mode  = document.getElementById('np-hw-mode');
  const btn   = document.getElementById('np-connect-btn');
  if (badge) { badge.textContent = _nsLabels[_ns.hw]||_ns.hw; badge.className='np-badge np-'+_ns.hw; }
  if (mode)  mode.textContent = _ns.hwLabel;
  if (btn)   btn.disabled = _ns.hw === 'connecting';
  if (activeView==='network') {
    _ncDraw();
    if (_ns.hw==='connecting' && !_netAnimFrame)
      _netAnimFrame = requestAnimationFrame(_ncDraw);
  }
}

// ── Side panel generators ────────────────────────────────────────────────────
function _ncUpdatePanel(hit) {
  const title = document.getElementById('net-panel-title');
  const body  = document.getElementById('net-panel-body');
  if (!title || !body) return;
  if (!hit) { title.textContent='Network'; body.innerHTML=_ncPanelOverview(); return; }
  if (hit.type === 'link') {
    const T = {ws:'Browser ↔ holOS', hw:'holOS ↔ Teensy 4.1', uart:'Teensy 4.1 ↔ 4.0'};
    title.textContent = T[hit.id]||hit.id;
    if (hit.id==='ws')   body.innerHTML = _ncPanelWS();
    if (hit.id==='hw')   { body.innerHTML = _ncPanelHW(); _ncPanelHWInit(); }
    if (hit.id==='uart') body.innerHTML = _ncPanelUART();
  } else {
    const nd = _nNodes.find(n=>n.id===hit.id)||{id:hit.id,icon:'?',label:hit.id,sub:''};
    title.textContent = nd.label; body.innerHTML = _ncPanelNode(nd);
  }
}

function _ncPanelOverview() {
  return `<div class="np-hint">Click a node or link to inspect and configure.</div>
    <div class="np-section">
      <div class="np-row"><span>WebSocket</span><span class="np-badge np-${_ns.ws}">${_nsLabels[_ns.ws]||_ns.ws}</span></div>
      <div class="np-row"><span>holOS ↔ Teensy 4.1</span><span class="np-badge np-${_ns.hw}">${_nsLabels[_ns.hw]||_ns.hw}</span></div>
      <div class="np-row"><span>Teensy 4.1 ↔ 4.0</span><span class="np-badge np-${_ns.uart}">${_nsLabels[_ns.uart]||_ns.uart}</span></div>
    </div>
    <div class="np-section">
      <div class="np-row"><span>Mode</span><span>${_ns.hwLabel}</span></div>
      <div class="np-row"><span>Server</span><span>${window.location.host}</span></div>
    </div>`;
}

function _ncPanelWS() {
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${_ns.ws}">${_nsLabels[_ns.ws]||_ns.ws}</span></div>
      <div class="np-row"><span>Server</span><span>${window.location.host}</span></div>
      <div class="np-row"><span>Transport</span><span>Socket.IO</span></div>
    </div>
    <div class="np-hint">The WebSocket connection is established automatically and maintained by Socket.IO. If disconnected, the page will reconnect automatically.</div>`;
}

function _ncPanelHW() {
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${_ns.hw}" id="np-hw-badge">${_nsLabels[_ns.hw]||_ns.hw}</span></div>
      <div class="np-row"><span>Mode</span><span id="np-hw-mode">${_ns.hwLabel}</span></div>
    </div>
    <div class="np-section">
      <label class="np-label">Serial port</label>
      <div style="display:flex;gap:6px;margin-bottom:8px">
        <select id="np-port-sel" class="input-sm" style="flex:1;min-width:0"></select>
        <button class="btn-tiny" onclick="_ncRefreshPorts()" title="Refresh">↺</button>
      </div>
      <label class="np-label">Baud rate</label>
      <select id="np-baud-sel" class="input-sm" style="width:100%;margin-bottom:10px">
        <option value="115200" selected>115 200 — USB direct</option>
        <option value="31250">31 250 — XBee / Jetson</option>
        <option value="57600">57 600</option>
        <option value="9600">9 600</option>
      </select>
      <div style="display:flex;gap:6px">
        <button id="np-connect-btn" class="btn green btn-sm" style="flex:1" onclick="_ncDoConnect()">Connect</button>
        <button class="btn red btn-sm" style="flex:1" onclick="serialDisconnect()">Disconnect</button>
      </div>
    </div>
    <div class="np-hint"><b>USB direct:</b> flash with <code>USB_INTERCOM</code> @ 115 200 bd.<br><b>XBee / Jetson:</b> comment out <code>USB_INTERCOM</code>, use Serial1 @ 31 250 bd.</div>`;
}

function _ncPanelHWInit() {
  _ncRefreshPorts();
  const mb = document.getElementById('serial-baud-sel');
  const nb = document.getElementById('np-baud-sel');
  if (mb && nb) nb.value = mb.value;
  const btn = document.getElementById('np-connect-btn');
  if (btn) btn.disabled = _ns.hw === 'connecting';
}

function _ncRefreshPorts() {
  const sel = document.getElementById('np-port-sel'); if (!sel) return;
  const cur = (document.getElementById('serial-port-sel')||{}).value || '';
  fetch('/api/serial/ports').then(r=>r.json()).then(ports => {
    sel.innerHTML = '<option value="">— port —</option>' +
      ports.map(p=>`<option value="${escHtml(p.port)}"${p.port===cur?' selected':''}>${escHtml(p.port)} — ${escHtml(p.desc)}</option>`).join('');
  });
}

function _ncDoConnect() {
  const ps = document.getElementById('np-port-sel');
  const bs = document.getElementById('np-baud-sel');
  if (!ps?.value) { showToast('Select a serial port first'); return; }
  const mp = document.getElementById('serial-port-sel');
  const mb = document.getElementById('serial-baud-sel');
  if (mp) mp.value = ps.value;
  if (mb) mb.value = bs ? bs.value : '115200';
  serialConnect();
}

function _ncPanelUART() {
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${_ns.uart}">${_nsLabels[_ns.uart]||_ns.uart}</span></div>
      <div class="np-row"><span>Physical</span><span>UART (hardware)</span></div>
      <div class="np-row"><span>Direction</span><span>Bidirectional</span></div>
    </div>
    <div class="np-hint">The UART between Teensy 4.1 and 4.0 is internal. Its state will be reported by firmware telemetry when T4.1 is connected.</div>`;
}

function _ncPanelNode(nd) {
  const st   = _nsNodeStatus(nd.id);
  const rows = {
    browser: [['Type','Browser (client)'],['Protocol','Socket.IO / HTTP']],
    holos:   [['Type','Flask + SocketIO'],['Host',window.location.host],['Physics','60 Hz sim']],
    t41:     [['MCU','Teensy 4.1 (600 MHz)'],['Role','Motion + Sensors'],['FW','PlatformIO / Arduino']],
    t40:     [['MCU','Teensy 4.0'],['Role','Actuators + I/O'],['FW','PlatformIO / Arduino']],
  };
  const extra = (rows[nd.id]||[]).map(([k,v])=>`<div class="np-row"><span>${k}</span><span>${v}</span></div>`).join('');
  return `<div class="np-section">
      <div class="np-title-row">${nd.icon} ${nd.label}</div>
      <div class="np-divider"></div>
      <div class="np-row"><span>Status</span><span class="np-badge np-${st}">${_nsLabels[st]||st}</span></div>
      ${extra}
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
  idle:    '○',
  running: '⋯',
  passed:  '✓',
  failed:  '✗',
  stopped: '—',
};

// ── Init ─────────────────────────────────────────────────────────────────────
function initTests() {
  // Fetch catalog from server
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

  // SocketIO events
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
}

// ── Catalog UI ───────────────────────────────────────────────────────────────
function _buildCatalogUI() {
  const container = document.getElementById('tests-catalog');
  if (!container) return;
  container.innerHTML = '';

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

  const icon = { running:'⋯', passed:'✓', failed:'✗', stopped:'—' }[status] || '?';
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
