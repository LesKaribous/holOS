// js/map_tools.js — Canvas interaction: pin, trajectory, opponent, tools, controls
// Depends on: canvas, ctx, socket, lastState, scale, wx, wy, wlen, canvasToWorld,
//   FIELD_W, FIELD_H, GRID_W, GRID_H, GRID_CELL, showToast, render, escHtml

'use strict';

// ── Small table rectangle ──────────────────────────────────────────────────
let smallTable = { enabled: false, x: 500, y: 500, w: 1200, h: 800 };
let _stDragStart = null;

function toggleSmallTable(on) {
  smallTable.enabled = on;
  if (lastState) render(lastState);
}

function updateSmallTable() {
  smallTable.x = +(document.getElementById('st-x')?.value || 0);
  smallTable.y = +(document.getElementById('st-y')?.value || 0);
  smallTable.w = +(document.getElementById('st-w')?.value || 0);
  smallTable.h = +(document.getElementById('st-h')?.value || 0);
  if (lastState) render(lastState);
}

function _syncSmallTableUI() {
  const el = (id) => document.getElementById(id);
  if (el('st-x')) el('st-x').value = Math.round(smallTable.x);
  if (el('st-y')) el('st-y').value = Math.round(smallTable.y);
  if (el('st-w')) el('st-w').value = Math.round(smallTable.w);
  if (el('st-h')) el('st-h').value = Math.round(smallTable.h);
  if (el('st-enabled')) el('st-enabled').checked = smallTable.enabled;
}

function drawSmallTable() {
  if (!smallTable.enabled) return;
  ctx.save();
  ctx.strokeStyle = 'rgba(255, 152, 0, 0.7)';
  ctx.lineWidth   = Math.max(1.5, scale * 3);
  ctx.setLineDash([wlen(30), wlen(15)]);
  ctx.strokeRect(wx(smallTable.x), wy(smallTable.y), wlen(smallTable.w), wlen(smallTable.h));
  ctx.setLineDash([]);
  ctx.fillStyle = 'rgba(255, 152, 0, 0.06)';
  ctx.fillRect(wx(smallTable.x), wy(smallTable.y), wlen(smallTable.w), wlen(smallTable.h));
  ctx.fillStyle = 'rgba(255, 152, 0, 0.65)';
  ctx.font = `${Math.max(9, wlen(40))}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.fillText('Small Table', wx(smallTable.x + smallTable.w/2), wy(smallTable.y + smallTable.h/2) + wlen(15));
  ctx.font = `${Math.max(8, wlen(28))}px sans-serif`;
  ctx.fillText(`${Math.round(smallTable.w)} × ${Math.round(smallTable.h)} mm`,
    wx(smallTable.x + smallTable.w/2), wy(smallTable.y + smallTable.h/2) + wlen(55));
  ctx.restore();
}

// ── Map pin & trajectory ───────────────────────────────────────────────────
let _mapPin  = null;
let _mapTool = null;
let _mapTraj = [];

function toggleMapTool(tool) {
  _mapTool = (_mapTool === tool) ? null : tool;
  document.getElementById('tool-obs-add')?.classList.toggle('active', _mapTool === 'obstacle');
  document.getElementById('tool-obs-del')?.classList.toggle('active', _mapTool === 'remove_obs');
  document.getElementById('tool-place-opp')?.classList.toggle('active', _mapTool === 'place_opp');
  document.getElementById('tool-draw-table')?.classList.toggle('active', _mapTool === 'draw_table');
  canvas.style.cursor = _mapTool ? 'crosshair' : '';
}

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

let _goInFlight = false;

function pinGoHere() {
  if (!_mapPin) return;
  if (_goInFlight) { showToast('Motion in progress…'); return; }
  _goInFlight = true;
  const currentThetaDeg = lastState?.robot ? lastState.robot.theta * 180 / Math.PI : null;
  const pinTheta = _mapPin.theta ?? 0;
  const rotationChanged = currentThetaDeg == null || Math.abs(pinTheta - currentThetaDeg) > 2.0;
  const body = { x: _mapPin.x, y: _mapPin.y };
  if (rotationChanged) body.theta = pinTheta;
  fetch('/api/go', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify(body)
  }).then(r => r.json()).then(d => {
    const tag = rotationChanged
      ? `(${_mapPin.x}, ${_mapPin.y}, ${pinTheta.toFixed(1)}°)`
      : `(${_mapPin.x}, ${_mapPin.y})`;
    showToast(d.ok ? `Go → ${tag}` : `Error: ${d.res}`);
  }).catch(() => showToast('Request failed'))
    .finally(() => { _goInFlight = false; });
}

function pinSetPos() {
  if (!_mapPin) return;
  socket.emit('set_robot_pos', { x: _mapPin.x, y: _mapPin.y, theta: _mapPin.theta || 0 });
  showToast(`Position set → (${_mapPin.x}, ${_mapPin.y}, ${_mapPin.theta || 0})`);
}

function pinCopy() {
  if (!_mapPin) return;
  const text = `(${_mapPin.x}, ${_mapPin.y}, ${_mapPin.theta || 0})`;
  navigator.clipboard.writeText(text).then(() => showToast(`Copied ${text}`)).catch(() => showToast('Copy failed'));
}

function pinAddWaypoint() {
  if (!_mapPin) return;
  _mapTraj.push({ x: _mapPin.x, y: _mapPin.y });
  _updateTrajBar();
  if (lastState) render(lastState);
  showToast(`Waypoint #${_mapTraj.length}: (${_mapPin.x}, ${_mapPin.y})`);
}

// ── Trajectory bar ─────────────────────────────────────────────────────────
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
  _mapTraj.splice(idx, 1); _updateTrajBar(); if (lastState) render(lastState);
}

function trajClear() {
  _mapTraj = []; _updateTrajBar(); if (lastState) render(lastState);
}

function trajCopy() {
  if (_mapTraj.length === 0) return;
  const text = '[' + _mapTraj.map(p => `(${p.x}, ${p.y})`).join(', ') + ']';
  navigator.clipboard.writeText(text).then(() => showToast(`Trajectory copied (${_mapTraj.length} pts)`)).catch(() => showToast('Copy failed'));
}

async function trajPaste() {
  try {
    const text = await navigator.clipboard.readText();
    const re = /\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)/g;
    let m, pts = [];
    while ((m = re.exec(text)) !== null) pts.push({ x: Math.round(+m[1]), y: Math.round(+m[2]) });
    if (pts.length === 0) { showToast('No valid points found in clipboard'); return; }
    _mapTraj = pts; _updateTrajBar(); if (lastState) render(lastState);
    showToast(`Pasted ${pts.length} waypoints`);
  } catch (e) { showToast('Paste failed — clipboard access denied'); }
}

async function trajExecute() {
  if (_mapTraj.length === 0) return;
  showToast(`Executing trajectory (${_mapTraj.length} pts)…`);
  try {
    const res = await fetch('/api/go_traj', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ waypoints: _mapTraj.map(p => ({ x: p.x, y: p.y })) })
    });
    const d = await res.json();
    if (!d.ok) { showToast(`Trajectory failed: ${d.res}`); return; }
  } catch (e) { showToast('Trajectory request failed'); return; }
  showToast(`Trajectory complete (${_mapTraj.length} pts)`);
}

// ── Canvas draw: pin + trajectory ──────────────────────────────────────────
function drawMapPin() {
  if (!_mapPin) return;
  const px = wx(_mapPin.x), py = wy(_mapPin.y);
  ctx.beginPath(); ctx.arc(px, py, 10, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(41,128,185,.15)'; ctx.fill();
  ctx.strokeStyle = '#2980b9'; ctx.lineWidth = 2; ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(px - 14, py); ctx.lineTo(px + 14, py);
  ctx.moveTo(px, py - 14); ctx.lineTo(px, py + 14);
  ctx.strokeStyle = 'rgba(41,128,185,.5)'; ctx.lineWidth = 1; ctx.stroke();
  ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2);
  ctx.fillStyle = '#2980b9'; ctx.fill();
  if (_mapPin.theta != null) {
    const a = -(_mapPin.theta * Math.PI / 180);
    const arrowLen = 22, headLen = 7;
    const ex = px + Math.cos(a) * arrowLen, ey = py + Math.sin(a) * arrowLen;
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(ex, ey);
    ctx.strokeStyle = '#2980b9'; ctx.lineWidth = 2; ctx.stroke();
    ctx.save(); ctx.translate(ex, ey); ctx.rotate(a);
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-headLen, -4); ctx.lineTo(-headLen, 4); ctx.closePath();
    ctx.fillStyle = '#2980b9'; ctx.fill();
    ctx.restore();
  }
}

function drawMapTraj() {
  if (_mapTraj.length === 0) return;
  ctx.beginPath();
  ctx.moveTo(wx(_mapTraj[0].x), wy(_mapTraj[0].y));
  for (let i = 1; i < _mapTraj.length; i++) ctx.lineTo(wx(_mapTraj[i].x), wy(_mapTraj[i].y));
  ctx.strokeStyle = '#e67e22'; ctx.lineWidth = 2;
  ctx.setLineDash([8, 4]); ctx.stroke(); ctx.setLineDash([]);
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

// ── Grid brush drag ────────────────────────────────────────────────────────
let _brushDown = false;
let _lastBrushCell = null;

function _paintGridCell(wx, wy) {
  const gx = Math.floor(wx / GRID_CELL);
  const gy = Math.floor(wy / GRID_CELL);
  if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H) return;
  const key = gx + ',' + gy;
  if (_lastBrushCell === key) return;
  _lastBrushCell = key;
  const value = (_mapTool === 'obstacle');
  socket.emit('paint_grid', { gx, gy, value });
}

// ── Canvas event listeners ─────────────────────────────────────────────────
canvas.addEventListener('mousedown', e => {
  if (e.button !== 0) return;
  const rect = canvas.getBoundingClientRect(), w = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
  if (w.x < 0 || w.x > FIELD_W || w.y < 0 || w.y > FIELD_H) return;
  if (_mapTool === 'obstacle' || _mapTool === 'remove_obs') {
    _brushDown = true; _lastBrushCell = null; _paintGridCell(w.x, w.y); return;
  }
  if (_mapTool === 'place_opp') { _placeOpponentAt(w.x, w.y); return; }
  if (_mapTool === 'draw_table') { _stDragStart = { x: w.x, y: w.y }; return; }
});

canvas.addEventListener('mousemove', e => {
  const rect = canvas.getBoundingClientRect(), w = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
  const inF = w.x >= 0 && w.x <= FIELD_W && w.y >= 0 && w.y <= FIELD_H;
  const el = document.getElementById('coord-display');
  if (el) el.textContent = inF ? `x: ${w.x.toFixed(0)} mm   y: ${w.y.toFixed(0)} mm` : 'x:— y:—';
  if (_brushDown && inF) _paintGridCell(w.x, w.y);
  if (_stDragStart && inF) {
    smallTable.x = Math.min(_stDragStart.x, w.x);
    smallTable.y = Math.min(_stDragStart.y, w.y);
    smallTable.w = Math.abs(w.x - _stDragStart.x);
    smallTable.h = Math.abs(w.y - _stDragStart.y);
    smallTable.enabled = true;
    _syncSmallTableUI();
    if (lastState) render(lastState);
  }
});

canvas.addEventListener('mouseup', () => {
  _brushDown = false; _lastBrushCell = null;
  if (_stDragStart) { _stDragStart = null; _syncSmallTableUI(); }
});

canvas.addEventListener('mouseleave', () => {
  _brushDown = false; _lastBrushCell = null;
  if (_stDragStart) { _stDragStart = null; }
});

canvas.addEventListener('click', e => {
  if (_brushDown) return;
  const rect = canvas.getBoundingClientRect(), w = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
  if (w.x < 0 || w.x > FIELD_W || w.y < 0 || w.y > FIELD_H) return;
  if (_mapTool === 'obstacle' || _mapTool === 'remove_obs' || _mapTool === 'place_opp' || _mapTool === 'draw_table') return;
  // Approach-picking mode for missions
  if (typeof _onMapClickForMission === 'function' && _onMapClickForMission(w.x, w.y)) return;
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
  const gx = Math.floor(w.x / GRID_CELL), gy = Math.floor(w.y / GRID_CELL);
  if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) {
    socket.emit('field_click', { x: w.x, y: w.y, button: 2 });
  }
});

// ── Opponent system ────────────────────────────────────────────────────────
let _oppSeq = [];
let _oppSeqPlaying = false;
let _oppSeqTimer = null;

function toggleOpponent(enabled) { socket.emit('set_opponent_enabled', { enabled }); }

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
  const cb = document.getElementById('opp-enabled');
  if (cb && !cb.checked) { cb.checked = true; toggleOpponent(true); }
  updateOpponentPos();
  toggleMapTool('place_opp');
}

function oppSeqAdd() {
  const x = parseFloat(document.getElementById('opp-x')?.value || 0);
  const y = parseFloat(document.getElementById('opp-y')?.value || 0);
  const theta = parseFloat(document.getElementById('opp-theta')?.value || 0);
  _oppSeq.push({ x, y, theta });
  _renderOppSeq();
}

function oppSeqClear() { _oppSeq = []; _renderOppSeq(); }

function oppSeqRemove(idx) { _oppSeq.splice(idx, 1); _renderOppSeq(); }

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
  let segIdx = 0, segProgress = 0;
  function tick() {
    if (!_oppSeqPlaying || segIdx >= _oppSeq.length - 1) {
      _oppSeqPlaying = false;
      document.getElementById('opp-seq-play')?.classList.remove('active');
      return;
    }
    const a = _oppSeq[segIdx], b = _oppSeq[segIdx + 1];
    const dist = Math.hypot(b.x - a.x, b.y - a.y);
    const dt = 1/30;
    const step = (speed * dt) / (dist || 1);
    segProgress += step;
    if (segProgress >= 1) { segProgress = 0; segIdx++; if (segIdx >= _oppSeq.length - 1) segIdx = 0; }
    const p = _oppSeq[segIdx], q = _oppSeq[Math.min(segIdx + 1, _oppSeq.length - 1)];
    const t = Math.min(segProgress, 1);
    socket.emit('set_opponent_pos', { x: p.x + (q.x - p.x) * t, y: p.y + (q.y - p.y) * t, theta: Math.atan2(q.y - p.y, q.x - p.x) });
    _oppSeqTimer = setTimeout(tick, dt * 1000);
  }
  tick();
}

function oppSeqStop() {
  _oppSeqPlaying = false;
  if (_oppSeqTimer) { clearTimeout(_oppSeqTimer); _oppSeqTimer = null; }
  document.getElementById('opp-seq-play')?.classList.remove('active');
}

// Init opponent sequence list on load
_renderOppSeq();

// ── Occupancy / features ───────────────────────────────────────────────────
function setFeature(k, v) { socket.emit('set_feature', {feature:k, value:v}); }

async function saveStaticMap() {
  const btn = document.getElementById('btn-occ-save');
  if (btn) { btn.disabled = true; btn.textContent = '…'; }
  try {
    const r = await fetch('/api/occupancy/static', { method: 'PUT' });
    const j = await r.json();
    if (btn) btn.textContent = j.ok ? '✓ Saved' : '✗ Error';
  } catch(e) { if (btn) btn.textContent = '✗ Error'; }
  setTimeout(() => { if (btn) { btn.disabled = false; btn.innerHTML = '&#128190; Save'; } }, 2000);
}

async function deployStaticMap() {
  const btn = document.getElementById('btn-occ-deploy');
  if (btn) { btn.disabled = true; btn.textContent = '…'; }
  try {
    const r = await fetch('/api/occupancy/deploy', { method: 'POST' });
    const j = await r.json();
    if (btn) btn.textContent = j.ok ? '✓ Sent' : '✗ ' + (j.error || 'Error');
  } catch(e) { if (btn) btn.textContent = '✗ Error'; }
  setTimeout(() => { if (btn) { btn.disabled = false; btn.innerHTML = '&#128225; Deploy'; } }, 2000);
}

function togglePathfinding(on) {
  fetch('/api/pathfinding', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({ enabled: on }) });
}

function toggleMotionMode(on) {
  fetch('/api/motion_mode', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({ mode: on ? 'pursuit' : 'waypoint' }) });
}

function setFeedrate(v) {
  document.getElementById('feedrate-val').textContent = parseFloat(v).toFixed(2) + '×';
  socket.emit('set_feedrate', {value: parseFloat(v)});
}

function toggleRobotImage(c) { showRobotImage = c; if (lastState) render(lastState); }

function clearLog() { document.getElementById('log-output').innerHTML = ''; }
