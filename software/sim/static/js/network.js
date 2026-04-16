// js/network.js — Hardware topology canvas view
// Depends on: socket, activeView, showToast, escHtml,
//   serialConnect, serialDisconnect, connectSim

'use strict';

// ── Runtime state ──────────────────────────────────────────────────────────
const _ns = {
  ws: 'disconnected', hw: 'idle', hwType: 'idle',
  hwLabel: 'Not connected', hwPort: null,
  t40: 'unknown', jetsonIp: '', selected: null, selType: null,
};

// ── Node definitions ───────────────────────────────────────────────────────
const _NN_W = 148, _NN_H = 106, _NN_R = 10;

const _nNodes = [
  { id:'pc',     label:'PC',            icon:'🖥', role:'Browser + holOS Local',    sw:['holOS UI (HTML+JS)','Flask · SocketIO · Python'],       xr:0.12, yr:0.25 },
  { id:'t41',    label:'Teensy 4.1',    icon:'🎛', role:'Main MCU — 600 MHz',       sw:['TwinSystem firmware','PlatformIO · C++17'],               xr:0.50, yr:0.25 },
  { id:'t40',    label:'Teensy 4.0',    icon:'🔧', role:'Secondary MCU',            sw:['TwinActuator firmware','PlatformIO · C++17','Static map (RAM)'], xr:0.88, yr:0.25 },
  { id:'sim',    label:'Simulator',     icon:'🎮', role:'Virtual robot',            sw:['SimBridge · Physics','VirtualTransport'],                 xr:0.12, yr:0.75 },
  { id:'jetson', label:'NVIDIA Jetson', icon:'🤖', role:'Edge computer',            sw:['holOS Server','ROS2 Humble'],                             xr:0.50, yr:0.75, offline:true },
  { id:'espcam', label:'ESP32-CAM',     icon:'📷', role:'Vision module',            sw:['TwinVision firmware','MJPEG stream'],                      xr:0.88, yr:0.75, offline:true },
];

const _nLinks = [
  { id:'hw',       from:'pc',     to:'t41',    proto:'USB-CDC',          baud:57600, configurable:true  },
  { id:'uart',     from:'t41',    to:'t40',    proto:'UART Intercom',    baud:31250, configurable:false },
  { id:'sim',      from:'sim',    to:'pc',     proto:'VirtualTransport', baud:null,  configurable:false },
  { id:'xbee',     from:'jetson', to:'t41',    proto:'XBee 868 MHz',     baud:57600, configurable:false, offline:true },
  { id:'espcam_j', from:'espcam', to:'jetson', proto:'WiFi · MJPEG',     baud:null,  configurable:false, offline:true },
  { id:'espcam_p', from:'espcam', to:'pc',     proto:'WiFi · MJPEG',     baud:null,  configurable:false, offline:true },
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

// ── Geometry helpers ───────────────────────────────────────────────────────
function _ncNodeById(id)  { return _nNodes.find(n => n.id === id); }
function _ncCanvas()      { return document.getElementById('net-canvas'); }
function _ncCol(st)       { return _nsColors[st] || '#94a3b8'; }

function _ncBox(nd, W, H) {
  const cx = W * nd.xr, cy = H * nd.yr;
  return { cx, cy, x: cx - _NN_W/2, y: cy - _NN_H/2, w: _NN_W, h: _NN_H };
}

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

// ── Node/link status ───────────────────────────────────────────────────────
function _nsNodeStatus(nid) {
  if (nid === 'pc')     return _ns.ws === 'connected' ? 'connected' : 'disconnected';
  if (nid === 'sim')    return _ns.hwType === 'sim' ? 'active' : 'disconnected';
  if (nid === 't41') {
    if (_ns.hw === 'connected')  return 'connected';
    if (_ns.hw === 'connecting') return 'connecting';
    return 'disconnected';
  }
  if (nid === 't40')    return _ns.t40 === 'ok' ? 'connected' : (_ns.t40 === 'unknown' ? 'unknown' : 'error');
  if (nid === 'jetson') return _ns.hwType === 'xbee' ? 'connected' : 'offline';
  if (nid === 'espcam') return 'offline';
  return 'unknown';
}

function _nsLinkStatus(lid) {
  if (lid === 'hw')       return _ns.hwType === 'usb' ? 'connected' : (_ns.hwType === 'xbee' ? 'disconnected' : 'disconnected');
  if (lid === 'uart')     return _ns.t40 === 'ok' ? 'connected' : (_ns.t40 === 'unknown' ? 'unknown' : 'disconnected');
  if (lid === 'sim')      return _ns.hwType === 'sim' ? 'active' : 'disconnected';
  if (lid === 'xbee')     return _ns.hwType === 'xbee' ? 'connected' : 'offline';
  if (lid === 'espcam_j') return 'offline';
  if (lid === 'espcam_p') return 'offline';
  return 'unknown';
}

function _nsLinkLabel(lid) {
  const lk = _nLinks.find(l => l.id === lid); if (!lk) return '';
  let proto = lk.proto;
  if (lid === 'hw') proto = _ns.hwType === 'xbee' ? 'XBee 868 MHz' : 'USB-CDC';
  return lk.baud ? `${proto} · ${(lk.baud/1000).toFixed(lk.baud%1000?1:0)}k` : proto;
}

// ── Main draw ──────────────────────────────────────────────────────────────
function _ncDraw() {
  _netAnimFrame = null;
  const c = _ncCanvas();
  if (!c || activeView !== 'network') return;
  const ctx2 = c.getContext('2d');
  const W = c.width, H = c.height;
  if (!W || !H) return;
  ctx2.clearRect(0, 0, W, H);
  ctx2.fillStyle = '#f0f4f8'; ctx2.fillRect(0, 0, W, H);
  ctx2.fillStyle = 'rgba(26,82,118,.05)';
  for (let x = 30; x < W; x += 40)
    for (let y = 30; y < H; y += 40) { ctx2.beginPath(); ctx2.arc(x, y, 1.5, 0, Math.PI*2); ctx2.fill(); }
  _nLinks.forEach(lk => _ncDrawLink(ctx2, lk, W, H));
  _nNodes.forEach(nd => _ncDrawNode(ctx2, nd, W, H));
  if (_ns.hw === 'connecting') _netAnimFrame = requestAnimationFrame(_ncDraw);
}

function _ncDrawNode(ctx2, nd, W, H) {
  const b = _ncBox(nd, W, H);
  const st = _nsNodeStatus(nd.id);
  const col = _ncCol(st);
  const sel = _ns.selected === nd.id && _ns.selType === 'node';
  const dim = !!(nd.offline);
  ctx2.save();
  if (dim) ctx2.globalAlpha = 0.55;
  ctx2.shadowColor = `rgba(26,82,118,${sel?.18:.07})`; ctx2.shadowBlur = sel?14:4; ctx2.shadowOffsetY = 2;
  ctx2.fillStyle = '#fff';
  _ncRR(ctx2, b.x, b.y, b.w, b.h, _NN_R); ctx2.fill();
  ctx2.shadowBlur = 0; ctx2.shadowOffsetY = 0;
  ctx2.strokeStyle = col + (sel?'dd':'44'); ctx2.lineWidth = sel?2.5:1.5;
  _ncRR(ctx2, b.x, b.y, b.w, b.h, _NN_R); ctx2.stroke();
  ctx2.fillStyle = col + '22';
  ctx2.beginPath();
  ctx2.moveTo(b.x+_NN_R, b.y); ctx2.lineTo(b.x+b.w-_NN_R, b.y);
  ctx2.quadraticCurveTo(b.x+b.w, b.y, b.x+b.w, b.y+_NN_R);
  ctx2.lineTo(b.x+b.w, b.y+22); ctx2.lineTo(b.x, b.y+22); ctx2.lineTo(b.x, b.y+_NN_R);
  ctx2.quadraticCurveTo(b.x, b.y, b.x+_NN_R, b.y); ctx2.closePath(); ctx2.fill();
  ctx2.save();
  if (st === 'connecting') ctx2.globalAlpha = .5 + .5*Math.sin(Date.now()/400);
  ctx2.fillStyle = col; ctx2.beginPath(); ctx2.arc(b.x+b.w-10, b.y+11, 4, 0, Math.PI*2); ctx2.fill();
  ctx2.restore();
  ctx2.font = '18px serif'; ctx2.textAlign = 'center'; ctx2.textBaseline = 'middle'; ctx2.fillText(nd.icon, b.x+18, b.y+11);
  ctx2.font = 'bold 11.5px "Segoe UI",system-ui,sans-serif'; ctx2.fillStyle = '#1a2332'; ctx2.textAlign = 'center'; ctx2.textBaseline = 'top';
  ctx2.fillText(nd.label, b.cx, b.y+26);
  ctx2.font = '10px "Segoe UI",system-ui,sans-serif'; ctx2.fillStyle = '#6b7a8d';
  ctx2.fillText(nd.role, b.cx, b.y+41);
  ctx2.font = '9px "Segoe UI",system-ui,sans-serif'; ctx2.fillStyle = '#94a3b8';
  (nd.sw||[]).forEach((line, i) => ctx2.fillText(line, b.cx, b.y+56+i*13));
  ctx2.restore();
}

function _ncDrawLink(ctx2, lk, W, H) {
  const n1 = _ncNodeById(lk.from), n2 = _ncNodeById(lk.to);
  if (!n1 || !n2) return;
  const b1 = _ncBox(n1, W, H), b2 = _ncBox(n2, W, H);
  const st = _nsLinkStatus(lk.id), col = _ncCol(st);
  const sel = _ns.selected === lk.id && _ns.selType === 'link';
  const ang12 = Math.atan2(b2.cy-b1.cy, b2.cx-b1.cx);
  const ang21 = ang12 + Math.PI;
  const p1 = _ncEdgePt(b1, ang12), p2 = _ncEdgePt(b2, ang21);
  ctx2.save();
  ctx2.strokeStyle = col; ctx2.lineWidth = sel ? 3 : 2; ctx2.lineCap = 'round';
  if (st === 'offline' || st === 'disconnected' || st === 'unknown') {
    ctx2.setLineDash([5,6]); ctx2.globalAlpha = .55;
  } else if (st === 'connecting') {
    ctx2.setLineDash([5,6]); ctx2.lineDashOffset = -((Date.now()/80)%11);
    ctx2.globalAlpha = .55+.45*Math.sin(Date.now()/250);
  }
  ctx2.beginPath(); ctx2.moveTo(p1.x, p1.y); ctx2.lineTo(p2.x, p2.y); ctx2.stroke();
  ctx2.restore();
  if (st !== 'offline' && st !== 'disconnected' && st !== 'unknown') {
    const AW = 7; ctx2.fillStyle = col;
    ctx2.beginPath(); ctx2.moveTo(p2.x, p2.y);
    ctx2.lineTo(p2.x-AW*Math.cos(ang12-Math.PI/6), p2.y-AW*Math.sin(ang12-Math.PI/6));
    ctx2.lineTo(p2.x-AW*Math.cos(ang12+Math.PI/6), p2.y-AW*Math.sin(ang12+Math.PI/6));
    ctx2.closePath(); ctx2.fill();
  }
  const mx = (p1.x+p2.x)/2, my = (p1.y+p2.y)/2 - (lk.offline?0:18);
  const lbl = _nsLinkLabel(lk.id);
  ctx2.save();
  ctx2.globalAlpha = (st === 'offline' || st === 'unknown') ? 0.5 : 1;
  ctx2.font = '9.5px "Segoe UI",system-ui,sans-serif';
  const PW = ctx2.measureText(lbl).width + 14, PH = 17;
  ctx2.fillStyle = sel ? col : '#fff'; ctx2.strokeStyle = col + (st === 'offline' ? '66' : '99'); ctx2.lineWidth = 1;
  _ncRR(ctx2, mx-PW/2, my-PH/2, PW, PH, 8); ctx2.fill(); ctx2.stroke();
  ctx2.fillStyle = sel ? '#fff' : col; ctx2.textAlign = 'center'; ctx2.textBaseline = 'middle';
  ctx2.fillText(lbl, mx, my);
  ctx2.restore();
  lk._hit = { p1, p2 };
}

function _ncRR(ctx2, x, y, w, h, r) {
  ctx2.beginPath();
  ctx2.moveTo(x+r,y); ctx2.lineTo(x+w-r,y); ctx2.quadraticCurveTo(x+w,y,x+w,y+r);
  ctx2.lineTo(x+w,y+h-r); ctx2.quadraticCurveTo(x+w,y+h,x+w-r,y+h);
  ctx2.lineTo(x+r,y+h); ctx2.quadraticCurveTo(x,y+h,x,y+h-r);
  ctx2.lineTo(x,y+r); ctx2.quadraticCurveTo(x,y,x+r,y); ctx2.closePath();
}

function _ncHit(mx, my) {
  const c = _ncCanvas(); if (!c) return null;
  const W = c.width, H = c.height;
  for (const nd of _nNodes) {
    const b = _ncBox(nd, W, H);
    if (mx >= b.x && mx <= b.x+b.w && my >= b.y && my <= b.y+b.h) return {type:'node', id:nd.id};
  }
  for (const lk of _nLinks) {
    if (!lk._hit) continue;
    const {p1, p2} = lk._hit;
    const dx = p2.x-p1.x, dy = p2.y-p1.y, lq = dx*dx+dy*dy;
    const t = lq ? Math.max(0, Math.min(1, ((mx-p1.x)*dx+(my-p1.y)*dy)/lq)) : 0;
    if (Math.hypot(mx-(p1.x+t*dx), my-(p1.y+t*dy)) <= 14) return {type:'link', id:lk.id};
  }
  return null;
}

function _ncResize() {
  const c = _ncCanvas(), area = document.getElementById('net-canvas-area');
  if (!c || !area) return;
  c.width = area.clientWidth; c.height = area.clientHeight;
  _ncDraw();
}

function initNetwork() {
  const c = _ncCanvas(); if (!c) return;
  if (!c._netWired) {
    c._netWired = true;
    c.addEventListener('click', evt => {
      const r = c.getBoundingClientRect();
      const h = _ncHit(evt.clientX-r.left, evt.clientY-r.top);
      _ns.selected = h ? h.id : null; _ns.selType = h ? h.type : null;
      _ncUpdatePanel(h); _ncDraw();
    });
    c.addEventListener('mousemove', evt => {
      const r = c.getBoundingClientRect();
      c.style.cursor = _ncHit(evt.clientX-r.left, evt.clientY-r.top) ? 'pointer' : 'default';
    });
    new ResizeObserver(() => { if (activeView === 'network') _ncResize(); })
      .observe(document.getElementById('net-canvas-area'));
  }
  _ncResize(); _ncUpdatePanel(null);
}

// ── Client node label (PC vs Jetson) ──────────────────────────────────────
function _ncSetClientLabel(isJetson) {
  const nd = _ncNodeById('pc');
  if (!nd) return;
  nd.label = isJetson ? 'Jetson' : 'PC';
  nd.icon  = isJetson ? '🤖' : '🖥';
  nd.role  = isJetson ? 'holOS Server · Edge computer' : 'Browser + holOS Local';
  if (activeView === 'network') _ncDraw();
}

// ── State update from socket ───────────────────────────────────────────────
function updateNetworkFromState(state) {
  const mode = state.connection_mode || 'idle';
  const newHw    = state.hw_connecting ? 'connecting' :
                   (mode==='usb'||mode==='xbee') ? 'connected' :
                   mode==='sim' ? 'sim' : 'idle';
  const newType  = mode;
  const newLabel = mode==='sim'  ? 'Virtual (SIM)' :
                   mode==='xbee' ? 'XBee 868 MHz'  :
                   mode==='usb'  ? 'USB Wired'      : 'Not connected';
  const newPort  = state.hw_serial_port || null;
  const newT40   = state.hw_t40 || 'unknown';
  const newJetIp = state.jetson_ip || '';
  const changed  = _ns.hw!==newHw || _ns.hwType!==newType || _ns.t40!==newT40 || _ns.hwPort!==newPort;
  _ns.hw=newHw; _ns.hwType=newType; _ns.hwLabel=newLabel;
  _ns.hwPort=newPort; _ns.t40=newT40; _ns.jetsonIp=newJetIp;
  if (!changed) return;
  const badge  = document.getElementById('np-hw-badge');
  const modeEl = document.getElementById('np-hw-mode');
  const btn    = document.getElementById('np-connect-btn');
  if (badge)  { badge.textContent = _nsLabels[_ns.hw] || _ns.hw; badge.className = 'np-badge np-' + _ns.hw; }
  if (modeEl) modeEl.textContent = _ns.hwPort ? `${_ns.hwLabel} (${_ns.hwPort})` : _ns.hwLabel;
  if (btn)    btn.disabled = _ns.hw === 'connecting';
  if (activeView === 'network') {
    _ncDraw();
    if (_ns.hw === 'connecting' && !_netAnimFrame) _netAnimFrame = requestAnimationFrame(_ncDraw);
  }
}

// ── Side panel ─────────────────────────────────────────────────────────────
function _ncUpdatePanel(hit) {
  const title = document.getElementById('net-panel-title');
  const body  = document.getElementById('net-panel-body');
  if (!title || !body) return;
  if (!hit) { title.textContent = 'Network'; body.innerHTML = _ncPanelOverview(); return; }
  if (hit.type === 'link') {
    const clientLbl = (_ncNodeById('pc') || {}).label || 'PC';
    const titles = {hw:`${clientLbl} ↔ Teensy 4.1`, uart:'T4.1 ↔ T4.0 Intercom', sim:`Simulator ↔ ${clientLbl}`,
                    xbee:'Jetson ↔ T4.1 XBee', espcam_j:'ESP32-CAM ↔ Jetson', espcam_p:`ESP32-CAM ↔ ${clientLbl}`};
    title.textContent = titles[hit.id] || hit.id;
    if (hit.id === 'hw')           { body.innerHTML = _ncPanelHW(); _ncPanelHWInit(); }
    else if (hit.id === 'sim')     body.innerHTML = _ncPanelSimLink();
    else if (hit.id === 'uart')    body.innerHTML = _ncPanelUART();
    else if (hit.id === 'xbee')    body.innerHTML = _ncPanelXBeeLink();
    else if (hit.id === 'espcam_j' || hit.id === 'espcam_p') body.innerHTML = _ncPanelEspCamLink();
  } else {
    const nd = _ncNodeById(hit.id) || {id:hit.id, icon:'?', label:hit.id, sw:[]};
    title.textContent = nd.label;
    if (hit.id === 'jetson') body.innerHTML = _ncPanelJetson();
    else                     body.innerHTML = _ncPanelNode(nd);
  }
}

function _ncPanelOverview() {
  const hwPortStr = _ns.hwPort ? ` (${_ns.hwPort})` : '';
  const simSt = _ns.hwType === 'sim' ? 'active' : 'disconnected';
  const jetSt = _ns.hwType === 'xbee' ? 'connected' : 'offline';
  const t40St = _ns.t40 === 'ok' ? 'connected' : _ns.t40;
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
      <div style="display:flex;gap:6px;margin-top:4px">
        <button id="np-connect-btn" class="btn green btn-sm" style="flex:1" onclick="_ncDoConnect()">Connect HW</button>
        <button class="btn red btn-sm" style="flex:1" onclick="serialDisconnect()">Disconnect</button>
      </div>
      <div style="margin-top:8px;padding-top:8px;border-top:1px solid rgba(255,255,255,.1)">
        <button class="btn btn-sm" style="width:100%;background:#6366f1;color:#fff" onclick="connectSim()">▶ Start Simulator</button>
      </div>
    </div>
    <div class="np-hint">Select a COM port and click <b>Connect HW</b>. Firmware auto-detects USB-CDC or XBee (57 600 bps).</div>`;
}

function _ncPanelHWInit() {
  _ncRefreshPorts();
  const btn = document.getElementById('np-connect-btn');
  if (btn) btn.disabled = _ns.hw === 'connecting';
}

function _ncRefreshPorts() {
  const sel = document.getElementById('np-port-sel'); if (!sel) return;
  const cur = (document.getElementById('serial-port-sel') || {}).value || '';
  fetch('/api/serial/ports').then(r => r.json()).then(ports => {
    sel.innerHTML = '<option value="">— port —</option>' +
      ports.map(p => `<option value="${escHtml(p.port)}"${p.port===cur?' selected':''}>${escHtml(p.port)} — ${escHtml(p.desc)}</option>`).join('');
  });
}

function _ncDoConnect() {
  const ps = document.getElementById('np-port-sel');
  if (!ps?.value) { showToast('Select a serial port first'); return; }
  const mp = document.getElementById('serial-port-sel');
  if (mp) mp.value = ps.value;
  serialConnect();
}

function _ncPanelUART() {
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${_ns.t40}">${_nsLabels[_ns.t40]||_ns.t40}</span></div>
      <div class="np-row"><span>Protocol</span><span>UART (Intercom)</span></div>
      <div class="np-row"><span>Baudrate</span><span>31 250 bps</span></div>
    </div>`;
}

function _ncPanelSimLink() {
  const active = _ns.hwType === 'sim';
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${active?'active':'disconnected'}">${active?'Active':'Inactive'}</span></div>
      <div class="np-row"><span>Type</span><span>In-process VirtualTransport</span></div>
    </div>
    <div class="np-section">
      <button class="btn btn-sm" style="width:100%;background:#6366f1;color:#fff" onclick="connectSim()">▶ Start Simulator</button>
    </div>`;
}

function _ncPanelXBeeLink() {
  return `<div class="np-section">
      <div class="np-row"><span>Protocol</span><span>XBee 868 MHz UART</span></div>
      <div class="np-row"><span>Baudrate</span><span>57 600 bps</span></div>
    </div>`;
}

function _ncPanelEspCamLink() {
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-offline">Offline</span></div>
      <div class="np-row"><span>Protocol</span><span>Serial2 · WiFi MJPEG</span></div>
    </div>`;
}

function _ncPanelJetson() {
  const ip = _ns.jetsonIp || '';
  return `<div class="np-section">
      <div class="np-row"><span>Platform</span><span>NVIDIA Jetson</span></div>
    </div>
    <div class="np-section">
      <label class="np-label">Jetson IP address</label>
      <div style="display:flex;gap:6px;margin-bottom:8px">
        <input id="np-jetson-ip" type="text" class="input-sm" style="flex:1" placeholder="192.168.x.x" value="${escHtml(ip)}">
        <button class="btn-tiny green" onclick="_ncSaveJetsonIp()">Save</button>
      </div>
    </div>`;
}

function _ncSaveJetsonIp() {
  const ip   = (document.getElementById('np-jetson-ip')   || {}).value || '';
  const user = (document.getElementById('np-jetson-user') || {}).value || 'robot';
  fetch('/api/jetson', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ip, user})
  }).then(() => { _ns.jetsonIp = ip; showToast('Jetson config saved'); });
}

function _ncPanelNode(nd) {
  const st = _nsNodeStatus(nd.id);
  const swHtml = (nd.sw||[]).map(s => `<div class="np-row"><span style="opacity:.6">SW</span><span>${escHtml(s)}</span></div>`).join('');
  return `<div class="np-section">
      <div class="np-row"><span>Status</span><span class="np-badge np-${st}">${_nsLabels[st]||st}</span></div>
      <div class="np-row"><span>Role</span><span>${escHtml(nd.role||'')}</span></div>
      ${swHtml}
    </div>`;
}
