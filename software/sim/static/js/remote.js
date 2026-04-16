// js/remote.js — Remote control drawer: heading dial, polar direction buttons
// Depends on: socket, activeView, lastState, switchView, ROBOT_IMG_OFFSET

'use strict';

// ── State ──────────────────────────────────────────────────────────────────────
let _remoteDrawerOpen = false;
let _remoteCurDeg     = 0;
let _remoteTgtDeg     = 0;
let _remoteSnapDeg    = 5;
let _remoteDragging   = false;
let _remoteMotionBusy = false;
let _remoteDialReady  = false;
let _remoteFrame      = 'table';

// ── Direction label maps ───────────────────────────────────────────────────────
const _REMOTE_TABLE_LABELS   = { 0:'E', 30:'ENE', 60:'NNE', 90:'N', 120:'NNW', 150:'WNW', 180:'W', 210:'WSW', 240:'SSW', 270:'S', 300:'SSE', 330:'ESE' };
const _REMOTE_TABLE_CARDINAL = { 90:'N', 0:'E', 180:'W', 270:'S' };
const _REMOTE_ROBOT_LABELS   = { 0:'A', 60:'AB', 120:'B', 180:'BC', 240:'C', 300:'CA' };
const _REMOTE_ROBOT_CARDINAL = { 0:'A', 60:'AB', 120:'B', 180:'BC', 240:'C', 300:'CA' };

// ── Dial constants ─────────────────────────────────────────────────────────────
const R_OUTER  = 145;
const R_TICK_M = 14;
const R_TICK_m = 7;
const R_LABELS = 113;
const R_ARC    = 100;
const R_CURIND = 110;
const R_TGTIND = 128;
const ROBOT_R  = 36;

// ── Geometry helpers ───────────────────────────────────────────────────────────
function _deg2xy(r, deg) {
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

// ── Drawer toggle ──────────────────────────────────────────────────────────────
function toggleRemoteDrawer(navBtn) {
  const drawer   = document.getElementById('remote-drawer');
  const navBtnEl = navBtn ?? document.getElementById('nav-remote');
  if (!drawer) return;

  _remoteDrawerOpen = !_remoteDrawerOpen;
  drawer.classList.toggle('closed', !_remoteDrawerOpen);
  navBtnEl?.classList.toggle('drawer-open', _remoteDrawerOpen);

  if (_remoteDrawerOpen) {
    if (activeView !== 'map') {
      const mapBtn = document.querySelector('.nav-btn[data-view="map"]');
      switchView('map', mapBtn);
    }
    _remoteInitDial();
    if (lastState?.robot) remoteUpdateTheta(lastState.robot.theta * 180 / Math.PI);
    _remoteUpdateDial();
  }
}

// ── Polar buttons ──────────────────────────────────────────────────────────────
function _remoteRebuildPolarButtons() {
  const POLAR_CX = 220, POLAR_CY = 220, POLAR_R = 185;
  const polarContainer = document.getElementById('remote-polar-btns');
  if (!polarContainer) return;
  polarContainer.innerHTML = '';

  const isRobot = _remoteFrame === 'robot';
  const labels   = isRobot ? _REMOTE_ROBOT_LABELS   : _REMOTE_TABLE_LABELS;
  const cardinal = isRobot ? _REMOTE_ROBOT_CARDINAL  : _REMOTE_TABLE_CARDINAL;
  const step     = isRobot ? 60 : 30;
  const count    = isRobot ? 6  : 12;

  for (let i = 0; i < count; i++) {
    const deg = i * step;
    const rad = deg * Math.PI / 180;
    const cx  = POLAR_CX + POLAR_R * Math.cos(rad);
    const cy  = POLAR_CY - POLAR_R * Math.sin(rad);
    const label  = labels[deg] ?? deg + '\u00B0';
    const isCard = cardinal[deg] !== undefined;
    const btn = document.createElement('button');
    btn.className = 'remote-polar-btn' + (isCard ? ' cardinal' : '');
    btn.style.left = cx.toFixed(1) + 'px';
    btn.style.top  = cy.toFixed(1) + 'px';
    btn.style.pointerEvents = 'auto';
    btn.textContent = label;
    btn.title = isRobot ? `Move toward face ${label}` : `Move ${label} (${deg}\u00B0)`;
    btn.addEventListener('click', () => remoteGoPolar(deg));
    polarContainer.appendChild(btn);
  }
}

function remoteFrameChanged() {
  _remoteFrame = document.getElementById('remote-frame-sel')?.value ?? 'table';
  _remoteRebuildPolarButtons();
}

// ── Dial init ──────────────────────────────────────────────────────────────────
function _remoteInitDial() {
  if (_remoteDialReady) return;
  _remoteDialReady = true;

  const svg = document.getElementById('remote-dial');
  if (!svg) return;

  const g = document.getElementById('dial-ticks-layer');

  const ring = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
  ring.setAttribute('cx', 0); ring.setAttribute('cy', 0); ring.setAttribute('r', R_OUTER);
  ring.setAttribute('fill', 'none'); ring.setAttribute('stroke', 'var(--surface3)'); ring.setAttribute('stroke-width', '1.5');
  g.appendChild(ring);

  const ring2 = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
  ring2.setAttribute('cx', 0); ring2.setAttribute('cy', 0); ring2.setAttribute('r', R_ARC + 12);
  ring2.setAttribute('fill', 'none'); ring2.setAttribute('stroke', 'var(--surface3)');
  ring2.setAttribute('stroke-width', '0.8'); ring2.setAttribute('stroke-dasharray', '2 4');
  g.appendChild(ring2);

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
      const lp  = _deg2xy(R_LABELS, deg);
      const lbl = document.createElementNS('http://www.w3.org/2000/svg', 'text');
      lbl.setAttribute('x', lp.x.toFixed(2)); lbl.setAttribute('y', lp.y.toFixed(2));
      lbl.setAttribute('text-anchor', 'middle'); lbl.setAttribute('dominant-baseline', 'middle');
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

  const cdot = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
  cdot.setAttribute('cx', 0); cdot.setAttribute('cy', 0); cdot.setAttribute('r', 4);
  cdot.setAttribute('fill', 'var(--brand)');
  g.appendChild(cdot);

  const curLine = document.getElementById('dial-cur-line');
  if (curLine) curLine.setAttribute('stroke', 'var(--text-dim)');
  const tgtLine = document.getElementById('dial-tgt-line');
  if (tgtLine) tgtLine.setAttribute('stroke', 'var(--brand)');
  const tgtDot = document.getElementById('dial-tgt-dot');
  if (tgtDot) { tgtDot.setAttribute('stroke', 'var(--brand)'); tgtDot.setAttribute('fill', 'var(--surface1)'); }
  const tgtLabel = document.getElementById('dial-tgt-label');
  if (tgtLabel) { tgtLabel.setAttribute('fill', 'var(--brand)'); tgtLabel.setAttribute('font-family', 'monospace'); }

  svg.addEventListener('mousedown',  _remoteDialMouseDown);
  svg.addEventListener('mousemove',  _remoteDialMouseMove);
  svg.addEventListener('mouseup',    _remoteDialMouseUp);
  svg.addEventListener('mouseleave', _remoteDialMouseUp);
  svg.addEventListener('touchstart', e => { e.preventDefault(); _remoteDialMouseDown(e.touches[0]); }, { passive: false });
  svg.addEventListener('touchmove',  e => { e.preventDefault(); _remoteDialMouseMove(e.touches[0]); }, { passive: false });
  svg.addEventListener('touchend',   () => { _remoteDialMouseUp(); }, { passive: false });

  _remoteRebuildPolarButtons();
  _remoteUpdateDial();
}

// ── Dial render ────────────────────────────────────────────────────────────────
function _remoteUpdateDial() {
  const cur   = _normDeg(_remoteCurDeg);
  const tgt   = _normDeg(_remoteTgtDeg);
  const delta = _shortDelta(cur, tgt);

  const robotImg  = document.getElementById('dial-robot-img');
  const imgOffDeg = ROBOT_IMG_OFFSET * 180 / Math.PI;
  if (robotImg) robotImg.style.transform = `translate(-50%, -50%) rotate(${(-cur + imgOffDeg).toFixed(2)}deg)`;

  const curLine = document.getElementById('dial-cur-line');
  if (curLine) {
    const p = _deg2xy(R_CURIND, cur);
    curLine.setAttribute('x2', p.x.toFixed(2)); curLine.setAttribute('y2', p.y.toFixed(2));
  }

  const tgtLine = document.getElementById('dial-tgt-line');
  const tgtDot  = document.getElementById('dial-tgt-dot');
  const tgtLbl  = document.getElementById('dial-tgt-label');
  const tp = _deg2xy(R_TGTIND, tgt);
  if (tgtLine) { tgtLine.setAttribute('x2', tp.x.toFixed(2)); tgtLine.setAttribute('y2', tp.y.toFixed(2)); }
  if (tgtDot)  { tgtDot.setAttribute('cx', tp.x.toFixed(2));  tgtDot.setAttribute('cy', tp.y.toFixed(2)); }
  if (tgtLbl) {
    const lp = _deg2xy(R_TGTIND + 18, tgt);
    tgtLbl.setAttribute('x', lp.x.toFixed(2)); tgtLbl.setAttribute('y', lp.y.toFixed(2));
    tgtLbl.textContent = tgt.toFixed(0) + '°';
  }

  const arc = document.getElementById('dial-delta-arc');
  if (arc) {
    arc.setAttribute('stroke', delta >= 0 ? '#3b82f6' : '#f59e0b');
    if (Math.abs(delta) < 0.5) {
      arc.setAttribute('d', '');
    } else {
      const p1 = _deg2xy(R_ARC, cur);
      const p2 = _deg2xy(R_ARC, tgt);
      const sweep = delta >= 0 ? 0 : 1;
      const large = Math.abs(delta) > 180 ? 1 : 0;
      arc.setAttribute('d',
        `M ${p1.x.toFixed(2)} ${p1.y.toFixed(2)} ` +
        `A ${R_ARC} ${R_ARC} 0 ${large} ${sweep} ${p2.x.toFixed(2)} ${p2.y.toFixed(2)}`);
    }
  }

  const sign = delta >= 0 ? '+' : '';
  document.getElementById('rr-cur')   && (document.getElementById('rr-cur').textContent   = cur.toFixed(1) + '°');
  document.getElementById('rr-tgt')   && (document.getElementById('rr-tgt').textContent   = tgt.toFixed(1) + '°');
  document.getElementById('rr-delta') && (document.getElementById('rr-delta').textContent = sign + delta.toFixed(1) + '°');

  const tgtInput = document.getElementById('remote-tgt-input');
  if (tgtInput && document.activeElement !== tgtInput) tgtInput.value = Math.round(tgt);

  const go = document.getElementById('remote-go-btn');
  if (go) {
    const sign2 = delta >= 0 ? '+' : '';
    go.textContent = `▶  TURN  ${sign2}${delta.toFixed(1)}°`;
    go.disabled    = _remoteMotionBusy;
    go.className   = 'remote-go-btn' + (_remoteMotionBusy ? ' busy' : '');
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
  let angle  = Math.atan2(-dy, dx) * 180 / Math.PI;
  if (angle < 0) angle += 360;
  return angle;
}

function _remoteDialMouseDown(e) { _remoteDragging = true;  remoteSetTarget(_remoteAngleFromEvent(e)); }
function _remoteDialMouseMove(e) { if (!_remoteDragging) return; remoteSetTarget(_remoteAngleFromEvent(e)); }
function _remoteDialMouseUp()    { _remoteDragging = false; }

// ── Public API ─────────────────────────────────────────────────────────────────
function remoteSetTarget(deg) {
  _remoteTgtDeg = _normDeg(Math.round(deg / _remoteSnapDeg) * _remoteSnapDeg);
  _remoteUpdateDial();
}

function remoteDelta(deltaDeg) {
  if (deltaDeg === 0) {
    _remoteTgtDeg = _normDeg(_remoteCurDeg);
  } else {
    _remoteTgtDeg = _normDeg(_remoteTgtDeg + deltaDeg);
  }
  _remoteUpdateDial();
}

function remoteSnapChanged() {
  _remoteSnapDeg = +(document.getElementById('remote-snap-sel')?.value ?? 5);
  remoteSetTarget(_remoteTgtDeg);
}

function remoteUpdateTheta(thetaDeg) {
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
    const res  = await fetch('/api/exec', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: `turn(${tgt.toFixed(2)})` }),
    });
    const data = await res.json();
    if (status) {
      if (data.ok) {
        const sign = delta >= 0 ? '+' : '';
        status.textContent = `✓ Done — turn → ${tgt.toFixed(1)}° (Δ${sign}${delta.toFixed(1)}°)`;
        status.className   = 'remote-status-ok';
      } else {
        status.textContent = `✗ ${data.res ?? 'error'}`;
        status.className   = 'remote-status-err';
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
  const dist    = parseInt(document.getElementById('remote-dist-sel')?.value ?? 200);
  const isRobot = _remoteFrame === 'robot';
  const relAngle = isRobot
    ? bearingDeg
    : _shortDelta(_remoteCurDeg, _normDeg(bearingDeg));

  _remoteMotionBusy = true;
  _remoteUpdateDial();

  const status = document.getElementById('remote-status');
  const labels = isRobot ? _REMOTE_ROBOT_LABELS : _REMOTE_TABLE_LABELS;
  const lbl      = labels[bearingDeg] ?? bearingDeg + '\u00B0';
  const frameTag = isRobot ? '[robot]' : '[table]';
  if (status) { status.textContent = `${frameTag} goPolar(${relAngle.toFixed(1)}°, ${dist}mm) → ${lbl}…`; status.className = 'remote-status-busy'; }

  try {
    const res  = await fetch('/api/exec', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: `goPolar(${relAngle.toFixed(2)},${dist})` }),
    });
    const data = await res.json();
    if (status) {
      if (data.ok) {
        status.textContent = `✓ ${frameTag} ${dist}mm → ${lbl}`;
        status.className   = 'remote-status-ok';
      } else {
        status.textContent = `✗ ${data.res ?? 'error'}`;
        status.className   = 'remote-status-err';
      }
    }
  } catch (e) {
    if (status) { status.textContent = `✗ Request failed`; status.className = 'remote-status-err'; }
  }

  _remoteMotionBusy = false;
  _remoteUpdateDial();
}
