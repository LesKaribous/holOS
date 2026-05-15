// js/terminal.js — Terminal I/O + Serial connectivity
// Depends on: socket, showToast, escHtml, cgSetRobotConnecting,
//   cgSetConnectionMode, loadCppBlocks

'use strict';

// ── Terminal ───────────────────────────────────────────────────────────────
let _termHistory = [], _termHistIdx = -1;

function appendTermLine(cmd, ok, res, fire, mode) {
  const out = document.getElementById('terminal-output'); if (!out) return;
  const d = document.createElement('div'); d.className = 'term-line';
  const rc = fire ? 'term-fire' : (ok ? 'term-ok' : 'term-fail');
  const modeHtml = mode
    ? `<span class="${mode === '[HW]' ? 'term-mode-hw' : 'term-mode-sim'}">${escHtml(mode)}</span>`
    : '';
  d.innerHTML = `${modeHtml}<span class="term-cmd">&gt; ${escHtml(cmd)}</span>` +
    (res ? `<span class="${rc}">${escHtml(res)}</span>` : '');
  out.appendChild(d); out.scrollTop = out.scrollHeight;
}

function terminalSend(fire) {
  const inp = document.getElementById('terminal-input'); if (!inp) return;
  const cmd = inp.value.trim(); if (!cmd) return;
  _termHistory.unshift(cmd); if (_termHistory.length > 60) _termHistory.pop();
  _termHistIdx = -1; inp.value = '';
  if (fire) { socket.emit('actuator_fire', {cmd}); appendTermLine(cmd, true, '(fired)', true); }
  else socket.emit('terminal_cmd', {cmd, timeout_ms: 5000});
}

function onTerminalKey(e) {
  if      (e.key === 'Enter')     terminalSend(false);
  else if (e.key === 'ArrowUp')   { e.preventDefault(); if (_termHistIdx < _termHistory.length - 1) document.getElementById('terminal-input').value = _termHistory[++_termHistIdx]; }
  else if (e.key === 'ArrowDown') { e.preventDefault(); if (_termHistIdx > 0) document.getElementById('terminal-input').value = _termHistory[--_termHistIdx]; else { _termHistIdx = -1; document.getElementById('terminal-input').value = ''; } }
}

function clearTerminal() { const o = document.getElementById('terminal-output'); if (o) o.innerHTML = ''; }

socket.on('terminal_rx', d => appendTermLine(d.cmd, d.ok, d.res, d._fire || false, d.mode));

// ── UART Raw view ─────────────────────────────────────────────────────────
function appendUartLine(dir, text) {
  const out = document.getElementById('uart-output'); if (!out) return;
  const now = new Date();
  const ts = now.toTimeString().slice(0, 8) + '.' + String(now.getMilliseconds()).padStart(3, '0');
  const d = document.createElement('div'); d.className = 'uart-line';
  const arrow = dir === 'rx' ? '←' : dir === 'tx' ? '→' : dir === 'dbg' ? '#' : '·';
  const dirCls = 'uart-dir-' + dir;
  const dataCls = 'uart-data' + (dir === 'tx' ? ' tx' : dir === 'sys' ? ' sys' : dir === 'dbg' ? ' dbg' : '');
  d.innerHTML = `<span class="${dirCls}">${arrow}</span>` +
                `<span class="uart-ts">${escHtml(ts)}</span>` +
                `<span class="${dataCls}">${escHtml(text)}</span>`;
  out.appendChild(d);
  // Auto-scroll only when already near the bottom
  if (out.scrollHeight - out.scrollTop - out.clientHeight < 100) out.scrollTop = out.scrollHeight;
}

function clearUart() { const o = document.getElementById('uart-output'); if (o) o.innerHTML = ''; }

function uartRawSend() {
  const inp = document.getElementById('uart-input'); if (!inp) return;
  const text = inp.value; if (!text) return;
  inp.value = '';
  socket.emit('uart_raw_tx', { text });
  // TX echo is handled server-side via _raw_tx subscriber
}

function onUartKey(e) { if (e.key === 'Enter') uartRawSend(); }

socket.on('uart_raw', d => appendUartLine(d.dir, d.line));

// ── Serial connectivity ────────────────────────────────────────────────────
function serialRefreshPorts() {
  fetch('/api/serial/ports')
    .then(r => { if (!r.ok) throw new Error(`HTTP ${r.status}`); return r.json(); })
    .then(ports => {
      ['serial-port-sel','t-port-sel'].forEach(id => {
        const sel = document.getElementById(id); if (!sel) return;
        const cur = sel.value;
        sel.innerHTML = '<option value="">— Select port —</option>' +
          ports.map(p => `<option value="${escHtml(p.port)}"${p.port === cur ? ' selected' : ''}>${escHtml(p.port)} — ${escHtml(p.desc)}</option>`).join('');
      });
    })
    .catch(err => console.error('[serial] port list fetch failed:', err));
}

// Persisted default fetched from the server (shared.config.BRIDGE_KIND).
// `null` until /api/bridge/default resolves, then 'xbee' or 'wifi'.
let _bridgeDefault = null;

function _bridgeApplyDefault(kind) {
  if (!kind) return;
  ['bridge-kind-sel','t-bridge-kind-sel'].forEach(id => {
    const sel = document.getElementById(id); if (sel) sel.value = kind;
  });
  // Re-run the help-blurb visibility logic.
  if (typeof onBridgeKindChange === 'function') onBridgeKindChange();
}

// Fired once at startup. Quietly fails on dev builds without the endpoint —
// the UI keeps whatever default is in the HTML.
function bridgeInitDefault() {
  fetch('/api/bridge/default')
    .then(r => r.ok ? r.json() : null)
    .then(d => { if (d && d.kind) { _bridgeDefault = d.kind; _bridgeApplyDefault(d.kind); } })
    .catch(() => {});
}

document.addEventListener('DOMContentLoaded', bridgeInitDefault);

// Keep both panels' bridge-kind selectors in sync, fetch its current value.
function _bridgeKind() {
  const a = document.getElementById('bridge-kind-sel');
  const b = document.getElementById('t-bridge-kind-sel');
  return (a && a.value) || (b && b.value) || _bridgeDefault || 'xbee';
}

function onBridgeKindChange() {
  const k = _bridgeKind();
  // Mirror the change to whichever selector wasn't the source.
  ['bridge-kind-sel','t-bridge-kind-sel'].forEach(id => {
    const sel = document.getElementById(id); if (sel) sel.value = k;
  });
  // Show the matching help blurb (sidebar only — terminal panel is compact).
  const helpX = document.getElementById('bridge-help-xbee');
  const helpW = document.getElementById('bridge-help-wifi');
  if (helpX) helpX.style.display = (k === 'xbee') ? '' : 'none';
  if (helpW) helpW.style.display = (k === 'wifi') ? '' : 'none';
}

function serialConnect() {
  const bridge = _bridgeKind();
  const port = (document.getElementById('serial-port-sel') || document.getElementById('t-port-sel'))?.value;
  // For WiFi, an empty port is fine — the server falls back to XIAO_HOST/PORT.
  if (!port && bridge !== 'wifi') { showToast('Select a serial port first'); return; }
  cgSetRobotConnecting();
  setSerialStatus('Connecting…', '');
  socket.emit('serial_connect', {port, bridge});
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
  _setSimBridgeStatus('running');
}

// ── Simulator-node panel (in the connectivity graph) ──────────────────────
// `Start simulator` here means: spin up the in-process VirtualTransport so
// the brain has someone to talk to. This is distinct from the topbar
// `▶ Start` (which runs the match strategy).
function simBridgeStart() {
  connectSim();
}

function simBridgeStop() {
  // The existing serialDisconnect handles both HW + sim disconnects on the
  // server side (it falls back to idle when no transport is open).
  if (typeof serialDisconnect === 'function') serialDisconnect();
  _setSimBridgeStatus('idle');
}

function _setSimBridgeStatus(state) {
  const el = document.getElementById('sim-bridge-status');
  if (!el) return;
  if (state === 'running') {
    el.textContent = 'running';
    el.style.color = 'var(--green)';
  } else {
    el.textContent = 'idle';
    el.style.color = 'var(--text-dim)';
  }
}

function setSerialStatus(msg, cls) {
  ['serial-status-val','t-serial-status'].forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.textContent = msg;
      el.style.color = cls === 'ok' ? 'var(--green)' : cls === 'err' ? 'var(--red)' : 'var(--text-dim)';
    }
  });
}

function _serialSetConnectBusy(busy) {
  const el = document.getElementById('serial-connect-btn');
  if (el) el.disabled = busy;
}

socket.on('serial_status', d => {
  setSerialStatus(d.msg, d.ok ? 'ok' : (d.connecting ? '' : 'err'));
  if (d.connecting) return;
  _serialSetConnectBusy(false);
  if (d.ok) {
    const mode = d.bridge === 'xbee' ? 'xbee' : 'usb';
    cgSetConnectionMode(mode, 'hw', d.port || '');
    loadCppBlocks();
  } else {
    cgSetConnectionMode('idle');
  }
});
