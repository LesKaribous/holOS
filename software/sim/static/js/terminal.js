// js/terminal.js — Terminal I/O + Serial connectivity
// Depends on: socket, showToast, escHtml, cgSetRobotConnecting,
//   cgSetConnectionMode, loadCppBlocks

'use strict';

// ── Terminal ───────────────────────────────────────────────────────────────
let _termHistory = [], _termHistIdx = -1;
let _termTab = 'cmd';

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

// ── UART Raw tab ───────────────────────────────────────────────────────────
function switchTermTab(tab) {
  _termTab = tab;
  document.getElementById('term-tab-cmd')?.classList.toggle('active', tab === 'cmd');
  document.getElementById('term-tab-uart')?.classList.toggle('active', tab === 'uart');
  document.getElementById('term-pane-cmd').style.display  = tab === 'cmd'  ? '' : 'none';
  document.getElementById('term-pane-uart').style.display = tab === 'uart' ? '' : 'none';
  if (tab === 'uart') {
    const o = document.getElementById('uart-output');
    if (o) o.scrollTop = o.scrollHeight;
  }
}

function appendUartLine(dir, text) {
  const out = document.getElementById('uart-output'); if (!out) return;
  const now = new Date();
  const ts = now.toTimeString().slice(0, 8) + '.' + String(now.getMilliseconds()).padStart(3, '0');
  const d = document.createElement('div'); d.className = 'uart-line';
  const arrow = dir === 'rx' ? '←' : dir === 'tx' ? '→' : '·';
  const dirCls = 'uart-dir-' + dir;
  const dataCls = 'uart-data' + (dir === 'tx' ? ' tx' : dir === 'sys' ? ' sys' : '');
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

function serialConnect() {
  const port = (document.getElementById('serial-port-sel') || document.getElementById('t-port-sel'))?.value;
  if (!port) { showToast('Select a serial port first'); return; }
  cgSetRobotConnecting();
  setSerialStatus('Connecting…', '');
  socket.emit('serial_connect', {port});
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
