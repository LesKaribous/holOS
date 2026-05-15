// js/match.js — Match control, C++ blocks, strategy/team/mode
// Depends on: socket, _currentConnectionMode, showToast, loadMacros, renderMissionList

'use strict';

// ── Match state ────────────────────────────────────────────────────────────
let _matchRunning = false;

// Single source of truth for "is the UI talking to real hardware".
// Add a new HW transport here when we grow another one.
function _isHwMode() {
  const m = _currentConnectionMode;
  return m === 'usb' || m === 'xbee' || m === 'wifi';
}

// ── Match controls ─────────────────────────────────────────────────────────
function runStrategy()  { socket.emit('run_strategy'); }
function stopStrategy() { socket.emit('stop_strategy'); }
function matchStart()   { socket.emit('match_start'); }
function matchStop()    { socket.emit('match_stop'); }
function matchResume()  { socket.emit('match_resume'); }

function unifiedStart() {
  if (_isHwMode()) {
    socket.emit('match_start');
  } else {
    socket.emit('run_strategy');
  }
  _matchRunning = true;
  _updateStartStopUI();
}

function unifiedStop() {
  if (_isHwMode()) socket.emit('match_stop');
  socket.emit('stop_strategy');
  _matchRunning = false;
  _updateStartStopUI();
}

function _updateStartStopUI() {
  const startBtn = document.getElementById('btn-start');
  const stopBtn  = document.getElementById('btn-stop');
  if (startBtn) { startBtn.textContent = _matchRunning ? '▶ Running…' : '▶ Start'; startBtn.disabled = _matchRunning; }
  if (stopBtn) stopBtn.disabled = !_matchRunning;
}

// ── Recalage (firmware routine) ────────────────────────────────────────────
// Two distinct flows:
//   • Classical recalage: homography lock + prise d'origine. Quick
//     (~5 s). Required before every match.
//   • Vision calib: multi-pose parallax sweep. Long (~30-60 s). Run
//     ONCE per camera mount — saved config sticks across reboots.
//     Requires classical recalage to have run first (gates on
//     homography lock, both client-side and server-side).
//
// `_recalageDone` tracks whether the classical recalage completed so
// we can (a) flip the button to a ✓ icon, and (b) early-block the
// vision-calib button without a server round-trip.
let _recalageDone = false;

function runRecalage() {
  if (!_isHwMode()) {
    showToast('Recalage is hardware only — connect to the robot first');
    return;
  }
  socket.emit('recalage');
}

socket.on('recalage_state', data => {
  const btn = document.getElementById('btn-recalage');
  if (data && data.homography_snapshot_ts) {
    // Server captured a fresh BEV snapshot — refresh the dashboard tile.
    // `?t=<ts>` busts the browser cache; img stays static between recalages.
    _updateHomographySnapshot(data.homography_snapshot_ts);
  }
  if (!btn) return;
  if (data && data.running) {
    btn.disabled = true;
    btn.textContent = '⌖ Recalage…';
    showToast('Recalage running on robot…');
  } else {
    btn.disabled = false;
    if (data && data.error === 'not_connected') {
      btn.textContent = '⌖ Recalage';
      showToast('Recalage failed: no robot connected');
    } else if (data && data.ok === false) {
      btn.textContent = '⌖ Recalage';
      showToast(`Recalage failed: ${data.res || 'firmware did not ack'}`);
    } else if (data && data.ok === true) {
      _recalageDone = true;
      btn.textContent = '✓ Recalage';
      showToast('Recalage complete');
    }
  }
});

function _updateHomographySnapshot(ts) {
  const img   = document.getElementById('vd-homography-img');
  const empty = document.getElementById('vd-homography-empty');
  const meta  = document.getElementById('vd-meta-homography');
  if (!img) return;
  img.onload  = () => { img.style.display = 'block'; if (empty) empty.style.display = 'none'; };
  img.onerror = () => { img.style.display = 'none';  if (empty) empty.style.display = 'block'; };
  img.src = `/api/vision/homography_snapshot.jpg?t=${ts}`;
  if (meta) {
    const d = new Date(ts);
    meta.textContent = 'captured at ' + d.toLocaleTimeString();
  }
}

// On page load, check whether the server already has a snapshot from a
// prior recalage (server keeps it in RAM across browser refreshes, not
// across server restarts). HEAD + read the timestamp from the header so
// we don't download the JPEG twice.
document.addEventListener('DOMContentLoaded', () => {
  fetch('/api/vision/homography_snapshot.jpg', { method: 'HEAD' })
    .then(r => {
      if (!r.ok) return;
      const ts = r.headers.get('X-Snapshot-Ts');
      if (ts) _updateHomographySnapshot(parseInt(ts, 10));
    })
    .catch(() => { /* no snapshot yet, leave the empty state */ });
});



socket.on('match_state', data => {
  if (data.running !== undefined) {
    _matchRunning = data.running;
    _updateStartStopUI();
    if (data.running) {
      showToast('Match started');
    } else if (data.error === 'not_connected') {
      showToast('No hardware connected — connect via Serial first');
    } else {
      showToast('Match stopped');
    }
  }
});

// ── C++ blocks ─────────────────────────────────────────────────────────────
let cppBlocks = [];

function loadCppBlocks() {
  fetch('/api/cpp_blocks').then(r => r.json()).then(data => {
    cppBlocks = Array.isArray(data) ? data : [];
    if (typeof renderMacroList === 'function') renderMacroList();
    if (typeof renderMissionList === 'function') renderMissionList();
  }).catch(() => { cppBlocks = []; });
}

function runCppBlock(name) { socket.emit('run_cpp_block', { name }); }

socket.on('cpp_block_result', data => {
  showToast(data.ok ? `C++ block "${data.name}" → SUCCESS` : `C++ block "${data.name}" → FAILED`);
  loadCppBlocks();
});

// ── Sim/strategy controls ──────────────────────────────────────────────────
function resetSim()       { socket.emit('reset'); }
function reloadStrategy() { socket.emit('reload_strategy'); }

function setTeam(team) {
  socket.emit('set_team', {team});
  document.getElementById('btn-yellow')?.classList.toggle('active', team === 'yellow');
  document.getElementById('btn-blue')?.classList.toggle('active', team === 'blue');
}

function setMode(mode) { socket.emit('set_mode', {mode}); }

// ── REC button ────────────────────────────────────────────────────────────
let _recOn = false;
let _recStart = 0;

function toggleRec() {
  const endpoint = _recOn ? '/api/log/stop' : '/api/log/start';
  fetch(endpoint, { method: 'POST' }).then(r => r.json()).then(d => {
    if (d.ok) {
      _recOn = !!(d.recording !== undefined ? d.recording : !_recOn);
      _refreshRecBtn(d);
      if (_recOn) showToast(`REC ${d.session_id || ''}`);
      else if (d.summary) showToast(`REC stopped (${d.summary.duration_s}s · ${d.summary.video_frames} frames)`);
    }
  }).catch(e => showToast('REC error: ' + e));
}

function _refreshRecBtn(status) {
  const dot   = document.getElementById('rec-dot');
  const label = document.getElementById('rec-label');
  if (!dot || !label) return;
  _recOn = !!(status && status.recording);
  if (_recOn) {
    dot.style.background = '#c0392b';
    dot.style.boxShadow  = '0 0 6px #c0392b';
    const t = (status && typeof status.elapsed_s === 'number') ? status.elapsed_s : 0;
    const mm = Math.floor(t / 60), ss = Math.floor(t % 60);
    label.textContent = `REC ${mm}:${String(ss).padStart(2,'0')}`;
  } else {
    dot.style.background = '#666';
    dot.style.boxShadow  = 'none';
    label.textContent = 'REC';
  }
}

setInterval(() => {
  fetch('/api/log/status').then(r => r.json()).then(_refreshRecBtn).catch(() => {});
}, 1000);
