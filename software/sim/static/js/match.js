// js/match.js — Match control, C++ blocks, strategy/team/mode
// Depends on: socket, _currentConnectionMode, showToast, loadMacros, renderMissionList

'use strict';

// ── Match state ────────────────────────────────────────────────────────────
let _matchRunning = false;

// ── Match controls ─────────────────────────────────────────────────────────
function runStrategy()  { socket.emit('run_strategy'); }
function stopStrategy() { socket.emit('stop_strategy'); }
function matchStart()   { socket.emit('match_start'); }
function matchStop()    { socket.emit('match_stop'); }
function matchResume()  { socket.emit('match_resume'); }

function unifiedStart() {
  const isHw = _currentConnectionMode === 'usb' || _currentConnectionMode === 'xbee';
  if (isHw) {
    socket.emit('match_start');
  } else {
    socket.emit('run_strategy');
  }
  _matchRunning = true;
  _updateStartStopUI();
}

function unifiedStop() {
  const isHw = _currentConnectionMode === 'usb' || _currentConnectionMode === 'xbee';
  if (isHw) socket.emit('match_stop');
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
// Hardware-only: fires the long-press behavior of the robot's physical
// reset button (wall-probe → drive to start pos → vision recalage). Same
// thing as if the operator pressed the on-robot button manually, just
// triggerable from the UI when the robot is reachable but the button
// isn't.
function runRecalage() {
  const isHw = _currentConnectionMode === 'usb' || _currentConnectionMode === 'xbee';
  if (!isHw) {
    showToast('Recalage is hardware only — connect to the robot first');
    return;
  }
  socket.emit('recalage');
}

socket.on('recalage_state', data => {
  const btn = document.getElementById('btn-recalage');
  if (!btn) return;
  if (data && data.running) {
    btn.disabled = true;
    btn.textContent = '⌖ Recalage…';
    showToast('Recalage running on robot…');
  } else {
    btn.disabled = false;
    btn.textContent = '⌖ Recalage';
    if (data && data.error === 'not_connected') {
      showToast('Recalage failed: no robot connected');
    } else if (data && data.ok === false) {
      showToast(`Recalage failed: ${data.res || 'firmware did not ack'}`);
    } else if (data && data.ok === true) {
      showToast('Recalage complete');
    }
  }
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
