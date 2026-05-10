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

// ── Vision calib (multi-pose parallax) ─────────────────────────────────────
function runVisionRecalage() {
  const isHw = _currentConnectionMode === 'usb' || _currentConnectionMode === 'xbee';
  if (!isHw) {
    showToast('Vision calib is hardware only — connect to the robot first');
    return;
  }
  if (!_recalageDone) {
    showToast('Run classical recalage first — homography must be locked');
    return;
  }
  socket.emit('vision_recalage');
}

socket.on('vision_recalage_state', data => {
  const btn = document.getElementById('btn-vision-recalage');
  if (!btn) return;
  if (data && data.running) {
    btn.disabled = true;
    btn.textContent = '📐 Vision calib…';
    showToast('Vision calibration sweep running…');
  } else {
    btn.disabled = false;
    btn.textContent = '📐 Vision calib';
    // Make sure the per-point modal is closed if the sweep ends for any
    // reason (server-side success, abort, firmware timeout).
    _hideVisionRecalageModal();
    if (data && data.error === 'homography_not_locked') {
      // Server-side guard caught what the client missed (rare race).
      showToast('Homography not locked — run classical recalage first');
    } else if (data && data.error === 'not_connected') {
      showToast('Vision calib failed: no robot connected');
    } else if (data && data.ok === false) {
      showToast(`Vision calib failed: ${data.res || 'firmware did not ack'}`);
    } else if (data && data.ok === true) {
      showToast('Vision calibration complete — config saved');
    }
  }
});

// ── Diagnostic: test syncToVision round-trip ─────────────────────────────
// Drives to (1000,1000), syncs OTOS to vision, drives to (1000,1000) again.
// With a detuned OTOS scale, the first move overshoots/undershoots; the
// second move should land on the physical target if vision sync works.
function runTestSyncVision() {
  const isHw = _currentConnectionMode === 'usb' || _currentConnectionMode === 'xbee';
  if (!isHw) {
    showToast('Test sync is hardware only — connect to the robot first');
    return;
  }
  if (!_recalageDone) {
    showToast('Run classical recalage first — vision must be calibrated');
    return;
  }
  socket.emit('test_sync_vision');
}

socket.on('test_sync_vision_state', data => {
  const btn = document.getElementById('btn-test-sync-vision');
  if (!btn) return;
  if (data && data.running) {
    btn.disabled = true;
    btn.textContent = '🎯 Testing…';
    showToast('Test sync running — watch the robot');
  } else {
    btn.disabled = false;
    btn.textContent = '🎯 Test sync';
    if (data && data.error === 'not_connected') {
      showToast('Test sync failed: no robot connected');
    } else if (data && data.ok === false) {
      showToast(`Test sync failed: ${data.res || 'firmware did not ack'}`);
    } else if (data && data.ok === true) {
      showToast('Test sync done — measure the actual position vs (1000, 1000)');
    }
  }
});


// ── Manual-confirm modal for the per-point capture ───────────────────────
// Driven by holOS's `vision_recalage_wait_user` event during the firmware
// vision_recalage() sweep. The firmware drives the robot near each
// target, disengages the steppers, and waits up to 125 s for holOS's
// reply. holOS in turn waits up to 120 s for the operator to push the
// robot to the precise target and click Confirm.
//
// Modal has two states:
//   waiting — instructions + Confirm/Cancel
//   error   — red banner with the failure reason + a single Close
//             button (Cancel relabelled). Sweep is already aborted
//             firmware-side, no further user action needed.
function _setVisionRecalageState(state) {
  const okBtn      = document.getElementById('vision-recalage-confirm');
  const cancelBtn  = document.getElementById('vision-recalage-cancel');
  const errBanner  = document.getElementById('vision-recalage-error');
  const target     = document.getElementById('vision-recalage-target');
  const instr      = document.getElementById('vision-recalage-instructions');
  if (state === 'error') {
    if (okBtn)     okBtn.classList.add('hidden');
    if (cancelBtn) cancelBtn.textContent = '✕ Close';
    if (target)    target.style.opacity = '0.5';
    if (instr)     instr.style.opacity = '0.5';
    if (errBanner) errBanner.classList.remove('hidden');
  } else { // 'waiting'
    if (okBtn)     okBtn.classList.remove('hidden');
    if (cancelBtn) cancelBtn.textContent = '✕ Cancel sweep';
    if (target)    target.style.opacity = '1';
    if (instr)     instr.style.opacity = '1';
    if (errBanner) {
      errBanner.classList.add('hidden');
      errBanner.textContent = '';
    }
  }
}

function _showVisionRecalageModal(target) {
  const modal = document.getElementById('vision-recalage-modal');
  const body  = document.getElementById('vision-recalage-target');
  if (!modal || !body) return;
  const tx = (typeof target.target_x === 'number') ? target.target_x : null;
  const ty = (typeof target.target_y === 'number') ? target.target_y : null;
  const tt = (typeof target.target_t_deg === 'number') ? target.target_t_deg : null;
  const xyLine = `Target:&nbsp;&nbsp;X = <strong>${tx === null ? '?' : Math.round(tx) + ' mm'}</strong>` +
                 `&nbsp;&nbsp;Y = <strong>${ty === null ? '?' : Math.round(ty) + ' mm'}</strong>`;
  const thetaLine = (tt !== null)
    ? `<br>θ ≈ ${tt >= 0 ? '+' : ''}${tt.toFixed(1)}° <span style="color:var(--muted,#6b7a8d);">(approximate — align by eye)</span>`
    : '';
  body.innerHTML = xyLine + thetaLine;
  _setVisionRecalageState('waiting');
  modal.classList.remove('hidden');
}

function _showVisionRecalageError(reason, message) {
  const modal     = document.getElementById('vision-recalage-modal');
  const errBanner = document.getElementById('vision-recalage-error');
  if (!modal || !errBanner) return;
  // If the modal isn't open (race: user already closed it), surface
  // the failure as a toast instead so they don't miss it entirely.
  if (modal.classList.contains('hidden')) {
    showToast(`Vision calib failed: ${message || reason || 'unknown'}`);
    return;
  }
  const tag = reason ? ` (${reason})` : '';
  errBanner.textContent = `Calibration failed${tag}: ${message || reason || 'unknown'}`;
  _setVisionRecalageState('error');
}

function _hideVisionRecalageModal() {
  const modal = document.getElementById('vision-recalage-modal');
  if (modal) modal.classList.add('hidden');
  _setVisionRecalageState('waiting');   // reset for next time
}

socket.on('vision_recalage_wait_user', target => {
  _showVisionRecalageModal(target || {});
});

socket.on('vision_recalage_wait_done', payload => {
  // Server reports per-point outcome via {ok, reason, message}. On
  // failure, keep the modal up and switch it into the error state so
  // the operator actually reads what went wrong. On success, close.
  if (payload && payload.ok === false) {
    _showVisionRecalageError(payload.reason, payload.message);
  } else {
    _hideVisionRecalageModal();
  }
});

document.addEventListener('DOMContentLoaded', () => {
  const okBtn     = document.getElementById('vision-recalage-confirm');
  const cancelBtn = document.getElementById('vision-recalage-cancel');
  if (okBtn) okBtn.addEventListener('click', () => {
    _hideVisionRecalageModal();
    socket.emit('vision_recalage_user_ok');
  });
  if (cancelBtn) cancelBtn.addEventListener('click', () => {
    // Two roles: in 'waiting' state this is "Cancel the sweep" and
    // we tell holOS so the firmware aborts. In 'error' state the
    // sweep is already dead — just dismiss.
    const errVisible = !document.getElementById('vision-recalage-error')
                          .classList.contains('hidden');
    _hideVisionRecalageModal();
    if (!errVisible) {
      socket.emit('vision_recalage_user_cancel');
      showToast('Vision calibration sweep cancelled');
    }
  });
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
