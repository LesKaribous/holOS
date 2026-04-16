// js/actuators.js — Servo controls, Pose library, Sequence builder
// Depends on: showToast

'use strict';

// ── Servo definitions (matches firmware groups.h) ──────────────────────────────
const ACT_GROUPS = {
  CA: {
    label: 'CA (Right Manip)',
    servos: [
      { id: 0, name: 'Grabber R',  min: 110, max: 170, def: 121, poses: { 0: 162, 1: 162, 2: 121 } },
      { id: 1, name: 'Elevator',   min: 0,   max: 60,  def: 5,   poses: { 0: 50, 1: 50, 2: 5 } },
      { id: 2, name: 'Grabber L',  min: 10,  max: 75,  def: 65,  poses: { 0: 20, 1: 20, 2: 65 } },
    ],
  },
  AB: {
    label: 'AB (Hugger)',
    servos: [
      { id: 3, name: 'Hug Elevator', min: 80,  max: 165, def: 155, poses: { 0: 155, 1: 155, 2: 90 } },
      { id: 4, name: 'Hug Grab',     min: 20,  max: 100, def: 90,  poses: { 0: 90, 1: 30, 2: 90 } },
    ],
  },
  BC: {
    label: 'BC (Banner)',
    servos: [],
  },
};

// ── State ──────────────────────────────────────────────────────────────────────
let _actGroup       = 'CA';
let _actServoAngles = {};
let _actPoses       = {};
let _actSequences   = {};
let _actSeqSteps    = [];
let _actSeqPlaying  = false;
let _actSeqAbort    = false;
let _actBusy        = false;
let _actSliderTimers = {};

// ── Init ───────────────────────────────────────────────────────────────────────
async function actInit() {
  try {
    const [posesRes, seqsRes] = await Promise.all([
      fetch('/api/actuator/poses').then(r => r.json()),
      fetch('/api/actuator/sequences').then(r => r.json()),
    ]);
    _actPoses     = posesRes || {};
    _actSequences = seqsRes  || {};
  } catch (e) { console.warn('actInit failed:', e); }
  actRenderServos();
  actRenderPoses();
  actRenderSeqSelect();
  _actSeqSteps = [];
  actRenderSeqSteps();
}

// ── Group tab switch ───────────────────────────────────────────────────────────
function actSelectGroup(group, btn) {
  _actGroup = group;
  document.querySelectorAll('.act-gtab').forEach(b => b.classList.remove('active'));
  btn?.classList.add('active');
  actRenderServos();
}

// ── Servo rendering ────────────────────────────────────────────────────────────
function actRenderServos() {
  const el = document.getElementById('act-servo-list');
  if (!el) return;
  const g = ACT_GROUPS[_actGroup];
  if (!g || g.servos.length === 0) {
    el.innerHTML = '<span class="act-empty">No servos in this group</span>';
    return;
  }
  el.innerHTML = g.servos.map(s => {
    const key      = `${_actGroup}:${s.id}`;
    const val      = _actServoAngles[key] ?? s.def;
    const poseNames = ['DROP', 'GRAB', 'STORE'];
    const poseBtns  = Object.entries(s.poses).map(([pi, a]) =>
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
          oninput="actSliderMove('${key}',this.value)">
        <button class="btn-tiny" onclick="actSliderSend('${_actGroup}',${s.id},'${key}')" title="Send to robot">Go</button>
      </div>
      <div class="act-servo-limits">
        <input type="number" class="act-lim-input" id="act-min-${key}" value="${s.min}" title="Min">
        <span class="act-lim-sep">–</span>
        <input type="number" class="act-lim-input" id="act-max-${key}" value="${s.max}" title="Max">
        <button class="btn-tiny" onclick="actSetLimits('${_actGroup}',${s.id},'${key}')" title="Apply new limits">Set</button>
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

function actSliderSend(group, servoId, key) {
  clearTimeout(_actSliderTimers[key]);
  _actSliderTimers[key] = setTimeout(() => {
    const angle = _actServoAngles[key] ?? 90;
    actCmd(`servo(${group},${servoId},${angle})`);
  }, 80);
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

async function actSetLimits(group, servoId, key) {
  const minEl = document.getElementById('act-min-' + key);
  const maxEl = document.getElementById('act-max-' + key);
  if (!minEl || !maxEl) return;
  const newMin = parseInt(minEl.value), newMax = parseInt(maxEl.value);
  if (isNaN(newMin) || isNaN(newMax) || newMin >= newMax) {
    minEl.style.borderColor = 'red'; maxEl.style.borderColor = 'red';
    return;
  }
  minEl.style.borderColor = ''; maxEl.style.borderColor = '';
  const r = await actCmd(`servo_limits(${group},${servoId},${newMin},${newMax})`);
  if (r && r.ok) {
    const g = ACT_GROUPS[group];
    if (g) {
      const s = g.servos.find(x => x.id === servoId);
      if (s) { s.min = newMin; s.max = newMax; }
    }
    actRenderServos();
  }
}

async function actCmd(cmd) {
  if (_actBusy) {
    const st = document.getElementById('act-seq-status');
    if (st) st.textContent = `⏳ busy — skipped ${cmd}`;
    return { ok: false, res: 'busy' };
  }
  _actBusy = true;
  try {
    const res = await fetch('/api/exec', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ cmd, timeout_ms: 3000 })
    });
    const d = await res.json();
    const st = document.getElementById('act-seq-status');
    if (st) st.textContent = d.ok ? `✓ ${cmd}` : `✗ ${cmd}: ${d.res}`;
    return d;
  } catch (e) {
    const st = document.getElementById('act-seq-status');
    if (st) st.textContent = `✗ ${cmd}: request failed`;
    return { ok: false, res: 'error' };
  } finally {
    _actBusy = false;
  }
}

// ── Pose Library ───────────────────────────────────────────────────────────────
function actSnapshotPose() {
  const name = prompt('Pose name:', `pose_${Object.keys(_actPoses).length}`);
  if (!name) return;
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
    const p       = _actPoses[name];
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

// ── Sequence Builder ───────────────────────────────────────────────────────────
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
    if (s.type === 'pose')        { icon = '🎯'; label = `Pose: <b>${s.pose}</b>`; }
    else if (s.type === 'delay')  { icon = '⏱'; label = `Wait ${s.ms} ms`; }
    else if (s.type === 'cmd')    { icon = '⚡'; label = `<code>${s.cmd}</code>`; }
    else                          { icon = '?';  label = JSON.stringify(s); }
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
  _actSeqAbort   = false;
  document.getElementById('act-seq-play-btn')?.classList.add('active');
  const st = document.getElementById('act-seq-status');

  for (let i = 0; i < _actSeqSteps.length; i++) {
    if (_actSeqAbort) break;
    const step = _actSeqSteps[i];
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
