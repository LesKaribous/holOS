// js/missions.js — Mission CRUD, approach picker, firmware fallback deploy
// Depends on: socket, macros, cppBlocks, poiData, STEP_DEFS,
//   showToast, escHtml, switchView, runCppBlock

'use strict';

// ── State ──────────────────────────────────────────────────────────────────
let missions = [];
let activeMissionIdx = -1;
let _missionPickingApproach = false;

// ── Load / save ────────────────────────────────────────────────────────────
function loadMissions() {
  fetch('/api/missions').then(r => r.json()).then(data => {
    missions = Array.isArray(data) ? data : [];
    renderMissionList();
  });
}

function saveMissions() {
  if (activeMissionIdx >= 0) syncActiveMissionFromUI();
  fetch('/api/missions', {
    method: 'PUT', headers: {'Content-Type':'application/json'},
    body: JSON.stringify(missions),
  }).then(() => showToast('Missions sauvegardées ✓'));
}

// ── List rendering ─────────────────────────────────────────────────────────
function renderMissionList() {
  const list = document.getElementById('mission-list'); if (!list) return;
  const sorted = [...missions.entries()].sort(([,a],[,b]) => b.priority - a.priority);
  const userHtml = sorted.map(([i, m]) => `
    <div class="file-item${i === activeMissionIdx ? ' active' : ''}" onclick="selectMission(${i})">
      <span class="file-item-name${m.enabled === false ? ' text-dim' : ''}">
        ${m.enabled === false ? '○' : '●'} ${escHtml(m.name)}
        <span style="font-size:9px;color:var(--text-dim);margin-left:4px">#${m.priority || 0} • ${m.score || 0}pts</span>
      </span>
      <span class="file-item-actions">
        <button class="btn-tiny red" onclick="event.stopPropagation();confirmDeleteMission(${i})">🗑</button>
      </span>
    </div>`).join('');
  const cppHtml = cppBlocks.map(b => `
    <div class="file-item cpp-block" title="C++ block — firmware embedded">
      <span class="file-item-name">
        <span class="badge-cpp">C++</span> ${escHtml(b.name)}
        <span style="font-size:9px;color:var(--text-dim);margin-left:4px">#${b.priority} • ${b.score}pts • ${(b.estimatedMs/1000).toFixed(1)}s${b.done ? ' ✓' : ''}</span>
      </span>
      <span class="file-item-actions">
        <button class="btn-tiny green" onclick="event.stopPropagation();runCppBlock('${escHtml(b.name)}')" title="Execute on robot">▶</button>
      </span>
    </div>`).join('');
  if (!missions.length && !cppBlocks.length) { list.innerHTML = '<span class="file-list-empty">Aucune mission</span>'; return; }
  list.innerHTML = userHtml + (cppBlocks.length ? '<div class="file-list-divider">Firmware Blocks</div>' + cppHtml : '');
}

// ── CRUD ───────────────────────────────────────────────────────────────────
function newMission() {
  const name = prompt('Nom de la mission (snake_case):');
  if (!name) return;
  const m = {
    id: 'm' + Date.now(), name: name.replace(/\s+/g, '_'), desc: '',
    approach: {x: null, y: null, angle: null}, macro_id: '',
    score: 20, priority: 5, time_ms: 8000, enabled: true,
  };
  missions.push(m);
  activeMissionIdx = missions.length - 1;
  saveMissions(); renderMissionList(); showMissionEditor(activeMissionIdx);
}

function selectMission(idx) {
  if (activeMissionIdx >= 0) syncActiveMissionFromUI();
  activeMissionIdx = idx; renderMissionList(); showMissionEditor(idx);
}

function showMissionEditor(idx) {
  const m = missions[idx]; if (!m) return;
  document.getElementById('mission-empty').classList.add('hidden');
  document.getElementById('mission-editor').classList.remove('hidden');
  document.getElementById('mission-name').value     = m.name || '';
  document.getElementById('mission-desc').value     = m.desc || '';
  document.getElementById('mission-score').value    = m.score ?? 20;
  document.getElementById('mission-priority').value = m.priority ?? 5;
  document.getElementById('mission-time-ms').value  = m.time_ms ?? 8000;
  const ap = m.approach || {};
  document.getElementById('mission-ax').value       = ap.x ?? '';
  document.getElementById('mission-ay').value       = ap.y ?? '';
  document.getElementById('mission-aa').value       = ap.angle ?? '';
  document.getElementById('mission-enabled').checked = m.enabled !== false;
  _refreshMacroSelector(m.macro_id || '');
  _refreshMissionMacroPreview(m.macro_id || '');
}

function _refreshMacroSelector(selectedId) {
  const sel = document.getElementById('mission-macro-sel'); if (!sel) return;
  sel.innerHTML = '<option value="">— aucune —</option>' +
    macros.map(m => `<option value="${escHtml(m.name)}"${m.name === selectedId ? ' selected' : ''}>${escHtml(m.name)}</option>`).join('');
}

function _refreshMissionMacroPreview(macroId) {
  const wrap = document.getElementById('mission-macro-preview');
  const container = document.getElementById('mission-steps-preview');
  if (!wrap || !container) return;
  const macro = macros.find(m => m.name === macroId);
  if (!macro || !macro.steps?.length) { wrap.classList.add('hidden'); return; }
  wrap.classList.remove('hidden');
  container.innerHTML = macro.steps.map((step) => {
    const def = STEP_DEFS.find(d => d.type === step.type) || STEP_DEFS[0];
    const summary = _stepSummary(step);
    return `<div class="mission-step-row">
      <span class="step-icon">${def.icon}</span>
      <span style="font-size:11px;flex:1">${def.label}: <code>${escHtml(summary)}</code></span>
      <button class="btn-tiny" onclick="runStepOnRobotDirect(${JSON.stringify(step).replace(/"/g,'&quot;')})" title="Exécuter sur robot">🤖</button>
    </div>`;
  }).join('');
}

function _stepSummary(step) {
  switch (step.type) {
    case 'move_to':     return `(${step.x||0}, ${step.y||0})`;
    case 'face':        return `${step.angle_deg||0}°`;
    case 'actuator':    return step.cmd || '';
    case 'wait':        return `${step.ms||0} ms`;
    case 'if_occupied': return `poi=${step.poi}`;
    case 'call_macro':  return `→ ${step.name}`;
    case 'log':         return step.msg || '';
    default:            return '';
  }
}

function runStepOnRobotDirect(step) {
  fetch('/api/missions/run-step', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify(step),
  }).then(r => r.json()).then(d => showToast(d.ok ? `✓ ${d.cmd}` : `✗ ${d.error || d.response}`))
    .catch(e => showToast('✗ ' + e));
}

function syncActiveMissionFromUI() {
  const m = missions[activeMissionIdx]; if (!m) return;
  m.name     = document.getElementById('mission-name')?.value || m.name;
  m.desc     = document.getElementById('mission-desc')?.value || '';
  m.score    = parseInt(document.getElementById('mission-score')?.value) || 0;
  m.priority = parseInt(document.getElementById('mission-priority')?.value) || 0;
  m.time_ms  = parseInt(document.getElementById('mission-time-ms')?.value) || 8000;
  m.enabled  = document.getElementById('mission-enabled')?.checked !== false;
  const ax = document.getElementById('mission-ax')?.value;
  const ay = document.getElementById('mission-ay')?.value;
  const aa = document.getElementById('mission-aa')?.value;
  m.approach = {
    x: ax !== '' ? parseFloat(ax) : null,
    y: ay !== '' ? parseFloat(ay) : null,
    angle: aa !== '' ? parseFloat(aa) : null,
  };
  m.macro_id = document.getElementById('mission-macro-sel')?.value || '';
}

function syncMissionApproach() {
  if (activeMissionIdx < 0) return;
  const m = missions[activeMissionIdx];
  const ax = document.getElementById('mission-ax')?.value;
  const ay = document.getElementById('mission-ay')?.value;
  if (m) m.approach = { ...(m.approach||{}), x: ax !== '' ? parseFloat(ax) : null, y: ay !== '' ? parseFloat(ay) : null };
}

function syncMissionMacro() {
  const sel = document.getElementById('mission-macro-sel');
  if (!sel || activeMissionIdx < 0) return;
  missions[activeMissionIdx].macro_id = sel.value;
  _refreshMissionMacroPreview(sel.value);
}

function syncMissionEnabled() {
  if (activeMissionIdx < 0) return;
  missions[activeMissionIdx].enabled = document.getElementById('mission-enabled')?.checked !== false;
  renderMissionList();
}

function deleteMission() { confirmDeleteMission(activeMissionIdx); }
function confirmDeleteMission(idx) {
  const m = missions[idx]; if (!m) return;
  if (!confirm(`Supprimer la mission "${m.name}" ?`)) return;
  missions.splice(idx, 1);
  activeMissionIdx = Math.min(activeMissionIdx, missions.length - 1);
  saveMissions(); renderMissionList();
  if (activeMissionIdx >= 0) showMissionEditor(activeMissionIdx);
  else {
    document.getElementById('mission-empty')?.classList.remove('hidden');
    document.getElementById('mission-editor')?.classList.add('hidden');
  }
}

function missionGoToMacro() {
  const sel = document.getElementById('mission-macro-sel');
  if (!sel?.value) return;
  const idx = macros.findIndex(m => m.name === sel.value);
  if (idx < 0) return;
  const btn = document.querySelector('[data-view=macros]');
  if (btn) switchView('macros', btn);
  selectMacro(idx);
}

// ── Approach point picker ──────────────────────────────────────────────────
function missionPickApproach() {
  _missionPickingApproach = true;
  showToast('Cliquez sur la carte pour définir le point d\'approche');
  const btn = document.querySelector('[data-view=map]');
  if (btn) switchView('map', btn);
}

function _onMapClickForMission(worldX, worldY) {
  if (!_missionPickingApproach) return false;
  _missionPickingApproach = false;
  document.getElementById('mission-ax').value = Math.round(worldX);
  document.getElementById('mission-ay').value = Math.round(worldY);
  syncMissionApproach();
  showToast(`Point d'approche: (${Math.round(worldX)}, ${Math.round(worldY)})`);
  const btn = document.querySelector('[data-view=missions]');
  if (btn) switchView('missions', btn);
  return true;
}

// ── Run mission on robot ───────────────────────────────────────────────────
function missionRunOnRobot() {
  if (activeMissionIdx < 0) return;
  syncActiveMissionFromUI();
  const m = missions[activeMissionIdx]; if (!m) return;
  const steps = [];
  const ap = m.approach || {};
  if (ap.x !== null && ap.y !== null) steps.push({type: 'move_to', x: ap.x, y: ap.y});
  if (ap.angle !== null && ap.angle !== undefined) steps.push({type: 'face', angle_deg: ap.angle});
  const macro = macros.find(mc => mc.name === m.macro_id);
  if (macro) steps.push(...macro.steps);
  if (!steps.length) { showToast('Aucune étape à exécuter'); return; }
  (async () => {
    const btn = document.getElementById('btn-mission-run');
    if (btn) btn.textContent = '⏳ Running…';
    for (const step of steps) {
      try {
        const res = await fetch('/api/missions/run-step', {
          method: 'POST', headers: {'Content-Type':'application/json'},
          body: JSON.stringify(step),
        }).then(r => r.json());
        if (!res.ok) { showToast(`✗ ${res.error || res.response}`); break; }
      } catch(e) { showToast('✗ ' + e); break; }
    }
    if (btn) btn.textContent = '▶ Run on robot';
  })();
}

// ── Firmware Fallback preview / deploy ─────────────────────────────────────
function missionPreviewFallback() {
  if (activeMissionIdx >= 0) syncActiveMissionFromUI();
  fetch('/api/missions/preview-fallback', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      const overlay = document.getElementById('mission-fallback-overlay');
      const pre     = document.getElementById('mission-fallback-text');
      if (!overlay || !pre) return;
      pre.textContent = d.cfg || '(empty)';
      overlay.style.display = 'flex';
    }).catch(e => showToast('✗ ' + e));
}

function missionCloseFallbackPreview() {
  const overlay = document.getElementById('mission-fallback-overlay');
  if (overlay) overlay.style.display = 'none';
}

function missionDeployFirmware() {
  if (activeMissionIdx >= 0) syncActiveMissionFromUI();
  const status = document.getElementById('mission-deploy-status');
  if (status) status.textContent = '⏳ Écriture…';
  const btn1 = document.getElementById('btn-deploy-fw');
  const btn2 = document.getElementById('btn-deploy-from-preview');
  [btn1, btn2].forEach(b => b && (b.disabled = true));
  fetch('/api/missions/deploy-firmware', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      const msg = d.ok ? `✓ ${d.lines} lignes déployées vers firmware` : `✗ ${d.error}`;
      if (status) status.textContent = msg;
      showToast(msg);
    }).catch(e => { if (status) status.textContent = '✗ ' + e; })
    .finally(() => [btn1, btn2].forEach(b => b && (b.disabled = false)));
}

// ── View activation ────────────────────────────────────────────────────────
function onMissionsViewActivated() {
  loadMissions();
  loadCppBlocks();
  if (activeMissionIdx >= 0) _refreshMacroSelector(missions[activeMissionIdx]?.macro_id || '');
}
