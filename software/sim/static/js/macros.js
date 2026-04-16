// js/macros.js — Macro CRUD, step builder, robot execution
// Depends on: socket, poiData, cppBlocks, showToast, escHtml,
//   renderMacroRefsInStrategy (strategy.js)

'use strict';

// ── State ──────────────────────────────────────────────────────────────────
let macros = [];
let activeMacroIdx = -1;

const STEP_DEFS = [
  { type:'move_to',     icon:'🎯', label:'Move to',          fields:['x','y'] },
  { type:'face',        icon:'↻',  label:'Face angle',       fields:['angle_deg'] },
  { type:'actuator',    icon:'🔧', label:'Actuator cmd',     fields:['cmd','wait_ms'] },
  { type:'wait',        icon:'⏱',  label:'Wait',             fields:['ms'] },
  { type:'if_occupied', icon:'❓', label:'If zone occupied', fields:['poi','skip'] },
  { type:'call_macro',  icon:'▶',  label:'Call macro',       fields:['name'] },
  { type:'log',         icon:'📝', label:'Log message',      fields:['msg'] },
];

// ── Load / save ────────────────────────────────────────────────────────────
function loadMacros() {
  fetch('/api/macros').then(r => r.json()).then(data => {
    macros = Array.isArray(data) ? data : [];
    renderMacroList();
    if (typeof renderMacroRefsInStrategy === 'function') renderMacroRefsInStrategy();
  });
}

function saveMacros() {
  if (activeMacroIdx >= 0) syncActiveMacroFromUI();
  fetch('/api/macros', {
    method: 'PUT', headers: {'Content-Type':'application/json'},
    body: JSON.stringify(macros)
  }).then(() => showToast('Macros saved ✓'));
}

// ── List rendering ─────────────────────────────────────────────────────────
function renderMacroList() {
  const list = document.getElementById('macro-list'); if (!list) return;
  const userHtml = macros.map((m, i) => `
    <div class="file-item${i === activeMacroIdx ? ' active' : ''}" onclick="selectMacro(${i})">
      <span class="file-item-name">⚡ ${escHtml(m.name)}</span>
      <span class="file-item-actions">
        <button class="btn-tiny red" onclick="event.stopPropagation();confirmDeleteMacro(${i})">🗑</button>
      </span>
    </div>`).join('');
  const cppHtml = cppBlocks.map(b => `
    <div class="file-item cpp-block" title="C++ block — read-only, registered in firmware">
      <span class="file-item-name">
        <span class="badge-cpp">C++</span> ${escHtml(b.name)}
        <span style="font-size:9px;color:var(--text-dim);margin-left:4px">#${b.priority} • ${b.score}pts${b.done ? ' ✓' : ''}</span>
      </span>
      <span class="file-item-actions">
        <button class="btn-tiny green" onclick="event.stopPropagation();runCppBlock('${escHtml(b.name)}')" title="Execute on robot">▶</button>
      </span>
    </div>`).join('');
  if (!macros.length && !cppBlocks.length) { list.innerHTML = '<span class="file-list-empty">No macros yet</span>'; return; }
  list.innerHTML = userHtml + (cppBlocks.length ? '<div class="file-list-divider">Firmware Blocks</div>' + cppHtml : '');
}

// ── CRUD ───────────────────────────────────────────────────────────────────
function newMacro() {
  const name = prompt('Macro name (snake_case):');
  if (!name) return;
  const m = { name: name.replace(/\s+/g, '_'), description: '', start_pos: null, steps: [] };
  macros.push(m);
  activeMacroIdx = macros.length - 1;
  saveMacros(); renderMacroList(); showMacroEditor(activeMacroIdx);
}

function selectMacro(idx) {
  if (activeMacroIdx >= 0) syncActiveMacroFromUI();
  activeMacroIdx = idx; renderMacroList(); showMacroEditor(idx);
}

function showMacroEditor(idx) {
  const m = macros[idx]; if (!m) return;
  document.getElementById('macro-empty').classList.add('hidden');
  document.getElementById('macro-editor').classList.remove('hidden');
  document.getElementById('macro-name').value = m.name;
  document.getElementById('macro-desc').value = m.description || '';
  document.getElementById('macro-sx').value = m.start_pos ? m.start_pos.x : '';
  document.getElementById('macro-sy').value = m.start_pos ? m.start_pos.y : '';
  renderMacroSteps(m.steps);
}

function syncActiveMacroFromUI() {
  const m = macros[activeMacroIdx]; if (!m) return;
  m.name        = document.getElementById('macro-name')?.value || m.name;
  m.description = document.getElementById('macro-desc')?.value || '';
  const sx = document.getElementById('macro-sx')?.value;
  const sy = document.getElementById('macro-sy')?.value;
  m.start_pos   = (sx && sy) ? {x: parseFloat(sx), y: parseFloat(sy)} : null;
}

function deleteMacro() { if (activeMacroIdx < 0) return; confirmDeleteMacro(activeMacroIdx); }
function confirmDeleteMacro(idx) {
  if (!confirm(`Delete macro "${macros[idx]?.name}"?`)) return;
  macros.splice(idx, 1);
  activeMacroIdx = Math.min(activeMacroIdx, macros.length - 1);
  saveMacros(); renderMacroList();
  if (activeMacroIdx >= 0) showMacroEditor(activeMacroIdx);
  else {
    document.getElementById('macro-empty').classList.remove('hidden');
    document.getElementById('macro-editor').classList.add('hidden');
  }
}

// ── Step rendering ─────────────────────────────────────────────────────────
function renderMacroSteps(steps) {
  const container = document.getElementById('macro-steps-list'); if (!container) return;
  container.innerHTML = steps.map((step, i) => buildStepHTML(step, i)).join('');
}

function buildStepHTML(step, idx) {
  const def = STEP_DEFS.find(d => d.type === step.type) || STEP_DEFS[0];
  const typeOptions = STEP_DEFS.map(d =>
    `<option value="${d.type}"${d.type === step.type ? ' selected' : ''}>${d.icon} ${d.label}</option>`
  ).join('');
  const fields = buildFieldsHTML(step, idx);
  return `<div class="macro-step" id="ms-${idx}">
    <span class="step-icon">${def.icon}</span>
    <select class="step-type-sel" onchange="changeStepType(${idx},this.value)">${typeOptions}</select>
    <div class="step-fields">${fields}</div>
    <button class="btn-tiny" onclick="runStepOnRobot(${idx})" title="Exécuter cette étape sur le robot connecté">🤖</button>
    <button class="btn-tiny" onclick="moveStep(${idx},-1)" title="Move up">↑</button>
    <button class="btn-tiny" onclick="moveStep(${idx},+1)" title="Move down">↓</button>
    <button class="btn-tiny red" onclick="deleteStep(${idx})">✕</button>
  </div>`;
}

function buildFieldsHTML(step, idx) {
  const f = (key, placeholder, val, type, w) =>
    `<label class="step-field-label">${key}</label>
     <input class="input-sm" type="${type || 'text'}" placeholder="${placeholder}" value="${val || ''}" style="width:${w || '80px'}"
       oninput="updateStepField(${idx},'${key}',this.value)">`;
  switch (step.type) {
    case 'move_to':     return f('x','0',step.x,'number','60px') + f('y','0',step.y,'number','60px');
    case 'face':        return f('angle_deg','0°',step.angle_deg,'number','70px');
    case 'actuator':    return f('cmd','arm(180)',step.cmd,'text','120px') + f('wait_ms','0',step.wait_ms,'number','55px');
    case 'wait':        return f('ms','500',step.ms,'number','70px');
    case 'if_occupied': return buildPOISelect(step, idx) + f('skip','1',step.skip,'number','45px');
    case 'call_macro':  return buildMacroSelect(step, idx);
    case 'log':         return f('msg','message...',step.msg,'text','180px');
    default:            return '';
  }
}

function buildPOISelect(step, idx) {
  const opts = poiData.map(p => `<option value="${p.name}"${p.name === step.poi ? ' selected' : ''}>${p.name}</option>`).join('');
  return `<label class="step-field-label">poi</label>
    <select class="input-sm" onchange="updateStepField(${idx},'poi',this.value)">
      <option value="">— choose —</option>${opts}</select>
    <label class="step-field-label">→ skip</label>`;
}

function buildMacroSelect(step, idx) {
  const opts = macros.map(m => `<option value="${m.name}"${m.name === step.name ? ' selected' : ''}>${m.name}</option>`).join('');
  return `<label class="step-field-label">macro</label>
    <select class="input-sm" onchange="updateStepField(${idx},'name',this.value)">
      <option value="">— choose —</option>${opts}</select>`;
}

function changeStepType(idx, newType) {
  const m = macros[activeMacroIdx]; if (!m) return;
  m.steps[idx] = {type: newType}; renderMacroSteps(m.steps);
}

function updateStepField(idx, key, val) {
  const m = macros[activeMacroIdx]; if (!m || !m.steps[idx]) return;
  const numFields = ['x','y','angle_deg','ms','wait_ms','skip'];
  m.steps[idx][key] = numFields.includes(key) ? parseFloat(val) || 0 : val;
}

function addMacroStep() {
  const m = macros[activeMacroIdx]; if (!m) return;
  m.steps.push({type: 'move_to', x: 0, y: 0}); renderMacroSteps(m.steps);
}

function deleteStep(idx) {
  const m = macros[activeMacroIdx]; if (!m) return;
  m.steps.splice(idx, 1); renderMacroSteps(m.steps);
}

function moveStep(idx, dir) {
  const m = macros[activeMacroIdx]; if (!m) return;
  const ni = idx + dir;
  if (ni < 0 || ni >= m.steps.length) return;
  [m.steps[idx], m.steps[ni]] = [m.steps[ni], m.steps[idx]];
  renderMacroSteps(m.steps);
}

// ── Robot execution ────────────────────────────────────────────────────────
function runMacroInSim() {
  syncActiveMacroFromUI();
  const m = macros[activeMacroIdx]; if (!m) return;
  const steps = [...m.steps];
  if (m.start_pos) steps.unshift({type:'move_to', x:m.start_pos.x, y:m.start_pos.y});
  socket.emit('run_macro', {steps, macros});
  showToast(`Running macro: ${m.name}`);
}

function runStepOnRobot(idx) {
  const m = macros[activeMacroIdx]; if (!m) return;
  const step = m.steps[idx]; if (!step) return;
  fetch('/api/missions/run-step', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify(step),
  }).then(r => r.json()).then(d => {
    const el = document.getElementById(`ms-${idx}`);
    const flash = d.ok ? '#22c55e44' : '#ef444444';
    if (el) { el.style.background = flash; setTimeout(() => el.style.background = '', 800); }
    showToast(d.ok ? `✓ ${d.cmd}` : `✗ ${d.error || d.response}`);
  }).catch(e => showToast('✗ ' + e));
}

let _macroRobotRunning = false;
function runMacroOnRobot() {
  if (_macroRobotRunning) { showToast('Macro déjà en cours'); return; }
  syncActiveMacroFromUI();
  const m = macros[activeMacroIdx]; if (!m) return;
  const steps = [...m.steps];
  if (m.start_pos) steps.unshift({type:'move_to', x:m.start_pos.x, y:m.start_pos.y});
  _macroRobotRunning = true;
  const btn = document.getElementById('btn-macro-run-robot');
  if (btn) btn.textContent = '⏳ Running…';
  (async () => {
    for (let i = 0; i < steps.length; i++) {
      document.querySelectorAll('.macro-step').forEach((el, idx) => { el.style.background = idx === i ? '#3b82f644' : ''; });
      try {
        const res = await fetch('/api/missions/run-step', {
          method: 'POST', headers: {'Content-Type':'application/json'},
          body: JSON.stringify(steps[i]),
        }).then(r => r.json());
        if (!res.ok) { showToast(`✗ Étape ${i+1} échouée: ${res.error || res.response}`); break; }
      } catch(e) { showToast('✗ Erreur réseau: ' + e); break; }
    }
    document.querySelectorAll('.macro-step').forEach(el => el.style.background = '');
    _macroRobotRunning = false;
    if (btn) btn.textContent = '🤖 Robot';
  })();
}

// ── Python code generation ─────────────────────────────────────────────────
function generateMacroPython() {
  syncActiveMacroFromUI();
  const m = macros[activeMacroIdx]; if (!m) return;
  const code = macroPythonCode(m);
  const pre = document.getElementById('macro-python-code');
  const wrap = document.getElementById('macro-python-preview');
  if (pre) pre.textContent = code;
  if (wrap) wrap.classList.remove('hidden');
}

function hidePythonPreview() { document.getElementById('macro-python-preview')?.classList.add('hidden'); }

function macroPythonCode(m) {
  const fn = m.name.replace(/[^a-z0-9_]/gi, '_').toLowerCase();
  let c = `def macro_${fn}(brain, transport):\n    """${m.description || m.name}"""\n`;
  if (m.start_pos) c += `    brain.exec("go(${m.start_pos.x},${m.start_pos.y})")\n    time.sleep(0.5)\n`;
  for (const s of m.steps) {
    switch (s.type) {
      case 'move_to':     c += `    brain.exec("go(${s.x||0},${s.y||0})")\n    time.sleep(0.3)\n`; break;
      case 'face':        c += `    # rotate to ${s.angle_deg||0}°\n`; break;
      case 'actuator':    c += `    transport.fire('${s.cmd||''}')\n`; if (s.wait_ms) c += `    time.sleep(${(s.wait_ms/1000).toFixed(2)})\n`; break;
      case 'wait':        c += `    time.sleep(${((s.ms||0)/1000).toFixed(2)})\n`; break;
      case 'if_occupied': c += `    # if zone ${s.poi} occupied → skip ${s.skip||1} step(s)\n`; break;
      case 'call_macro':  c += `    macro_${(s.name||'').replace(/[^a-z0-9_]/gi,'_').toLowerCase()}(brain, transport)\n`; break;
      case 'log':         c += `    brain.log('${s.msg||''}')\n`; break;
    }
  }
  return c;
}

// ── Socket: seq_done (from run_macro) ──────────────────────────────────────
socket.on('seq_done', d => {
  document.querySelectorAll('.macro-step').forEach(el => el.classList.remove('running'));
  if (!d.stopped) showToast('Macro done ✓');
});
