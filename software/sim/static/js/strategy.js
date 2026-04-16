// js/strategy.js — Strategy file editor (CodeMirror) + Monitor view
// Depends on: socket, poiData, macros, showToast, setText, escHtml, poiCategory

'use strict';

// ── Strategy editor state ──────────────────────────────────────────────────
let activeStrategyFile = 'match.py';
let strategyEditor = null;

function initCodeMirror() {
  if (typeof CodeMirror === 'undefined') return;
  const ta = document.getElementById('strategy-code');
  if (!ta || strategyEditor) return;
  strategyEditor = CodeMirror.fromTextArea(ta, {
    mode: 'python', theme: 'eclipse', lineNumbers: true,
    indentUnit: 4, indentWithTabs: false,
    extraKeys: { 'Ctrl-S': () => saveStrategy(), 'Ctrl-Enter': () => saveAndActivate() },
  });
  const scroller = strategyEditor.getScrollerElement();
  scroller.style.height = '100%';
  strategyEditor.refresh();
}

function refreshStrategyList() {
  fetch('/api/strategies').then(r => r.json()).then(files => {
    const list = document.getElementById('strategy-file-list');
    if (!list) return;
    list.innerHTML = files.map(f => `
      <div class="file-item${f === activeStrategyFile ? ' active' : ''}" onclick="loadStrategyFile('${f}')">
        <span class="file-item-name">📄 ${escHtml(f)}</span>
        <span class="file-item-actions">
          <button class="btn-tiny" onclick="event.stopPropagation();renameStrategyFile('${escHtml(f)}')">✏</button>
          ${f !== 'match.py' ? `<button class="btn-tiny red" onclick="event.stopPropagation();deleteStrategyFile('${escHtml(f)}')">🗑</button>` : ''}
        </span>
      </div>`).join('');
  });
  renderMacroRefsInStrategy();
  renderPoiRefs();
}

function loadStrategyFile(name) {
  activeStrategyFile = name;
  fetch(`/api/strategy/${encodeURIComponent(name)}`).then(r => r.json()).then(d => {
    if (strategyEditor) strategyEditor.setValue(d.content || '');
    else { const ta = document.getElementById('strategy-code'); if (ta) ta.value = d.content || ''; }
    setText('strategy-filename', name);
    const delBtn = document.getElementById('del-strat-btn');
    if (delBtn) delBtn.disabled = (name === 'match.py');
    refreshStrategyList();
    setStrategyStatus('');
  });
}

function saveStrategy() {
  const content = strategyEditor ? strategyEditor.getValue() : (document.getElementById('strategy-code')?.value || '');
  fetch(`/api/strategy/${encodeURIComponent(activeStrategyFile)}`, {
    method: 'PUT', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({content})
  }).then(() => setStrategyStatus('Saved ✓', 'ok'));
}

function saveAndActivate() {
  saveStrategy();
  socket.emit('activate_strategy', {name: activeStrategyFile});
  setStrategyStatus('Saved & activated ✓', 'ok');
}

function newStrategy() {
  const name = prompt('Strategy filename (without .py):');
  if (!name) return;
  fetch('/api/strategies', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({name})
  }).then(r => r.json()).then(d => { if (d.ok) { refreshStrategyList(); loadStrategyFile(d.name); } });
}

function renameStrategy() { renameStrategyFile(activeStrategyFile); }
function renameStrategyFile(name) {
  const newName = prompt(`Rename "${name}" to:`, name.replace('.py', ''));
  if (!newName) return;
  fetch(`/api/strategy/${encodeURIComponent(name)}/rename`, {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({to: newName})
  }).then(r => r.json()).then(d => {
    if (d.ok) { if (activeStrategyFile === name) activeStrategyFile = d.name; refreshStrategyList(); }
  });
}

function deleteStrategy() { deleteStrategyFile(activeStrategyFile); }
function deleteStrategyFile(name) {
  if (!confirm(`Delete "${name}"?`)) return;
  fetch(`/api/strategy/${encodeURIComponent(name)}`, {method: 'DELETE'}).then(() => {
    if (activeStrategyFile === name) { activeStrategyFile = 'match.py'; loadStrategyFile('match.py'); }
    else refreshStrategyList();
  });
}

function setStrategyStatus(msg, cls) {
  const el = document.getElementById('strategy-status');
  if (!el) return; el.textContent = msg; el.className = 'editor-status ' + (cls || '');
  if (msg) setTimeout(() => { el.textContent = ''; el.className = 'editor-status'; }, 3000);
}

function renderMacroRefsInStrategy() {
  const el = document.getElementById('macro-refs'); if (!el) return;
  if (!macros.length) { el.innerHTML = '<span class="file-list-empty">No macros yet</span>'; return; }
  el.innerHTML = macros.map(m => `
    <div class="file-item" onclick="insertMacroCall('${escHtml(m.name)}')">
      <span class="file-item-name">⚡ ${escHtml(m.name)}</span>
    </div>`).join('');
}

function insertMacroCall(name) {
  const call = `    run_macro_${name}(brain)  # TODO: define this macro\n`;
  if (strategyEditor) { const cur = strategyEditor.getCursor(); strategyEditor.replaceRange(call, cur); }
  else { const ta = document.getElementById('strategy-code'); if (ta) ta.value += call; }
  setStrategyStatus('Macro call inserted', 'ok');
}

function renderPoiRefs() {
  const el = document.getElementById('poi-refs'); if (!el) return;
  el.innerHTML = poiData.map(p => {
    const cat = poiCategory(p.name);
    return `<div class="poi-ref-item" onclick="insertPoiRef('${p.name}')">
      <span class="poi-ref-dot" style="background:${cat.color}"></span>
      <span>${p.name}</span>
      <span style="color:var(--text-dim);margin-left:auto;font-size:10px">(${Math.round(p.x)},${Math.round(p.y)})</span>
    </div>`;
  }).join('');
}

function insertPoiRef(name) {
  const ref = `POI.${name}`;
  if (strategyEditor) { const cur = strategyEditor.getCursor(); strategyEditor.replaceRange(ref, cur); }
  else { const ta = document.getElementById('strategy-code'); if (ta) { const pos = ta.selectionStart; ta.value = ta.value.slice(0, pos) + ref + ta.value.slice(pos); } }
}

// ── Monitor view ───────────────────────────────────────────────────────────
function updateMonitor(s) {
  const r = s.robot;
  setText('mon-x',     r.x.toFixed(0));
  setText('mon-y',     r.y.toFixed(0));
  setText('mon-theta', (r.theta * 180 / Math.PI).toFixed(1) + '°');
  setText('mon-speed', Math.hypot(r.vx, r.vy).toFixed(0));
  setText('mon-vx',    r.vx.toFixed(0));
  setText('mon-vy',    r.vy.toFixed(0));
  setText('mon-omega', (r.omega ? (r.omega * 180 / Math.PI).toFixed(1) : '—'));
  const ms = document.getElementById('mon-mstate');
  if (ms) { ms.textContent = s.motion.state; ms.className = 'mon-val ' + (MOTION_CLASS[s.motion.state] || ''); }
  const left = s.chrono.left, total = left + s.chrono.elapsed;
  setText('mon-time',  left.toFixed(1) + ' s');
  setText('mon-score', s.score + ' pts');
  const pct = total > 0 ? left / total * 100 : 100;
  const cb = document.getElementById('mon-chrono-bar');
  if (cb) { cb.style.width = pct + '%'; cb.style.background = left < 10 ? '#c0392b' : left < 30 ? '#b7770d' : '#2980b9'; }
  const safe = s.safety.enabled && s.safety.detected;
  const sf = document.getElementById('mon-safety');
  if (sf) { sf.textContent = safe ? '⚠ OBSTACLE' : 'OK'; sf.style.color = safe ? '#c0392b' : '#1a8c3c'; }
  setText('mon-obs-detected', safe ? 'Yes' : 'No');
  setText('mon-collision',    r.collided ? 'Yes' : 'No');

  const ledHb     = document.getElementById('led-heartbeat');
  const ledSafety = document.getElementById('led-safety');
  const ledMotion = document.getElementById('led-motion');
  const ledOtos   = document.getElementById('led-otos');
  if (ledHb) {
    if (s.mode === 'sim')       ledHb.style.background = '#555';
    else if (s.hw_heartbeat_ok) ledHb.style.background = '#1a8c3c';
    else                        ledHb.style.background = '#c0392b';
  }
  if (ledSafety) {
    if (!s.safety.enabled)      ledSafety.style.background = '#555';
    else if (s.safety.detected) ledSafety.style.background = '#c0392b';
    else                        ledSafety.style.background = '#1a8c3c';
  }
  if (ledMotion) {
    const ms2 = s.motion.state;
    if (ms2 === 'RUNNING')      ledMotion.style.background = '#1a8c3c';
    else if (ms2 === 'PAUSED')  ledMotion.style.background = '#b7770d';
    else                        ledMotion.style.background = '#555';
  }
  if (ledOtos) {
    if (s.mode === 'sim')       ledOtos.style.background = '#1a8c3c';
    else if (s.hw_mode)         ledOtos.style.background = '#1a8c3c';
    else                        ledOtos.style.background = '#555';
  }

  const go = document.getElementById('mon-game-objs');
  if (go) go.innerHTML = s.game_objs.filter(o => o.color !== 'UNKNOWN' && o.color !== 'NONE').map(o =>
    `<span style="display:inline-block;width:12px;height:12px;border-radius:50%;background:${COLOR_HEX[o.color] || '#888'};border:1px solid rgba(0,0,0,.2)"></span>`
  ).join('');

  const logEl = document.getElementById('mon-event-log');
  if (logEl && s.log && s.log.length) {
    const colored = s.log.slice(-20).map(line => {
      let color = '#c8ccd4';
      if (line.includes('FAILED') || line.includes('error') || line.includes('ERROR')) color = '#e06c75';
      else if (line.includes('DETECT') || line.includes('OBSTACLE') || line.includes('CLEAR')) color = '#e5c07b';
      else if (line.includes('OK') || line.includes('loaded') || line.includes('connected')) color = '#98c379';
      return `<div style="color:${color}">${line.replace(/</g, '&lt;')}</div>`;
    }).join('');
    logEl.innerHTML = colored;
    logEl.scrollTop = logEl.scrollHeight;
  }
}
