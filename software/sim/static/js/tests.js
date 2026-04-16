// js/tests.js — Hardware test runner view
// Depends on: socket, showToast

'use strict';

// ── State ──────────────────────────────────────────────────────────────────
let _testCatalog   = {};
let _testStatus    = {};
let _testDuration  = {};
let _testRunning   = false;
let _selectedSuite = null;

const TEST_ICONS = {
  idle:'○', running:'⋯', passed:'✓', failed:'✗',
  stopped:'—', skipped:'⊖', prompt:'⏸',
};

// ── Init ───────────────────────────────────────────────────────────────────
function initTests() {
  fetch('/api/tests/catalog')
    .then(r => r.json())
    .then(catalog => {
      _testCatalog = catalog;
      Object.values(catalog).forEach(suite => {
        suite.tests.forEach(t => { _testStatus[t.id] = 'idle'; _testDuration[t.id] = null; });
      });
      _buildCatalogUI();
    })
    .catch(() => {});
}

// ── Socket listeners ───────────────────────────────────────────────────────
socket.on('test_progress', data => {
  _setTestStatus(data.id, data.status);
  if (data.status === 'running') _logEntry(data.id, 'running', '…', null);
});

socket.on('test_result', data => {
  const status = data.passed ? 'passed' : 'failed';
  _setTestStatus(data.id, status, data.duration_ms);
  _logEntry(data.id, status, data.msg, data.duration_ms);
  _updateSummary();
});

socket.on('test_done', () => {
  _testRunning = false; _updateButtons(); _updateSummary();
});

socket.on('tests_catalog_changed', data => {
  _testStatus = {}; _testDuration = {}; _testCatalog = {}; _testRunning = false;
  _updateButtons();
  if (data && data.mode === 'idle') { clearTestLog(); _logHeader('⚠ Connexion perdue — reconnectez le robot'); }
  initTests();
});

socket.on('test_prompt', data => { _showTestPrompt(data.msg || ''); });

// ── Catalog UI ─────────────────────────────────────────────────────────────
function _buildCatalogUI() {
  const container = document.getElementById('tests-catalog'); if (!container) return;
  container.innerHTML = '';
  if (Object.keys(_testCatalog).length === 0) {
    container.innerHTML = `<div style="padding:32px 16px;text-align:center;color:var(--text-dim);font-size:12px;line-height:1.7">
      <div style="font-size:2rem;margin-bottom:10px">🔌</div>
      <div style="font-weight:600;margin-bottom:6px">Robot non connecté</div>
      <div>Connectez-vous au robot (USB ou XBee)<br>pour accéder aux tests matériels.</div>
    </div>`;
    ['btn-run-all','btn-run-suite'].forEach(id => { const b = document.getElementById(id); if (b) b.disabled = true; });
    return;
  }
  Object.entries(_testCatalog).forEach(([suiteId, suite]) => {
    const suiteEl = document.createElement('div');
    suiteEl.className = 'test-suite'; suiteEl.id = `test-suite-${suiteId}`;
    const hdr = document.createElement('div'); hdr.className = 'test-suite-header';
    hdr.innerHTML = `<span class="test-suite-icon">${suite.icon}</span><span>${suite.label}</span>
      <button class="test-suite-run" title="Run ${suite.label}" onclick="testRunSuite('${suiteId}',event)">▶</button>`;
    hdr.addEventListener('click', e => { if (e.target.classList.contains('test-suite-run')) return; _selectSuite(suiteId); });
    suiteEl.appendChild(hdr);
    suite.tests.forEach(t => {
      const item = document.createElement('div');
      item.className = 'test-item idle'; item.id = `test-item-${t.id}`; item.title = t.desc;
      item.innerHTML = `<span class="test-item-icon idle" id="test-icon-${t.id}">${TEST_ICONS.idle}</span>
        <span class="test-item-name">${t.name}</span><span class="test-item-dur" id="test-dur-${t.id}"></span>`;
      item.addEventListener('click', () => testRunOne(t.id));
      suiteEl.appendChild(item);
    });
    container.appendChild(suiteEl);
  });
}

function _selectSuite(suiteId) {
  _selectedSuite = suiteId;
  document.querySelectorAll('.test-suite-header').forEach(h => h.classList.remove('selected'));
  const hdr = document.querySelector(`#test-suite-${suiteId} .test-suite-header`);
  if (hdr) hdr.classList.add('selected');
}

// ── Status updates ─────────────────────────────────────────────────────────
function _setTestStatus(id, status, durMs) {
  _testStatus[id] = status;
  if (durMs != null) _testDuration[id] = durMs;
  const item = document.getElementById(`test-item-${id}`);
  const icon = document.getElementById(`test-icon-${id}`);
  const dur  = document.getElementById(`test-dur-${id}`);
  if (item) item.className = `test-item ${status}`;
  if (icon) { icon.className = `test-item-icon ${status}`; icon.textContent = TEST_ICONS[status] || '?'; }
  if (dur && durMs != null) dur.textContent = durMs < 1000 ? `${durMs}ms` : `${(durMs/1000).toFixed(1)}s`;
}

// ── Log ─────────────────────────────────────────────────────────────────────
function _logEntry(id, status, msg, durMs) {
  const log = document.getElementById('tests-log'); if (!log) return;
  const existing = document.getElementById(`tlog-${id}`); if (existing) existing.remove();
  const entry = document.createElement('div');
  entry.className = `tlog-entry ${status}`; entry.id = `tlog-${id}`;
  const icon = { running:'⋯', passed:'✓', failed:'✗', stopped:'—', skipped:'⊖' }[status] || '?';
  const durStr = durMs != null ? (durMs < 1000 ? `${durMs}ms` : `${(durMs/1000).toFixed(1)}s`) : '';
  const meta = _findTest(id);
  const name = meta ? meta.name : id;
  entry.innerHTML = `<span class="tlog-icon">${icon}</span><span class="tlog-id">${name}</span>
    <span class="tlog-msg">${_escHtmlTest(msg)}</span><span class="tlog-dur">${durStr}</span>`;
  log.appendChild(entry); log.scrollTop = log.scrollHeight;
}

function _logHeader(text) {
  const log = document.getElementById('tests-log'); if (!log) return;
  const entry = document.createElement('div');
  entry.className = 'tlog-entry header'; entry.textContent = text;
  log.appendChild(entry); log.scrollTop = log.scrollHeight;
}

function clearTestLog() { const log = document.getElementById('tests-log'); if (log) log.innerHTML = ''; }

// ── Prompt overlay ─────────────────────────────────────────────────────────
function _showTestPrompt(msg) {
  let overlay = document.getElementById('test-prompt-overlay');
  if (!overlay) {
    overlay = document.createElement('div'); overlay.id = 'test-prompt-overlay';
    overlay.style.cssText = 'position:fixed;inset:0;z-index:9999;display:flex;align-items:center;justify-content:center;background:rgba(0,0,0,0.60);backdrop-filter:blur(4px)';
    document.body.appendChild(overlay);
  }
  let title = '', body = _escHtmlTest(msg);
  const stepMatch = msg.match(/^(ÉTAPE\s+\d+\/\d+\s*[—\-–]\s*[^\n]+)\n+([\s\S]*)$/);
  if (stepMatch) { title = _escHtmlTest(stepMatch[1]); body = _escHtmlTest(stepMatch[2].trim()); }
  overlay.innerHTML = `<div style="background:#1a2235;border:1px solid #3a4a65;border-radius:12px;padding:28px 32px;max-width:520px;width:92%;box-shadow:0 12px 48px rgba(0,0,0,0.7);text-align:center">
    <div style="font-size:2rem;margin-bottom:10px">⏸</div>
    ${title ? `<div style="font-size:0.85rem;font-weight:700;letter-spacing:.06em;text-transform:uppercase;color:#7eb8e8;margin-bottom:14px">${title}</div>` : ''}
    <div style="font-size:0.93rem;color:#d8e4f2;margin-bottom:26px;line-height:1.65;white-space:pre-wrap;text-align:left">${body}</div>
    <button onclick="_testPromptAck()" style="background:#2980b9;color:#fff;border:none;border-radius:6px;padding:10px 32px;font-size:0.95rem;font-weight:600;cursor:pointer">Continuer ▶</button>
  </div>`;
  overlay.style.display = 'flex';
  _logHeader('⏸ ' + (title || msg).split('\n')[0]);
}

function _testPromptAck() {
  const overlay = document.getElementById('test-prompt-overlay');
  if (overlay) overlay.style.display = 'none';
  socket.emit('test_prompt_ack');
  _logHeader('▶ Continuer');
}

// ── Summary badge ──────────────────────────────────────────────────────────
function _updateSummary() {
  const badge = document.getElementById('test-summary'); if (!badge) return;
  const passed = Object.values(_testStatus).filter(s => s === 'passed').length;
  const failed = Object.values(_testStatus).filter(s => s === 'failed').length;
  const ran = passed + failed;
  if (ran === 0) { badge.textContent = ''; badge.style.color = ''; }
  else { badge.textContent = `${passed}/${ran}`; badge.style.color = failed > 0 ? 'var(--red)' : 'var(--green)'; }
}

function _updateButtons() {
  const btnRunAll   = document.getElementById('btn-run-all');
  const btnRunSuite = document.getElementById('btn-run-suite');
  const btnStop     = document.getElementById('btn-stop-tests');
  if (btnRunAll)   btnRunAll.disabled   = _testRunning;
  if (btnRunSuite) btnRunSuite.disabled = _testRunning;
  if (btnStop)     btnStop.disabled     = !_testRunning;
}

// ── Public API ─────────────────────────────────────────────────────────────
function testRunAll() {
  if (_testRunning) return;
  _resetAllStatus(); _logHeader('▶ Run All'); _startRun();
  socket.emit('run_tests', {});
}

function testRunSuite(suiteId, evt) {
  if (evt) evt.stopPropagation();
  const sid = suiteId || _selectedSuite;
  if (!sid || _testRunning) return;
  _resetSuiteStatus(sid);
  const label = (_testCatalog[sid] || {}).label || sid;
  _logHeader(`▶ Suite: ${label}`); _startRun();
  socket.emit('run_tests', { suite: sid });
}

function testRunOne(testId) {
  if (_testRunning) return;
  _setTestStatus(testId, 'idle');
  const meta = _findTest(testId);
  _logHeader(`▶ ${meta ? meta.name : testId}`); _startRun();
  socket.emit('run_tests', { test: testId });
}

function testStop() { socket.emit('stop_tests'); }

// ── Helpers ────────────────────────────────────────────────────────────────
function _startRun() { _testRunning = true; _updateButtons(); }

function _resetAllStatus() {
  Object.keys(_testStatus).forEach(id => { _testStatus[id] = 'idle'; _setTestStatus(id, 'idle'); });
  _updateSummary();
}

function _resetSuiteStatus(suiteId) {
  const suite = _testCatalog[suiteId]; if (!suite) return;
  suite.tests.forEach(t => { _testStatus[t.id] = 'idle'; _setTestStatus(t.id, 'idle'); });
  _updateSummary();
}

function _findTest(id) {
  for (const suite of Object.values(_testCatalog)) {
    const t = suite.tests.find(t => t.id === id); if (t) return t;
  }
  return null;
}

// Local escHtml variant (avoids dependency on global escHtml for safety)
function _escHtmlTest(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
