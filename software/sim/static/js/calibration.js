// js/calibration.js — Calibration wizard & manual tuning
// Depends on: socket, showToast

'use strict';

// ── State ──────────────────────────────────────────────────────────────────────
let _calibDirty = {};
let _calibData  = {
  cx: 1.089, cy: -1.089, cr: 0.831,
  ha: 1.0,   hb: 1.0,   hc: 1.0,
  ol: 0.9714, oa: 1.0,
};

let _calibStatusTimer = null;

let _calibWiz = {
  axis: 'x',
  commanded: 0,
  otosMeasured: 0,
  unit: 'mm',
  suggested: null,
};

// ── Init ───────────────────────────────────────────────────────────────────────
function calibInit() {
  fetch('/api/calibration')
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        _calibData = Object.assign(_calibData, d.calib);
        calibRefreshUI(d.connected);
      }
    })
    .catch(() => {});
}

// ── Socket ─────────────────────────────────────────────────────────────────────
socket.on('calib_updated', data => {
  if (data && data.calib) {
    _calibData = Object.assign(_calibData, data.calib);
    _calibDirty = {};
    calibRefreshInputs();
    _calibStatusMsg('✓ Calibration synchronisée depuis le robot');
  }
});

// ── Refresh UI ─────────────────────────────────────────────────────────────────
function calibRefreshInputs() {
  const fields = ['cx','cy','cr','ha','hb','hc','ol','oa'];
  fields.forEach(k => {
    const el = document.getElementById(`calib-${k}`);
    if (el && document.activeElement !== el) {
      el.value = _calibData[k];
      el.classList.remove('dirty');
    }
  });
}

function calibRefreshUI(connected) {
  calibRefreshInputs();
  const dot  = document.getElementById('calib-banner-dot');
  const text = document.getElementById('calib-banner-text');
  if (!dot || !text) return;
  if (connected) {
    dot.style.background = '#22c55e';
    text.textContent = 'Robot connecté — les commandes sont envoyées en temps réel';
  } else {
    dot.style.background = '#6b7280';
    text.textContent = 'Non connecté — les valeurs seront appliquées à la connexion';
  }
}

// ── Mark dirty ─────────────────────────────────────────────────────────────────
function calibMarkDirty(group) {
  _calibDirty[group] = true;
  const groups = {
    cart:   ['cx','cy','cr'],
    holo:   ['ha','hb','hc'],
    otos_l: ['ol'],
    otos_a: ['oa'],
  };
  (groups[group] || []).forEach(k => {
    const el = document.getElementById(`calib-${k}`);
    if (el) el.classList.add('dirty');
  });
}

// ── Apply helpers ──────────────────────────────────────────────────────────────
function _calibPost(payload) {
  return fetch('/api/calibration', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(payload),
  })
  .then(r => r.json())
  .then(d => {
    if (d.ok) {
      _calibData = Object.assign(_calibData, d.calib);
      _calibStatusMsg('✓ Appliqué');
    } else {
      _calibStatusMsg('✗ Erreur: ' + (d.error || '?'));
    }
    return d;
  });
}

function _calibGetFloat(id) {
  const el = document.getElementById(id);
  return el ? parseFloat(el.value) : NaN;
}

function calibApplyCart() {
  const cx = _calibGetFloat('calib-cx');
  const cy = _calibGetFloat('calib-cy');
  const cr = _calibGetFloat('calib-cr');
  if (isNaN(cx) || isNaN(cy) || isNaN(cr)) return;
  _calibPost({ cx, cy, cr }).then(() => {
    ['cx','cy','cr'].forEach(k => {
      const el = document.getElementById(`calib-${k}`);
      if (el) el.classList.remove('dirty');
    });
    delete _calibDirty.cart;
  });
}

function calibApplyHolo() {
  const ha = _calibGetFloat('calib-ha');
  const hb = _calibGetFloat('calib-hb');
  const hc = _calibGetFloat('calib-hc');
  if (isNaN(ha) || isNaN(hb) || isNaN(hc)) return;
  _calibPost({ ha, hb, hc }).then(() => {
    ['ha','hb','hc'].forEach(k => {
      const el = document.getElementById(`calib-${k}`);
      if (el) el.classList.remove('dirty');
    });
    delete _calibDirty.holo;
  });
}

function calibApplyOtosLinear() {
  const ol = _calibGetFloat('calib-ol');
  if (isNaN(ol)) return;
  if (ol < 0.872 || ol > 1.127) {
    _calibStatusMsg('⚠ Valeur hors plage OTOS (0.872–1.127)');
    return;
  }
  _calibPost({ ol }).then(() => {
    const el = document.getElementById('calib-ol');
    if (el) el.classList.remove('dirty');
    delete _calibDirty.otos_l;
  });
}

function calibApplyOtosAngular() {
  const oa = _calibGetFloat('calib-oa');
  if (isNaN(oa)) return;
  if (oa < 0.872 || oa > 1.127) {
    _calibStatusMsg('⚠ Valeur hors plage OTOS (0.872–1.127)');
    return;
  }
  _calibPost({ oa }).then(() => {
    const el = document.getElementById('calib-oa');
    if (el) el.classList.remove('dirty');
    delete _calibDirty.otos_a;
  });
}

function calibApplyAll() {
  const cx = _calibGetFloat('calib-cx'), cy = _calibGetFloat('calib-cy'), cr = _calibGetFloat('calib-cr');
  const ha = _calibGetFloat('calib-ha'), hb = _calibGetFloat('calib-hb'), hc = _calibGetFloat('calib-hc');
  const ol = _calibGetFloat('calib-ol'), oa = _calibGetFloat('calib-oa');
  _calibPost({ cx, cy, cr, ha, hb, hc, ol, oa }).then(() => {
    _calibDirty = {};
    document.querySelectorAll('.calib-input').forEach(el => el.classList.remove('dirty'));
  });
}

// ── Server-side save / load operations ─────────────────────────────────────────
function calibSave() {
  fetch('/api/calibration/save', { method: 'POST' })
    .then(r => r.json())
    .then(d => _calibStatusMsg(d.ok ? '💾 Sauvegardé (serveur)' : '✗ Erreur: ' + d.error));
}

function calibLoad() {
  fetch('/api/calibration/load', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        if (d.calib) {
          _calibData = Object.assign(_calibData, d.calib);
          _calibDirty = {};
          calibRefreshInputs();
        }
        _calibStatusMsg('📂 Chargé et poussé vers firmware');
      } else {
        _calibStatusMsg('✗ Erreur: ' + d.error);
      }
    });
}

function calibReset() {
  if (!confirm('Remettre tous les paramètres aux valeurs par défaut ?')) return;
  fetch('/api/calibration/reset', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.ok && d.calib) {
        _calibData = Object.assign(_calibData, d.calib);
        _calibDirty = {};
        calibRefreshInputs();
        _calibStatusMsg('↺ Réinitialisé aux défauts');
      }
    });
}

// ── Wizard ─────────────────────────────────────────────────────────────────────
function _calibWizHideAll() {
  for (const id of ['calib-wiz-measure', 'calib-wiz-suggest', 'calib-wiz-error']) {
    const el = document.getElementById(id);
    if (el) el.classList.add('hidden');
  }
}

function _calibWizShowError(msg) {
  const el = document.getElementById('calib-wiz-error');
  if (el) { el.textContent = '✗ ' + msg; el.classList.remove('hidden'); }
}

document.addEventListener('DOMContentLoaded', () => {
  const sel = document.getElementById('calib-wiz-axis');
  if (sel) sel.addEventListener('change', () => {
    const lbl  = document.getElementById('calib-wiz-distlbl');
    const dist = document.getElementById('calib-wiz-dist');
    if (sel.value === 'theta') {
      if (lbl)  lbl.textContent  = 'Angle (°)';
      if (dist) { dist.value = 90; dist.min = 5; dist.max = 720; }
    } else {
      if (lbl)  lbl.textContent  = 'Distance (mm)';
      if (dist) { dist.value = 500; dist.min = 50; dist.max = 1500; }
    }
  });
});

function calibWizStart() {
  const axis = document.getElementById('calib-wiz-axis').value;
  const val  = parseFloat(document.getElementById('calib-wiz-dist').value);
  if (isNaN(val) || val <= 0) { _calibWizShowError('Valeur invalide'); return; }
  _calibWiz.axis      = axis;
  _calibWiz.commanded = val;
  _calibWiz.unit      = (axis === 'theta') ? '°' : 'mm';
  _calibWizHideAll();

  const btn = document.getElementById('btn-calib-wiz-start');
  btn.disabled = true;
  btn.textContent = '⏳ Déplacement...';

  const isTurn = (axis === 'theta');
  const url    = isTurn ? '/api/calibration/open_turn' : '/api/calibration/open_move';
  const body   = isTurn ? { angle_deg: val } : { dist_mm: val, axis };

  const abort   = new AbortController();
  const abortMs = 45000;
  const abortId = setTimeout(() => abort.abort(), abortMs);
  const resetBtn = () => { btn.disabled = false; btn.textContent = '▶ Déplacer'; };

  fetch(url, {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(body),
    signal: abort.signal,
  })
  .then(r => r.json())
  .then(d => {
    clearTimeout(abortId);
    resetBtn();
    if (!d.ok) {
      const raw = d.raw ? ` — raw: ${d.raw}` : '';
      _calibWizShowError((d.error || 'Échec du déplacement') + raw);
      return;
    }
    const otos = isTurn ? d.otos_dth_deg : d.otos_dist;
    _calibWiz.otosMeasured = otos;
    document.getElementById('calib-wiz-cmd').textContent  = `${val.toFixed(2)} ${_calibWiz.unit}`;
    document.getElementById('calib-wiz-otos').textContent = `${otos.toFixed(2)} ${_calibWiz.unit}`;
    const actualEl = document.getElementById('calib-wiz-actual');
    actualEl.value = '';
    actualEl.focus();
    document.getElementById('calib-wiz-measure').classList.remove('hidden');
    _calibStatusMsg(`✓ Déplacement ${axis} terminé`);
  })
  .catch(e => {
    clearTimeout(abortId);
    resetBtn();
    if (e.name === 'AbortError') {
      _calibWizShowError(`Timeout client (${abortMs/1000}s). Le robot est peut-être bloqué — vérifie son état et relance.`);
    } else {
      _calibWizShowError(e.message);
    }
  });
}

function calibWizCompute() {
  const actual = parseFloat(document.getElementById('calib-wiz-actual').value);
  if (isNaN(actual) || actual <= 0) { _calibWizShowError('Mesure réelle invalide'); return; }
  fetch('/api/calibration/compute', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      axis:          _calibWiz.axis,
      commanded:     _calibWiz.commanded,
      otos_measured: _calibWiz.otosMeasured,
      actual:        actual,
    }),
  })
  .then(r => r.json())
  .then(d => {
    if (!d.ok) { _calibWizShowError(d.error || 'Calcul échoué'); return; }
    _calibWiz.suggested = d;
    const lines = [];
    lines.push(`Axe: ${d.axis}`);
    lines.push(`Commandé     : ${d.commanded} ${_calibWiz.unit}`);
    lines.push(`OTOS mesuré  : ${d.otos_measured} ${_calibWiz.unit}`);
    lines.push(`Réel (règle) : ${d.actual} ${_calibWiz.unit}`);
    lines.push('');
    lines.push(`Motion ratio : ${d.motion_ratio}`);
    lines.push(`OTOS ratio   : ${d.otos_ratio}`);
    lines.push('');
    lines.push('Corrections proposées:');
    for (const k of Object.keys(d.suggested)) {
      const oldV = d.current[k];
      const newV = d.suggested[k];
      const pct  = ((newV - oldV) / oldV * 100).toFixed(2);
      lines.push(`  ${k}: ${oldV} → ${newV}  (${pct >= 0 ? '+' : ''}${pct}%)`);
    }
    document.getElementById('calib-wiz-suggest-body').textContent = lines.join('\n');
    const warnEl = document.getElementById('calib-wiz-warnings');
    warnEl.textContent = (d.warnings && d.warnings.length) ? '⚠ ' + d.warnings.join(' · ') : '';
    document.getElementById('calib-wiz-suggest').classList.remove('hidden');
    document.getElementById('calib-wiz-measure').classList.add('hidden');
  })
  .catch(e => _calibWizShowError(e.message));
}

function calibWizApply() {
  if (!_calibWiz.suggested || !_calibWiz.suggested.suggested) return;
  const payload = Object.assign({}, _calibWiz.suggested.suggested);
  fetch('/api/calibration', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(payload),
  })
  .then(r => r.json())
  .then(d => {
    if (d.ok) {
      _calibData = Object.assign(_calibData, d.calib || {});
      calibRefreshInputs();
      _calibStatusMsg('✓ Calibration appliquée — pensez à sauvegarder');
      calibWizCancel();
    } else {
      _calibWizShowError(d.error || 'Application échouée');
    }
  })
  .catch(e => _calibWizShowError(e.message));
}

function calibWizCancel() {
  _calibWiz.suggested = null;
  _calibWizHideAll();
}

// ── Status message ─────────────────────────────────────────────────────────────
function _calibStatusMsg(msg) {
  const el = document.getElementById('calib-status-msg');
  if (!el) return;
  el.textContent = msg;
  clearTimeout(_calibStatusTimer);
  _calibStatusTimer = setTimeout(() => { el.textContent = ''; }, 5000);
}

// ── Probe / Recalage ──────────────────────────────────────────────────────────
function probeStart() {
  const wall      = document.getElementById('probe-wall').value;
  const face      = document.getElementById('probe-face').value;
  const clearance = parseFloat(document.getElementById('probe-clearance').value) || 100;

  const btn = document.getElementById('btn-probe-start');
  const resEl = document.getElementById('probe-result');
  const errEl = document.getElementById('probe-error');
  resEl.classList.add('hidden');
  errEl.classList.add('hidden');
  btn.disabled = true;
  btn.textContent = '⏳ Approche...';

  const abort   = new AbortController();
  const abortMs = 50000;
  const abortId = setTimeout(() => abort.abort(), abortMs);
  const resetBtn = () => { btn.disabled = false; btn.textContent = '⚡ Lancer'; };

  fetch('/api/calibration/probe', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ wall, face, clearance }),
    signal: abort.signal,
  })
  .then(r => r.json())
  .then(d => {
    clearTimeout(abortId);
    resetBtn();
    if (!d.ok) {
      errEl.textContent = '✗ ' + (d.error || 'Probe failed');
      errEl.classList.remove('hidden');
      return;
    }
    const thetaDeg = (d.theta * 180 / Math.PI).toFixed(1);
    resEl.innerHTML =
      `<strong>Position corrigée</strong><br>` +
      `X = ${parseFloat(d.x).toFixed(1)} mm &nbsp; Y = ${parseFloat(d.y).toFixed(1)} mm &nbsp; θ = ${thetaDeg}°<br>` +
      `<span style="color:var(--text-dim)">Mur: ${d.wall} &nbsp; Face: ${d.face}</span>`;
    resEl.classList.remove('hidden');
    _calibStatusMsg(`✓ Recalage ${wall} terminé`);
  })
  .catch(e => {
    clearTimeout(abortId);
    resetBtn();
    if (e.name === 'AbortError') {
      errEl.textContent = `✗ Timeout (${abortMs/1000}s). Le robot est peut-être bloqué.`;
    } else {
      errEl.textContent = '✗ ' + e.message;
    }
    errEl.classList.remove('hidden');
  });
}

// ── Stall Probe (stall-detection calibration) ─────────────────────────────────
function stallProbeStart() {
  const wall       = document.getElementById('stall-probe-wall').value;
  const face       = document.getElementById('stall-probe-face').value;
  const clearance  = parseFloat(document.getElementById('stall-probe-clearance').value) || 200;
  const degagement = parseFloat(document.getElementById('stall-probe-degagement').value) || 80;

  const btn   = document.getElementById('btn-stall-probe-start');
  const resEl = document.getElementById('stall-probe-result');
  const errEl = document.getElementById('stall-probe-error');
  resEl.classList.add('hidden');
  errEl.classList.add('hidden');
  btn.disabled = true;
  btn.textContent = '⏳ Approche...';

  const abort   = new AbortController();
  const abortMs = 50000;
  const abortId = setTimeout(() => abort.abort(), abortMs);
  const resetBtn = () => { btn.disabled = false; btn.textContent = '⚡ Lancer'; };

  fetch('/api/calibration/stall_probe', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ wall, face, clearance, degagement }),
    signal: abort.signal,
  })
  .then(r => r.json())
  .then(d => {
    clearTimeout(abortId);
    resetBtn();
    if (!d.ok) {
      errEl.textContent = '✗ ' + (d.error || 'Stall probe failed');
      errEl.classList.remove('hidden');
      return;
    }
    const thetaDeg = (d.theta * 180 / Math.PI).toFixed(1);
    const stalledTxt = d.stalled ? '✓ Stall détecté' : '✗ Pas de stall';
    resEl.innerHTML =
      `<strong>Position corrigée</strong><br>` +
      `X = ${parseFloat(d.x).toFixed(1)} mm &nbsp; Y = ${parseFloat(d.y).toFixed(1)} mm &nbsp; θ = ${thetaDeg}°<br>` +
      `<strong>Stall</strong>: ${stalledTxt}<br>` +
      `Durée: ${d.dur_ms} ms &nbsp; Distance parcourue: ${parseFloat(d.travel_mm).toFixed(1)} mm &nbsp; Min trans: ${parseFloat(d.stall_min_trans).toFixed(2)} mm<br>` +
      `<span style="color:var(--text-dim)">Mur: ${d.wall} &nbsp; Face: ${d.face}</span>`;
    resEl.classList.remove('hidden');
    _calibStatusMsg(`✓ Stall probe ${wall} terminé — ${stalledTxt}`);
  })
  .catch(e => {
    clearTimeout(abortId);
    resetBtn();
    if (e.name === 'AbortError') {
      errEl.textContent = `✗ Timeout (${abortMs/1000}s). Le robot est peut-être bloqué.`;
    } else {
      errEl.textContent = '✗ ' + e.message;
    }
    errEl.classList.remove('hidden');
  });
}

// ── Stall detection parameter tuning ──────────────────────────────────────────
function stallPushParams() {
  const params = [
    { id: 'stall-vel-cmd-min',  key: 'stall.vel_cmd_min'  },
    { id: 'stall-vel-otos-max', key: 'stall.vel_otos_max' },
    { id: 'stall-vel-time',     key: 'stall.vel_time'     },
    { id: 'stall-vel-rot-min',  key: 'stall.vel_rot_min'  },
    { id: 'stall-vel-rot-max',  key: 'stall.vel_rot_max'  },
    { id: 'stall-delay-ms',     key: 'stall.delay_ms'     },
    { id: 'stall-period-ms',    key: 'stall.period_ms'    },
    { id: 'stall-trans-mm',     key: 'stall.trans_mm'     },
    { id: 'stall-angle-rad',    key: 'stall.angle_rad'    },
  ];

  const statusEl = document.getElementById('stall-params-status');
  let sent = 0, failed = 0;

  const promises = params.map(p => {
    const el = document.getElementById(p.id);
    if (!el) return Promise.resolve();
    const val = el.value;
    return fetch('/api/settings', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ key: p.key, value: val }),
    })
    .then(r => r.json())
    .then(d => { if (d.ok && d.pushed) sent++; else failed++; })
    .catch(() => { failed++; });
  });

  Promise.all(promises).then(() => {
    if (failed === 0) {
      statusEl.textContent = `✓ ${sent} paramètres envoyés`;
      statusEl.style.color = '#22c55e';
    } else {
      statusEl.textContent = `⚠ ${sent} envoyés, ${failed} échoués`;
      statusEl.style.color = '#c0392b';
    }
    setTimeout(() => { statusEl.textContent = ''; }, 4000);
  });
}

// ── Global motion protection toggles ─────────────────────────────────────────
function motionProtectionToggle() {
  const snap = document.getElementById('chk-border-snap').checked ? 1 : 0;
  const coll = document.getElementById('chk-collision').checked  ? 1 : 0;
  const statusEl = document.getElementById('protection-status');

  const promises = [
    fetch('/api/settings', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ key: 'motion.border_snap', value: snap }),
    }),
    fetch('/api/settings', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ key: 'motion.collision', value: coll }),
    }),
  ];

  Promise.all(promises)
    .then(rs => Promise.all(rs.map(r => r.json())))
    .then(ds => {
      const ok = ds.every(d => d.ok && d.pushed);
      statusEl.textContent = ok ? '✓ envoyé' : '⚠ erreur push';
      statusEl.style.color = ok ? '#22c55e' : '#c0392b';
      setTimeout(() => { statusEl.textContent = ''; }, 3000);
    })
    .catch(() => {
      statusEl.textContent = '⚠ erreur réseau';
      statusEl.style.color = '#c0392b';
    });
}

// ── View activation ────────────────────────────────────────────────────────────
function onCalibViewActivated() {
  calibInit();
}
