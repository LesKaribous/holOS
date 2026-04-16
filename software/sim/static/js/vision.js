// js/vision.js — Vision view: camera feed, processing pipeline, object registry
// Depends on: socket, activeView

'use strict';

// ── State ──────────────────────────────────────────────────────────────────────
let _visionObjects     = [];
let _visionActiveObjId = null;
let _visionFrameInfo   = {};
let _visionEnabled     = false;

// ── Color presets (match _COLOR_RANGES_HSV in vision_backend.py) ───────────────
const _VP_COLOR_PRESETS = {
  red:    { lo_h: 0,  lo_s: 80,  lo_v: 60,  hi_h: 10,  hi_h2: 180, lo_h2: 160 },
  green:  { lo_h: 40, lo_s: 60,  lo_v: 50,  hi_h: 85,  hi_h2: -1,  lo_h2: 160 },
  blue:   { lo_h: 95, lo_s: 60,  lo_v: 50,  hi_h: 135, hi_h2: -1,  lo_h2: 160 },
  yellow: { lo_h: 20, lo_s: 80,  lo_v: 80,  hi_h: 38,  hi_h2: -1,  lo_h2: 160 },
  white:  { lo_h: 0,  lo_s: 0,   lo_v: 190, hi_h: 180, hi_h2: -1,  lo_h2: 160 },
  black:  { lo_h: 0,  lo_s: 0,   lo_v: 0,   hi_h: 180, hi_h2: -1,  lo_h2: 160 },
  brown:  { lo_h: 8,  lo_s: 50,  lo_v: 30,  hi_h: 22,  hi_h2: -1,  lo_h2: 160 },
};

// ── Socket: receive vision frames ──────────────────────────────────────────────
socket.on('vision_frame', data => {
  if (activeView !== 'vision') return;

  const rawImg = document.getElementById('vision-raw-img');
  if (rawImg && data.raw) rawImg.src = 'data:image/jpeg;base64,' + data.raw;

  const rectImg  = document.getElementById('vision-rect-img');
  const rectPlch = document.getElementById('vision-rect-placeholder');
  if (rectImg && rectPlch) {
    if (data.rect) {
      rectImg.src = 'data:image/jpeg;base64,' + data.rect;
      rectImg.style.display  = 'block';
      rectPlch.style.display = 'none';
    } else {
      rectImg.style.display  = 'none';
      rectPlch.style.display = 'flex';
    }
  }

  const procWrap = document.getElementById('vision-proc-wrap');
  const procImg  = document.getElementById('vision-proc-img');
  const procMode = data.proc_mode || 'none';
  if (procWrap) procWrap.style.display = (procMode !== 'none' && data.proc) ? 'flex' : 'none';
  if (procImg && data.proc) procImg.src = 'data:image/jpeg;base64,' + data.proc;
  const procBadge = document.getElementById('vision-proc-mode-badge');
  if (procBadge) procBadge.textContent = procMode !== 'none' ? procMode : '—';

  const hBadge = document.getElementById('vision-h-badge');
  if (hBadge) {
    if (data.h_fresh)    { hBadge.textContent = 'LIVE';  hBadge.className = 'vision-h-badge h-live'; }
    else if (data.has_h) { hBadge.textContent = 'CACHE'; hBadge.className = 'vision-h-badge h-cache'; }
    else                 { hBadge.textContent = 'NO H';  hBadge.className = 'vision-h-badge h-noh'; }
  }

  _visionFrameInfo = data;
  const fc   = data.frame_count ?? -1;
  const fi   = data.frame_idx  ?? 0;
  const pb   = document.getElementById('vision-playback');
  const sl   = document.getElementById('vision-seek-sl');
  const info = document.getElementById('vision-frame-info');
  if (pb) {
    pb.classList.toggle('hidden', fc <= 0);
    if (fc > 0) {
      if (sl) { sl.max = fc - 1; sl.value = fi; }
      if (info) info.textContent = `${fi} / ${fc - 1}`;
    }
  }

  _renderDetections(data.detections || []);
});

function _renderDetections(dets) {
  const el = document.getElementById('vision-det-list');
  const ct = document.getElementById('vision-det-count');
  if (!el) return;
  if (ct) ct.textContent = dets.length;
  el.innerHTML = dets.map(d => {
    const pos = (d.x_mm !== undefined)
      ? ` → table (${d.x_mm.toFixed(0)}, ${d.y_mm.toFixed(0)}) mm`
      : '';
    return `<div class="vision-det-row">ID <b>${d.id}</b> px=(${d.px.toFixed(0)},${d.py.toFixed(0)})${pos}</div>`;
  }).join('');
}

// ── View activation ────────────────────────────────────────────────────────────
async function onVisionViewActivated() {
  try {
    const res  = await fetch('/api/vision/state');
    const data = await res.json();
    _visionEnabled = data.enabled;

    const cb = document.getElementById('vision-enabled-cb');
    if (cb) cb.checked = _visionEnabled;
    _visionUpdateBadge(_visionEnabled);

    if (data.source) _visionSyncSourceSelector(data.source);

    const cfg = data.config || {};
    _visionSyncCheckbox('v-undistort', cfg.undistort);
    _visionSyncCheckbox('v-aruco',     cfg.show_aruco !== false);
    _visionSyncCheckbox('v-ids',       cfg.show_ids !== false);
    _visionSyncCheckbox('v-rej',       cfg.show_rejected);
    _visionSyncCheckbox('v-grid',      cfg.show_grid !== false);
    _visionSyncCheckbox('v-overlay',   cfg.show_table_overlay !== false);
    _visionSyncCheckbox('v-autocolor', cfg.auto_color);
    const dictSel = document.getElementById('vision-dict-sel');
    if (dictSel && cfg.dict) dictSel.value = cfg.dict;
    const qsl = document.getElementById('vision-quality-sl');
    const qv  = document.getElementById('vision-quality-val');
    if (qsl && cfg.jpeg_quality) { qsl.value = cfg.jpeg_quality; if (qv) qv.textContent = cfg.jpeg_quality; }

    if (cfg.anchors) _visionSyncAnchors(cfg.anchors);
    _visionSyncProcUI(cfg);
    await visionLoadObjects();
  } catch (e) {
    console.error('Vision state load error:', e);
  }
}

function _visionSyncProcUI(cfg) {
  const mode = cfg.proc_mode ?? 'none';
  const modeEl = document.getElementById('vp-mode');
  if (modeEl) modeEl.value = mode;

  const bm = document.getElementById('vp-bin-method');
  if (bm) bm.value = cfg.binary_method ?? 'global';
  _vSetSlider('vp-bin-thresh', 'vp-bin-thresh-val', cfg.binary_threshold ?? 128);
  _vSetSlider('vp-bin-block',  'vp-bin-block-val',  cfg.binary_block_size ?? 11);
  _visionSyncCheckbox('vp-bin-invert', cfg.binary_invert);

  _vSetSlider('vp-h-lo',  'vp-h-lo-val',  cfg.color_lo_h  ?? 0);
  _vSetSlider('vp-h-hi',  'vp-h-hi-val',  cfg.color_hi_h  ?? 10);
  _vSetSlider('vp-h2-lo', 'vp-h2-lo-val', cfg.color_lo_h2 ?? 160);
  _vSetSlider('vp-h2-hi', 'vp-h2-hi-val', cfg.color_hi_h2 ?? -1);
  _vSetSlider('vp-s-lo',  'vp-s-lo-val',  cfg.color_lo_s  ?? 80);
  _vSetSlider('vp-s-hi',  'vp-s-hi-val',  cfg.color_hi_s  ?? 255);
  _vSetSlider('vp-v-lo',  'vp-v-lo-val',  cfg.color_lo_v  ?? 60);
  _vSetSlider('vp-v-hi',  'vp-v-hi-val',  cfg.color_hi_v  ?? 255);
  _visionSyncCheckbox('vp-color-masked', cfg.color_show_masked !== false);
  const cpEl = document.getElementById('vp-color-preset');
  if (cpEl && cfg.color_target) cpEl.value = cfg.color_target;

  _vSetSlider('vp-canny-lo', 'vp-canny-lo-val', cfg.canny_low  ?? 50);
  _vSetSlider('vp-canny-hi', 'vp-canny-hi-val', cfg.canny_high ?? 150);
  _vSetSlider('vp-blur-k',   'vp-blur-k-val',   cfg.blur_kernel ?? 5);

  const morphOp = document.getElementById('vp-morph-op');
  if (morphOp) morphOp.value = cfg.morph_op ?? 'none';
  _vSetSlider('vp-morph-k', 'vp-morph-k-val', cfg.morph_kernel ?? 3);

  visionProcModeChanged();
}

function _visionSyncCheckbox(id, val) {
  const el = document.getElementById(id);
  if (el) el.checked = !!val;
}

function _visionUpdateBadge(enabled) {
  const badge = document.getElementById('vision-status-badge');
  if (!badge) return;
  badge.textContent = enabled ? 'ON' : 'OFF';
  badge.className   = 'vision-badge ' + (enabled ? 'vision-badge-on' : 'vision-badge-off');
}

// ── Enable / Disable ───────────────────────────────────────────────────────────
async function visionToggleEnabled(checked) {
  const url  = checked ? '/api/vision/enable' : '/api/vision/disable';
  const res  = await fetch(url, { method: 'POST' });
  const data = await res.json();
  _visionEnabled = checked && data.ok;
  _visionUpdateBadge(_visionEnabled);
}

// ── Source selector ────────────────────────────────────────────────────────────
function visionSourceTypeChanged() {
  const type = document.querySelector('input[name="vsrc-type"]:checked')?.value ?? 'usb';
  ['usb','ip','video','image'].forEach(t => {
    const el = document.getElementById('vsrc-' + t);
    if (el) el.classList.toggle('hidden', t !== type);
  });
}

function _visionGetSourceString() {
  const type = document.querySelector('input[name="vsrc-type"]:checked')?.value ?? 'usb';
  switch (type) {
    case 'usb':   return document.getElementById('vsrc-usb-idx')?.value?.trim()    ?? '0';
    case 'ip':    return document.getElementById('vsrc-ip-url')?.value?.trim()     ?? '';
    case 'video': return document.getElementById('vsrc-video-path')?.value?.trim() ?? '';
    case 'image': return document.getElementById('vsrc-image-path')?.value?.trim() ?? '';
    default:      return '0';
  }
}

function _visionSyncSourceSelector(source) {
  if (!source) return;
  const s = String(source);
  let type = 'usb';
  if (/^rtsp:|^http:|^https:/.test(s)) {
    type = 'ip';
    const el = document.getElementById('vsrc-ip-url');
    if (el) el.value = s;
  } else if (/\.(mp4|mkv|avi|mov|webm)$/i.test(s)) {
    type = 'video';
    const el = document.getElementById('vsrc-video-path');
    if (el) el.value = s;
  } else if (/\.(jpg|jpeg|png|bmp|tiff?)$/i.test(s)) {
    type = 'image';
    const el = document.getElementById('vsrc-image-path');
    if (el) el.value = s;
  } else {
    const el = document.getElementById('vsrc-usb-idx');
    if (el) el.value = isNaN(+s) ? '0' : s;
  }
  const radio = document.querySelector(`input[name="vsrc-type"][value="${type}"]`);
  if (radio) { radio.checked = true; visionSourceTypeChanged(); }
}

async function visionApplySource() {
  const type = document.querySelector('input[name="vsrc-type"]:checked')?.value ?? 'usb';
  const src = _visionGetSourceString();
  if (!src && src !== '0') {
    const msgs = { ip: 'Enter an RTSP/HTTP URL', video: 'Enter a video file path', image: 'Enter an image file path' };
    const badge = document.getElementById('v-status-badge');
    if (badge) { badge.textContent = msgs[type] || 'Select a source'; badge.className = 'v-badge v-badge-err'; }
    return;
  }
  try {
    const badge = document.getElementById('v-status-badge');
    if (badge) { badge.textContent = 'Connecting...'; badge.className = 'v-badge v-badge-warn'; }
    const resp = await fetch('/api/vision/source', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ source: src }),
    });
    if (!resp.ok) {
      const err = await resp.text();
      if (badge) { badge.textContent = `Error: ${err.slice(0,40)}`; badge.className = 'v-badge v-badge-err'; }
    }
  } catch (e) {
    const badge = document.getElementById('v-status-badge');
    if (badge) { badge.textContent = `Connection failed`; badge.className = 'v-badge v-badge-err'; }
  }
}

// ── Processing pipeline ────────────────────────────────────────────────────────
function _vSetSlider(id, valId, val) {
  const sl = document.getElementById(id);
  const vl = document.getElementById(valId);
  if (sl) sl.value = val;
  if (vl) vl.textContent = (id === 'vp-h2-hi' && val < 0) ? 'off' : val;
}

function visionProcSlider(sliderId, valId) {
  const sl = document.getElementById(sliderId);
  const vl = document.getElementById(valId);
  if (sl && vl) {
    const v = +sl.value;
    vl.textContent = (sliderId === 'vp-h2-hi' && v < 0) ? 'off' : v;
  }
}

function visionProcModeChanged() {
  const mode = document.getElementById('vp-mode')?.value ?? 'none';
  ['binarize','color','canny','blur'].forEach(g => {
    const el = document.getElementById('vp-' + g);
    if (el) el.classList.toggle('hidden', g !== {
      binarize: 'binarize', color_mask: 'color', canny: 'canny', blur: 'blur',
    }[mode]);
  });
  const morphRow = document.getElementById('vp-morph-row');
  if (morphRow) morphRow.style.display = (mode === 'none' || mode === 'blur') ? 'none' : '';
  visionProcCfg();
}

function _visionBinMethodChanged() {
  const method = document.getElementById('vp-bin-method')?.value ?? 'global';
  const thRow  = document.getElementById('vp-bin-thresh-row');
  const blRow  = document.getElementById('vp-bin-block-row');
  if (thRow) thRow.classList.toggle('hidden', method === 'otsu');
  if (blRow) blRow.classList.toggle('hidden', !method.startsWith('adaptive'));
}

function visionColorPreset() {
  const preset = document.getElementById('vp-color-preset')?.value ?? 'custom';
  const p = _VP_COLOR_PRESETS[preset];
  if (!p) return;
  _vSetSlider('vp-h-lo',  'vp-h-lo-val',  p.lo_h);
  _vSetSlider('vp-h-hi',  'vp-h-hi-val',  p.hi_h);
  _vSetSlider('vp-h2-lo', 'vp-h2-lo-val', p.lo_h2 ?? 160);
  _vSetSlider('vp-h2-hi', 'vp-h2-hi-val', p.hi_h2 ?? -1);
  visionProcCfg();
}

async function visionProcCfg() {
  const mode = document.getElementById('vp-mode')?.value ?? 'none';
  const cfg = { proc_mode: mode };

  if (mode === 'binarize') {
    const method = document.getElementById('vp-bin-method')?.value ?? 'global';
    _visionBinMethodChanged();
    cfg.binary_method     = method;
    cfg.binary_threshold  = +(document.getElementById('vp-bin-thresh')?.value  ?? 128);
    cfg.binary_block_size = +(document.getElementById('vp-bin-block')?.value   ?? 11);
    cfg.binary_invert     = !!(document.getElementById('vp-bin-invert')?.checked);
  }
  if (mode === 'color_mask') {
    cfg.color_target      = document.getElementById('vp-color-preset')?.value ?? 'red';
    cfg.color_lo_h        = +(document.getElementById('vp-h-lo')?.value  ?? 0);
    cfg.color_hi_h        = +(document.getElementById('vp-h-hi')?.value  ?? 10);
    cfg.color_lo_h2       = +(document.getElementById('vp-h2-lo')?.value ?? 160);
    cfg.color_hi_h2       = +(document.getElementById('vp-h2-hi')?.value ?? -1);
    cfg.color_lo_s        = +(document.getElementById('vp-s-lo')?.value  ?? 80);
    cfg.color_hi_s        = +(document.getElementById('vp-s-hi')?.value  ?? 255);
    cfg.color_lo_v        = +(document.getElementById('vp-v-lo')?.value  ?? 60);
    cfg.color_hi_v        = +(document.getElementById('vp-v-hi')?.value  ?? 255);
    cfg.color_show_masked = !!(document.getElementById('vp-color-masked')?.checked);
  }
  if (mode === 'canny') {
    cfg.canny_low  = +(document.getElementById('vp-canny-lo')?.value ?? 50);
    cfg.canny_high = +(document.getElementById('vp-canny-hi')?.value ?? 150);
  }
  if (mode === 'blur') {
    cfg.blur_kernel = +(document.getElementById('vp-blur-k')?.value ?? 5);
  }
  if (mode !== 'none' && mode !== 'blur') {
    cfg.morph_op     = document.getElementById('vp-morph-op')?.value ?? 'none';
    cfg.morph_kernel = +(document.getElementById('vp-morph-k')?.value ?? 3);
  }

  await fetch('/api/vision/config', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(cfg),
  });
}

// ── Config helper ──────────────────────────────────────────────────────────────
async function visionCfg(key, value) {
  if (key === 'jpeg_quality') {
    const qv = document.getElementById('vision-quality-val');
    if (qv) qv.textContent = value;
  }
  await fetch('/api/vision/config', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ [key]: value }),
  });
}

// ── Anchors ────────────────────────────────────────────────────────────────────
function _visionSyncAnchors(anchors) {
  for (const [corner, vals] of Object.entries(anchors)) {
    document.querySelectorAll(`.va-id[data-corner="${corner}"]`).forEach(el => { el.value = vals.tag_id; });
    document.querySelectorAll(`.va-x[data-corner="${corner}"]`).forEach(el  => { el.value = vals.x_mm;  });
    document.querySelectorAll(`.va-y[data-corner="${corner}"]`).forEach(el  => { el.value = vals.y_mm;  });
  }
}

function _visionReadAnchors() {
  const corners = ['top_left', 'top_right', 'bottom_right', 'bottom_left'];
  const anchors = {};
  for (const c of corners) {
    const id = document.querySelector(`.va-id[data-corner="${c}"]`)?.value;
    const x  = document.querySelector(`.va-x[data-corner="${c}"]`)?.value;
    const y  = document.querySelector(`.va-y[data-corner="${c}"]`)?.value;
    anchors[c] = { tag_id: parseInt(id||0), x_mm: parseFloat(x||0), y_mm: parseFloat(y||0) };
  }
  return anchors;
}

async function visionAnchorChanged() {
  const anchors = _visionReadAnchors();
  await fetch('/api/vision/config', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ anchors }),
  });
}

// ── Playback ───────────────────────────────────────────────────────────────────
async function visionPlayPause() {
  await fetch('/api/vision/playback', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'play_pause' }),
  });
}

async function visionSeek(frame) {
  await fetch('/api/vision/playback', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'seek', frame }),
  });
}

async function visionStep(delta) {
  await fetch('/api/vision/playback', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'step', delta }),
  });
}

async function visionSetSpeed(speed) {
  await fetch('/api/vision/playback', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'speed', speed }),
  });
}

// ── Object registry ────────────────────────────────────────────────────────────
async function visionLoadObjects() {
  const res  = await fetch('/api/vision/objects');
  _visionObjects = await res.json();
  _visionRenderObjects();
}

async function visionSaveObjects() {
  await fetch('/api/vision/objects', {
    method: 'PUT', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(_visionObjects),
  });
}

function _visionRenderObjects() {
  const list = document.getElementById('vision-objects-list');
  if (!list) return;
  const COLOR_DOT = {
    unknown: '#888', red: '#e53', green: '#2a2', blue: '#36f',
    yellow: '#fb0', white: '#eee', black: '#555', brown: '#854',
  };
  list.innerHTML = _visionObjects.map(obj => {
    const dot    = COLOR_DOT[obj.color] || '#888';
    const live   = obj.last_seen_ms ? '●' : '○';
    const table  = obj.on_table !== false ? '' : ' <span style="color:#f66;font-size:9px">gone</span>';
    const active = obj.id === _visionActiveObjId ? ' vision-obj-row-active' : '';
    return `<div class="vision-obj-row${active}" onclick="visionSelectObject('${obj.id}')">
      <span class="vision-obj-dot" style="background:${dot}"></span>
      <span class="vision-obj-name">${obj.name || obj.id}</span>
      <span style="font-size:9px;color:#888">${obj.type || ''}</span>
      ${obj.aruco_id !== null && obj.aruco_id !== undefined ? `<span class="vision-obj-tag">#${obj.aruco_id}</span>` : ''}
      ${table}
      <span style="margin-left:auto;color:${obj.last_seen_ms ? '#4a4' : '#666'};font-size:10px">${live}</span>
    </div>`;
  }).join('');
}

function visionSelectObject(objId) {
  _visionActiveObjId = objId;
  _visionRenderObjects();
  const obj = _visionObjects.find(o => o.id === objId);
  if (!obj) return;
  const ed = document.getElementById('vision-obj-editor');
  if (ed) ed.classList.remove('hidden');
  document.getElementById('voe-title').textContent   = obj.name || obj.id;
  document.getElementById('voe-name').value          = obj.name  || '';
  document.getElementById('voe-type').value          = obj.type  || 'other';
  document.getElementById('voe-aruco').value         = obj.aruco_id !== null && obj.aruco_id !== undefined ? obj.aruco_id : '';
  document.getElementById('voe-color').value         = obj.color || 'unknown';
  document.getElementById('voe-ix').value            = obj.initial_pos?.x ?? '';
  document.getElementById('voe-iy').value            = obj.initial_pos?.y ?? '';
  document.getElementById('voe-ontable').checked     = obj.on_table !== false;
  const ls = document.getElementById('voe-lastseen');
  if (ls) {
    if (obj.last_seen_ms) {
      const secs = ((Date.now() % 1e9) / 1000 - obj.last_seen_ms / 1000);
      ls.textContent = `Last seen ${Math.abs(secs).toFixed(1)}s ago  |  cur=(${obj.current_pos?.x?.toFixed(0)??'?'}, ${obj.current_pos?.y?.toFixed(0)??'?'}) mm`;
    } else {
      ls.textContent = 'Not yet detected';
    }
  }
}

function visionObjChanged() {
  const obj = _visionObjects.find(o => o.id === _visionActiveObjId);
  if (!obj) return;
  obj.name     = document.getElementById('voe-name').value;
  obj.type     = document.getElementById('voe-type').value;
  const arucoRaw = document.getElementById('voe-aruco').value;
  obj.aruco_id = arucoRaw !== '' ? parseInt(arucoRaw) : null;
  obj.color    = document.getElementById('voe-color').value;
  obj.on_table = document.getElementById('voe-ontable').checked;
  const ix     = parseFloat(document.getElementById('voe-ix').value);
  const iy     = parseFloat(document.getElementById('voe-iy').value);
  if (!isNaN(ix) && !isNaN(iy)) obj.initial_pos = { x: ix, y: iy };
  document.getElementById('voe-title').textContent = obj.name || obj.id;
  _visionRenderObjects();
  clearTimeout(visionObjChanged._timer);
  visionObjChanged._timer = setTimeout(visionSaveObjects, 800);
}

function visionNewObject() {
  const id  = 'obj_' + Date.now();
  const obj = {
    id,
    name: 'New Object', type: 'cylinder', aruco_id: null, color: 'unknown',
    initial_pos: { x: 1500, y: 1000 }, current_pos: { x: 1500, y: 1000 },
    on_table: true, last_seen_ms: null, notes: '',
  };
  _visionObjects.push(obj);
  _visionRenderObjects();
  visionSelectObject(id);
}

async function visionDeleteObject() {
  if (!_visionActiveObjId) return;
  if (!confirm(`Delete object "${_visionActiveObjId}"?`)) return;
  _visionObjects = _visionObjects.filter(o => o.id !== _visionActiveObjId);
  _visionActiveObjId = null;
  document.getElementById('vision-obj-editor')?.classList.add('hidden');
  _visionRenderObjects();
  await visionSaveObjects();
}

// ── Periodic position refresh ──────────────────────────────────────────────────
setInterval(() => {
  if (activeView !== 'vision') return;
  fetch('/api/vision/objects').then(r => r.json()).then(objs => {
    objs.forEach(serverObj => {
      const local = _visionObjects.find(o => o.id === serverObj.id);
      if (local) {
        local.current_pos  = serverObj.current_pos;
        local.last_seen_ms = serverObj.last_seen_ms;
        local.on_table     = serverObj.on_table;
      }
    });
    _visionRenderObjects();
    if (_visionActiveObjId) {
      const obj = _visionObjects.find(o => o.id === _visionActiveObjId);
      if (obj) {
        const ls = document.getElementById('voe-lastseen');
        if (ls && obj.last_seen_ms) {
          ls.textContent = `cur=(${obj.current_pos?.x?.toFixed(0)??'?'}, ${obj.current_pos?.y?.toFixed(0)??'?'}) mm`;
        }
      }
    }
  }).catch(() => {});
}, 2000);
