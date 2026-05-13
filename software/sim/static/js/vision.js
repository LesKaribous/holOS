// js/vision.js — Match-time vision view: live ArUco + tracker overlay.
// The visio tab is now a window into the running pipeline (not a test bench).
// Depends on: socket, activeView

'use strict';

let _visionEnabled  = false;
let _visionTeam     = 'blue';
// Latest detections + tracker state, kept around so the Objects subtab can
// render without waiting for the next frame.
let _visionLastFrame = null;
// Dashboard feed cache: feed_id → { jpeg, label, t (ms), pipeline, meta }.
// Populated by `vision_feed` SocketIO events; rendered into the grid.
const _visionFeeds = {};

function _isVisionView() {
  return typeof activeView === 'string' && activeView.startsWith('vision-');
}

// ── Socket: receive live frames + tracker state ─────────────────────────────
socket.on('vision_frame', data => {
  // Always cache the latest frame so the Objects subtab can render at any time
  _visionLastFrame = data;
  // Update the Objects table if that tab is active
  if (activeView === 'vision-objects') _renderObjectsTable(data);
  // Dashboard-only updates below
  if (activeView !== 'vision-dashboard') return;

  const rawImg  = document.getElementById('vision-raw-img');
  const rawPlch = document.getElementById('vision-raw-placeholder');
  if (rawImg && rawPlch) {
    if (data.raw) {
      rawImg.src = 'data:image/jpeg;base64,' + data.raw;
      rawImg.style.display  = 'block';
      rawPlch.style.display = 'none';
    } else {
      rawImg.style.display  = 'none';
      rawPlch.style.display = 'flex';
    }
  }

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

  // Homography badge on the raw feed label
  const hBadge = document.getElementById('vision-h-badge');
  if (hBadge) {
    if (data.h_fresh)    { hBadge.textContent = 'LIVE';  hBadge.className = 'vision-h-badge h-live'; }
    else if (data.has_h) { hBadge.textContent = 'CACHE'; hBadge.className = 'vision-h-badge h-cache'; }
    else                 { hBadge.textContent = 'NO H';  hBadge.className = 'vision-h-badge h-noh'; }
  }

  // Tracker status panel
  _renderTrackerStatus(data);
  _renderDetections(data.detections || []);
});

socket.on('vision_correction', data => {
  // Flash the last-correction line so the user can see corrections happening
  const el = document.getElementById('vt-last-corr');
  if (!el) return;
  el.textContent =
    `(${data.x_mm.toFixed(0)}, ${data.y_mm.toFixed(0)}) mm, ${data.theta_deg.toFixed(1)}°  ✓`;
  el.style.color = '#3a3';
  setTimeout(() => { el.style.color = ''; }, 1500);
});

socket.on('vision_heading_synced', data => {
  const el = document.getElementById('vt-heading');
  if (el) el.textContent = `${data.offset_deg.toFixed(2)}°  (synced)`;
});

// ── Dashboard feed grid ───────────────────────────────────────────────────
// Pipeline output nodes publish JPEGs via SocketIO `vision_feed` events.
// We cache per-feed and re-render the grid whenever the set of seen
// feed-ids changes (cheap path: just swap the <img src=>).

socket.on('vision_feed', data => {
  if (!data || !data.feed_id) return;
  // Four payload kinds today:
  //   frame      — jpeg, drawn as <img> tile
  //   pose_list  — table of poses, drawn as a tile in the dashboard
  //   objects    — same shape as pose_list but feeds the Objects subtab's
  //                persistent tracking (age, fade, classification)
  //   json       — anything JSON-serializable, drawn pretty-printed
  const kind = data.kind || (data.jpeg ? 'frame' : 'json');
  _visionFeeds[data.feed_id] = {
    kind,
    jpeg: data.jpeg,        // present when kind === 'frame'
    payload: data.data,     // present when kind !== 'frame'
    pipeline: data.pipeline,
    meta: data.meta || {},
    label: (data.meta && data.meta.label) || data.feed_id,
    t:    Date.now(),
  };
  // Object feeds are non-persistent: the Objects tab shows ONLY what's
  // in the current frame. If a tag is no longer detected, it must
  // disappear immediately (no stale carry-over).
  if (kind === 'objects' && Array.isArray(data.data)) {
    _vobjLatest = data.data;
    _vobjLatestT = Date.now();
    if (activeView === 'vision-objects') _renderObjectsTableFromState();
  }
  // Dashboard state must accrue even when the view isn't active so the
  // ArUco persistence and the last-known pose readings survive tab switches.
  if (data.feed_id === 'aruco_list' && Array.isArray(data.data)) {
    _ingestArucoList(data.data);
  } else if (data.feed_id === 'poses_corrected' && Array.isArray(data.data)) {
    _ingestPosesCorrected(data.data);
  }
  if (activeView === 'vision-dashboard') {
    if (!_visionPipelinesCache) refreshPipelineBar();
    _renderFeedGrid();        // safe no-op if old DOM is gone
    _renderDebugTiles();      // persistent ArUco list + Own/Opp cards
  } else if (activeView === 'vision-detection') {
    _renderDetectionTiles();
  }
});

// ── Vision dashboard state (persistent ArUco list + Own/Opp pose cards) ─
// ArUco markers stay colored (green = own team, red = opponent, neutral
// for anchors / unknowns) while fresh, then gray out the moment we
// stop seeing them and fade to invisible over _ARU_FADE_MS before
// being dropped. Avoids the flicker of a tag that drops out for a
// single frame.
const _arucoSeen = new Map();             // tag_id → {px, py, num_corners, lastSeen}
const _poseSeen  = { own: null, opp: null }; // {tag_id, label, naive_x, naive_y, x, y, theta, lastSeen}
const _ARU_FRESH_MS  = 250;               // 5 FPS pose feed = 200 ms; +50 ms margin
const _ARU_FADE_MS   = 2000;              // opacity ramp 1 → 0 over this window
// Total time a tag stays in the list after disappearing. Decoupled from
// the fade so the SLOT survives long after the row has gone visually
// invisible — that's what stops the list shrinking and other rows
// shifting when a flaky tag drops out for a frame or two.
const _ARU_REMOVE_MS = 5000;
const _LOC_FRESH_MS  = 500;

// Tag-id → team affiliation. Blue robots wear tags 1-5, yellow 6-10.
// Tags 20-23 are field anchors (configured per match in vision_config.json).
// Anything else is "neutral".
let _currentTeam = 'blue';
socket.on('state', s => {
  if (s && (s.team === 'blue' || s.team === 'yellow')) _currentTeam = s.team;
});

// Anchor map — populated from /api/vision_config on load. Maps tag_id to
// a short corner abbreviation (TL/TR/BL/BR) for the role column.
const _ANCHOR_ABBREV = {
  'top_left':     'TL',
  'top_right':    'TR',
  'bottom_left':  'BL',
  'bottom_right': 'BR',
};
let _anchorMap = {};   // tag_id (int) → abbrev like 'TL'

async function _loadAnchorMap() {
  try {
    const r = await fetch('/api/vision_config', {cache: 'no-store'});
    if (!r.ok) return;
    const j = await r.json();
    const anchors = (j && j.config && j.config.anchors) || {};
    const next = {};
    for (const [name, a] of Object.entries(anchors)) {
      if (a && a.tag_id != null) {
        next[Number(a.tag_id)] =
          _ANCHOR_ABBREV[name] || name.slice(0, 3).toUpperCase();
      }
    }
    _anchorMap = next;
  } catch (e) { /* ignore — fallback shows tag id alone */ }
}
window.addEventListener('load', _loadAnchorMap);

function _tagAffiliation(tagId) {
  if (tagId >= 1 && tagId <= 5)  return 'blue';
  if (tagId >= 6 && tagId <= 10) return 'yellow';
  return null;
}
function _tagClassForTeam(tagId, team) {
  if (_anchorMap[tagId] != null) return 'anchor';
  const a = _tagAffiliation(tagId);
  if (a === null) return 'neutral';
  return (a === team) ? 'own' : 'opponent';
}
// Role label is intentionally independent of fresh/lost — when a tag
// flickers, the label stays put. Only the COLOR transitions to gray
// (handled by `data-cls="lost"` on the row).
function _roleLabel(tagId, team) {
  if (_anchorMap[tagId] != null) return _anchorMap[tagId] || 'anc';
  const a = _tagAffiliation(tagId);
  if (a === null) return '—';
  return (a === team) ? 'own' : 'opp';
}

function _ingestArucoList(items) {
  if (!Array.isArray(items)) return;
  const now = Date.now();
  for (const m of items) {
    if (m == null || m.tag_id == null) continue;
    _arucoSeen.set(m.tag_id, {
      px: m.px, py: m.py,
      num_corners: m.num_corners,
      lastSeen: now,
    });
  }
}

function _ingestPosesCorrected(items) {
  if (!Array.isArray(items)) return;
  const now = Date.now();
  for (const p of items) {
    if (!p) continue;
    const cls = p.classification;
    const slot = cls === 'own' ? 'own' : cls === 'opponent' ? 'opp' : null;
    if (!slot) continue;
    _poseSeen[slot] = {
      tag_id:  p.tag_id, label: p.label,
      naive_x: p.naive_x_mm, naive_y: p.naive_y_mm,
      x:       p.x_mm,       y:       p.y_mm,
      theta:   p.theta_rad,
      lastSeen: now,
    };
  }
}

function _fmtAge(ms) {
  if (ms < 1000) return `${Math.round(ms)} ms`;
  if (ms < 60000) return `${(ms/1000).toFixed(1)} s`;
  return `${Math.floor(ms/60000)} m ${Math.floor((ms % 60000)/1000)} s`;
}

function _renderDebugTiles() {
  const now = Date.now();
  _renderArucoListPersistent(now);
  _renderLocCard('own', _poseSeen.own, now);
  _renderLocCard('opp', _poseSeen.opp, now);
}

function _renderArucoListPersistent(now) {
  const body = document.getElementById('vd-tile-arulist');
  if (!body) return;
  const meta = document.getElementById('vd-meta-arulist');

  const rows = [];
  let freshCount = 0;
  for (const [id, m] of _arucoSeen) {
    const age = now - m.lastSeen;
    if (age > _ARU_REMOVE_MS) { _arucoSeen.delete(id); continue; }
    const fresh = age < _ARU_FRESH_MS;
    if (fresh) freshCount++;
    // Fresh: opacity 1, color = own/opponent/neutral. Lost: opacity
    // ramps 1→0 over _ARU_FADE_MS, color = neutral gray (so the row
    // visibly "fades out" toward the white surface).
    const opacity = fresh ? 1
      : Math.max(0, 1 - (age - _ARU_FRESH_MS) / _ARU_FADE_MS);
    const cls = fresh ? _tagClassForTeam(id, _currentTeam) : 'lost';
    rows.push({ id, m, fresh, opacity, age, cls });
  }
  rows.sort((a, b) => a.id - b.id);

  if (meta) meta.textContent = `${freshCount}/${rows.length} fresh`;

  if (rows.length === 0) {
    if (!body.querySelector('.vd-empty')) {
      body.innerHTML = `<div class="vd-empty">no markers seen yet</div>`;
    }
    return;
  }
  // Compact table: tag · role · px,py · age. Role is derived from the
  // tag id alone (NOT from the fresh/lost state) so a flickering tag
  // doesn't make the role column oscillate. Only the row COLOR follows
  // the fresh/lost state (data-cls).
  body.innerHTML = `<table class="vd-pose-table vd-pose-table-compact">
    <thead><tr><th>id</th><th>role</th><th>px</th><th>py</th><th>age</th></tr></thead>
    <tbody>${rows.map(r => `
      <tr class="vd-aru-row" data-fresh="${r.fresh ? 1 : 0}" data-cls="${r.cls}" style="opacity:${r.opacity.toFixed(2)}">
        <td>${r.id}</td>
        <td>${_roleLabel(r.id, _currentTeam)}</td>
        <td>${r.m.px != null ? Math.round(r.m.px) : '—'}</td>
        <td>${r.m.py != null ? Math.round(r.m.py) : '—'}</td>
        <td>${_fmtAge(r.age)}</td>
      </tr>`).join('')}
    </tbody>
  </table>`;
}

function _renderLocCard(slot, p, now) {
  const card = document.getElementById(`vd-loc-${slot}`);
  if (!card) return;
  const tagEl = document.getElementById(`vd-loc-${slot}-tag`);
  const ageEl = document.getElementById(`vd-loc-${slot}-age`);
  const naEl  = document.getElementById(`vd-loc-${slot}-naive`);
  const coEl  = document.getElementById(`vd-loc-${slot}-corr`);
  const thEl  = document.getElementById(`vd-loc-${slot}-theta`);

  if (!p) {
    card.classList.remove('vd-loc-fresh', 'vd-loc-stale');
    card.classList.add('vd-loc-never');
    card.style.opacity = '';
    if (tagEl) tagEl.textContent = '—';
    if (ageEl) ageEl.textContent = 'never';
    if (naEl)  naEl.textContent  = '—, —';
    if (coEl)  coEl.textContent  = '—, —';
    if (thEl)  thEl.textContent  = '—';
    return;
  }
  card.classList.remove('vd-loc-never');
  const age = now - p.lastSeen;
  const fresh = age < _LOC_FRESH_MS;
  card.classList.toggle('vd-loc-fresh', fresh);
  card.classList.toggle('vd-loc-stale', !fresh);
  const opacity = fresh ? 1 : Math.max(0.35, 1 - (age - _LOC_FRESH_MS) / 10000);
  card.style.opacity = opacity.toFixed(2);

  if (tagEl) tagEl.textContent = p.label ? `#${p.tag_id} · ${p.label}` : `#${p.tag_id}`;
  if (ageEl) ageEl.textContent = _fmtAge(age);
  const fmt = v => (v == null || isNaN(v)) ? '—' : (+v).toFixed(0);
  if (naEl)  naEl.textContent  = `${fmt(p.naive_x)}, ${fmt(p.naive_y)}`;
  if (coEl)  coEl.textContent  = `${fmt(p.x)}, ${fmt(p.y)}`;
  if (thEl)  thEl.textContent  = (p.theta == null || isNaN(p.theta))
    ? '—' : (p.theta * 180 / Math.PI).toFixed(1);
}

// ── Per-view pipeline binding ───────────────────────────────────────────
// Each subtab (vision-dashboard, vision-detection) picks its own active
// pipeline. They run concurrently — the registry no longer enforces a
// single active. We persist the picks in localStorage so the user's
// choices stick across reloads.
const _VIEW_PIPELINE_KEY = 'visionViewPipelines';
function _loadViewPipelines() {
  try { return JSON.parse(localStorage.getItem(_VIEW_PIPELINE_KEY)) || {}; }
  catch { return {}; }
}
function _saveViewPipelines(map) {
  try { localStorage.setItem(_VIEW_PIPELINE_KEY, JSON.stringify(map)); }
  catch {}
}
function visionSetViewPipeline(viewId, pipelineName) {
  const map = _loadViewPipelines();
  map[viewId] = pipelineName || null;
  _saveViewPipelines(map);
  if (!pipelineName) return;
  // Enable the picked pipeline (parallel-safe — doesn't touch others).
  fetch(`/api/vision/pipelines/${encodeURIComponent(pipelineName)}/enable`, {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({enabled: true}),
  });
  // Also keep set_active in sync for the legacy pipeline-bar UI bits.
  if (typeof visionSetActivePipeline === 'function' && viewId === 'vision-dashboard') {
    visionSetActivePipeline(pipelineName);
  }
}

// ── Detection view tiles ────────────────────────────────────────────────
const _DETECT_TILE_IDS = {
  detect_raw:     'vd-tile-detraw',
  detect_preview: 'vd-tile-detpreview',
  detect_results: 'vd-tile-detresults',
};
const _DETECT_META_IDS = {
  detect_raw:     'vd-meta-detraw',
  detect_preview: 'vd-meta-detpreview',
  detect_results: 'vd-meta-detresults',
};

// Embed-cam status pill — gets updated from `detect_status` payloads
// whenever the firmware or the UI fires a detection.  Lets the
// operator see "request received" the instant the host gets the
// T:vis embed_detect frame, even if the camera fetch later times
// out so no JPEG comes back.
let _embedCamLastStatusT = 0;
socket.on('vision_feed', (data) => {
  if (!data || data.feed_id !== 'detect_status') return;
  const p = data.data || {};
  _embedCamLastStatusT = Date.now();
  const pill = document.getElementById('vd-det-status');
  if (!pill) return;
  const stage = String(p.stage || '?');
  const src   = String(p.source || '?');
  // Timing breakdown (only attached to done/error stages from the host).
  // Showing the split tells the operator whether the bottleneck is the
  // ESP32 wire (fetch), JPEG decode, or ArUco itself — instead of just
  // a lump "total" number that hides where the round-trip went.
  const fetchMs   = (p.fetch_ms   ?? null);
  const decodeMs  = (p.decode_ms  ?? null);
  const analyzeMs = (p.analyze_ms ?? null);
  const timings = (fetchMs != null)
    ? ` ⏱ f=${fetchMs} d=${decodeMs ?? 0} a=${analyzeMs ?? 0}ms`
    : '';
  const txt = (() => {
    if (stage === 'request')  return `▶ request from ${src}`;
    if (stage === 'fetching') return `… fetching (${src})`;
    if (stage === 'done')     return `✓ ${src} n=${p.n ?? '?'}/${p.expected ?? 4} off=${(p.offset_mm ?? 0).toFixed(1)}mm${timings}`;
    if (stage === 'error')    return `✗ ${src} ${p.reason || 'error'}${timings}`;
    return `${stage} (${src})`;
  })();
  pill.textContent = txt;
  pill.classList.remove('vd-pill-ok', 'vd-pill-err', 'vd-pill-busy');
  if (stage === 'done')                              pill.classList.add('vd-pill-ok');
  else if (stage === 'error')                        pill.classList.add('vd-pill-err');
  else if (stage === 'request' || stage === 'fetching') pill.classList.add('vd-pill-busy');
  // Brief flash so the operator notices the transition.
  pill.style.transition = 'background-color 200ms';
  pill.style.backgroundColor = (stage === 'error') ? '#5c1a1a'
                              : (stage === 'done')  ? '#1a4d2e'
                              : '#3a3a1a';
  setTimeout(() => { pill.style.backgroundColor = ''; }, 1500);
});

// Tick the frames-since counter so the operator can spot a stalled
// stream at a glance.
setInterval(() => {
  if (typeof activeView === 'string' && activeView === 'vision-detection') {
    const lbl = document.getElementById('vd-det-frames');
    if (lbl && _embedCamLastStatusT) {
      const dt = ((Date.now() - _embedCamLastStatusT) / 1000).toFixed(1);
      lbl.textContent = `last: ${dt}s ago`;
    }
  }
}, 500);

function _renderDetectionTiles() {
  // Embed-cam feeds are pinned: the operator wants to see the LAST
  // requested capture even if it landed long ago, so we don't apply
  // a staleness cutoff here (unlike the dashboard tiles).
  const now = Date.now();
  for (const [fid, slot] of Object.entries(_DETECT_TILE_IDS)) {
    const body = document.getElementById(slot);
    if (!body) continue;
    const meta = document.getElementById(_DETECT_META_IDS[fid]);
    const feed = _visionFeeds[fid];
    if (!feed) {
      if (!body.querySelector('.vd-empty')) {
        body.innerHTML =
          `<div class="vd-empty">no capture yet — press ▶ Detect or trigger from the robot</div>`;
      }
      if (meta) meta.textContent = '';
      continue;
    }
    if (meta) meta.textContent = `${((now - feed.t)/1000).toFixed(1)} s ago`;

    if (feed.kind === 'frame' && feed.jpeg) {
      let img = body.querySelector('img');
      if (!img) {
        body.innerHTML = '';
        img = document.createElement('img');
        img.style.maxWidth = '100%';
        img.style.maxHeight = '100%';
        body.appendChild(img);
      }
      img.src = `data:image/jpeg;base64,${feed.jpeg}`;
    } else if (Array.isArray(feed.payload)) {
      _renderChecksList(body, feed.payload);
    } else if (feed.payload != null) {
      _renderChecksList(body, [feed.payload]);
    }
  }
}

// Called by app.js switchView when the Detection tab activates.
// Asks the host to re-emit the cached feeds so the preview tile
// repaints with the most recent processed frame.
function onVisionDetectionActivated() {
  _renderDetectionTiles();    // paint what's already in the cache
  fetch('/api/embed_cam/replay', {method: 'POST'})
    .then(r => r.json())
    .catch(() => {});         // no detection yet → ignore
}
window.onVisionDetectionActivated = onVisionDetectionActivated;

function _renderChecksList(container, checks) {
  if (!Array.isArray(checks) || checks.length === 0) {
    container.innerHTML = `<div class="vd-empty">no checks reported this frame</div>`;
    return;
  }
  const rows = checks.map(c => {
    if (typeof c !== 'object' || c == null) c = {value: c};
    const name   = c.name   || c.id || '—';
    const status = String(c.status || 'unknown').toLowerCase();
    const value  = (c.value !== undefined && c.value !== null) ? c.value : '';
    const color  = c.color;            // {r,g,b} 0..255 OR '#hex' OR null
    let swatch = '';
    if (color) {
      let hex = '';
      if (typeof color === 'string') hex = color;
      else if (typeof color === 'object' && 'r' in color)
        hex = `rgb(${color.r|0}, ${color.g|0}, ${color.b|0})`;
      if (hex) swatch = `<span class="vd-check-color-swatch" style="background:${hex}"></span>`;
    }
    const valStr = (typeof value === 'object')
      ? JSON.stringify(value) : String(value);
    return `<tr>
      <td>${swatch}${name}</td>
      <td><span class="vd-check-status vd-check-status-${status}">${status}</span></td>
      <td>${valStr}</td>
    </tr>`;
  }).join('');
  container.innerHTML = `<table class="vd-checks-list">
    <thead><tr><th>Check</th><th>Status</th><th>Value</th></tr></thead>
    <tbody>${rows}</tbody>
  </table>`;
}

// ── Embed cam (ESP32-CAM) one-shot detect ───────────────────────────────
// Drives services/embed_cam.py via /api/embed_cam/detect. The annotated
// preview + check list arrive over SocketIO as standard `vision_feed`
// frames (detect_preview / detect_results) so the existing tile renderer
// just picks them up — no extra glue here.
let _embedCamLoopTimer = null;

function embedCamDetect() {
  // Apply any tuning the user changed in the config drawer before firing.
  const cfg = _embedCamReadConfig();
  fetch('/api/embed_cam/detect', {
    method:  'POST',
    headers: {'Content-Type': 'application/json'},
    body:    JSON.stringify({config: cfg}),
  })
  .then(r => r.json())
  .then(j => {
    if (!j.ok) {
      console.warn('[embed_cam] detect failed:', j.error);
    }
  })
  .catch(e => console.warn('[embed_cam] detect error:', e));
}

function embedCamSetLoop(on) {
  if (_embedCamLoopTimer) {
    clearInterval(_embedCamLoopTimer);
    _embedCamLoopTimer = null;
  }
  if (on) {
    embedCamDetect();
    _embedCamLoopTimer = setInterval(embedCamDetect, 1000);
  }
}

function embedCamToggleConfig() {
  const el = document.getElementById('embed-cam-cfg');
  if (el) el.classList.toggle('hidden');
}

function _embedCamReadConfig() {
  const get = (id) => {
    const el = document.getElementById(id);
    return el ? el.value : null;
  };
  return {
    url:                  get('ec-url'),
    aruco_dict:           get('ec-aruco-dict'),
    refine:               get('ec-refine'),
    team:                 get('ec-team'),
    expected_count:       parseInt(get('ec-expected-count'), 10),
    expected_spread_mm:   parseFloat(get('ec-spread-mm')),
    fallback_scale_mm_per_px: parseFloat(get('ec-fallback-scale')),
  };
}

function embedCamSaveConfig() {
  fetch('/api/embed_cam/config', {
    method:  'POST',
    headers: {'Content-Type': 'application/json'},
    body:    JSON.stringify(_embedCamReadConfig()),
  })
  .then(r => r.json())
  .then(j => {
    if (j.ok) console.log('[embed_cam] config saved');
  });
}

// Pull current server-side defaults on first show.
(function _embedCamInit() {
  fetch('/api/embed_cam/config')
    .then(r => r.json())
    .then(j => {
      if (!j.ok || !j.config) return;
      const set = (id, v) => {
        const el = document.getElementById(id);
        if (el && v !== undefined && v !== null) el.value = v;
      };
      const c = j.config;
      set('ec-url',                  c.url);
      set('ec-aruco-dict',           c.aruco_dict);
      set('ec-refine',               c.refine);
      set('ec-team',                 c.team);
      set('ec-expected-count',       c.expected_count);
      set('ec-spread-mm',            c.expected_spread_mm);
      set('ec-fallback-scale',       c.fallback_scale_mm_per_px);
    })
    .catch(() => {});  // endpoint not ready — defaults stay in the inputs
})();

// ── Playback strip wiring ───────────────────────────────────────────────
// Buttons live inside `.vd-playback[data-view=...]`. They translate into
// /api/vision/pipelines/<name>/source POST requests using the pipeline
// bound to that view (via _loadViewPipelines()).
let _vdPlaybackState = { playback: 'pause', frame_idx: 0, frame_count: -1 };

function _vdPostSource(viewId, params) {
  const pname = (_loadViewPipelines()[viewId]) || _visionPipelinesCache?.active;
  if (!pname) return;
  return fetch(`/api/vision/pipelines/${encodeURIComponent(pname)}/source`, {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({params}),
  });
}

function _vdSyncPlaybackUI() {
  const strip = document.getElementById('vd-playback');
  if (!strip) return;
  // Show/hide based on current pipeline's source state — populated via poll.
  const mode = _vdPlaybackState.playback || 'pause';
  strip.querySelectorAll('[data-toggle-mode]').forEach(b => {
    b.classList.toggle('active', b.dataset.toggleMode === mode);
  });
  const frameLbl = document.getElementById('vd-pb-frame');
  if (frameLbl) {
    const idx = _vdPlaybackState.frame_idx;
    const tot = _vdPlaybackState.frame_count;
    if (idx === undefined) frameLbl.textContent = '';
    else frameLbl.textContent = (tot > 0) ? `${idx} / ${tot}` : `frame ${idx}`;
  }
}

function _vdInitPlaybackButtons() {
  document.querySelectorAll('.vd-playback').forEach(strip => {
    const viewId = strip.dataset.view || 'vision-dashboard';
    strip.querySelectorAll('.vd-pb-btn').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.preventDefault();
        const act = btn.dataset.act;
        if (act === 'step_back_10' || act === 'step_10') {
          const cur = _vdPlaybackState.frame_idx ?? 0;
          const delta = (act === 'step_back_10') ? -10 : 10;
          _vdPostSource(viewId, {seek_target: Math.max(0, cur + delta), playback: 'seek'});
        } else {
          _vdPostSource(viewId, {playback: act});
        }
      });
    });
    const speedSel = strip.querySelector('.vd-pb-speed');
    if (speedSel) speedSel.addEventListener('change', () => {
      _vdPostSource(viewId, {speed: parseFloat(speedSel.value)});
    });
  });
}
document.addEventListener('DOMContentLoaded', _vdInitPlaybackButtons);

// Walks the named pipeline's nodes, finds the source.* one, and pulls
// playback / frame_idx / frame_count out of its state for the strip.
// Hides the strip when the pipeline doesn't have a video source.
function _vdRefreshPlaybackState(registry, pipelineName) {
  const strip = document.getElementById('vd-playback');
  if (!strip) return;
  if (!pipelineName) { strip.classList.add('hidden'); return; }
  const pipe = (registry?.pipelines || []).find(p => p.name === pipelineName);
  const srcNode = pipe?.state?.nodes?.find(n => n.kind === 'source.video');
  if (!srcNode) { strip.classList.add('hidden'); return; }
  strip.classList.remove('hidden');
  _vdPlaybackState = {
    playback:    srcNode.state?.playback    ?? 'pause',
    frame_idx:   srcNode.state?.frame_idx,
    frame_count: srcNode.state?.frame_count,
  };
  _vdSyncPlaybackUI();
  // Reflect the speed in the dropdown if the user changed it elsewhere.
  const sp = strip.querySelector('.vd-pb-speed');
  const liveSpeed = pipe?.state?.nodes?.find(n => n.id === srcNode.id)?.state?.speed;
  if (sp && liveSpeed != null && +sp.value !== +liveSpeed) sp.value = String(liveSpeed);
}

function _renderFeedGrid() {
  const grid = document.getElementById('vision-feeds-grid');
  const emptyEl = document.getElementById('vision-feeds-empty');
  if (!grid) return;

  // Drop feeds we haven't heard from in 8 s (stale source).
  const now = Date.now();
  const STALE_MS = 8000;
  const fresh = Object.entries(_visionFeeds)
    .filter(([_, f]) => (now - f.t) < STALE_MS)
    .map(([fid, f]) => ({ fid, ...f }));

  // Only show feeds belonging to the currently-active pipeline.
  const activePipeline = _visionPipelinesCache?.active;
  const visible = activePipeline
    ? fresh.filter(f => f.pipeline === activePipeline)
    : fresh;

  grid.dataset.feedCount = String(visible.length);

  if (visible.length === 0) {
    if (emptyEl) emptyEl.style.display = 'flex';
    // Remove any stale tile elements
    grid.querySelectorAll('.vision-feed-tile').forEach(el => el.remove());
    return;
  }
  if (emptyEl) emptyEl.style.display = 'none';

  // Resolve per-feed playback controls: walk the active pipeline graph,
  // find the upstream source.* node feeding each output, and surface its
  // playback param.
  const sourceForFeed = _resolveFeedSources(_visionPipelinesCache, activePipeline);

  // Diff-update: keep existing tiles, swap src for known feeds, add new ones.
  const existing = new Map(
    Array.from(grid.querySelectorAll('.vision-feed-tile')).map(el => [el.dataset.feedId, el])
  );
  const seen = new Set();
  for (const f of visible) {
    seen.add(f.fid);
    let tile = existing.get(f.fid);
    const wantKind = f.kind || 'frame';
    // If a tile previously rendered a different kind for the same feed_id
    // (rare but possible if the user re-wires the editor), drop it so the
    // body is rebuilt from scratch.
    if (tile && tile.dataset.kind !== wantKind) {
      tile.remove();
      tile = null;
    }
    if (!tile) {
      tile = document.createElement('div');
      tile.className = 'vision-feed-tile';
      tile.dataset.feedId = f.fid;
      tile.dataset.kind = wantKind;
      // Body markup depends on the feed kind — frame gets <img>, data
      // tiles get a scrollable body region.
      if (wantKind === 'frame') {
        tile.innerHTML = `
          <div class="vfg-label">
            <span class="vfg-name"></span>
            <span class="vfg-pipeline"></span>
          </div>
          <img class="vfg-img" alt="" draggable="false">
          <div class="vfg-playback" data-pb="hidden"></div>
        `;
      } else {
        tile.innerHTML = `
          <div class="vfg-label">
            <span class="vfg-name"></span>
            <span class="vfg-pipeline"></span>
            <span class="vfg-kind">${wantKind}</span>
          </div>
          <div class="vfg-data"></div>
        `;
      }
      grid.appendChild(tile);
    }
    tile.querySelector('.vfg-name').textContent = f.label;
    tile.querySelector('.vfg-pipeline').textContent = f.pipeline ? `[${f.pipeline}]` : '';

    if (wantKind === 'frame') {
      tile.querySelector('.vfg-img').src = 'data:image/jpeg;base64,' + f.jpeg;
      const src = sourceForFeed[f.fid];
      const pbEl = tile.querySelector('.vfg-playback');
      _renderPlaybackControls(pbEl, src, activePipeline);
    } else if (wantKind === 'pose_list') {
      _renderPoseListBody(tile.querySelector('.vfg-data'), f.payload);
    } else {
      _renderJsonBody(tile.querySelector('.vfg-data'), f.payload);
    }
  }
  // Drop tiles whose feed has gone stale
  for (const [fid, el] of existing) {
    if (!seen.has(fid)) el.remove();
  }
}

// Compact pose-list table renderer (memoized via JSON signature so we
// don't reflow the DOM every tick).
function _renderPoseListBody(el, poses) {
  if (!el) return;
  if (!Array.isArray(poses) || poses.length === 0) {
    if (el.dataset.sig !== 'empty') {
      el.dataset.sig = 'empty';
      el.innerHTML = '<div class="vfg-data-empty">no poses</div>';
    }
    return;
  }
  const sig = JSON.stringify(poses);
  if (el.dataset.sig === sig) return;
  el.dataset.sig = sig;
  const fmt = (v) => (v == null || isNaN(v)) ? '—' : v.toFixed(0);
  const fmtTheta = (v) =>
    (v == null || isNaN(v)) ? '—' : ((v * 180 / Math.PI).toFixed(1) + '°');
  el.innerHTML = `
    <table class="vfg-pose-table">
      <thead><tr>
        <th>tag</th><th>class</th><th>x mm</th><th>y mm</th><th>θ</th>
      </tr></thead>
      <tbody>${poses.map(p => {
        const cls = (p.classification || 'other').replace(' ', '-');
        return `<tr class="vfg-pose-${cls}">
          <td>#${p.tag_id ?? '?'}</td>
          <td>${escHtml(p.classification || 'other')}</td>
          <td>${fmt(p.x_mm)}</td>
          <td>${fmt(p.y_mm)}</td>
          <td>${fmtTheta(p.theta_rad)}</td>
        </tr>`;
      }).join('')}</tbody>
    </table>`;
}

// JSON pretty-printer with the same memoization trick.
function _renderJsonBody(el, value) {
  if (!el) return;
  let sig;
  try { sig = JSON.stringify(value); } catch (_) { sig = '__unstringifiable__'; }
  if (el.dataset.sig === sig) return;
  el.dataset.sig = sig;
  let pretty;
  try { pretty = JSON.stringify(value, null, 2); }
  catch (_) { pretty = String(value); }
  el.innerHTML = `<pre class="vfg-json">${escHtml(pretty || '—')}</pre>`;
}

// Walk the registry's graph for `pipelineName` and build {feedId → sourceNode}.
function _resolveFeedSources(registry, pipelineName) {
  const out = {};
  if (!registry || !pipelineName) return out;
  const p = (registry.pipelines || []).find(x => x.name === pipelineName);
  if (!p) return out;
  const nodes = p.graph?.nodes || [];
  const byId = Object.fromEntries(nodes.map(n => [n.id, n]));
  // For each output node, follow its 'frame' input back to the nearest source.
  // Cycle protection via a visited set (was depth-based, but a misbuilt
  // graph can still hit infinite paths through a self-loop).
  function findSource(nid, visited) {
    if (!nid || visited.has(nid)) return null;
    visited.add(nid);
    const node = byId[nid];
    if (!node) return null;
    if (node.kind && node.kind.startsWith('source.')) return node;
    for (const i of (node.inputs || [])) {
      const r = findSource(i.src_node, visited);
      if (r) return r;
    }
    return null;
  }
  for (const node of nodes) {
    if (node.kind !== 'output') continue;
    // The backend defaults feed_id → node id when the param is empty
    // (OutputNode._effective_feed_id). Mirror that here so the playback-
    // control resolver finds the source for empty-feed_id output nodes.
    const fid = (node.params?.feed_id && String(node.params.feed_id).trim())
              || node.id;
    if (!fid) continue;
    out[fid] = findSource(node.id, new Set());
  }
  return out;
}

function _renderPlaybackControls(el, sourceNode, pipelineName) {
  if (!el) return;
  if (!pipelineName) {
    if (el.dataset.sig !== '__hidden__') {
      el.dataset.pb = 'hidden';
      el.dataset.sig = '__hidden__';
      el.innerHTML = '';
    }
    return;
  }
  if (!sourceNode) {
    if (el.dataset.sig !== '__no_src__') {
      el.dataset.pb = 'shown';
      el.dataset.sig = '__no_src__';
      el.innerHTML = '<span style="color:var(--text-dim);font-size:11px">'
                   + 'no upstream source — wait a moment or reload</span>';
    }
    return;
  }
  const kind = sourceNode.kind;
  const params = sourceNode.params || {};
  const path = String(params.path || '').toLowerCase();
  // Transport controls only make sense for a seekable video FILE. Cameras
  // and live-URL video sources (http/https/rtsp) always run "live" — show
  // only the indicator so a stale Pause click can't freeze the stream.
  const isLiveSource =
    (kind === 'source.camera') ||
    (kind === 'source.video' &&
     (path.startsWith('http://')  || path.startsWith('https://') ||
      path.startsWith('rtsp://')));
  const mode = params.playback || (isLiveSource ? 'live' : 'play');
  const speed = parseFloat(params.speed ?? 1.0);

  // ── Memoize ────────────────────────────────────────────────────────
  // Without this, every vision_feed event (~20 Hz) replaced innerHTML,
  // destroying button DOM mid-hover (= blink) and mid-click (= the
  // <button> that was clicked vanishes before the click handler fires,
  // so the click does nothing). Skip the rebuild when the user-visible
  // state hasn't changed.
  const sig = `${sourceNode.id}|${kind}|${isLiveSource}|${mode}|${speed}|${params.refresh ?? ''}`;
  if (el.dataset.sig === sig) return;
  el.dataset.sig = sig;

  const btn = (act, label, active, title) =>
    `<button class="vfg-pb-btn${active ? ' vfg-pb-active' : ''}"
             data-act="${act}" title="${title || act}">${label}</button>`;

  let html = '';
  if (isLiveSource) {
    html = btn('live', '● live', true, 'live source — transport disabled');
  } else if (kind === 'source.video') {
    html =
      btn('step_back_10', '⏮⏮',  false, 'back 10 frames') +
      btn('step_back',    '⏮',   false, 'back 1 frame') +
      btn('pause',        '⏸',   mode === 'pause') +
      btn('play',         '▶',   mode === 'play') +
      btn('step',         '⏭',   false, 'forward 1 frame') +
      btn('step_10',      '⏭⏭',  false, 'forward 10 frames') +
      `<select class="vfg-pb-speed" title="Playback speed">
         ${[0.25, 0.5, 1, 2, 4, 8].map(s =>
           `<option value="${s}"${s===speed?' selected':''}>${s}×</option>`).join('')}
       </select>`;
  } else if (kind === 'source.image') {
    const isOnce = (params.playback === 'once');
    html = btn(isOnce ? 'once' : 'every-tick',
               isOnce ? 'once' : '↻ tick', true) +
           btn('refresh', '⟳', false, 'emit one new frame');
  }
  el.dataset.pb = 'shown';
  el.innerHTML = html;

  // Re-wire click handlers (we just replaced innerHTML).
  el.querySelectorAll('.vfg-pb-btn').forEach(b => {
    b.onclick = (e) => {
      e.preventDefault();
      e.stopPropagation();
      _sendPlaybackAction(pipelineName, sourceNode.id, kind, b.dataset.act);
    };
  });
  const speedSel = el.querySelector('.vfg-pb-speed');
  if (speedSel) {
    speedSel.onchange = (e) => {
      e.stopPropagation();
      _sendPlaybackAction(pipelineName, sourceNode.id, kind,
                          'speed:' + speedSel.value);
    };
    speedSel.onclick = (e) => e.stopPropagation();
  }
}

async function _sendPlaybackAction(pipelineName, nodeId, kind, action) {
  let params = {};
  // Helper: read current source-node graph params from the cached registry.
  const curParams = (_visionPipelinesCache?.pipelines || [])
    .find(p => p.name === pipelineName)?.graph?.nodes
    ?.find(n => n.id === nodeId)?.params || {};

  if (kind === 'source.image') {
    if (action === 'refresh') {
      params = { playback: 'once',
                 refresh: (curParams.refresh ?? 0) + 1 };
    } else if (action === 'once' || action === 'every-tick') {
      params = { playback: action === 'once' ? 'once' : 'every_tick' };
    }
  } else if (kind === 'source.video') {
    if (action === 'step_back_10') {
      // 10 frames back via seek to current_idx - 10
      const cur = curParams.seek_target ?? 0;
      // Use the live state (frame_idx) for current position
      const liveState = _visionPipelinesCache?.pipelines
        ?.find(p => p.name === pipelineName)?.state?.nodes
        ?.find(n => n.id === nodeId)?.state || {};
      const idx = liveState.frame_idx ?? cur;
      params = { playback: 'seek', seek_target: Math.max(0, idx - 10) };
    } else if (action === 'step_10') {
      const liveState = _visionPipelinesCache?.pipelines
        ?.find(p => p.name === pipelineName)?.state?.nodes
        ?.find(n => n.id === nodeId)?.state || {};
      const idx = liveState.frame_idx ?? 0;
      params = { playback: 'seek', seek_target: idx + 10 };
    } else if (action.startsWith('speed:')) {
      params = { speed: parseFloat(action.split(':')[1]) };
    } else {
      // play / pause / step / step_back
      params = { playback: action };
    }
  } else {
    params = { playback: action };
  }

  await fetch(`/api/vision/pipelines/${encodeURIComponent(pipelineName)}/node/${encodeURIComponent(nodeId)}/params`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ params }),
  });
  // Refresh the registry cache so the UI reflects the new state immediately.
  refreshPipelineBar();
}

// Periodic re-render so the dashboard fade animations advance even
// when no new feed arrives. 1 s tick — the age column ticks once per
// second (readable) and the opacity fade has 2-3 visible steps over
// the 2 s window. Anything faster made the age text feel jittery.
setInterval(() => {
  if (activeView === 'vision-dashboard') {
    _renderFeedGrid();
    _renderDebugTiles();
  } else if (activeView === 'vision-detection') {
    _renderDetectionTiles();
  }
}, 1000);

function _renderTrackerStatus(data) {
  // Heading offset
  const hd = document.getElementById('vt-heading');
  if (hd) {
    if (data.heading_offset_deg !== null && data.heading_offset_deg !== undefined) {
      hd.textContent = `${data.heading_offset_deg.toFixed(2)}°`;
      hd.style.color = '#3a3';
    } else {
      hd.textContent = 'not synced — recalage required';
      hd.style.color = '#c33';
    }
  }
  // Anchor status
  const an = document.getElementById('vt-anchors');
  const a = data.anchor_status || {};
  if (an) {
    const parts = [];
    if (a.live?.length)    parts.push(`${a.live.length}/4 live`);
    if (a.cached?.length)  parts.push(`${a.cached.length} cached`);
    if (a.missing?.length) parts.push(`${a.missing.length} missing`);
    an.textContent = parts.length ? parts.join(', ') : '—';
    an.style.color = (a.live?.length === 4) ? '#3a3'
                   : (a.missing?.length     ) ? '#c33'
                   :                           '#fa4';
  }
  // Own / opp poses
  const own = document.getElementById('vt-own');
  if (own) {
    if (data.robot_pose) {
      const p = data.robot_pose;
      own.textContent = `#${p.tag_id}  (${p.x_mm.toFixed(0)}, ${p.y_mm.toFixed(0)}) mm  θ=${(p.theta_rad * 180 / Math.PI).toFixed(1)}°`;
      own.style.color = '#3a3';
    } else {
      own.textContent = 'not visible';
      own.style.color = '#888';
    }
  }
  const opp = document.getElementById('vt-opp');
  if (opp) {
    if (data.opponent_pose) {
      const p = data.opponent_pose;
      opp.textContent = `#${p.tag_id}  (${p.x_mm.toFixed(0)}, ${p.y_mm.toFixed(0)}) mm`;
      opp.style.color = '#fa4';
    } else {
      opp.textContent = 'not visible';
      opp.style.color = '#888';
    }
  }
  const lc = document.getElementById('vt-last-corr');
  if (lc && lc.style.color !== 'rgb(51, 170, 51)') {
    if (data.last_correction) {
      lc.textContent = `(${data.last_correction.x_mm.toFixed(0)}, ${data.last_correction.y_mm.toFixed(0)}) mm — ${data.last_correction.age_s.toFixed(1)} s ago`;
    } else {
      lc.textContent = '— (none yet)';
    }
  }

  // Read-only team display — actual selection lives in the topbar / robot switch.
  if (data.team) {
    _visionTeam = data.team;
    const isBlue = data.team === 'blue';
    const longLbl = (isBlue ? 'Blue' : 'Yellow') +
                    ` (own = ${isBlue ? '1..5' : '6..10'})`;
    const shortLbl = isBlue ? 'Blue (1-5)' : 'Yellow (6-10)';
    const color = isBlue ? '#3b82f6' : '#fbbf24';
    // Old (legacy) tracker-info pane
    const ro = document.getElementById('vt-team-readout');
    if (ro) { ro.textContent = longLbl; ro.style.color = color; }
    // New (simplified debug view) bar pill
    const vd = document.getElementById('vd-team-readout');
    if (vd) { vd.textContent = shortLbl; vd.style.color = color; }
  }
}

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

// ── View activation ────────────────────────────────────────────────────────

// Dashboard subtab: pipeline picker, live feeds, tracker info (collapsed),
// log strip at the bottom. The legacy source / ArUco / dict / JPEG-Q
// controls are gone — those parameters now live on individual nodes in
// the pipeline editor.
async function onVisionViewActivated() {
  refreshPipelineBar();   // populates the toolbar + node-state strip
  // Pull legacy match-time backend state for the tracker info panel only
  // (heading offset, anchors, robot/opp poses come from there for now).
  try {
    const res  = await fetch('/api/vision/state');
    const data = await res.json();
    if (data.team) _visionTeam = data.team;
    const hd = document.getElementById('vt-heading');
    if (hd) {
      if (data.heading_offset_deg !== null && data.heading_offset_deg !== undefined) {
        hd.textContent = `${data.heading_offset_deg.toFixed(2)}°`;
        hd.style.color = '#3a3';
      } else {
        hd.textContent = 'not synced — recalage required';
        hd.style.color = '#c33';
      }
    }
  } catch (e) { /* ignore — tracker info panel is supplemental */ }
}

// ── Vision toggle (drives the active pipeline's enabled state) ─────────────
async function visionToggleEnabled(checked) {
  const active = _visionPipelinesCache?.active;
  if (!active) {
    // Nothing to enable — un-tick the box and tell the user where to go.
    const cb = document.getElementById('vision-enabled-cb');
    if (cb) cb.checked = false;
    return;
  }
  try {
    await fetch(`/api/vision/pipelines/${encodeURIComponent(active)}/enable`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: !!checked }),
    });
  } catch (_) { /* swallow — bar will refresh on next poll */ }
  refreshPipelineBar();
}

// ── Match-time controls ────────────────────────────────────────────────────
async function visionSetTeam(team) {
  const res = await fetch('/api/vision/team', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ team }),
  });
  const data = await res.json();
  if (data.ok) _visionTeam = data.team;
}

async function visionSyncHeading() {
  const res = await fetch('/api/vision/sync_heading', { method: 'POST' });
  const data = await res.json();
  const el = document.getElementById('vt-heading');
  if (!el) return;
  if (data.ok) {
    el.textContent = `${data.offset_deg.toFixed(2)}°  (synced)`;
    el.style.color = '#3a3';
  } else {
    el.textContent = `failed — ${data.reason}`;
    el.style.color = '#c33';
  }
}

// ── Diagnostics line ───────────────────────────────────────────────────────
function _renderDiag(state) {
  const ws = state?.diag?.worker_status || '—';
  const wsEl = document.getElementById('vd-status');
  if (wsEl) {
    wsEl.textContent = `worker: ${ws}`;
    wsEl.className = 'vd-pill ' + (
      ws === 'running'                     ? 'vd-ok'
      : ws.startsWith('init failed')       ? 'vd-err'
      : ws.startsWith('frame error')       ? 'vd-err'
      : ws === 'no source' || ws === 'no frame' ? 'vd-warn'
      : ''
    );
  }

  const srcEl = document.getElementById('vd-source');
  if (srcEl) {
    const src = state?.source || '—';
    const open = state?.source_open;
    srcEl.textContent = `source: ${src.slice(-50)}${open ? ' ✓' : ''}`;
    srcEl.className = 'vd-pill ' + (open ? 'vd-ok' : (src === '—' ? '' : 'vd-warn'));
  }

  const fr = document.getElementById('vd-frames');
  if (fr) {
    const p = state?.diag?.frames_processed ?? 0;
    const e = state?.diag?.frames_encoded ?? 0;
    fr.textContent = `frames ${e}/${p}`;
  }

  const ageEl = document.getElementById('vd-age');
  if (ageEl) {
    const age = state?.diag?.last_frame_age_s;
    if (age == null) {
      ageEl.textContent = 'last frame —';
      ageEl.className = 'vd-pill';
    } else if (age < 1.0) {
      ageEl.textContent = `last frame ${(age * 1000).toFixed(0)} ms ago`;
      ageEl.className = 'vd-pill vd-ok';
    } else {
      ageEl.textContent = `last frame ${age.toFixed(1)} s ago`;
      ageEl.className = 'vd-pill ' + (age > 5 ? 'vd-err' : 'vd-warn');
    }
  }

  const errEl = document.getElementById('vd-error');
  if (errEl) {
    const err = state?.diag?.last_error;
    if (err) {
      errEl.textContent = err.length > 120 ? err.slice(0, 117) + '…' : err;
      errEl.classList.remove('hidden');
    } else {
      errEl.classList.add('hidden');
    }
  }
}

// Poll state every 1s so the diag line keeps moving even when no frame
// stream is flowing (e.g. source not opened, worker idle). Runs on the
// dashboard only — the Editor / Objects tabs don't show the diag line.
setInterval(async () => {
  if (activeView !== 'vision-dashboard') return;
  try {
    const res = await fetch('/api/vision/state');
    const data = await res.json();
    _renderDiag(data);
  } catch (_) {}
}, 1000);

async function visionResetHeading() {
  await fetch('/api/vision/reset_heading', { method: 'POST' });
  const el = document.getElementById('vt-heading');
  if (el) {
    el.textContent = 'not synced — recalage required';
    el.style.color = '#c33';
  }
}

// ── Anchors ────────────────────────────────────────────────────────────────
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

// ── Active-pipeline picker (dashboard) ─────────────────────────────────────
let _visionPipelinesCache = null;   // last response from /api/vision/pipelines

async function refreshPipelineBar() {
  try {
    const res = await fetch('/api/vision/pipelines');
    const data = await res.json();
    _visionPipelinesCache = data;
    const viewMap = _loadViewPipelines();
    // Per-view pipeline picker: any <select> tagged with data-view-pipeline
    // gets populated. Selection persists in localStorage.
    document.querySelectorAll('select[data-view-pipeline]').forEach(sel => {
      const viewId = sel.dataset.viewPipeline;
      const cur = viewMap[viewId] || (viewId === 'vision-dashboard' ? data.active : '') || '';
      sel.innerHTML = '<option value="">— pipeline —</option>' +
        (data.pipelines || []).map(p =>
          `<option value="${p.name}"${p.name === cur ? ' selected' : ''}>` +
          `${p.name}${p.name === data.default ? ' ★' : ''}` +
          `</option>`).join('');
    });
    // Update playback state for the localization view (look up its source.* node).
    _vdRefreshPlaybackState(data, viewMap['vision-dashboard'] || data.active);
    const btn = document.getElementById('vp-bar-default-btn');
    if (btn) {
      const cur = data.active || '';
      const isDefault = cur && cur === data.default;
      btn.textContent = isDefault ? '★ Default' : '☆ Set default';
      btn.disabled = !cur;
      btn.classList.toggle('btn-active', isDefault);
    }
    const st = document.getElementById('vp-bar-status');
    const active = data.active
      ? (data.pipelines || []).find(x => x.name === data.active)
      : null;
    const s = active?.state || {};
    if (st) {
      if (!data.active) {
        st.textContent = (data.pipelines && data.pipelines.length)
          ? 'no active pipeline — pick one above to drive the feeds'
          : 'no pipelines yet — go to Vision · Editor to build one';
        st.classList.remove('vp-bar-status-ok');
      } else {
        st.textContent = `'${data.active}' is the active pipeline`;
        st.classList.add('vp-bar-status-ok');
      }
    }
    // Worker-state pills
    const wEl = document.getElementById('vp-bar-worker');
    if (wEl) {
      const ws = s.worker_status || (data.active ? '—' : 'no pipeline');
      wEl.textContent = `worker: ${ws}`;
      wEl.className = 'vd-pill ' + (
        ws === 'running'                ? 'vd-ok'
        : ws.startsWith('init failed')  ? 'vd-err'
        : ws.startsWith('frame error')  ? 'vd-err'
        : ws.startsWith('loop error')   ? 'vd-err'
        : ws === 'disabled'             ? ''
        :                                 'vd-warn'
      );
    }
    const fEl = document.getElementById('vp-bar-frames');
    if (fEl) fEl.textContent = `frames: ${s.frames_processed ?? 0}`;
    const aEl = document.getElementById('vp-bar-age');
    if (aEl) {
      const age = s.last_frame_age_s;
      if (age == null) {
        aEl.textContent = 'last: —'; aEl.className = 'vd-pill';
      } else if (age < 1) {
        aEl.textContent = `last: ${(age*1000).toFixed(0)} ms`; aEl.className = 'vd-pill vd-ok';
      } else {
        aEl.textContent = `last: ${age.toFixed(1)} s`;
        aEl.className   = 'vd-pill ' + (age > 5 ? 'vd-err' : 'vd-warn');
      }
    }
    // Sync the Vision toggle from the active pipeline's enabled flag
    // (since the toggle now controls pipeline.enable() / .disable()).
    const cb = document.getElementById('vision-enabled-cb');
    if (cb) cb.checked = !!(active && active.enabled);
    // Per-node state strip
    _renderNodeStates(active);
    // Logs — always refresh while the dashboard is open. The strip is
    // visible by default; the toggle button just hides/shows it.
    const logsEl = document.getElementById('vp-logs');
    if (logsEl && !logsEl.classList.contains('hidden')) _refreshPipelineLogs();
  } catch (e) { /* ignore — server may be restarting */ }
}

function _renderNodeStates(activePipeline) {
  const host = document.getElementById('vp-nodestates');
  if (!host) return;
  if (!activePipeline) { host.innerHTML = ''; return; }
  const nodes = activePipeline.state?.nodes || [];
  host.innerHTML = nodes.map(n => {
    const st         = n.state  || {};
    const err        = st.last_error;
    const declared   = n.declared_inputs || [];
    const wired      = n.wired_inputs    || [];
    const missing    = declared.filter(p => !wired.includes(p));
    const isSource   = (n.kind || '').startsWith('source.');
    const produced   = n.produced_last_tick === true;
    const inputsOK   = isSource ? true : (n.inputs_satisfied === true);

    // Coloring rules:
    //   red   = error
    //   amber = declared but unwired input port (graph is incomplete)
    //   green = produced output last tick
    //   amber = wired but upstream produced None (waiting for data)
    //   grey  = idle (e.g. node ran but returned {})
    let cls, hint;
    if (err) {
      cls = 'vp-ns-err'; hint = `⚠ error`;
    } else if (!isSource && missing.length) {
      cls = 'vp-ns-warn'; hint = `unwired: ${missing.join(',')}`;
    } else if (produced) {
      cls = 'vp-ns-ok';   hint = '● running';
    } else if (!inputsOK) {
      cls = 'vp-ns-warn'; hint = isSource ? 'no frame yet' : 'no upstream data';
    } else {
      cls = '';           hint = 'idle';
    }

    let extra;
    if (err) {
      extra = `<small>${escHtml(err.slice(0, 80))}</small>`;
    } else if (st.frame_idx != null && st.frame_count != null && st.frame_count > 0) {
      extra = `<small>${st.frame_idx}/${st.frame_count}${st.playback ? ' · ' + st.playback : ''}</small>`;
    } else if (st.playback) {
      extra = `<small>${st.playback}</small>`;
    } else {
      extra = `<small>${escHtml(hint)}</small>`;
    }
    return `<span class="vp-nodestate ${cls}"
                  title="${escHtml(JSON.stringify({...st, declared_inputs: declared, wired_inputs: wired, output_ports: n.output_ports, ticks: n.tick_count}))}">
      <b>${n.id}</b>${extra ? ' ' + extra : ''}
    </span>`;
  }).join('');
}

function visionToggleLogs() {
  const el  = document.getElementById('vp-logs');
  const btn = document.getElementById('vp-logs-toggle');
  if (!el) return;
  const willBeHidden = !el.classList.contains('hidden');
  el.classList.toggle('hidden');
  if (btn) btn.textContent = willBeHidden ? 'show' : 'hide';
  if (!willBeHidden) _refreshPipelineLogs();
}

async function _refreshPipelineLogs() {
  const active = _visionPipelinesCache?.active;
  if (!active) return;
  try {
    const r = await fetch(`/api/vision/pipelines/${encodeURIComponent(active)}/logs?n=50`);
    const data = await r.json();
    const el = document.getElementById('vp-logs');
    if (!el) return;
    const logs = data.logs || [];
    if (!logs.length) {
      el.innerHTML = '<div class="vp-logs-empty">No logs yet</div>';
      return;
    }
    el.innerHTML = logs.map(L => {
      const t = new Date((L.t || 0) * 1000).toLocaleTimeString();
      return `<div class="vp-log-row">
        <span class="vp-log-t">${t}</span>
        <span class="vp-log-lv vp-lv-${L.level || 'info'}">${(L.level||'info').toUpperCase()}</span>
        <span class="vp-log-msg">${escHtml(L.msg || '')}</span>
      </div>`;
    }).join('');
    el.scrollTop = el.scrollHeight;
  } catch (_) {}
}

// minimal HTML escaper (used by the log panel + node-state tooltips)
function escHtml(s) {
  return String(s).replace(/[&<>"']/g, c => ({
    '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'
  }[c]));
}

async function visionSetActivePipeline(name) {
  await fetch('/api/vision/active', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name: name || null }),
  });
  refreshPipelineBar();
}

async function visionSetDefaultPipeline() {
  // Sets the default to whatever is currently active.
  const sel = document.getElementById('vp-bar-active');
  const name = sel ? sel.value : '';
  if (!name) return;
  await fetch('/api/vision/default', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  });
  refreshPipelineBar();
}

// Refresh the pipeline bar every 2s while the dashboard is visible.
setInterval(() => {
  if (activeView === 'vision-dashboard') refreshPipelineBar();
}, 2000);

// ── Editor subtab ──────────────────────────────────────────────────────────
function onVisionEditorActivated() {
  // The editor is a React island loaded lazily. The mount function is
  // idempotent — safe to call every time the tab is shown.
  if (typeof window.visionMountEditor === 'function') {
    try { window.visionMountEditor(); } catch (e) { console.error(e); }
  }
  // Notify the React component that the user just switched in, so it can
  // pause the active pipeline's video source and refresh the freeze-frame
  // snapshot. The listener is set up inside vision_editor.jsx.
  setTimeout(() => {
    window.dispatchEvent(new CustomEvent('vision-editor-activated'));
  }, 50);
}

// ── Objects subtab ─────────────────────────────────────────────────────────
function onVisionObjectsActivated() {
  // Render whatever is in the persistent state — could have been
  // populated by either the legacy vision_frame events or by an
  // output.objects pipeline node.
  _renderObjectsTableFromState();
}

// Snapshot of the latest output.objects pose_list. We deliberately do NOT
// keep persistent state per-tag-id any more — when a tag isn't in the
// current frame (paused source, occlusion, lost track) the user wants it
// gone from the list immediately.
let _vobjLatest = [];
let _vobjLatestT = 0;

function _classifyTag(id, team) {
  // CdR convention: blue → own=1..5, opp=6..10; yellow swaps.
  // Anchors are 20..23 (table corners).
  if (id >= 20 && id <= 23) return 'anchor';
  const isBlueOwn = id >= 1 && id <= 5;
  const isYellowOwn = id >= 6 && id <= 10;
  if (team === 'blue')   return isBlueOwn ? 'own robot' : (isYellowOwn ? 'opponent' : 'other');
  if (team === 'yellow') return isYellowOwn ? 'own robot' : (isBlueOwn ? 'opponent' : 'other');
  return 'other';
}

// Legacy vision_frame ingestion — used as a fallback when no pipeline
// output.objects node is wired. Replaces the snapshot wholesale on each
// frame: not detected → not in the snapshot → not on the table.
function _renderObjectsTable(data) {
  const team = data.team || _visionTeam || 'blue';
  const snap = (data.detections || []).map(d => ({
    tag_id:     d.id,
    px:         d.px,
    py:         d.py,
    x_mm:       d.x_mm,
    y_mm:       d.y_mm,
    theta_rad:  // overlay heading from tracker poses if present
      (data.robot_pose    && data.robot_pose.tag_id    === d.id)
        ? data.robot_pose.theta_rad
      : (data.opponent_pose && data.opponent_pose.tag_id === d.id)
        ? data.opponent_pose.theta_rad
        : null,
    classification: _classifyTag(d.id, team),
  }));
  _vobjLatest = snap;
  _vobjLatestT = Date.now();
  _renderObjectsTableFromState();
}

// Read-only renderer — rebuilds from _vobjLatest each call. A tag
// missing from the latest snapshot disappears immediately.
function _renderObjectsTableFromState() {
  const ageMs = _vobjLatestT ? (Date.now() - _vobjLatestT) : null;
  // If the last snapshot is older than this, we treat the source as
  // "lost" and clear the table — typical when the pipeline gets paused.
  const SNAPSHOT_FRESH_MS = 2000;
  const fresh = ageMs != null && ageMs < SNAPSHOT_FRESH_MS;

  let entries = fresh
    ? _vobjLatest.map(p => ({
        id: p.tag_id,
        x_mm: p.x_mm, y_mm: p.y_mm,
        theta_rad: p.theta_rad,
        px: p.px, py: p.py,
        classification: p.classification
                        || _classifyTag(p.tag_id, _visionTeam),
      })).filter(e => e.id != null)
    : [];
  entries.sort((a, b) => a.id - b.id);

  const tbody = document.getElementById('vobj-tbody');
  if (!tbody) return;
  if (entries.length === 0) {
    const msg = (ageMs == null)
      ? 'No detections yet — wire an output.objects node in the active pipeline'
      : `No active detections (last snapshot ${(ageMs/1000).toFixed(1)} s ago)`;
    tbody.innerHTML = `<tr class="vobj-empty"><td colspan="6" style="text-align:center;color:var(--text-dim);padding:18px">
      ${msg}
    </td></tr>`;
  } else {
    tbody.innerHTML = entries.map(e => {
      const klass = String(e.classification).replace(' ', '-');
      const tabMM = (e.x_mm != null && e.y_mm != null)
        ? `(${(+e.x_mm).toFixed(0)}, ${(+e.y_mm).toFixed(0)})`
        : '—';
      const theta = (e.theta_rad != null && !isNaN(e.theta_rad))
        ? `${(e.theta_rad * 180 / Math.PI).toFixed(1)}°`
        : '—';
      const px = e.px != null ? Math.round(e.px) : '—';
      const py = e.py != null ? Math.round(e.py) : '—';
      return `<tr class="vobj-row vobj-${klass}">
        <td><span class="vobj-id">#${e.id}</span></td>
        <td><span class="vobj-tag vobj-tag-${klass}">${e.classification}</span></td>
        <td>${px},${py}</td>
        <td>${tabMM}</td>
        <td>${theta}</td>
        <td>${ageMs != null ? (ageMs < 1000 ? `${ageMs} ms` : `${(ageMs/1000).toFixed(1)} s`) : '—'}</td>
      </tr>`;
    }).join('');
  }
  const sum = document.getElementById('vobj-summary');
  if (sum) sum.textContent = `${entries.length} detection${entries.length === 1 ? '' : 's'}`;
}

// Tick once per second so the table goes blank when the snapshot stales out.
setInterval(() => {
  if (activeView === 'vision-objects') _renderObjectsTableFromState();
}, 1000);
