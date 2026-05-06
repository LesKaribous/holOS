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
  if (activeView === 'vision-dashboard') {
    // Make sure the registry cache is populated — the playback-control
    // resolver needs the graph to walk back from output → source. Without
    // this, the very first feed arrives before the 2 s pipeline-bar poll
    // has loaded the cache and the controls render as "no upstream source".
    if (!_visionPipelinesCache) refreshPipelineBar();
    _renderFeedGrid();
  }
});

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
  const mode = params.playback || (kind === 'source.camera' ? 'live' : 'play');
  const speed = parseFloat(params.speed ?? 1.0);

  // ── Memoize ────────────────────────────────────────────────────────
  // Without this, every vision_feed event (~20 Hz) replaced innerHTML,
  // destroying button DOM mid-hover (= blink) and mid-click (= the
  // <button> that was clicked vanishes before the click handler fires,
  // so the click does nothing). Skip the rebuild when the user-visible
  // state hasn't changed.
  const sig = `${sourceNode.id}|${kind}|${mode}|${speed}|${params.refresh ?? ''}`;
  if (el.dataset.sig === sig) return;
  el.dataset.sig = sig;

  const btn = (act, label, active, title) =>
    `<button class="vfg-pb-btn${active ? ' vfg-pb-active' : ''}"
             data-act="${act}" title="${title || act}">${label}</button>`;

  let html = '';
  if (kind === 'source.camera') {
    html = btn('live',  '● live',  mode === 'live') +
           btn('pause', '⏸',       mode === 'pause') +
           btn('step',  '⤳',       false, 'capture next frame');
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

// Re-render once per second to clear stale tiles even when no new frames arrive.
setInterval(() => {
  if (activeView === 'vision-dashboard') _renderFeedGrid();
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
    const ro = document.getElementById('vt-team-readout');
    if (ro) {
      const isBlue = data.team === 'blue';
      ro.textContent = (isBlue ? 'Blue' : 'Yellow') +
                       ` (own = ${isBlue ? '1..5' : '6..10'})`;
      ro.style.color = isBlue ? '#3b82f6' : '#fbbf24';
    }
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
    const sel = document.getElementById('vp-bar-active');
    if (sel) {
      const cur = data.active || '';
      sel.innerHTML = '<option value="">— none —</option>' +
        (data.pipelines || []).map(p =>
          `<option value="${p.name}"${p.name === cur ? ' selected' : ''}>` +
          `${p.name}${p.name === data.default ? ' ★' : ''}` +
          `</option>`).join('');
    }
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
