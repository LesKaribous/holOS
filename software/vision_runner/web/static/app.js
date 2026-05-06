/* vision-runner UI:
 *   - subscribe to SocketIO 'feed' (frames + data) and 'runner_status'
 *   - render one tile per feed_id, auto-discovered
 *   - playback controls call /api/source for video sources
 *   - top-right rebuild button calls /api/reload
 */

const socket = io();
const tilesEl = document.getElementById('tiles');
const playbackEl = document.getElementById('playback');
const reloadBadge = document.getElementById('reload-badge');
const fpsBadge = document.getElementById('fps-badge');
const errBadge = document.getElementById('err-badge');
const srcBadge = document.getElementById('src-badge');
const speedSel = document.getElementById('speed-sel');
const frameInfo = document.getElementById('frame-info');

// Track tiles by feed_id
const tiles = new Map();

function ensureTile(feedId, label) {
  if (tiles.has(feedId)) return tiles.get(feedId);
  const t = document.createElement('section');
  t.className = 'tile';
  t.innerHTML = `
    <div class="tile-head">
      <div class="tile-title"></div>
      <div class="tile-meta"></div>
    </div>
    <div class="tile-body"></div>
  `;
  tilesEl.appendChild(t);
  const tile = {
    el: t,
    head: t.querySelector('.tile-title'),
    meta: t.querySelector('.tile-meta'),
    body: t.querySelector('.tile-body'),
    kind: null,
    lastT: 0,
    fps: 0,
    framesSinceTick: 0,
  };
  tile.head.textContent = label || feedId;
  tiles.set(feedId, tile);
  return tile;
}

// Periodic FPS calc per tile
setInterval(() => {
  for (const [, t] of tiles) {
    t.fps = t.framesSinceTick;
    t.framesSinceTick = 0;
    if (t.kind === 'frame') {
      t.meta.textContent = `${t.fps} fps`;
    }
  }
}, 1000);

socket.on('feed', (msg) => {
  const tile = ensureTile(msg.feed_id, (msg.meta && msg.meta.label) || msg.feed_id);
  tile.framesSinceTick++;
  tile.kind = msg.kind;

  if (msg.kind === 'frame') {
    let img = tile.body.querySelector('img');
    if (!img) {
      tile.body.innerHTML = '';
      img = document.createElement('img');
      tile.body.appendChild(img);
    }
    img.src = `data:image/jpeg;base64,${msg.jpeg_b64}`;
  } else if (msg.kind === 'data') {
    renderData(tile, msg.payload);
    tile.meta.textContent = `${tile.fps} updates/s`;
  }
});

function renderData(tile, payload) {
  // pose_list → table
  if (Array.isArray(payload) && payload.length > 0
      && typeof payload[0] === 'object'
      && 'tag_id' in payload[0]) {
    const cols = ['tag_id', 'classification', 'x_mm', 'y_mm', 'theta_rad'];
    const rows = payload.map(p => {
      const cls = p.classification || 'object';
      const tds = cols.map(c => {
        let v = p[c];
        if (typeof v === 'number') v = v.toFixed(c === 'theta_rad' ? 3 : 1);
        return `<td>${v ?? ''}</td>`;
      }).join('');
      return `<tr class="pose-cls-${cls}">${tds}</tr>`;
    }).join('');
    tile.body.innerHTML = `
      <table class="pose-table">
        <thead><tr>${cols.map(c => `<th>${c}</th>`).join('')}</tr></thead>
        <tbody>${rows}</tbody>
      </table>`;
    return;
  }
  // objects-list / generic JSON → pretty-print
  let pre = tile.body.querySelector('pre');
  if (!pre) {
    tile.body.innerHTML = '';
    pre = document.createElement('pre');
    tile.body.appendChild(pre);
  }
  pre.textContent = JSON.stringify(payload, null, 2);
}

// ── Status (FPS, reload count, errors, source state) ────────────────────
let currentSourceState = {};   // last seen source node state from /api/status

function fetchStatus() {
  fetch('/api/status').then(r => r.json()).then(applyStatus).catch(() => {});
}
function applyStatus(st) {
  if (!st) return;
  reloadBadge.textContent = `build #${st.reload_count ?? '—'}`;
  fpsBadge.textContent =
    st.frames_processed !== undefined
      ? `${st.frames_processed} frames` : '— frames';
  errBadge.textContent = st.last_error ? `⚠ ${st.last_error}` : '';
  if (st.source_kind) {
    srcBadge.textContent = `${st.source_kind} · ${st.source_path || ''}`;
  }
  // Show playback controls only for video sources
  playbackEl.classList.toggle('hidden', st.source_kind !== 'video');

  // Sync the source-node state — playback mode + frame index — into the UI
  // so the buttons reflect ground truth (the worker may have advanced
  // playback past `step` into `pause` without us asking, etc).
  const srcNode = (st.nodes || []).find(n => n.id === 'src');
  if (srcNode && srcNode.state) {
    currentSourceState = srcNode.state;
    syncPlaybackUI();
  }
}
function syncPlaybackUI() {
  const mode = currentSourceState.playback || 'pause';
  document.querySelectorAll('[data-toggle="mode"]').forEach(b => {
    b.classList.toggle('active', b.dataset.mode === mode);
  });
  const idx = currentSourceState.frame_idx;
  const tot = currentSourceState.frame_count;
  if (idx !== undefined && tot !== undefined) {
    frameInfo.textContent = (tot > 0)
      ? `frame ${idx} / ${tot}`
      : `frame ${idx}`;
  } else {
    frameInfo.textContent = '';
  }
}
socket.on('runner_status', applyStatus);
setInterval(fetchStatus, 1000);
fetchStatus();

// ── Playback controls ────────────────────────────────────────────────────
function postSource(params) {
  return fetch('/api/source', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({params}),
  }).then(() => fetchStatus());   // refresh UI immediately, don't wait for poll
}

document.querySelectorAll('.pb-btn').forEach(btn => {
  btn.addEventListener('click', (e) => {
    e.preventDefault();
    const act = btn.dataset.act;
    // Translate ⏮⏮/⏭⏭ to a `seek` because source.video doesn't know
    // about step_back_10/step_10 — only `seek` with `seek_target`.
    if (act === 'step_back_10' || act === 'step_10') {
      const cur = currentSourceState.frame_idx ?? 0;
      const delta = (act === 'step_back_10') ? -10 : 10;
      postSource({seek_target: Math.max(0, cur + delta), playback: 'seek'});
      return;
    }
    postSource({playback: act});
  });
});

speedSel.addEventListener('change', () => {
  postSource({speed: parseFloat(speedSel.value)});
});

// Keyboard shortcuts: space = play/pause, ←/→ = step back/forward
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (e.code === 'Space') {
    e.preventDefault();
    const isPlaying = currentSourceState.playback === 'play';
    postSource({playback: isPlaying ? 'pause' : 'play'});
  } else if (e.code === 'ArrowLeft') {
    e.preventDefault();
    postSource({playback: e.shiftKey ? 'step_back' : 'step_back'});
    if (e.shiftKey) {
      const cur = currentSourceState.frame_idx ?? 0;
      postSource({seek_target: Math.max(0, cur - 10), playback: 'seek'});
    }
  } else if (e.code === 'ArrowRight') {
    e.preventDefault();
    if (e.shiftKey) {
      const cur = currentSourceState.frame_idx ?? 0;
      postSource({seek_target: cur + 10, playback: 'seek'});
    } else {
      postSource({playback: 'step'});
    }
  }
});

// ── Force rebuild ────────────────────────────────────────────────────────
document.getElementById('btn-reload').addEventListener('click', () => {
  fetch('/api/reload', {method: 'POST'});
});
