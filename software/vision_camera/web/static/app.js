/* Virtual-camera UI: playback controls + status sync.
 * Polls /api/status to track current playback mode + frame index, sends
 * /api/control for any user action. Keyboard shortcuts: space = play/pause,
 * ←/→ = step back/forward, Shift+←/→ = -10/+10 frames. */

const frameBadge = document.getElementById('frame-badge');
const errBadge   = document.getElementById('err-badge');
const speedSel   = document.getElementById('speed-sel');

let state = {playback: 'pause', speed: 1.0, frame_idx: 0, frame_count: -1};

async function fetchStatus() {
  try {
    const r = await fetch('/api/status');
    const s = await r.json();
    if (s && s.playback !== undefined) applyState(s);
  } catch {}
}
function applyState(s) {
  state = {...state, ...s};
  errBadge.textContent = s.last_error ? `⚠ ${s.last_error}` : '';
  if (s.frame_count > 0) frameBadge.textContent = `frame ${s.frame_idx} / ${s.frame_count}`;
  else                    frameBadge.textContent = `frame ${s.frame_idx}`;
  document.querySelectorAll('[data-toggle-mode]').forEach(b => {
    b.classList.toggle('active', b.dataset.toggleMode === s.playback);
  });
  if (speedSel && +speedSel.value !== +s.speed) speedSel.value = String(s.speed);
}
setInterval(fetchStatus, 750);
fetchStatus();

async function postControl(params) {
  try {
    const r = await fetch('/api/control', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({params}),
    });
    const j = await r.json();
    if (j && j.state) applyState(j.state);
  } catch {}
}

document.querySelectorAll('.pb-btn').forEach(btn => {
  btn.addEventListener('click', e => {
    e.preventDefault();
    const act = btn.dataset.act;
    if (act === 'step_back_10' || act === 'step_10') {
      const cur = state.frame_idx ?? 0;
      const delta = (act === 'step_back_10') ? -10 : 10;
      postControl({seek_target: Math.max(0, cur + delta), playback: 'seek'});
    } else {
      postControl({playback: act});
    }
  });
});
speedSel.addEventListener('change', () => {
  postControl({speed: parseFloat(speedSel.value)});
});

document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (e.code === 'Space') {
    e.preventDefault();
    postControl({playback: state.playback === 'play' ? 'pause' : 'play'});
  } else if (e.code === 'ArrowLeft') {
    e.preventDefault();
    if (e.shiftKey) {
      const cur = state.frame_idx ?? 0;
      postControl({seek_target: Math.max(0, cur - 10), playback: 'seek'});
    } else {
      postControl({playback: 'step_back'});
    }
  } else if (e.code === 'ArrowRight') {
    e.preventDefault();
    if (e.shiftKey) {
      const cur = state.frame_idx ?? 0;
      postControl({seek_target: cur + 10, playback: 'seek'});
    } else {
      postControl({playback: 'step'});
    }
  }
});
