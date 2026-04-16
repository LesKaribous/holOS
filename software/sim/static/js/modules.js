// js/modules.js — Telemetry channel toggles + firmware service controls
// Depends on: socket, showToast, activeView

'use strict';

const _TEL_CHANNELS = [
  { id:'pos',    name:'Position',  desc:'X / Y / θ from the odometry estimator', freq:'10 Hz' },
  { id:'motion', name:'Motion',    desc:'RUNNING / IDLE state + current target',  freq:'10 Hz' },
  { id:'safety', name:'Safety',    desc:'Obstacle detection & collision flags',   freq:'10 Hz' },
  { id:'chrono', name:'Chrono',    desc:'Match elapsed time',                     freq:'10 Hz' },
  { id:'occ',    name:'Occupancy', desc:'Compressed lidar occupancy grid',        freq:'2 Hz'  },
];

const _HW_SERVICES = [
  { id:'SAFETY', name:'Safety', desc:'Lidar-based obstacle detection & emergency stop' },
  { id:'LIDAR',  name:'Lidar',  desc:'Lidar scan processing & occupancy map update'    },
  { id:'MOTION', name:'Motion', desc:'PID motion controller & trajectory planner'      },
  { id:'CHRONO', name:'Chrono', desc:'Match timer (start / stop / reset)'              },
];

const _telOverride   = {};
const TEL_OVERRIDE_TTL = 2000;

let _modInited   = false;
let _modUpdating = false;

function initModulesView() {
  if (_modInited) return;
  _modInited = true;
  const grid = document.getElementById('tel-grid');
  if (grid) {
    grid.innerHTML = _TEL_CHANNELS.map(ch => `
      <div class="tel-card" id="tel-card-${ch.id}">
        <div class="tel-card-name">${ch.name}</div>
        <div class="tel-card-desc">${ch.desc}</div>
        <div class="tel-card-footer">
          <span class="tel-freq">${ch.freq}</span>
          <label class="toggle-sw" title="Toggle ${ch.name} telemetry">
            <input type="checkbox" id="tel-chk-${ch.id}" onchange="modSetTelChannel('${ch.id}', this.checked)">
            <span class="toggle-slider"></span>
          </label>
        </div>
      </div>`).join('');
  }
  const list = document.getElementById('svc-list');
  if (list) {
    list.innerHTML = _HW_SERVICES.map(svc => `
      <div class="svc-row" id="svc-row-${svc.id}">
        <span class="svc-dot disabled" id="svc-dot-${svc.id}"></span>
        <span class="svc-name">${svc.name}</span>
        <span class="svc-desc">${svc.desc}</span>
        <span class="svc-btns">
          <button class="btn-svc enable"  onclick="modSvcCmd('enable','${svc.id}')"  title="Enable ${svc.name}">Enable</button>
          <button class="btn-svc disable" onclick="modSvcCmd('disable','${svc.id}')" title="Disable ${svc.name}">Disable</button>
        </span>
      </div>`).join('');
  }
}

function updateModulesView(state) {
  if (!_modInited || _modUpdating) return;
  _modUpdating = true;
  const connected = !!(state && state.hw_mode);
  const telMask   = (state && state.hw_tel_mask) ? state.hw_tel_mask : {};

  const banner  = document.getElementById('mod-banner');
  const dot     = document.getElementById('mod-banner-dot');
  const bannerT = document.getElementById('mod-banner-text');
  if (banner && dot && bannerT) {
    if (connected) {
      const hwType = (state.hw_type || 'usb').toUpperCase();
      banner.classList.add('connected');
      dot.className = 'mod-status-dot on';
      bannerT.textContent = `Connected — ${hwType} bridge active`;
    } else {
      banner.classList.remove('connected');
      dot.className = 'mod-status-dot';
      bannerT.textContent = 'Not connected — connect to a robot to manage modules';
    }
  }

  const now = Date.now();
  _TEL_CHANNELS.forEach(ch => {
    const chk  = document.getElementById(`tel-chk-${ch.id}`);
    const card = document.getElementById(`tel-card-${ch.id}`);
    if (!chk) return;
    const ov = _telOverride[ch.id];
    let enabled;
    if (ov && (now - ov.ts) < TEL_OVERRIDE_TTL) {
      enabled = ov.value;
      if (telMask[ch.id] !== undefined && telMask[ch.id] === ov.value) delete _telOverride[ch.id];
    } else {
      delete _telOverride[ch.id];
      enabled = telMask[ch.id] !== undefined ? telMask[ch.id] : true;
    }
    chk.checked  = enabled;
    chk.disabled = !connected;
    if (card) { card.classList.toggle('active', enabled); card.classList.toggle('inactive', !enabled); }
  });

  _HW_SERVICES.forEach(svc => {
    const dot = document.getElementById(`svc-dot-${svc.id}`);
    const btns = document.querySelectorAll(`#svc-row-${svc.id} .btn-svc`);
    if (dot) dot.className = connected ? 'svc-dot enabled' : 'svc-dot disabled';
    btns.forEach(b => { b.disabled = !connected; });
  });

  _modUpdating = false;
}

function modSetTelChannel(channelId, enabled) {
  _telOverride[channelId] = { value: enabled, ts: Date.now() };
  socket.emit('hw_fire', { cmd: `tel(${channelId},${enabled ? 1 : 0})` });
}

function modSvcCmd(action, serviceId) {
  socket.emit('hw_fire', { cmd: `${action}(${serviceId})` });
  showToast(`${action}(${serviceId}) sent`);
}
