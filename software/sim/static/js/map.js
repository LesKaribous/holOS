// js/map.js — Canvas rendering: field, robot, overlays, map sidebar
// Depends on globals from app.js: canvas, ctx, miniCanvas, mctx,
//   scale, offX, offY, wx, wy, wlen, canvasToWorld,
//   lastState, poiData, terrainImg, robotImg, showRobotImage,
//   colorPanelOpen, activePoi, socket,
//   FIELD_W, FIELD_H, GRID_W, GRID_H, GRID_CELL,
//   COLOR_HEX, ALL_COLORS, MOTION_CLASS, poiCategory, escHtml, setText

'use strict';

// ── Main render ────────────────────────────────────────────────────────────
function render(s) {
  ctx.fillStyle = '#e8ecf0';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawField(ctx, wx, wy, wlen, s);
  if (s.features.show_grid)  drawGrid(s);
  if (s.features.show_trail) drawTrail(s);
  if (s.features.show_path)  drawPath(s);
  drawOpponent(s);
  drawGameObjects(s);
  drawPOIs(ctx, wx, wy, wlen, poiData);
  drawSmallTable();
  drawMapTraj();
  drawMapPin();
  drawRobot(s);
}

// ── Mini-map renderer ──────────────────────────────────────────────────────
function renderMiniMap(s) {
  const MW = miniCanvas.width, MH = miniCanvas.height;
  const ms  = Math.min(MW / FIELD_W, MH / FIELD_H);
  const mox = (MW - FIELD_W * ms) / 2;
  const moy = (MH - FIELD_H * ms) / 2;
  const mwx = mm => mox + mm * ms;
  const mwy = mm => moy + (FIELD_H - mm) * ms;
  const mwl = mm => mm * ms;

  mctx.fillStyle = '#c8d8c8';
  mctx.fillRect(0, 0, MW, MH);
  mctx.fillStyle = terrainImg ? 'transparent' : '#bdc8b4';
  if (terrainImg) mctx.drawImage(terrainImg, mwx(0), mwy(FIELD_H), mwl(FIELD_W), mwl(FIELD_H));
  mctx.strokeStyle = 'rgba(26,82,118,.5)';
  mctx.lineWidth = 1;
  mctx.strokeRect(mwx(0), mwy(FIELD_H), mwl(FIELD_W), mwl(FIELD_H));
  for (const p of poiData) {
    const cat = poiCategory(p.name);
    mctx.beginPath();
    mctx.arc(mwx(p.x), mwy(p.y), Math.max(1.5, mwl(6)), 0, Math.PI*2);
    mctx.fillStyle = cat.color + 'aa';
    mctx.fill();
  }
  if (s.features.show_path && s.path && s.path.length > 1) {
    mctx.strokeStyle = 'rgba(41,128,185,.6)';
    mctx.lineWidth = 1;
    mctx.setLineDash([3,2]);
    mctx.beginPath();
    mctx.moveTo(mwx(s.path[0][0]), mwy(s.path[0][1]));
    for (let i = 1; i < s.path.length; i++) mctx.lineTo(mwx(s.path[i][0]), mwy(s.path[i][1]));
    mctx.stroke();
    mctx.setLineDash([]);
  }
  const r = s.robot;
  const rcx = mwx(r.x), rcy = mwy(r.y);
  mctx.beginPath();
  mctx.arc(rcx, rcy, Math.max(3, mwl(r.radius * 0.6)), 0, Math.PI*2);
  mctx.fillStyle = s.team === 'yellow' ? '#d4ac0d' : '#2980b9';
  mctx.fill();
  mctx.strokeStyle = 'rgba(255,255,255,.8)';
  mctx.lineWidth = 1;
  mctx.stroke();
  const arr = mwl(r.radius * 1.0);
  mctx.strokeStyle = '#fff';
  mctx.lineWidth = 1.5;
  mctx.beginPath();
  mctx.moveTo(rcx, rcy);
  mctx.lineTo(rcx + arr * Math.cos(-r.theta), rcy + arr * Math.sin(-r.theta));
  mctx.stroke();
}

// ── Field drawing ──────────────────────────────────────────────────────────
function drawField(c, wxf, wyf, wlf, s) {
  const fx = wxf(0), fy = wyf(0), fw = wlf(FIELD_W), fh = wlf(FIELD_H);
  if (terrainImg) {
    c.drawImage(terrainImg, fx, fy, fw, fh);
    c.fillStyle = 'rgba(0,0,0,0.15)';
    c.fillRect(fx, fy, fw, fh);
  } else {
    c.fillStyle = '#c8d4be';
    c.fillRect(fx, fy, fw, fh);
    c.fillStyle = 'rgba(180,140,0,.09)';
    c.fillRect(wxf(0), wyf(0), wlf(1500), wlf(FIELD_H));
    c.fillStyle = 'rgba(30,80,200,.06)';
    c.fillRect(wxf(1500), wyf(0), wlf(1500), wlf(FIELD_H));
  }
  c.strokeStyle = 'rgba(26,82,118,.5)';
  c.lineWidth = Math.max(1.5, wlf(4));
  c.strokeRect(fx, fy, fw, fh);
}

function drawGrid(s) {
  ctx.strokeStyle = 'rgba(16, 49, 70, 0.75)'; ctx.lineWidth = .5;
  for (let gx=0; gx<=GRID_W; gx++) {
    ctx.beginPath(); ctx.moveTo(wx(gx*GRID_CELL), wy(0)); ctx.lineTo(wx(gx*GRID_CELL), wy(FIELD_H)); ctx.stroke();
  }
  for (let gy=0; gy<=GRID_H; gy++) {
    ctx.beginPath(); ctx.moveTo(wx(0), wy(gy*GRID_CELL)); ctx.lineTo(wx(FIELD_W), wy(gy*GRID_CELL)); ctx.stroke();
  }
  for (const c of s.occupancy) {
    ctx.fillStyle = (c.layer === 'dynamic')
      ? 'rgba(255, 140, 0, 0.82)'
      : 'rgba(41, 128, 185, 0.72)';
    ctx.fillRect(wx(c.gx*GRID_CELL)+1, wy(c.gy*GRID_CELL)+1, wlen(GRID_CELL)-2, wlen(GRID_CELL)-2);
  }
}

function drawOpponent(s) {
  const opp = s.opponent;
  if (!opp || !opp.enabled) return;
  const cx = wx(opp.x), cy = wy(opp.y), r = wlen(120);
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(231,76,60,.2)'; ctx.fill();
  ctx.strokeStyle = '#e74c3c'; ctx.lineWidth = 2; ctx.stroke();
  const th = opp.theta || 0;
  const dx = Math.cos(th) * r, dy = -Math.sin(th) * r;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + dx, cy + dy);
  ctx.strokeStyle = '#e74c3c'; ctx.lineWidth = 2; ctx.stroke();
  ctx.fillStyle = '#e74c3c'; ctx.font = '11px monospace'; ctx.textAlign = 'center';
  ctx.fillText('OPP', cx, cy - r - 4);
  ctx.textAlign = 'start';
}

function drawPath(s) {
  if (!s.path || s.path.length < 2) return;
  ctx.strokeStyle='rgba(41,128,185,.7)'; ctx.lineWidth=2; ctx.setLineDash([6,4]);
  ctx.beginPath(); ctx.moveTo(wx(s.path[0][0]), wy(s.path[0][1]));
  for (let i=1; i<s.path.length; i++) ctx.lineTo(wx(s.path[i][0]), wy(s.path[i][1]));
  ctx.stroke();
  for (let i=1; i<s.path.length-1; i++) {
    const px=wx(s.path[i][0]), py=wy(s.path[i][1]), d=wlen(7);
    ctx.fillStyle='rgba(41,128,185,.8)';
    ctx.beginPath(); ctx.moveTo(px,py-d); ctx.lineTo(px+d,py); ctx.lineTo(px,py+d); ctx.lineTo(px-d,py); ctx.closePath(); ctx.fill();
  }
  ctx.setLineDash([]);
  if (s.motion_target) {
    const tx = wx(s.motion_target[0]), ty = wy(s.motion_target[1]);
    const r = wlen(18);
    ctx.strokeStyle = '#e74c3c'; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(tx, ty, r, 0, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(tx - r*1.4, ty); ctx.lineTo(tx + r*1.4, ty); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(tx, ty - r*1.4); ctx.lineTo(tx, ty + r*1.4); ctx.stroke();
  }
}

function drawTrail(s) {
  const tr = s.robot.trail; if (!tr || tr.length < 2) return;
  for (let i=1; i<tr.length; i++) {
    const a = i/tr.length;
    ctx.strokeStyle=`rgba(41,128,185,${a*.3})`; ctx.lineWidth=1.5;
    ctx.beginPath(); ctx.moveTo(wx(tr[i-1][0]),wy(tr[i-1][1])); ctx.lineTo(wx(tr[i][0]),wy(tr[i][1])); ctx.stroke();
  }
}

function drawPOIs(c, wxf, wyf, wlf, pois) {
  for (const p of pois) {
    const px=wxf(p.x), py=wyf(p.y), cat=poiCategory(p.name), r=wlf(cat.r);
    c.beginPath(); c.arc(px,py,r,0,Math.PI*2);
    c.fillStyle=cat.color+'22'; c.fill();
    c.strokeStyle=cat.color+'cc'; c.lineWidth=1.5; c.stroke();
    if (wlf(1) > 0.09) {
      c.fillStyle=cat.color; c.font=`bold ${Math.max(9,Math.round(wlf(18)))}px monospace`;
      c.textAlign='center'; c.shadowColor='rgba(255,255,255,.8)'; c.shadowBlur=3;
      c.fillText(cat.label(p.name), px, py-r-2); c.shadowBlur=0;
    }
  }
}

function drawGameObjects(s) {
  for (const o of s.game_objs) {
    if (o.color==='UNKNOWN'||o.color==='NONE') continue;
    const hex=COLOR_HEX[o.color]||'#888', px=wx(o.x), py=wy(o.y), r=wlen(25);
    ctx.beginPath(); ctx.arc(px,py,r,0,Math.PI*2);
    ctx.fillStyle=hex+'cc'; ctx.fill();
    ctx.strokeStyle=hex; ctx.lineWidth=2; ctx.stroke();
  }
}

function drawRobot(s) {
  const r=s.robot, cx=wx(r.x), cy=wy(r.y), rad=wlen(r.radius), th=-r.theta;
  ctx.beginPath(); ctx.arc(cx,cy,rad,0,Math.PI*2);
  ctx.strokeStyle=r.collided?'rgba(192,57,43,.85)':'rgba(26,82,118,.28)';
  ctx.lineWidth=1.5; ctx.stroke();
  ctx.save(); ctx.translate(cx,cy); ctx.rotate(th);
  if (showRobotImage && robotImg) {
    ctx.rotate(ROBOT_IMG_OFFSET);
    if (r.collided) ctx.globalAlpha=0.55;
    const sz=rad*2.1; ctx.drawImage(robotImg, -sz/2,-sz/2, sz,sz);
    ctx.globalAlpha=1;
  } else {
    const rs=rad*.85;
    ctx.beginPath(); ctx.moveTo(rs,0); ctx.lineTo(-rs*.6,rs*.87); ctx.lineTo(-rs*.6,-rs*.87); ctx.closePath();
    ctx.fillStyle=r.collided?'#e74c3c':(s.team==='yellow'?'#d4ac0d':'#2980b9');
    ctx.fill(); ctx.strokeStyle='rgba(255,255,255,.6)'; ctx.lineWidth=1.5; ctx.stroke();
    ctx.strokeStyle='#fff'; ctx.lineWidth=2;
    ctx.beginPath(); ctx.moveTo(rs*.3,0); ctx.lineTo(rs*.95,0);
    const ar=rs*.15; ctx.lineTo(rs*.85,ar*.5); ctx.moveTo(rs*.95,0); ctx.lineTo(rs*.85,-ar*.5); ctx.stroke();
  }
  ctx.restore();
  if (s.safety.enabled && s.safety.detected) {
    ctx.beginPath(); ctx.arc(cx,cy,rad+wlen(15),0,Math.PI*2);
    ctx.strokeStyle='rgba(180,100,0,.7)'; ctx.lineWidth=3; ctx.setLineDash([5,4]); ctx.stroke(); ctx.setLineDash([]);
  }
}

// ── Map sidebar ────────────────────────────────────────────────────────────
let prevLog='[]', prevObjs='[]';

function updateMapSidebar(s) {
  const r=s.robot;
  setText('s-pos',   `(${r.x.toFixed(0)}, ${r.y.toFixed(0)})`);
  setText('s-theta', `${(r.theta*180/Math.PI).toFixed(1)}°`);
  setText('s-speed', `${Math.hypot(r.vx,r.vy).toFixed(0)} mm/s`);
  const me=document.getElementById('s-mstate');
  if (me) { me.textContent=s.motion.state; me.className='status-val '+(MOTION_CLASS[s.motion.state]||''); }
  const left=s.chrono.left, total=left+s.chrono.elapsed;
  setText('s-time',  `${left.toFixed(1)} s`);
  setText('s-score', `${s.score} pts`);
  const pct=total>0?left/total*100:100;
  const bar=document.getElementById('chrono-bar');
  if (bar) { bar.style.width=pct+'%'; bar.style.background=left<10?'#c0392b':left<30?'#b7770d':'#2980b9'; }
  const lk=JSON.stringify(s.log.length);
  if (lk!==prevLog) { prevLog=lk;
    const logEl=document.getElementById('log-output');
    if (logEl) logEl.innerHTML=[...s.log].reverse().map(l=>{
      let c=''; if(l.includes('✓')||l.includes('SUCCESS'))c='ok'; else if(l.includes('✗')||l.includes('error'))c='err'; else if(l.includes('Stall')||l.includes('warn'))c='warn';
      return `<div class="log-line ${c}">${escHtml(l)}</div>`;
    }).join('');
  }
  updateColorPanel(s.game_objs);
}

function updateColorPanel(objs) {
  const key=JSON.stringify(objs.map(o=>o.color));
  if (key===prevObjs) return; prevObjs=key;
  const list=document.getElementById('color-list');
  if (!list) return;
  list.innerHTML=objs.map(o=>{
    const cat=poiCategory(o.name);
    return `<div class="color-item" onclick="openColorPopup(event,'${o.name}')">
      <div class="color-dot" style="background:${COLOR_HEX[o.color]||'#888'}"></div>
      <span class="color-name" style="color:${cat.color}">${cat.label(o.name)}</span></div>`;
  }).join('');
}

function openColorPopup(e, poi) {
  e.stopPropagation(); activePoi=poi;
  const pop=document.getElementById('color-popup');
  const ch =document.getElementById('color-choices');
  const ttl=document.getElementById('color-popup-title');
  if (!pop||!ch||!ttl) return;
  ttl.textContent='Color: '+poi;
  ch.innerHTML=ALL_COLORS.map(c=>`<div class="color-choice" onclick="applyColor('${c}')">
    <div class="color-dot" style="background:${COLOR_HEX[c]}"></div><span>${c}</span></div>`).join('');
  const rect=e.currentTarget.getBoundingClientRect();
  pop.style.left=`${rect.right+6}px`; pop.style.top=`${Math.min(rect.top,window.innerHeight-260)}px`;
  pop.classList.remove('hidden');
}

function applyColor(c) {
  if (!activePoi) return;
  socket.emit('set_color',{name:activePoi,color:c});
  document.getElementById('color-popup')?.classList.add('hidden'); activePoi=null;
}

function toggleColorPanel() {
  colorPanelOpen=!colorPanelOpen;
  const l=document.getElementById('color-list'); if (l) l.style.display=colorPanelOpen?'':'none';
}
