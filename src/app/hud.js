// Head-up display: flight data, transition status, battery, wind, rescue
// timer/score, status messages, mini-map, attitude indicator, stick monitor.

import { G0, DEG } from '../core/math.js';
import { AIRCRAFT } from '../core/params.js';
import { shoreNorth } from '../core/terrain.js';
import { CAMERA_LABELS } from './cameras.js';

const el = (tag, cls, parent, html) => {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (html != null) e.innerHTML = html;
  if (parent) parent.appendChild(e);
  return e;
};

const fmtTime = (s) => {
  if (s == null || !isFinite(s)) return '--';
  const m = Math.floor(s / 60);
  const ss = Math.floor(s % 60);
  return `${m}:${String(ss).padStart(2, '0')}`;
};

export class Hud {
  constructor(root) {
    this.root = root;
    this.build();
    this.lastText = 0;
    this.msgCount = 0;
  }

  build() {
    const r = this.root;
    // top centre: mode + transition
    this.top = el('div', 'hud-top', r);
    this.modeEl = el('div', 'hud-mode', this.top, 'QLOITER');
    this.armEl = el('div', 'hud-arm', this.top, 'DISARMED');
    this.transEl = el('div', 'hud-trans', this.top, 'VTOL');
    this.transBar = el('div', 'hud-transbar', this.top);
    this.transFill = el('div', 'hud-transfill', this.transBar);
    this.missionEl = el('div', 'hud-mission', this.top, '');
    // left: speeds
    this.left = el('div', 'hud-panel hud-left', r);
    this.aspdEl = this.big(this.left, 'AIRSPEED', 'm/s');
    this.gspdEl = this.small(this.left, 'GROUNDSPEED', 'm/s');
    this.stallEl = this.small(this.left, 'STALL / TRANSITION', 'm/s');
    this.aoaEl = this.small(this.left, 'ANGLE OF ATTACK', '°');
    // right: altitude
    this.right = el('div', 'hud-panel hud-right', r);
    this.altEl = this.big(this.right, 'ALTITUDE (rel. home)', 'm');
    this.haglEl = this.small(this.right, 'HEIGHT ABOVE SURFACE', 'm');
    this.vsEl = this.small(this.right, 'CLIMB RATE', 'm/s');
    this.hdgEl = this.small(this.right, 'HEADING', '°');
    // attitude indicator
    this.att = el('canvas', 'hud-att', r);
    this.att.width = 160;
    this.att.height = 160;
    // bottom left: battery
    this.batt = el('div', 'hud-panel hud-batt', r);
    el('div', 'hud-label', this.batt, 'BATTERY 6S2P 13 Ah');
    this.battV = el('div', 'hud-batt-v', this.batt, '--');
    this.battBar = el('div', 'hud-battbar', this.batt);
    this.battFill = el('div', 'hud-battfill', this.battBar);
    this.battInfo = el('div', 'hud-batt-info', this.batt, '');
    // bottom right: wind + motors
    this.motors = el('div', 'hud-panel hud-motors', r);
    el('div', 'hud-label', this.motors, 'WIND');
    this.windEl = el('div', 'hud-wind', this.motors, '');
    this.windCanvas = el('canvas', 'hud-windarrow', this.motors);
    this.windCanvas.width = this.windCanvas.height = 56;
    el('div', 'hud-label', this.motors, 'LIFT MOTORS / PUSHER');
    this.motorBars = el('div', 'hud-motorbars', this.motors);
    this.mbars = [];
    for (const lbl of ['1', '2', '3', '4', 'P']) {
      const c = el('div', 'hud-mbar', this.motorBars);
      const f = el('div', 'hud-mfill', c);
      el('div', 'hud-mlbl', c, lbl);
      this.mbars.push(f);
    }
    this.pusherInfo = el('div', 'hud-small', this.motors, '');
    // rescue panel
    this.rescue = el('div', 'hud-panel hud-rescue', r);
    el('div', 'hud-label', this.rescue, 'RESCUE');
    this.rescueTimer = el('div', 'hud-rescue-timer', this.rescue, '0.0 s');
    this.rescueInfo = el('div', 'hud-rescue-info', this.rescue, '');
    // messages
    this.msgs = el('div', 'hud-msgs', r);
    // minimap
    this.map = el('canvas', 'hud-map', r);
    this.map.width = 200;
    this.map.height = 200;
    // sticks
    this.sticks = el('div', 'hud-sticks', r);
    this.stickL = el('div', 'hud-stick', this.sticks);
    this.stickLDot = el('div', 'hud-stickdot', this.stickL);
    this.stickR = el('div', 'hud-stick', this.sticks);
    this.stickRDot = el('div', 'hud-stickdot', this.stickR);
    this.inputEl = el('div', 'hud-input', this.sticks, '');
    // camera label & marker
    this.camEl = el('div', 'hud-cam', r, '');
    this.marker = el('div', 'hud-marker', r, '<div class="hud-marker-box"></div><div class="hud-marker-txt"></div>');
    this.markerTxt = this.marker.querySelector('.hud-marker-txt');
    this.swMarker = el('div', 'hud-marker hud-swmarker', r, '<div class="hud-marker-box"></div><div class="hud-marker-txt">swimmer</div>');
    this.swMarkerTxt = this.swMarker.querySelector('.hud-marker-txt');
    // warnings + crash
    this.warn = el('div', 'hud-warn', r, '');
    this.crashEl = el('div', 'hud-crash hidden', r, '');
    this.pausedEl = el('div', 'hud-paused hidden', r, 'PAUSED (P to resume)');
  }

  big(parent, label, unit) {
    const box = el('div', 'hud-big', parent);
    el('div', 'hud-label', box, label);
    const v = el('div', 'hud-bigval', box, '--');
    el('span', 'hud-unit', box, unit);
    return v;
  }

  small(parent, label, unit) {
    const box = el('div', 'hud-smallrow', parent);
    el('span', 'hud-label', box, label);
    const v = el('span', 'hud-smallval', box, '--');
    el('span', 'hud-unit', box, unit);
    return v;
  }

  update(tel, ctx) {
    const now = performance.now();
    const textDue = now - this.lastText > 80;
    const fc = tel.fc;
    if (textDue) {
      this.lastText = now;
      this.modeEl.textContent = fc.mode;
      this.armEl.textContent = fc.armed ? 'ARMED' : 'DISARMED';
      this.armEl.className = 'hud-arm ' + (fc.armed ? 'armed' : '');
      this.transEl.textContent = fc.transition;
      this.transEl.className = 'hud-trans ' + (fc.assist ? 'assist' : fc.phase === 'fw' ? 'fw' : fc.phase.startsWith('trans') ? 'trans' : '');
      if (fc.transitionPct != null) {
        this.transBar.style.display = 'block';
        this.transFill.style.width = `${fc.transitionPct.toFixed(0)}%`;
      } else this.transBar.style.display = 'none';
      this.missionEl.textContent = fc.missionItem ? `AUTO ${fc.missionItem}` : '';

      // speeds
      const m = tel.payloadAttached ? AIRCRAFT.mass : AIRCRAFT.mass - AIRCRAFT.payloadMass;
      const rho = tel.est.rho;
      const vs = Math.sqrt((2 * m * G0) / (rho * AIRCRAFT.ref.S * AIRCRAFT.aero.CLmax));
      this.aspdEl.textContent = tel.airspeed.toFixed(1);
      const inFw = fc.phase !== 'vtol';
      this.aspdEl.className = 'hud-bigval ' + (inFw && tel.airspeed < vs ? 'bad' : inFw && tel.airspeed < ctx.fbwMin ? 'caution' : '');
      this.gspdEl.textContent = tel.groundspeed.toFixed(1);
      this.stallEl.textContent = `${vs.toFixed(1)} / ${ctx.fbwMin.toFixed(1)}`;
      this.aoaEl.textContent = tel.trueAirspeed > 8 ? (tel.alpha / DEG).toFixed(1) : '--';
      this.altEl.textContent = tel.alt.toFixed(1);
      this.haglEl.textContent = tel.hagl.toFixed(1);
      this.haglEl.className = 'hud-smallval ' + (tel.hagl < 3 && !tel.est.onGround ? 'caution' : '');
      this.vsEl.textContent = (tel.climb >= 0 ? '+' : '') + tel.climb.toFixed(1);
      this.hdgEl.textContent = tel.heading.toFixed(0).padStart(3, '0');

      // battery
      const b = tel.battery;
      this.battV.textContent = `${b.voltage.toFixed(2)} V  (${b.cell.toFixed(2)} V/cell)`;
      this.battV.className = 'hud-batt-v ' + (b.cell < 3.4 ? 'bad' : b.cell < 3.5 ? 'caution' : '');
      const pct = b.soc * 100;
      this.battFill.style.width = `${pct.toFixed(0)}%`;
      this.battFill.className = 'hud-battfill ' + (pct < 20 ? 'bad' : pct < 35 ? 'caution' : '');
      const usable = Math.max(0, (b.soc - 0.2) * 13.0 * 3.75 * 6); // Wh above 20 % reserve
      const pAvg = (this.pAvg = this.pAvg == null ? b.power : this.pAvg + (b.power - this.pAvg) * 0.02);
      const tRem = pAvg > 50 ? (usable / pAvg) * 3600 : null;
      this.battInfo.innerHTML = `${pct.toFixed(0)}% · ${b.current.toFixed(0)} A · ${b.power.toFixed(0)} W<br>${(b.usedAh * 1000).toFixed(0)} mAh used · ${tRem != null ? fmtTime(tRem) : '--'} to 20% reserve`;

      // wind
      const w = tel.wind;
      const ws = Math.hypot(w[0], w[1]);
      const from = ((Math.atan2(-w[1], -w[0]) / DEG) + 360) % 360;
      this.windEl.textContent = `${ws.toFixed(1)} m/s from ${from.toFixed(0).padStart(3, '0')}°${tel.gust > 0 ? ' · GUST' : ''}`;
      this.drawWind(w, tel.est.yaw);

      // motors
      const duties = (tel.rotors || []).map((r) => r.duty);
      duties.push(tel.pusher ? tel.pusher.duty : 0);
      duties.forEach((d, i) => {
        this.mbars[i].style.height = `${Math.min(100, d * 100).toFixed(0)}%`;
        this.mbars[i].className = 'hud-mfill ' + (d > 0.95 ? 'bad' : '');
      });
      const pIinst = tel.pusher ? tel.pusher.I : 0;
      this.pIf = this.pIf == null ? pIinst : this.pIf + (pIinst - this.pIf) * 0.05;
      const pI = this.pIf;
      const pRpm = tel.pusher ? tel.pusher.rpm : 0;
      this.pusherInfo.innerHTML = `Pusher ${pIinst.toFixed(0)} A · ${pRpm.toFixed(0)} rpm` + (pI > AIRCRAFT.pusherMotor.maxCurrent ? ' <span class="bad">&gt; 85 A RATING</span>' : '') + (pRpm > AIRCRAFT.pusher.maxRpmApc ? ' <span class="caution">over APC rpm limit</span>' : '');

      // rescue
      this.updateRescue(tel);
      this.updateMessages(tel.messages);

      // input monitor
      this.inputEl.textContent = ctx.inputLabel;
      this.camEl.textContent = CAMERA_LABELS[ctx.cameraMode] + (ctx.cameraMode === 'los' ? ` · FOV ${ctx.fov.toFixed(0)}° (mouse wheel / +- to zoom)` : '') + (ctx.timeScale !== 1 ? ` · TIME x${ctx.timeScale}` : '');

      // warnings
      const warns = [];
      if (tel.stalled && fc.phase !== 'vtol') warns.push('STALL');
      if (fc.assist) warns.push('Q_ASSIST');
      if (b.cell < 3.5) warns.push('LOW BATTERY');
      if (fc.failsafe.rc) warns.push('RADIO FAILSAFE');
      if (pI > AIRCRAFT.pusherMotor.maxCurrent) warns.push('PUSHER OVERCURRENT');
      if (tel.landingNote && tel.est.onGround) warns.push(tel.landingNote.toUpperCase());
      this.warn.textContent = warns.join('  ·  ');
      this.warn.style.display = warns.length ? 'block' : 'none';

      // crash
      if (tel.crash) {
        this.crashEl.classList.remove('hidden');
        this.crashEl.innerHTML = `<div class="crash-title">CRASH: ${tel.crash.reason}</div><div>${tel.crash.detail}</div><div class="crash-hint">Press <b>R</b> to reset</div>`;
      } else this.crashEl.classList.add('hidden');
      this.pausedEl.classList.toggle('hidden', !ctx.paused);
    }
    this.drawAttitude(tel.est);
    this.drawMap(tel, ctx);
    this.drawSticks(ctx.pilot);
    this.drawMarkers(ctx);
  }

  updateRescue(tel) {
    const s = tel.scenario;
    const lim = s.timeLimit;
    let t;
    if (!s.running) t = 'Ready (timer starts at lift-off)';
    else if (s.reachTime != null) t = `Reached in ${s.reachTime.toFixed(1)} s`;
    else t = `${s.elapsed.toFixed(1)} s / ${lim} s`;
    this.rescueTimer.textContent = t;
    this.rescueTimer.className = 'hud-rescue-timer ' + (!s.running ? 'idle' : !s.onTime ? 'bad' : s.reachTime != null ? 'good' : '');
    const v = tel.est;
    const lines = [];
    const sw = this.swimmer;
    if (sw) {
      const d = Math.hypot(v.pos[0] - sw[0], v.pos[1] - sw[1]);
      lines.push(`Swimmer ${d.toFixed(0)} m`);
    }
    if (s.release) lines.push(`Released at ${s.release.height.toFixed(1)} m${s.release.time != null ? ` (t=${s.release.time.toFixed(1)} s)` : ''}`);
    else if (tel.payloadAttached) lines.push('Flotation device: ON BOARD (B / payload switch)');
    if (s.splash) lines.push(s.splash.onLand ? 'Device landed on the beach' : `Miss distance ${s.splash.miss.toFixed(1)} m`);
    if (s.score != null) lines.push(`<b>Score ${s.score}/100</b>`);
    if (s.returnTime != null) lines.push(`Back on the beach after ${s.returnTime.toFixed(0)} s`);
    this.rescueInfo.innerHTML = lines.join('<br>');
  }

  updateMessages(messages) {
    const last = messages.slice(-6);
    const key = last.map((m) => m.t + m.text).join('|');
    if (key === this.msgKey) return;
    this.msgKey = key;
    this.msgs.innerHTML = last
      .map((m) => `<div class="msg ${m.severity}"><span class="msg-t">${m.t.toFixed(1)}</span> ${m.text}</div>`)
      .join('');
  }

  drawWind(w, yaw) {
    const c = this.windCanvas.getContext('2d');
    const W = this.windCanvas.width;
    c.clearRect(0, 0, W, W);
    c.strokeStyle = 'rgba(255,255,255,0.35)';
    c.beginPath();
    c.arc(W / 2, W / 2, W / 2 - 3, 0, Math.PI * 2);
    c.stroke();
    // arrow points where the wind blows TO, relative to north-up
    const ang = Math.atan2(w[1], w[0]);
    const s = Math.min(1, Math.hypot(w[0], w[1]) / 12);
    c.save();
    c.translate(W / 2, W / 2);
    c.rotate(ang);
    c.strokeStyle = '#7fd3ff';
    c.fillStyle = '#7fd3ff';
    c.lineWidth = 3;
    c.beginPath();
    c.moveTo(0, 18 * s + 4);
    c.lineTo(0, -18 * s - 4);
    c.stroke();
    c.beginPath();
    c.moveTo(0, -22 * s - 6);
    c.lineTo(-6, -12 * s - 2);
    c.lineTo(6, -12 * s - 2);
    c.fill();
    c.restore();
    c.fillStyle = 'rgba(255,255,255,0.6)';
    c.font = '9px sans-serif';
    c.fillText('N', W / 2 - 3, 9);
    void yaw;
  }

  drawAttitude(est) {
    const c = this.att.getContext('2d');
    const W = this.att.width, H = this.att.height;
    c.clearRect(0, 0, W, H);
    c.save();
    c.beginPath();
    c.arc(W / 2, H / 2, W / 2 - 2, 0, Math.PI * 2);
    c.clip();
    c.translate(W / 2, H / 2);
    c.rotate(-est.roll);
    const ppd = 2.2; // pixels per degree of pitch
    const off = (est.pitch / DEG) * ppd;
    c.fillStyle = 'rgba(70,140,220,0.55)';
    c.fillRect(-W, -2 * H + off, 2 * W, 2 * H);
    c.fillStyle = 'rgba(150,105,60,0.55)';
    c.fillRect(-W, off, 2 * W, 2 * H);
    c.strokeStyle = 'white';
    c.lineWidth = 1.5;
    c.beginPath();
    c.moveTo(-W, off);
    c.lineTo(W, off);
    c.stroke();
    c.font = '9px sans-serif';
    c.fillStyle = 'white';
    for (let p = -30; p <= 30; p += 10) {
      if (p === 0) continue;
      const y = off - p * ppd;
      c.beginPath();
      c.moveTo(-18, y);
      c.lineTo(18, y);
      c.stroke();
      c.fillText(String(p), 21, y + 3);
    }
    c.restore();
    c.strokeStyle = '#ffd23f';
    c.lineWidth = 3;
    c.beginPath();
    c.moveTo(W / 2 - 34, H / 2);
    c.lineTo(W / 2 - 10, H / 2);
    c.lineTo(W / 2, H / 2 + 7);
    c.lineTo(W / 2 + 10, H / 2);
    c.lineTo(W / 2 + 34, H / 2);
    c.stroke();
    c.strokeStyle = 'rgba(255,255,255,0.5)';
    c.lineWidth = 1;
    c.beginPath();
    c.arc(W / 2, H / 2, W / 2 - 2, 0, Math.PI * 2);
    c.stroke();
    c.fillStyle = 'white';
    c.font = '11px sans-serif';
    c.fillText(`R ${(est.roll / DEG).toFixed(0)}°  P ${(est.pitch / DEG).toFixed(0)}°`, W / 2 - 34, H - 10);
  }

  drawMap(tel, ctx) {
    const c = this.map.getContext('2d');
    const W = this.map.width, H = this.map.height;
    c.clearRect(0, 0, W, H);
    c.fillStyle = 'rgba(10,30,50,0.55)';
    c.fillRect(0, 0, W, H);
    const scale = ctx.mapScale; // m per pixel
    const cx = W / 2, cy = H * 0.72;
    const toPx = (n, e) => [cx + e / scale, cy - n / scale];
    // beach
    c.fillStyle = 'rgba(233,215,169,0.55)';
    c.beginPath();
    for (let px = 0; px <= W; px += 4) {
      const e = (px - cx) * scale;
      const [, y] = toPx(shoreNorth(e), e);
      if (px === 0) c.moveTo(px, y);
      else c.lineTo(px, y);
    }
    c.lineTo(W, H);
    c.lineTo(0, H);
    c.fill();
    // range rings around home
    c.strokeStyle = 'rgba(255,255,255,0.15)';
    for (const r of [250, 500, 750, 1000]) {
      c.beginPath();
      c.arc(cx, cy, r / scale, 0, Math.PI * 2);
      c.stroke();
    }
    // mission path
    c.strokeStyle = 'rgba(255,210,63,0.5)';
    c.setLineDash([4, 4]);
    const [sn, se] = this.swimmer || [0, 0];
    c.beginPath();
    c.moveTo(...toPx(0, 0));
    c.lineTo(...toPx(sn, se));
    c.stroke();
    c.setLineDash([]);
    // home & swimmer
    c.fillStyle = '#ffd23f';
    c.fillRect(cx - 3, cy - 3, 6, 6);
    const [sx, sy] = toPx(sn, se);
    c.fillStyle = '#ff3355';
    c.beginPath();
    c.arc(sx, sy, 4, 0, Math.PI * 2);
    c.fill();
    // payload
    if (tel.payload) {
      const [px, py] = toPx(tel.payload.pos[0], tel.payload.pos[1]);
      c.fillStyle = '#ff8a00';
      c.beginPath();
      c.arc(px, py, 3, 0, Math.PI * 2);
      c.fill();
    }
    // aircraft
    const p = tel.est.pos;
    const [ax, ay] = toPx(p[0], p[1]);
    c.save();
    c.translate(ax, ay);
    c.rotate(tel.est.yaw);
    c.fillStyle = '#ffffff';
    c.beginPath();
    c.moveTo(0, -7);
    c.lineTo(5, 6);
    c.lineTo(0, 3);
    c.lineTo(-5, 6);
    c.closePath();
    c.fill();
    c.restore();
    c.fillStyle = 'rgba(255,255,255,0.7)';
    c.font = '10px sans-serif';
    c.fillText(`N ↑   ring = 250 m`, 6, 12);
  }

  drawSticks(pilot) {
    if (!pilot) return;
    // Mode 2: left = throttle (y) / yaw (x); right = pitch (y) / roll (x)
    const set = (dot, x, y) => {
      dot.style.left = `${(50 + x * 42).toFixed(1)}%`;
      dot.style.top = `${(50 - y * 42).toFixed(1)}%`;
    };
    set(this.stickLDot, pilot.yaw, pilot.throttle * 2 - 1);
    set(this.stickRDot, pilot.roll, -pilot.pitch);
  }

  drawMarkers(ctx) {
    const m = ctx.marker;
    if (m && ctx.showMarker) {
      this.marker.style.display = 'block';
      this.marker.style.transform = `translate(${m.x.toFixed(0)}px, ${m.y.toFixed(0)}px)`;
      this.markerTxt.textContent = `${m.dist.toFixed(0)} m`;
    } else this.marker.style.display = 'none';
    const s = ctx.swMarker;
    if (s && ctx.showMarker) {
      this.swMarker.style.display = 'block';
      this.swMarker.style.transform = `translate(${s.x.toFixed(0)}px, ${s.y.toFixed(0)}px)`;
      this.swMarkerTxt.textContent = `swimmer ${s.dist.toFixed(0)} m`;
    } else this.swMarker.style.display = 'none';
  }
}
