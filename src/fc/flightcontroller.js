// Built-in flight controller that emulates ArduPlane with QuadPlane:
// flight modes, VTOL <-> fixed-wing transitions, Q_ASSIST, arming, failsafes,
// and AUTO missions. It reads an ideal state estimate (as from a converged
// EKF) and writes actuator commands to the vehicle.
//
// This is an approximation of ArduPilot's behaviour for pilot training, not
// a port of its code: mode logic, limits and parameter names follow ArduPilot,
// while gains are scheduled from the aircraft model (like an autotuned plane).

import { G0, DEG, clamp, wrapPi, sq } from '../core/math.js';
import { DEFAULT_FC_PARAMS } from './params.js';
import {
  LiftThrustModel, PusherThrustModel, LiftMixer, VtolAttitude, VtolPosition, FwAttitude,
  Tecs, L1Nav, lowpass,
} from './controllers.js';

export const ALL_MODES = ['MANUAL', 'FBWA', 'CRUISE', 'AUTO', 'RTL', 'QHOVER', 'QLOITER', 'QLAND', 'QRTL'];
const VTOL_MODES = new Set(['QHOVER', 'QLOITER', 'QLAND', 'QRTL']);
const MSG_KEEP = 40;

function thrToClimb(thr, p) {
  if (thr > 0.55) return ((thr - 0.55) / 0.45) * p.Q_PILOT_SPD_UP;
  if (thr < 0.45) return -((0.45 - thr) / 0.45) * p.Q_PILOT_SPD_DN;
  return 0;
}
const deadband = (v, d = 0.03) => (Math.abs(v) < d ? 0 : (v - Math.sign(v) * d) / (1 - d));

export class FlightController {
  constructor(ac, params = {}) {
    this.ac = ac;
    this.p = { ...DEFAULT_FC_PARAMS, ...params };
    this.liftModel = new LiftThrustModel(ac);
    this.pusherModel = new PusherThrustModel(ac);
    this.mixer = new LiftMixer(ac, this.liftModel);
    this.vAtt = new VtolAttitude(ac);
    this.vPos = new VtolPosition(ac);
    this.fwAtt = new FwAttitude(ac);
    this.tecs = new Tecs(ac, this.pusherModel);
    this.l1 = new L1Nav();
    this.messages = [];
    this.mission = [];
    this.reset();
  }

  reset() {
    this.time = 0;
    this.armed = false;
    this.crashed = false;
    this.messages = [];
    this.mode = 'QLOITER';
    this.phase = 'vtol'; // vtol | trans_wait | trans_timer | fw
    this.transStart = 0;
    this.transTimerStart = 0;
    this.assist = false;
    this.assistTimer = 0;
    this.landed = true;
    this.landedTimer = 0;
    this.home = [0, 0];
    this.altT = 0;
    this.posT = [0, 0];
    this.velT = [0, 0];
    this.rollF = 0;
    this.pitchF = 0;
    this.courseLock = null;
    this.courseTimer = 0;
    this.missionIdx = -1;
    this.item = null;
    this.itemState = {};
    this.missionSpeed = this.p.AIRSPEED_CRUISE;
    this.rtl = null;
    this.releaseRequest = false;
    this.lastPusher = 0;
    this.rcLostTime = 0;
    this.failsafe = { rc: false, battLow: false, battCrit: false };
    this.out = { lift: [0, 0, 0, 0], pusher: 0, ailL: 0, ailR: 0, rvL: 0, rvR: 0 };
    this.status = {
      mode: this.mode, armed: false, phase: 'vtol', transition: 'VTOL', transitionPct: null, assist: false,
      landed: true, missionItem: '', failsafe: { rc: false, battLow: false, battCrit: false }, home: [0, 0],
    };
    this.vAtt.reset();
    this.vPos.reset();
    this.fwAtt.reset();
    this.tecs.reset();
  }

  msg(text, severity = 'info') {
    this.messages.push({ t: this.time, text, severity });
    if (this.messages.length > MSG_KEEP) this.messages.shift();
  }

  setParams(params) {
    Object.assign(this.p, params);
  }

  setMission(items) {
    this.mission = items;
  }

  get isVtolMode() {
    return VTOL_MODES.has(this.mode);
  }

  setMode(mode, reason = '') {
    if (!ALL_MODES.includes(mode) || mode === this.mode) return;
    const prev = this.mode;
    this.mode = mode;
    this.msg(`Mode ${mode}${reason ? ' (' + reason + ')' : ''}`);
    this.onModeEnter(prev);
  }

  onModeEnter(prev) {
    const e = this.est;
    if (!e) return;
    this.altT = e.alt;
    this.posT = [e.pos[0], e.pos[1]];
    this.courseLock = null;
    this.rtl = null;
    if (this.mode === 'QLOITER' || this.mode === 'QLAND') {
      // keep current velocity and brake from it (back-transition braking)
      this.velT = [e.vel[0], e.vel[1]];
    } else {
      this.velT = [0, 0];
    }
    if (this.mode === 'AUTO') this.startMission();
    if (this.mode === 'RTL' || this.mode === 'QRTL') this.startRtl(this.mode === 'QRTL');
    if (!VTOL_MODES.has(prev) && this.isVtolMode) this.vAtt.yawTarget = null;
    this.tecs.reset(e);
  }

  // ---------------------------------------------------------------- arming
  tryArm() {
    if (this.armed) return;
    const e = this.est;
    if (this.pilot && this.pilot.throttle > 0.12 && this.mode !== 'AUTO') {
      this.msg('Arm: throttle not low', 'warn');
      return;
    }
    if (this.crashed) {
      this.msg('Arm: vehicle crashed - reset first', 'warn');
      return;
    }
    this.armed = true;
    this.armTime = this.time;
    this.landed = !!e.onGround;
    this.landedTimer = 0;
    this.home = [e.pos[0], e.pos[1]];
    this.vAtt.reset();
    this.vPos.reset();
    this.fwAtt.reset();
    this.altT = e.alt;
    this.posT = [e.pos[0], e.pos[1]];
    this.velT = [0, 0];
    this.phase = e.airspeed > this.p.ARSPD_FBW_MIN && !this.isVtolMode ? 'fw' : 'vtol';
    this.msg('Armed');
    if (this.mode === 'AUTO') this.startMission();
  }

  disarm(reason = '') {
    if (!this.armed) return;
    this.armed = false;
    const airborne = this.est && !this.est.onGround && this.est.hagl > 0.6;
    this.msg(`Disarmed${reason ? ' (' + reason + ')' : ''}${airborne ? ' - IN FLIGHT, motors stopped' : ''}`, airborne ? 'error' : 'info');
  }

  // ---------------------------------------------------------------- main update
  /**
   * @param est   state estimate (see Simulation.estimate)
   * @param pilot {roll, pitch, yaw, throttle (0..1), rcOk, events: [{type, mode}]}
   */
  update(dt, est, pilot) {
    this.time += dt;
    this.est = est;
    this.pilot = pilot;
    this.releaseRequest = false;
    const p = this.p;

    for (const ev of pilot.events || []) {
      if (ev.type === 'arm') this.tryArm();
      else if (ev.type === 'disarm') this.disarm('switch');
      else if (ev.type === 'mode') this.setMode(ev.mode, ev.source || '');
      else if (ev.type === 'release') {
        this.releaseRequest = true;
      }
    }

    this.checkFailsafes(dt, est, pilot);

    const out = this.out;
    if (!this.armed) {
      this.lastPusher = 0;
      this.disarmedOutputs(est, pilot);
      this.updateStatus(est);
      return out;
    }

    // --- run the mode: produces a demand {wantFw, fw:{roll,pitch,throttle,rudder}, vtol:{...}}
    const dmd = this.runMode(dt, est, pilot);

    // --- transitions
    this.updatePhase(dt, est, dmd);

    // --- outputs
    this.computeOutputs(dt, est, dmd);
    // THR_SLEWRATE: limit how fast the pusher throttle may change
    const slew = (p.THR_SLEWRATE / 100) * dt;
    if (p.THR_SLEWRATE > 0 && !dmd.manual) {
      out.pusher = clamp(out.pusher, this.lastPusher - slew, this.lastPusher + slew);
    }
    this.lastPusher = out.pusher;
    this.updateStatus(est, dmd);
    return out;
  }

  disarmedOutputs(est, pilot) {
    const out = this.out;
    out.lift = [0, 0, 0, 0];
    out.pusher = 0;
    const md = this.ac.aero.maxDeflection;
    this.setSurfaces(pilot.roll * md, -pilot.pitch * md, pilot.yaw * md);
    this.phase = this.isVtolMode || est.airspeed < this.p.ARSPD_FBW_MIN ? 'vtol' : 'fw';
    this.landed = true;
    this.vAtt.reset();
    this.vPos.reset();
  }

  setSurfaces(da, de, dr) {
    const out = this.out;
    out.ailL = da;
    out.ailR = -da;
    out.rvL = de + dr;
    out.rvR = de - dr;
  }

  // ---------------------------------------------------------------- failsafes
  checkFailsafes(dt, est, pilot) {
    const p = this.p;
    // radio
    if (pilot.rcOk === false) {
      this.rcLostTime += dt;
      if (!this.failsafe.rc && this.rcLostTime > p.FS_LONG_TIMEOUT) {
        this.failsafe.rc = true;
        this.msg('Radio failsafe', 'error');
        if (this.armed && !est.onGround) {
          if (p.FS_LONG_ACTN === 1) this.setMode('RTL', 'radio failsafe');
          else if (p.FS_LONG_ACTN === 2) this.setMode('QLAND', 'radio failsafe');
        }
      }
    } else {
      if (this.failsafe.rc) this.msg('Radio regained');
      this.failsafe.rc = false;
      this.rcLostTime = 0;
    }
    // battery (voltage filtered over ~5 s like ArduPilot's BATT_LOW_TIMER)
    this.vFilt = this.vFilt == null ? est.vBus : lowpass(this.vFilt, est.vBus, dt, 3);
    if (!this.failsafe.battLow && this.vFilt < p.BATT_LOW_VOLT && this.time > 5) {
      this.failsafe.battLow = true;
      this.msg(`Battery low ${this.vFilt.toFixed(1)} V`, 'warn');
      if (this.armed && p.BATT_FS_LOW_ACT === 1) this.setMode('RTL', 'battery');
      if (this.armed && p.BATT_FS_LOW_ACT === 2) this.setMode('QLAND', 'battery');
    }
    if (!this.failsafe.battCrit && this.vFilt < p.BATT_CRT_VOLT && this.time > 5) {
      this.failsafe.battCrit = true;
      this.msg(`Battery CRITICAL ${this.vFilt.toFixed(1)} V`, 'error');
      if (this.armed && p.BATT_FS_CRT_ACT === 1) this.setMode('RTL', 'battery');
      if (this.armed && p.BATT_FS_CRT_ACT === 2) this.setMode('QLAND', 'battery');
    }
  }

  // ---------------------------------------------------------------- modes
  runMode(dt, est, pilot) {
    switch (this.mode) {
      case 'MANUAL': return this.modeManual(est, pilot);
      case 'FBWA': return this.modeFbwa(dt, est, pilot);
      case 'CRUISE': return this.modeCruise(dt, est, pilot);
      case 'QHOVER': return this.modeQhover(dt, est, pilot);
      case 'QLOITER': return this.modeQloiter(dt, est, pilot, false);
      case 'QLAND': return this.modeQloiter(dt, est, pilot, true);
      case 'QRTL':
      case 'RTL': return this.runRtl(dt, est, pilot);
      case 'AUTO': return this.runMission(dt, est, pilot);
      default: return this.modeQhover(dt, est, pilot);
    }
  }

  rollLimit(est) {
    const p = this.p;
    let lim = p.LIM_ROLL_CD * DEG;
    if (p.STALL_PREVENTION) {
      const n = sq(est.airspeed / Math.max(p.ARSPD_FBW_MIN, 1));
      lim = n <= 1 ? Math.min(lim, 25 * DEG) : Math.min(lim, Math.max(25 * DEG, Math.acos(1 / n)));
    }
    return lim;
  }

  modeManual(est, pilot) {
    return { wantFw: true, manual: true, fw: { throttle: pilot.throttle * (this.p.THR_MAX / 100) } };
  }

  modeFbwa(dt, est, pilot) {
    const p = this.p;
    const roll = deadband(pilot.roll) * this.rollLimit(est);
    let pitch = pilot.pitch >= 0 ? pilot.pitch * p.LIM_PITCH_MAX * DEG : pilot.pitch * -p.LIM_PITCH_MIN * DEG;
    pitch -= p.STAB_PITCH_DOWN * DEG * clamp(1 - pilot.throttle / 0.5, 0, 1);
    return {
      wantFw: true,
      fw: { roll, pitch, throttle: pilot.throttle * (p.THR_MAX / 100), rudder: deadband(pilot.yaw) },
      climbForAssist: clamp((pitch / (15 * DEG)) * 2, -2, 2),
    };
  }

  modeCruise(dt, est, pilot) {
    const p = this.p;
    // airspeed from throttle stick, altitude from pitch stick
    const vDem = p.ARSPD_FBW_MIN + clamp(pilot.throttle, 0, 1) * (p.ARSPD_FBW_MAX - p.ARSPD_FBW_MIN);
    const climb = deadband(pilot.pitch, 0.05) * p.FBWB_CLIMB_RATE;
    this.altT += climb * dt;
    this.altT = clamp(this.altT, est.alt - 15, est.alt + 15);
    const tec = this.tecs.update(dt, est, this.altT, vDem, est.rho, this.ac.mass, p, { climbFF: climb, integrate: this.phase === 'fw' });
    let roll;
    const rIn = deadband(pilot.roll, 0.05);
    if (rIn !== 0 || this.phase !== 'fw') {
      roll = rIn * this.rollLimit(est);
      this.courseLock = null;
      this.courseTimer = 0;
    } else {
      this.courseTimer += dt;
      if (!this.courseLock && this.courseTimer > 0.5) {
        const crs = Math.atan2(est.vel[1], est.vel[0]);
        const A = [est.pos[0], est.pos[1]];
        this.courseLock = { A, B: [A[0] + 5000 * Math.cos(crs), A[1] + 5000 * Math.sin(crs)] };
        this.msg('CRUISE: course locked');
      }
      roll = this.courseLock ? this.l1.waypoint(est.pos, est.vel, this.courseLock.A, this.courseLock.B).roll : 0;
      roll = clamp(roll, -this.rollLimit(est), this.rollLimit(est));
    }
    return {
      wantFw: true,
      fw: { roll, pitch: tec.pitch, throttle: tec.throttle, rudder: deadband(pilot.yaw), autoThrottle: true },
      transAlt: this.altT,
      climbForAssist: tec.hdotDem,
    };
  }

  /** Shared vertical handling for pilot-controlled Q modes. */
  pilotVertical(dt, est, pilot, landing) {
    const p = this.p;
    let climb;
    if (landing) {
      climb = est.alt > p.Q_LAND_FINAL_ALT ? -p.Q_WP_SPEED_DN : -p.Q_LAND_SPEED;
    } else {
      climb = thrToClimb(pilot.throttle, p);
    }
    // landed / take-off logic
    if (this.landed) {
      if (!landing && climb > 0.2) {
        this.landed = false;
        this.msg('Takeoff');
      }
    } else if (est.onGround && climb <= 0 && -est.vel[2] < 0.3) {
      this.landedTimer += dt;
      if (this.landedTimer > 0.3) {
        this.landed = true;
        this.landedTimer = 0;
        this.msg('Landed');
      }
    } else {
      this.landedTimer = 0;
    }
    if (this.landed) {
      this.altT = est.alt;
      // auto-disarm: after a landing, or after 10 s sitting on the ground with the throttle low
      const lowThr = pilot.throttle < 0.1;
      this.landedTimer += landing || lowThr ? dt : 0;
      const delay = landing ? p.LAND_DISARMDELAY : 10;
      if ((landing || lowThr) && this.landedTimer > delay) this.disarm(landing ? 'landed' : 'auto disarm');
    } else {
      this.altT += climb * dt;
      this.altT = clamp(this.altT, est.alt - 3, est.alt + 3);
    }
    return climb;
  }

  modeQhover(dt, est, pilot) {
    const p = this.p;
    const climb = this.pilotVertical(dt, est, pilot, false);
    const amax = p.Q_ANGLE_MAX * DEG;
    this.rollF = lowpass(this.rollF, deadband(pilot.roll) * amax, dt, p.Q_A_INPUT_TC);
    this.pitchF = lowpass(this.pitchF, deadband(pilot.pitch) * amax, dt, p.Q_A_INPUT_TC);
    this.posT = [est.pos[0], est.pos[1]];
    return {
      wantFw: false,
      vtol: {
        roll: this.rollF,
        pitch: this.pitchF,
        yawRate: deadband(pilot.yaw, 0.05) * p.Q_YAW_RATE_MAX * DEG,
        alt: this.altT,
        climbFF: climb,
        fwdThr: p.Q_FWD_THR_USE === 2,
      },
    };
  }

  modeQloiter(dt, est, pilot, landing) {
    const p = this.p;
    const climb = this.pilotVertical(dt, est, pilot, landing);
    // stick -> desired velocity in the heading frame
    const c = Math.cos(est.yaw), s = Math.sin(est.yaw);
    const vF = -deadband(pilot.pitch, 0.05) * p.Q_LOIT_SPEED;
    const vR = deadband(pilot.roll, 0.05) * p.Q_LOIT_SPEED;
    const vDes = [vF * c - vR * s, vF * s + vR * c];
    const sticks = vF !== 0 || vR !== 0;
    const braking = this.phaseWasFw ? p.Q_TRANS_DECEL : p.Q_LOIT_BRK_ACCEL;
    const acc = sticks ? p.Q_LOIT_ACC_MAX : braking;
    const dv = [vDes[0] - this.velT[0], vDes[1] - this.velT[1]];
    const dvn = Math.hypot(dv[0], dv[1]);
    const step = Math.min(dvn, acc * dt);
    const accFF = [0, 0];
    if (dvn > 1e-6) {
      this.velT[0] += (dv[0] / dvn) * step;
      this.velT[1] += (dv[1] / dvn) * step;
      accFF[0] = (dv[0] / dvn) * (step / dt);
      accFF[1] = (dv[1] / dvn) * (step / dt);
    }
    if (Math.hypot(this.velT[0], this.velT[1]) < 0.3) this.phaseWasFw = false;
    this.posT[0] += this.velT[0] * dt;
    this.posT[1] += this.velT[1] * dt;
    this.leashPos(est, 6);
    if (this.landed) {
      this.posT = [est.pos[0], est.pos[1]];
      this.velT = [0, 0];
    }
    const vmax = Math.max(p.Q_LOIT_SPEED, Math.hypot(this.velT[0], this.velT[1])) + 3;
    const h = this.vPos.horizontal(dt, est, this.posT, this.velT, accFF, vmax, !this.landed);
    let yawRate = deadband(pilot.yaw, 0.05) * p.Q_YAW_RATE_MAX * DEG;
    return this.vtolFromAccel(est, h.acc, this.altT, climb, yawRate, true, { braking: this.phaseWasFw });
  }

  /**
   * Move the altitude target toward alt with a sqrt (constant-deceleration)
   * profile limited by Q_WP_SPEED_UP/DN. Returns the climb-rate feed-forward.
   */
  rampAlt(dt, alt, accel = 1.0) {
    const p = this.p;
    const dz = alt - this.altT;
    if (Math.abs(dz) < 1e-4) {
      this.altT = alt;
      return 0;
    }
    const vmax = dz > 0 ? p.Q_WP_SPEED_UP : p.Q_WP_SPEED_DN;
    const v = Math.sign(dz) * Math.min(vmax, Math.sqrt(2 * accel * Math.abs(dz)));
    if (Math.abs(v * dt) >= Math.abs(dz)) this.altT = alt;
    else this.altT += v * dt;
    return v;
  }

  leashPos(est, leash) {
    const d = [this.posT[0] - est.pos[0], this.posT[1] - est.pos[1]];
    const dn = Math.hypot(d[0], d[1]);
    if (dn > leash) {
      this.posT[0] = est.pos[0] + (d[0] * leash) / dn;
      this.posT[1] = est.pos[1] + (d[1] * leash) / dn;
    }
  }

  /** Build a VTOL demand from a desired NE acceleration (position modes). */
  vtolFromAccel(est, acc, alt, climbFF, yawRate, positionMode, opts = {}) {
    const p = this.p;
    const ang = VtolPosition.accelToAngles(acc[0], acc[1], est.yaw, p.Q_ANGLE_MAX * DEG);
    let pitch = ang.pitch;
    // Q_BCK_PIT_LIM applies while the wing is still flying; once slow, the
    // normal VTOL lean limit is available for braking (e.g. in a tailwind).
    if (opts.braking && est.airspeed > 0.6 * p.ARSPD_FBW_MIN) pitch = Math.min(pitch, p.Q_BCK_PIT_LIM * DEG);
    // With the wing flying (lift motors near idle), nose-up braking makes the
    // aircraft balloon. Once well above the target height, ease the nose down
    // toward the 1 g angle of attack so the wing stops climbing.
    const above = est.alt - alt;
    const bThr = 5;
    if (est.airspeed > 8 && above > bThr && this.mixer.limited.thrLow) {
      const a1g = this.fwAtt.alphaRequired(est.airspeed, est.rho, 1, this.ac.mass);
      const lim = a1g + 2 * DEG - 0.01 * (above - bThr) - 0.03 * Math.max(-est.vel[2], 0);
      pitch = Math.min(pitch, Math.max(lim, -p.Q_FWD_PIT_LIM * DEG));
    }
    const wv = p.Q_WVANE_ENABLE && positionMode && yawRate === 0 && !this.landed;
    if (wv) yawRate = clamp(p.Q_WVANE_GAIN * ang.roll, -20 * DEG, 20 * DEG);
    // While the wing is still flying fast, let the heading follow the airflow
    // (the rudder and wing keep it coordinated) instead of fighting it with
    // the weak differential-thrust yaw of the lift motors.
    const yawFree = est.airspeed > 12 && !this.landed;
    return {
      wantFw: false,
      vtol: {
        roll: ang.roll,
        pitch,
        yaw: yawFree ? null : opts.yaw ?? null,
        yawFree,
        yawRate: yawFree ? 0 : yawRate,
        alt,
        climbFF,
        fwdThr: p.Q_FWD_THR_USE >= 1 && positionMode,
      },
    };
  }

  // ---------------------------------------------------------------- mission / RTL
  startMission() {
    if (!this.mission || this.mission.length === 0) {
      this.msg('AUTO: no mission', 'warn');
      return;
    }
    this.missionIdx = -1;
    this.missionSpeed = this.p.AIRSPEED_CRUISE;
    this.missionDone = false;
    this.nextItem();
  }

  nextItem() {
    this.missionIdx++;
    if (this.missionIdx >= this.mission.length) {
      this.item = null;
      if (!this.missionDone) this.msg('Mission complete');
      this.missionDone = true;
      return;
    }
    this.item = this.mission[this.missionIdx];
    this.itemState = { t0: this.time, start: this.est ? [this.est.pos[0], this.est.pos[1]] : [0, 0] };
    this.msg(`Mission ${this.missionIdx + 1}/${this.mission.length}: ${this.item.label || this.item.type}`);
  }

  runMission(dt, est, pilot) {
    // execute instantaneous items until a navigation item is active
    let guard = 0;
    while (this.item && guard++ < 10) {
      const it = this.item;
      if (it.type === 'CHANGE_SPEED') {
        this.missionSpeed = it.speed;
        this.nextItem();
      } else if (it.type === 'PAYLOAD_RELEASE') {
        this.releaseRequest = true;
        this.msg('Payload release');
        this.nextItem();
      } else break;
    }
    if (!this.item) return this.holdVtol(dt, est);
    const r = this.runNavItem(dt, est, this.item, this.itemState);
    if (r.done) this.nextItem();
    return r.dmd;
  }

  startRtl(vtolOnly) {
    const e = this.est;
    const p = this.p;
    const dist = Math.hypot(this.home[0] - e.pos[0], this.home[1] - e.pos[1]);
    const useFw = !vtolOnly && p.Q_RTL_MODE === 1 && (this.phase !== 'vtol' || dist > 300);
    const alt = useFw ? p.RTL_ALTITUDE : Math.max(e.alt, p.Q_RTL_ALT);
    this.rtl = {
      item: { type: 'VTOL_LAND', n: this.home[0], e: this.home[1], alt, speed: p.AIRSPEED_CRUISE, vtolOnly: !useFw },
      state: { t0: this.time, start: [e.pos[0], e.pos[1]] },
    };
  }

  runRtl(dt, est, pilot) {
    if (!this.rtl) this.startRtl(this.mode === 'QRTL');
    const r = this.runNavItem(dt, est, this.rtl.item, this.rtl.state);
    return r.dmd;
  }

  holdVtol(dt, est) {
    const h = this.vPos.horizontal(dt, est, this.posT, [0, 0], [0, 0], this.p.Q_WP_SPEED, !this.landed);
    if (this.landed) {
      this.landedTimer += dt;
      if (this.landedTimer > this.p.LAND_DISARMDELAY) this.disarm('mission complete');
    }
    return this.vtolFromAccel(est, h.acc, this.altT, 0, 0, true);
  }

  /** Fixed-wing navigation toward B along A->B at altitude/speed. */
  fwNav(dt, est, A, B, alt, speed) {
    const nav = this.l1.waypoint(est.pos, est.vel, A, B);
    const lim = this.rollLimit(est);
    const tec = this.tecs.update(dt, est, alt, speed, est.rho, this.ac.mass, this.p, { integrate: this.phase === 'fw' });
    return {
      nav,
      dmd: {
        wantFw: true,
        fw: { roll: clamp(nav.roll, -lim, lim), pitch: tec.pitch, throttle: tec.throttle, rudder: 0, autoThrottle: true },
        transAlt: alt,
        climbForAssist: tec.hdotDem,
      },
    };
  }

  /** VTOL move toward a 3-D point at Q_WP_SPEED. */
  vtolMove(dt, est, target, alt, st, speed = this.p.Q_WP_SPEED) {
    const p = this.p;
    const d = [target[0] - est.pos[0], target[1] - est.pos[1]];
    const dist = Math.hypot(d[0], d[1]);
    // horizontal velocity profile: accelerate, cruise, decelerate (sqrt controller)
    const vmag = Math.min(speed, Math.sqrt(2 * p.Q_LOIT_BRK_ACCEL * Math.max(dist - 0.3, 0)));
    const vff = dist > 0.5 ? [(d[0] / dist) * vmag, (d[1] / dist) * vmag] : [0, 0];
    this.posT = dist > 3 ? [est.pos[0], est.pos[1]] : [target[0], target[1]];
    if (dist <= 3) {
      vff[0] *= 0.3;
      vff[1] *= 0.3;
    }
    const climbFF = this.rampAlt(dt, alt);
    const h = this.vPos.horizontal(dt, est, this.posT, vff, [0, 0], speed + 3, true);
    const yaw = st.holdYaw ?? null;
    const dmd = this.vtolFromAccel(est, h.acc, this.altT, climbFF, 0, true, { yaw });
    const gs = Math.hypot(est.vel[0], est.vel[1]);
    const altDone = Math.abs(this.altT - alt) < 0.01 && Math.abs(est.alt - alt) < 0.3;
    return { dmd, dist, arrived: dist < 1.5 && altDone && gs < 1.0 };
  }

  runNavItem(dt, est, it, st) {
    const p = this.p;
    switch (it.type) {
      case 'VTOL_TAKEOFF': {
        if (st.phase == null) {
          st.phase = est.onGround ? 'spool' : 'climb';
          this.posT = [est.pos[0], est.pos[1]];
          st.holdYaw = est.yaw;
          this.altT = est.alt;
        }
        if (st.phase === 'spool') {
          this.landed = true;
          if (this.time - st.t0 > 1.0) {
            st.phase = 'climb';
            this.landed = false;
            this.msg('VTOL takeoff');
          }
          return { done: false, dmd: this.vtolFromAccel(est, [0, 0], est.alt, 0, 0, true, { yaw: st.holdYaw }) };
        }
        this.landed = false;
        const climbFF = this.rampAlt(dt, it.alt);
        const h = this.vPos.horizontal(dt, est, this.posT, [0, 0], [0, 0], p.Q_WP_SPEED, true);
        const dmd = this.vtolFromAccel(est, h.acc, this.altT, climbFF, 0, true, { yaw: st.holdYaw });
        return { done: est.alt >= it.alt - 0.5, dmd };
      }
      case 'WAYPOINT': {
        const B = [it.n, it.e];
        const r = this.fwNav(dt, est, st.start, B, it.alt, it.speed ?? this.missionSpeed);
        return { done: r.nav.distB < p.WP_RADIUS || r.nav.passed, dmd: r.dmd };
      }
      case 'VTOL_MOVE': {
        if (st.holdYaw == null) st.holdYaw = est.yaw;
        if (st.alt0 == null) this.altT = est.alt;
        st.alt0 = true;
        const r = this.vtolMove(dt, est, [it.n, it.e], it.alt, st);
        return { done: r.arrived, dmd: r.dmd };
      }
      case 'LOITER_TIME': {
        if (st.hold == null) {
          // hold the previous navigation target (not wherever we happen to be)
          st.hold = [this.posT[0], this.posT[1]];
          st.holdYaw = est.yaw;
          st.alt = this.altT;
        }
        const r = this.vtolMove(dt, est, st.hold, st.alt, st);
        return { done: this.time - st.t0 >= it.seconds, dmd: r.dmd };
      }
      case 'VTOL_APPROACH':
      case 'VTOL_LAND':
        return this.runApproach(dt, est, it, st);
      default:
        return { done: true, dmd: this.holdVtol(dt, est) };
    }
  }

  /**
   * Fixed-wing approach to a point, planned back-transition (stopping distance
   * from Q_TRANS_DECEL), braking in VTOL, then hover (VTOL_APPROACH) or land
   * (VTOL_LAND). Mirrors ArduPlane's NAV_VTOL_LAND approach sequence.
   */
  runApproach(dt, est, it, st) {
    const p = this.p;
    const B = [it.n, it.e];
    const d = [B[0] - est.pos[0], B[1] - est.pos[1]];
    const dist = Math.hypot(d[0], d[1]);
    const alt = it.alt ?? est.alt;
    if (st.phase == null) {
      const farEnough = dist > 150 && !it.vtolOnly;
      st.phase = this.phase === 'vtol' && !farEnough ? 'move' : 'fw';
      if (this.phase === 'vtol' && !farEnough) {
        st.holdYaw = Math.atan2(d[1], d[0]);
        this.altT = est.alt;
      }
    }
    const landing = it.type === 'VTOL_LAND';
    if (st.phase === 'fw') {
      const speed = it.speed ?? this.missionSpeed;
      const r = this.fwNav(dt, est, st.start, B, alt, speed);
      const gs = Math.hypot(est.vel[0], est.vel[1]);
      const stopDist = (gs * gs) / (2 * p.Q_TRANS_DECEL) + gs * 1.0;
      const remaining = Math.max(Math.hypot(B[0] - est.pos[0], B[1] - est.pos[1]), 0);
      if (this.phase === 'fw' && remaining <= stopDist) {
        st.phase = 'decel';
        st.v0 = gs;
        st.dir = [d[0] / Math.max(dist, 1e-3), d[1] / Math.max(dist, 1e-3)];
        st.holdYaw = est.yaw;
        this.altT = est.alt;
        this.msg('VTOL approach: back-transition');
      }
      return { done: false, dmd: r.dmd };
    }
    if (st.phase === 'decel') {
      // along-track distance remaining (negative once we overshoot)
      const along = d[0] * st.dir[0] + d[1] * st.dir[1];
      const cross = -d[0] * st.dir[1] + d[1] * st.dir[0];
      const vmax = along >= 0 ? st.v0 : p.Q_WP_SPEED;
      const vmag = Math.sign(along) * Math.min(vmax, Math.sqrt(2 * p.Q_TRANS_DECEL * Math.max(Math.abs(along) - 0.5, 0)));
      const vc = clamp(0.5 * cross, -3, 3); // pull back onto the approach line
      const vff = [st.dir[0] * vmag - st.dir[1] * vc, st.dir[1] * vmag + st.dir[0] * vc];
      this.posT = dist > 4 ? [est.pos[0], est.pos[1]] : [B[0], B[1]];
      const climbFF = this.rampAlt(dt, alt);
      const h = this.vPos.horizontal(dt, est, this.posT, vff, [0, 0], st.v0 + 3, true);
      const dmd = this.vtolFromAccel(est, h.acc, this.altT, climbFF, 0, true, { yaw: st.holdYaw, braking: true });
      const gs = Math.hypot(est.vel[0], est.vel[1]);
      if (dist < 2.5 && gs < 1.5) {
        st.phase = landing ? 'land' : 'hold';
        this.posT = [B[0], B[1]];
      }
      return { done: false, dmd };
    }
    if (st.phase === 'move') {
      const r = this.vtolMove(dt, est, B, alt, st);
      if (r.arrived) st.phase = landing ? 'land' : 'hold';
      return { done: false, dmd: r.dmd };
    }
    if (st.phase === 'hold') {
      const r = this.vtolMove(dt, est, B, alt, st);
      return { done: true, dmd: r.dmd };
    }
    // land
    const desc = est.alt > p.Q_LAND_FINAL_ALT ? p.Q_WP_SPEED_DN : p.Q_LAND_SPEED;
    if (!this.landed) {
      // descend the target at the landing rate, leashed close to the aircraft
      this.altT = clamp(this.altT - desc * dt, est.alt - 0.5, est.alt + 1);
      if (est.onGround && -est.vel[2] < 0.3) {
        this.landedTimer += dt;
        if (this.landedTimer > 0.3) {
          this.landed = true;
          this.landedTimer = 0;
          this.msg('Landed');
        }
      } else this.landedTimer = 0;
    } else {
      this.altT = est.alt;
      this.landedTimer += dt;
      if (this.landedTimer > p.LAND_DISARMDELAY) {
        this.disarm('landed');
        if (this.mode === 'AUTO') this.missionDone = true;
      }
    }
    this.posT = [B[0], B[1]];
    const h = this.vPos.horizontal(dt, est, this.posT, [0, 0], [0, 0], p.Q_WP_SPEED, !this.landed);
    const dmd = this.vtolFromAccel(est, h.acc, this.altT, -desc, 0, true, { yaw: st.holdYaw ?? null });
    return { done: false, dmd };
  }

  // ---------------------------------------------------------------- transition state machine
  updatePhase(dt, est, dmd) {
    const p = this.p;
    const prev = this.phase;
    if (dmd.manual) {
      this.phase = 'fw';
    } else if (dmd.wantFw) {
      if (this.phase === 'vtol') {
        if (est.onGround) {
          // no fixed-wing take-off from skids: stay put
        } else {
          this.phase = est.airspeed >= p.ARSPD_FBW_MIN ? 'trans_timer' : 'trans_wait';
          this.transStart = this.time;
          this.transTimerStart = this.time;
          this.transAlt = dmd.transAlt ?? est.alt;
          this.vPos.iz = Math.max(this.vPos.iz, 0);
          this.tecs.reset(est, p.ARSPD_FBW_MIN);
          this.msg('Transition to fixed-wing');
        }
      }
      if (this.phase === 'trans_wait') {
        if (est.airspeed >= p.ARSPD_FBW_MIN) {
          this.phase = 'trans_timer';
          this.transTimerStart = this.time;
          this.msg(`Transition airspeed reached ${est.airspeed.toFixed(1)} m/s`);
        } else if (p.Q_TRANS_FAIL > 0 && this.time - this.transStart > p.Q_TRANS_FAIL && !this.transFailWarned) {
          this.transFailWarned = true;
          this.msg('Transition failure: airspeed not reached', 'warn');
        }
      }
      if (this.phase === 'trans_timer' && this.time - this.transTimerStart >= p.Q_TRANSITION_MS / 1000) {
        this.phase = 'fw';
        this.tecs.reset(est, p.ARSPD_FBW_MIN);
        this.tecs.hDem = dmd.transAlt ?? est.alt;
        this.msg('Transition done');
      }
    } else if (this.phase !== 'vtol') {
      this.phase = 'vtol';
      this.phaseWasFw = true;
      this.msg('Back-transition to VTOL');
      // seed the VTOL controllers for a smooth take-over
      this.vAtt.reset();
      this.vPos.iz = -G0 * 0.5;
      this.altT = est.alt;
    }
    if (prev !== this.phase && this.phase === 'fw') this.transFailWarned = false;

    // Q_ASSIST in fixed-wing flight
    const fwModeAssist = this.phase === 'fw' && !dmd.manual && p.Q_ASSIST_SPEED > 0;
    if (fwModeAssist) {
      const attErr = Math.max(Math.abs(wrapPi((dmd.fw.roll ?? 0) - est.roll)), Math.abs((dmd.fw.pitch ?? 0) - est.pitch));
      const want = est.airspeed < p.Q_ASSIST_SPEED || (p.Q_ASSIST_ANGLE > 0 && attErr > p.Q_ASSIST_ANGLE * DEG);
      if (want) {
        this.assistTimer += dt;
        if (!this.assist && this.assistTimer > p.Q_ASSIST_DELAY) {
          this.assist = true;
          this.assistAlt = est.alt;
          const V = est.airspeed;
          const vs = p.ARSPD_FBW_MIN / 1.2;
          this.vPos.iz = -G0 * clamp(sq(V / vs), 0, 0.95);
          this.vAtt.reset();
          this.msg(`Q_ASSIST active (${V.toFixed(1)} m/s)`, 'warn');
        }
      } else {
        this.assistTimer = 0;
        if (this.assist && est.airspeed > p.Q_ASSIST_SPEED + 1) {
          this.assist = false;
          this.msg('Q_ASSIST off');
        }
      }
    } else {
      this.assist = false;
      this.assistTimer = 0;
    }
  }

  // ---------------------------------------------------------------- outputs
  computeOutputs(dt, est, dmd) {
    const p = this.p;
    const out = this.out;
    const rho = est.rho;
    const m = this.ac.mass;
    const md = this.ac.aero.maxDeflection;
    const pilot = this.pilot;

    if (dmd.manual) {
      out.lift = [0, 0, 0, 0];
      out.pusher = clamp(dmd.fw.throttle, 0, 1);
      this.setSurfaces(pilot.roll * md, -pilot.pitch * md, pilot.yaw * md);
      return;
    }

    const runLift = (att, altT, climbFF, scale, climbMax, sinkMax) => {
      const a = this.vAtt.update(dt, est, att, this.mixer.limited);
      const v = this.vPos.vertical(dt, est, altT, climbFF, climbMax, sinkMax, m);
      const tilt = Math.max(Math.cos(est.roll) * Math.cos(est.pitch), 0.5);
      let coll = (v.force / tilt) * scale;
      const torque = a.torque.map((t) => t * Math.max(scale, 0));
      return { coll, torque };
    };

    const spinMin = p.Q_M_SPIN_MIN, spinMax = p.Q_M_SPIN_MAX;
    const surfShare = (torque) => {
      // let the surfaces deliver what they can of the demanded torque
      const V = est.airspeed;
      if (V < 4) return { torque, surf: { da: 0, de: 0, dr: 0 } };
      const a = this.ac.aero;
      const qS = 0.5 * rho * V * V * this.ac.ref.S;
      const b = this.ac.ref.b, c = this.ac.ref.c;
      const da = clamp(torque[0] / (qS * b * a.Clda), -md, md);
      const de = clamp(torque[1] / (qS * c * a.Cmde), -md, md);
      const dr = clamp(torque[2] / (qS * b * a.Cndr), -md, md);
      const eff = 0.8; // conservative estimate of what the surfaces deliver
      const rem = [
        torque[0] - eff * qS * b * a.Clda * da,
        torque[1] - eff * qS * c * a.Cmde * de,
        torque[2] - eff * qS * b * a.Cndr * dr,
      ];
      return { torque: rem, surf: { da, de, dr } };
    };

    if (this.phase === 'vtol') {
      const v = dmd.vtol;
      if (this.landed) {
        out.lift = [p.Q_M_SPIN_ARM, p.Q_M_SPIN_ARM, p.Q_M_SPIN_ARM, p.Q_M_SPIN_ARM];
        out.pusher = 0;
        this.setSurfaces(0, 0, 0);
        this.vAtt.reset();
        this.vPos.reset();
        return;
      }
      let pitch = v.pitch;
      let fwd = 0;
      const lim = p.Q_FWD_PIT_LIM * DEG;
      if (v.fwdThr && pitch < -lim) {
        const T = (p.Q_FWD_THR_GAIN / 2) * m * G0 * (Math.tan(-pitch) - Math.tan(lim));
        fwd = clamp(this.pusherModel.duty(T, est.airspeed, est.vBus, rho), 0, p.THR_MAX / 100);
        pitch = -lim;
      }
      const L = runLift({ roll: v.roll, pitch, yaw: v.yaw, yawRate: v.yawRate, yawFree: v.yawFree }, v.alt, v.climbFF, 1,
        p.Q_PILOT_SPD_UP + 1, p.Q_PILOT_SPD_DN + 1);
      const sh = surfShare(L.torque);
      out.lift = this.mixer.mix(L.coll, sh.torque, est.vBus, rho, spinMin, spinMax);
      out.pusher = fwd;
      this.setSurfaces(sh.surf.da, sh.surf.de, sh.surf.dr);
      this.fwAtt.reset();
      return;
    }

    // fixed-wing demands (FW, transition)
    const f = dmd.fw;
    const yawDamp = p.YAW2SRV_DAMP;
    if (this.phase === 'trans_wait') {
      const tr = 15 * DEG;
      const att = {
        roll: clamp(f.roll ?? 0, -tr, tr),
        pitch: clamp(f.pitch ?? 0, -p.Q_TRAN_PIT_MAX * DEG, p.Q_TRAN_PIT_MAX * DEG),
        yawRate: 0,
        yawFree: this.time - this.transStart > 0.5 && est.airspeed > 8,
      };
      const L = runLift(att, this.transAlt, 0, 1, 2, 2);
      const sh = surfShare(L.torque);
      out.lift = this.mixer.mix(L.coll, sh.torque, est.vBus, rho, spinMin, spinMax);
      out.pusher = clamp(f.autoThrottle ? p.THR_MAX / 100 : f.throttle, 0, p.THR_MAX / 100);
      this.setSurfaces(sh.surf.da, sh.surf.de, sh.surf.dr + (f.rudder || 0) * md);
      return;
    }

    const fwOut = this.fwAtt.update(dt, est, f.roll, f.pitch, rho, m, { rudderIn: f.rudder, damper: yawDamp, coordinate: p.YAW2SRV_RLL });
    this.setSurfaces(fwOut.da, fwOut.de, fwOut.dr);
    out.pusher = clamp(f.throttle, 0, 1);

    if (this.phase === 'trans_timer') {
      const k = clamp(1 - (this.time - this.transTimerStart) / (p.Q_TRANSITION_MS / 1000), 0, 1);
      const L = runLift({ roll: f.roll, pitch: f.pitch, yawRate: 0, yawFree: true }, this.transAlt, 0, k, 2, 2);
      out.lift = k > 0.02 ? this.mixer.mix(L.coll, L.torque, est.vBus, rho, 0, spinMax) : [0, 0, 0, 0];
      return;
    }
    if (this.assist) {
      const climb = clamp(dmd.climbForAssist ?? 0, -2, 2);
      this.assistAlt += climb * dt;
      this.assistAlt = clamp(this.assistAlt, est.alt - 3, est.alt + 3);
      const L = runLift({ roll: f.roll, pitch: f.pitch, yawRate: 0, yawFree: true }, this.assistAlt, climb, 1, 2.5, 2.5);
      out.lift = this.mixer.mix(Math.max(L.coll, 0), L.torque, est.vBus, rho, 0, spinMax);
      return;
    }
    out.lift = [0, 0, 0, 0];
  }

  // ---------------------------------------------------------------- status for the HUD
  updateStatus(est, dmd) {
    const p = this.p;
    let trans = '';
    let transPct = null;
    switch (this.phase) {
      case 'vtol': trans = this.armed && this.landed ? 'VTOL (landed)' : 'VTOL'; break;
      case 'trans_wait':
        trans = 'TRANSITION: building airspeed';
        transPct = clamp(est.airspeed / p.ARSPD_FBW_MIN, 0, 1) * 80;
        break;
      case 'trans_timer':
        trans = 'TRANSITION: lift motors ramping down';
        transPct = 80 + 20 * clamp((this.time - this.transTimerStart) / (p.Q_TRANSITION_MS / 1000), 0, 1);
        break;
      case 'fw': trans = this.assist ? 'FIXED-WING + Q_ASSIST' : 'FIXED-WING'; break;
      default: trans = this.phase;
    }
    if (this.mode === 'MANUAL') trans = 'FIXED-WING (MANUAL, lift motors off)';
    let item = '';
    if (this.mode === 'AUTO') {
      item = this.item ? `${this.missionIdx + 1}/${this.mission.length} ${this.item.label || this.item.type}` : this.missionDone ? 'complete' : '';
    }
    this.status = {
      mode: this.mode,
      armed: this.armed,
      phase: this.phase,
      transition: trans,
      transitionPct: transPct,
      assist: this.assist,
      landed: this.landed,
      missionItem: item,
      failsafe: { ...this.failsafe },
      home: this.home,
    };
  }
}
