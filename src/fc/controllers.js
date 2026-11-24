// Control building blocks for the built-in ArduPilot-style flight controller.
//
// The loops mirror ArduPilot's structure (angle -> rate -> output for the
// multicopter side, AP_RollController/AP_PitchController style for the wing,
// TECS for speed/height and L1 for lateral navigation). Gains are scheduled
// from the aircraft model, i.e. they behave like a well-tuned (autotuned)
// aircraft rather than out-of-the-box defaults.

import {
  G0, DEG, clamp, quatMul, eulerToQuat, wrapPi, bisect, sq,
} from '../core/math.js';
import { Motor, LiftRotorAero, PusherAero, steadySpeed } from '../core/propulsion.js';

const RHO_REF = 1.225;
const V_REF = 22.2;

// ====================================================================== thrust models known to the FC
export class LiftThrustModel {
  constructor(ac) {
    const motor = new Motor(ac.liftMotor);
    const rotor = new LiftRotorAero(ac.liftRotor);
    const st = {};
    const zero = [0, 0, 0];
    this.table = []; // [effective duty, static thrust at RHO_REF]
    for (let i = 0; i <= 50; i++) {
      const d = i / 50;
      const w = steadySpeed(motor, d, V_REF, (x) => rotor.compute(x, zero, RHO_REF, 100, st).Q);
      this.table.push([d, rotor.compute(w, zero, RHO_REF, 100, st).T]);
    }
    // yaw torque per newton of thrust around hover (dQ/dT)
    const Th = (ac.mass * G0) / 4;
    const find = (T) => {
      const w = bisect((x) => rotor.compute(x, zero, RHO_REF, 100, st).T - T, 10, 3000, 1e-6, 60);
      return rotor.compute(w, zero, RHO_REF, 100, st).Q;
    };
    this.kq = (find(Th * 1.05) - find(Th * 0.95)) / (0.1 * Th);
  }
  thrust(duty, vBus, rho) {
    const de = clamp((duty * vBus) / V_REF, 0, 1.2);
    const n = this.table.length - 1;
    const x = de * n;
    const i = Math.min(Math.floor(x), n - 1);
    const f = x - i;
    const t0 = this.table[i][1];
    const t1 = i + 1 <= n ? this.table[i + 1][1] : t0;
    return (t0 + (t1 - t0) * f) * (rho / RHO_REF);
  }
  duty(thrust, vBus, rho) {
    const target = (thrust * RHO_REF) / rho;
    const tab = this.table;
    if (target <= 0) return 0;
    let lo = 0, hi = tab.length - 1;
    if (target >= tab[hi][1]) return (V_REF / vBus) * 1.0;
    while (hi - lo > 1) {
      const m = (lo + hi) >> 1;
      if (tab[m][1] < target) lo = m;
      else hi = m;
    }
    const [d0, t0] = tab[lo];
    const [d1, t1] = tab[hi];
    const de = d0 + ((d1 - d0) * (target - t0)) / (t1 - t0);
    return (de * V_REF) / vBus;
  }
}

export class PusherThrustModel {
  constructor(ac) {
    const motor = new Motor(ac.pusherMotor);
    const prop = new PusherAero(ac.pusher);
    this.speeds = [];
    for (let v = 0; v <= 40; v += 2) this.speeds.push(v);
    this.duties = [];
    for (let i = 0; i <= 20; i++) this.duties.push(i / 20);
    this.table = this.speeds.map((V) =>
      this.duties.map((d) => {
        const w = steadySpeed(motor, d, V_REF, (x) => prop.compute(x, V, RHO_REF).Q);
        return prop.compute(w, V, RHO_REF).T;
      }),
    );
  }
  thrust(duty, V, vBus, rho) {
    const de = clamp((duty * vBus) / V_REF, 0, 1);
    const vi = clamp(V / 2, 0, this.speeds.length - 1.001);
    const di = de * (this.duties.length - 1);
    const i0 = Math.floor(vi), j0 = Math.min(Math.floor(di), this.duties.length - 2);
    const fv = vi - i0, fd = di - j0;
    const t = this.table;
    const a = t[i0][j0] + (t[i0][j0 + 1] - t[i0][j0]) * fd;
    const b = t[i0 + 1][j0] + (t[i0 + 1][j0 + 1] - t[i0 + 1][j0]) * fd;
    return (a + (b - a) * fv) * (rho / RHO_REF);
  }
  duty(thrust, V, vBus, rho) {
    if (thrust <= this.thrust(0, V, vBus, rho)) return 0;
    if (thrust >= this.thrust(1, V, vBus, rho)) return 1;
    return bisect((d) => this.thrust(d, V, vBus, rho) - thrust, 0, 1, 1e-4, 30);
  }
}

// ====================================================================== helpers
export class RateLimiter {
  constructor(value = 0) {
    this.value = value;
  }
  update(target, rateUp, rateDown, dt) {
    const d = target - this.value;
    this.value += clamp(d, -rateDown * dt, rateUp * dt);
    return this.value;
  }
}

export function lowpass(prev, x, dt, tau) {
  if (tau <= 0) return x;
  return prev + (x - prev) * Math.min(1, dt / tau);
}

/** Rotation vector (body frame) taking the current attitude q to the target qt. */
function attitudeError(q, qt) {
  const qc = [q[0], -q[1], -q[2], -q[3]];
  let e = quatMul(qc, qt);
  if (e[0] < 0) e = e.map((v) => -v);
  const s = Math.hypot(e[1], e[2], e[3]);
  if (s < 1e-9) return [0, 0, 0];
  const ang = 2 * Math.atan2(s, e[0]);
  return [(e[1] / s) * ang, (e[2] / s) * ang, (e[3] / s) * ang];
}

// ====================================================================== lift-motor mixer
export class LiftMixer {
  constructor(ac, thrustModel) {
    this.ac = ac;
    this.tm = thrustModel;
    this.rotors = ac.liftRotors;
    this.sx2 = this.rotors.reduce((s, r) => s + r.pos[0] * r.pos[0], 0);
    this.sy2 = this.rotors.reduce((s, r) => s + r.pos[1] * r.pos[1], 0);
    this.limited = { rp: false, yaw: false, thrHigh: false, thrLow: false };
  }

  /**
   * @param collective  total thrust along -z body (N)
   * @param torque      [L, M, N] desired body torques (N m)
   * @returns duties[4]
   */
  mix(collective, torque, vBus, rho, spinMin, spinMax) {
    const n = this.rotors.length;
    const tMin = this.tm.thrust(spinMin, vBus, rho);
    const tMax = this.tm.thrust(spinMax, vBus, rho);
    const span = tMax - tMin;
    const rp = [], yw = [];
    for (const r of this.rotors) {
      rp.push((-torque[0] * r.pos[1]) / this.sy2 + (torque[1] * r.pos[0]) / this.sx2);
      yw.push((torque[2] * r.spin) / (n * this.tm.kq));
    }
    const lim = this.limited;
    lim.rp = lim.yaw = lim.thrHigh = lim.thrLow = false;
    // Priorities (as in ArduPilot's matrix mixer): roll/pitch first, then
    // collective thrust, then yaw, with a small reserved yaw headroom.
    const range = (arr) => Math.max(...arr) - Math.min(...arr);
    const yawHeadroom = 0.1 * span;
    const rpRange = range(rp);
    const rpSpan = span - 2 * yawHeadroom;
    if (rpRange > rpSpan) {
      const k = rpSpan / rpRange;
      for (let i = 0; i < n; i++) rp[i] *= k;
      lim.rp = true;
    }
    // collective: keep it inside the band the roll/pitch spread leaves free
    let base = collective / n;
    const lo = tMin - Math.min(...rp);
    const hi = tMax - Math.max(...rp);
    if (base > hi) {
      base = hi;
      lim.thrHigh = true;
    }
    if (base < lo) {
      base = lo;
      lim.thrLow = true;
    }
    // yaw: whatever fits around the collective, but never less than the headroom
    let k = 1;
    for (let i = 0; i < n; i++) {
      const y = yw[i];
      if (y > 1e-9) k = Math.min(k, Math.max(tMax - base - rp[i], 0) / y);
      else if (y < -1e-9) k = Math.min(k, Math.max(base + rp[i] - tMin, 0) / -y);
    }
    const yMax = Math.max(...yw.map(Math.abs));
    if (yMax > 1e-9 && k * yMax < yawHeadroom) k = Math.min(1, yawHeadroom / yMax);
    if (k < 1) lim.yaw = true;
    const comb = rp.map((v, i) => v + k * yw[i]);
    // final shift if the reserved yaw headroom pushed a motor out of range
    const over = Math.max(...comb.map((v) => base + v - tMax), 0);
    const under = Math.max(...comb.map((v) => tMin - base - v), 0);
    base += under - over;
    return comb.map((v) => clamp(this.tm.duty(base + v, vBus, rho), spinMin, spinMax));
  }
}

// ====================================================================== VTOL attitude
export class VtolAttitude {
  constructor(ac) {
    this.I = [ac.Ixx, ac.Iyy, ac.Izz];
    this.kAng = [6, 6, 3];
    this.kRate = [14, 12, 4];
    this.kI = [4, 4, 1];
    this.rateMax = [100 * DEG, 100 * DEG, 60 * DEG];
    this.reset();
  }
  reset() {
    this.integ = [0, 0, 0];
    this.yawTarget = null;
  }
  /**
   * @param q       current attitude quaternion
   * @param omega   body rates
   * @param tgt     {roll, pitch, yaw (abs, rad) | null, yawRate (rad/s)}
   * @param limited mixer saturation flags (anti-windup)
   * @returns torque [L, M, N] (N m) and rate targets
   */
  update(dt, est, tgt, limited) {
    const yawNow = est.yaw;
    if (this.yawTarget == null || tgt.yawFree) this.yawTarget = yawNow;
    if (tgt.yaw != null) this.yawTarget = tgt.yaw;
    else this.yawTarget = wrapPi(this.yawTarget + (tgt.yawRate || 0) * dt);
    // keep the yaw target within reach (weak yaw authority / weathervaning)
    const yErr = wrapPi(this.yawTarget - yawNow);
    if (Math.abs(yErr) > 30 * DEG) this.yawTarget = wrapPi(yawNow + Math.sign(yErr) * 30 * DEG);
    const qt = eulerToQuat(tgt.roll, tgt.pitch, this.yawTarget);
    const e = attitudeError(est.q, qt);
    const yr = tgt.yaw != null ? 0 : tgt.yawRate || 0;
    const sp = Math.sin(est.roll), cp = Math.cos(est.roll);
    const st = Math.sin(est.pitch), ct = Math.cos(est.pitch);
    const ff = [-st * yr, sp * ct * yr, cp * ct * yr];
    const rateT = [0, 0, 0];
    const torque = [0, 0, 0];
    const w = est.omega;
    const sat = limited && (limited.rp || limited.thrHigh);
    for (let i = 0; i < 3; i++) {
      rateT[i] = clamp(this.kAng[i] * e[i] + ff[i], -this.rateMax[i], this.rateMax[i]);
      const err = rateT[i] - w[i];
      const yawSat = i === 2 && limited && limited.yaw;
      if (!(sat || yawSat) && !est.onGround) {
        this.integ[i] = clamp(this.integ[i] + this.kI[i] * err * dt, -3, 3);
      }
      if (est.onGround) this.integ[i] *= 0.95;
      const acc = this.kRate[i] * err + this.integ[i];
      torque[i] = this.I[i] * acc;
    }
    return { torque, rateT, err: e };
  }
}

// ====================================================================== VTOL position
export class VtolPosition {
  constructor(ac) {
    this.m = ac.mass;
    this.kPz = 1.0;
    this.kVz = 5.0;
    this.kIz = 2.0;
    this.kPxy = 1.0;
    this.kVxy = 2.0;
    this.kIxy = 0.8;
    this.reset();
  }
  reset() {
    this.iz = 0;
    this.ixy = [0, 0];
  }
  /**
   * Vertical: returns required upward force (N, world vertical).
   * zT target altitude (rel home, up positive); vzFF climb feed-forward (m/s, up +).
   */
  vertical(dt, est, altT, climbFF, climbMax, sinkMax, mass) {
    const climbT = clamp(this.kPz * (altT - est.alt) + climbFF, -sinkMax, climbMax);
    const climb = -est.vel[2];
    const err = climbT - climb;
    const accT = clamp(this.kVz * err + this.iz, -G0, 6);
    if (!est.onGround) this.iz = clamp(this.iz + this.kIz * err * dt, -9, 6);
    else this.iz *= 0.98;
    return { force: mass * (G0 + accT), climbT };
  }
  /** Horizontal: returns desired NE acceleration. */
  horizontal(dt, est, posT, velFF, accFF, velMax, allowI = true) {
    const vT = [0, 1].map((i) => velFF[i] + this.kPxy * (posT[i] - est.pos[i]));
    const vn = Math.hypot(vT[0], vT[1]);
    if (vn > velMax) {
      vT[0] *= velMax / vn;
      vT[1] *= velMax / vn;
    }
    const acc = [0, 1].map((i) => {
      const err = vT[i] - est.vel[i];
      // integrate only near the target velocity (anti-windup while braking hard)
      if (allowI && !est.onGround && Math.abs(err) < 2) this.ixy[i] = clamp(this.ixy[i] + this.kIxy * err * dt, -4, 4);
      if (est.onGround) this.ixy[i] *= 0.95;
      return this.kVxy * err + this.ixy[i] + accFF[i];
    });
    return { acc, vT };
  }
  /** Convert a desired NE acceleration into lean angles for the current heading. */
  static accelToAngles(aN, aE, yaw, maxAngle) {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const aF = aN * c + aE * s;
    const aR = -aN * s + aE * c;
    let pitch = Math.atan2(-aF, G0);
    let roll = Math.atan2(aR * Math.cos(pitch), G0);
    // limit total lean
    const lean = Math.acos(clamp(Math.cos(roll) * Math.cos(pitch), -1, 1));
    if (lean > maxAngle) {
      const k = maxAngle / lean;
      roll *= k;
      pitch *= k;
    }
    return { roll, pitch };
  }
}

// ====================================================================== fixed-wing attitude
export class FwAttitude {
  constructor(ac) {
    this.ac = ac;
    this.a = ac.aero;
    this.S = ac.ref.S;
    this.b = ac.ref.b;
    this.c = ac.ref.c;
    this.CL0 = ac.aero.trim.CL - ac.aero.CLa * ac.aero.trim.alpha;
    this.Cm0 = -ac.aero.Cma * ac.aero.trim.alpha;
    this.reset();
  }
  reset() {
    this.iRoll = 0;
    this.iPitch = 0;
  }
  /** Angle of attack needed to support load factor n at airspeed V. */
  alphaRequired(V, rho, n, mass) {
    const qS = 0.5 * rho * Math.max(V, 8) ** 2 * this.S;
    const CL = (n * mass * G0) / qS;
    return clamp((CL - this.CL0) / this.a.CLa, -10 * DEG, 11 * DEG);
  }
  /**
   * @returns {da, de, dr} effective surface commands (rad)
   * opts: {rudderIn (-1..1), damper (scale), coordinate (scale), tauRoll, tauPitch, integrate}
   */
  update(dt, est, rollT, pitchT, rho, mass, opts = {}) {
    const a = this.a;
    const V = clamp(est.airspeed, 8, 45);
    const qbar = 0.5 * rho * V * V;
    const qSb = qbar * this.S * this.b;
    const qSc = qbar * this.S * this.c;
    const [p, q, r] = est.omega;
    const phi = est.roll, theta = est.pitch;
    const maxD = a.maxDeflection;
    const integrate = opts.integrate !== false && est.airspeed > 10 && !est.onGround;
    // roll
    const tauR = opts.tauRoll ?? 0.5;
    const pT = clamp(wrapPi(rollT - phi) / tauR, -75 * DEG, 75 * DEG);
    if (integrate) this.iRoll = clamp(this.iRoll + 0.15 * wrapPi(rollT - phi) * dt, -5 * DEG, 5 * DEG);
    let da = (this.ac.Ixx * 8 * (pT - p)) / qSb - (a.Clp * pT * this.b) / (2 * V);
    da = da / a.Clda + this.iRoll;
    // pitch
    const tauP = opts.tauPitch ?? 0.5;
    let qT = clamp((pitchT - theta) / tauP, -60 * DEG, 60 * DEG);
    const cphi = Math.cos(phi);
    if (Math.abs(phi) < 80 * DEG) qT += (G0 / V) * Math.sin(phi) * Math.tan(phi);
    const n = clamp(1 / Math.max(cphi, 0.2), 1, 4);
    const alphaReq = this.alphaRequired(est.airspeed, rho, n, mass);
    if (integrate) this.iPitch = clamp(this.iPitch + 0.2 * (pitchT - theta) * dt, -8 * DEG, 8 * DEG);
    let de = (this.ac.Iyy * 6 * (qT - q)) / qSc - (a.Cmq * qT * this.c) / (2 * V) - (this.Cm0 + a.Cma * alphaReq);
    de = de / a.Cmde - this.iPitch;
    // yaw: coordination feed-forward + damper
    const rT = (opts.coordinate ?? 1) * (G0 / V) * Math.sin(phi) * Math.cos(theta);
    const damp = opts.damper ?? 1;
    let dr = (this.ac.Izz * 2.5 * damp * (rT - r)) / qSb;
    dr += -(a.Cnr * rT * this.b) / (2 * V) * (opts.coordinate ?? 1);
    dr = dr / a.Cndr - (a.Cnda / a.Cndr) * clamp(da, -maxD, maxD) * (opts.coordinate ?? 1);
    dr += (opts.rudderIn || 0) * maxD;
    return { da: clamp(da, -maxD, maxD), de: clamp(de, -maxD, maxD), dr: clamp(dr, -maxD, maxD) };
  }
}

// ====================================================================== TECS
export class Tecs {
  constructor(ac, pusherModel) {
    this.ac = ac;
    this.pm = pusherModel;
    this.CL0 = ac.aero.trim.CL - ac.aero.CLa * ac.aero.trim.alpha;
    this.reset();
  }
  reset(est = null, vMin = 0) {
    this.iThr = 0;
    this.iPitch = 0;
    this.hDem = est ? est.alt : 0;
    this.vDem = Math.max(est ? est.airspeed : 20, vMin);
    this.vdotF = 0;
    this.lastV = est ? est.airspeed : null;
  }
  /**
   * @returns {throttle (0..1), pitch (rad), hdotDem}
   * p = FC params; opts {throttleMin, throttleMax, pitchMin, pitchMax, climbFF}
   */
  update(dt, est, hDemand, vDemand, rho, mass, p, opts = {}) {
    const a = this.ac.aero;
    const V = Math.max(est.airspeed, 5);
    // airspeed rate (filtered)
    if (this.lastV == null) this.lastV = est.airspeed;
    const vdotRaw = (est.airspeed - this.lastV) / dt;
    this.lastV = est.airspeed;
    this.vdotF = lowpass(this.vdotF, vdotRaw, dt, 0.4);
    // demands
    const climbMax = p.TECS_CLMB_MAX, sinkMax = p.TECS_SINK_MAX;
    this.hDem += clamp(hDemand - this.hDem, -sinkMax * dt, climbMax * dt);
    this.vDem += clamp(vDemand - this.vDem, -1.5 * dt, 1.5 * dt);
    const hdotDem = clamp(0.5 * (this.hDem - est.alt) + (opts.climbFF || 0), -sinkMax, climbMax);
    const vdotDem = clamp(0.6 * (this.vDem - est.airspeed), -2, 2);
    const hdot = -est.vel[2];
    // throttle: total energy rate
    const qS = 0.5 * rho * V * V * this.ac.ref.S;
    const phi = est.roll;
    const CL = (mass * G0) / (qS * Math.max(Math.cos(phi), 0.3));
    const drag = qS * (a.CD0 + a.k * CL * CL) * a.dragMultiplier;
    const steDotDem = G0 * hdotDem + V * vdotDem;
    const steDot = G0 * hdot + V * this.vdotF;
    const err = steDotDem - steDot;
    const thrMin = opts.throttleMin ?? p.THR_MIN / 100;
    const thrMax = opts.throttleMax ?? p.THR_MAX / 100;
    let T = drag + (mass * steDotDem) / V + (mass * 0.4 * err) / V + this.iThr;
    let thr = this.pm.duty(T, est.airspeed, est.vBus, rho);
    const satHi = thr >= thrMax, satLo = thr <= thrMin;
    const integ = opts.integrate !== false;
    if (integ && ((err > 0 && !satHi) || (err < 0 && !satLo))) {
      this.iThr = clamp(this.iThr + (mass * 0.1 * err * dt) / V, -15, 15);
    }
    thr = clamp(thr, thrMin, thrMax);
    // pitch: energy balance
    const sebDotDem = G0 * hdotDem - V * vdotDem;
    const sebDot = G0 * hdot - V * this.vdotF;
    const alphaReq = clamp((CL - this.CL0) / a.CLa, -10 * DEG, 11 * DEG);
    const gammaDem = Math.asin(clamp(sebDotDem / (G0 * V), -0.5, 0.5));
    const pErr = (sebDotDem - sebDot) / (G0 * V);
    const pMin = (opts.pitchMin ?? p.LIM_PITCH_MIN) * DEG;
    const pMax = (opts.pitchMax ?? p.LIM_PITCH_MAX) * DEG;
    let pitch = alphaReq + gammaDem + 0.8 * pErr + this.iPitch;
    if (integ && (pitch < pMax || pErr < 0) && (pitch > pMin || pErr > 0)) {
      this.iPitch = clamp(this.iPitch + 0.15 * pErr * dt, -8 * DEG, 8 * DEG);
    }
    pitch = clamp(pitch, pMin, pMax);
    return { throttle: thr, pitch, hdotDem, thrust: T, drag };
  }
}

// ====================================================================== L1 navigation
export class L1Nav {
  constructor() {
    this.period = 14;
    this.damping = 0.75;
  }
  /**
   * Track the line A->B. Returns lateral-acceleration-derived bank demand.
   * pos, vel: NE; A, B: [N, E]
   */
  waypoint(pos, velNE, A, B) {
    const gs = Math.max(Math.hypot(velNE[0], velNE[1]), 3);
    const L1 = Math.max((1 / Math.PI) * this.damping * this.period * gs, 20);
    const ab = [B[0] - A[0], B[1] - A[1]];
    const abLen = Math.hypot(ab[0], ab[1]);
    const u = abLen > 1e-3 ? [ab[0] / abLen, ab[1] / abLen] : [1, 0];
    const ap = [pos[0] - A[0], pos[1] - A[1]];
    const along = ap[0] * u[0] + ap[1] * u[1];
    const xtrack = u[0] * ap[1] - u[1] * ap[0]; // + = right of track
    let target;
    if (Math.abs(xtrack) < L1) {
      const ahead = along + Math.sqrt(L1 * L1 - xtrack * xtrack);
      target = [A[0] + u[0] * ahead, A[1] + u[1] * ahead];
    } else {
      target = [A[0] + u[0] * Math.max(along, 0), A[1] + u[1] * Math.max(along, 0)];
    }
    const distB = Math.hypot(B[0] - pos[0], B[1] - pos[1]);
    // when close to B, aim at B directly
    if (distB < L1) target = B;
    const tp = [target[0] - pos[0], target[1] - pos[1]];
    const vv = Math.hypot(velNE[0], velNE[1]) > 1 ? velNE : [Math.cos(this.headingHint || 0), Math.sin(this.headingHint || 0)];
    const eta = Math.atan2(vv[0] * tp[1] - vv[1] * tp[0], vv[0] * tp[0] + vv[1] * tp[1]);
    const kL1 = 4 * this.damping * this.damping;
    const latAcc = (kL1 * gs * gs * Math.sin(clamp(eta, -Math.PI / 2, Math.PI / 2))) / Math.max(Math.hypot(tp[0], tp[1]), 1);
    return {
      roll: Math.atan(latAcc / G0),
      xtrack,
      along,
      distB,
      passed: along >= abLen,
      bearingB: Math.atan2(B[1] - pos[1], B[0] - pos[0]),
    };
  }
}

export { sq };
