// 6-DOF rigid-body vehicle: aerodynamics, four lift rotors, pusher, battery,
// servos, ground/water contact. Integrated with fixed-step RK4.
//
// The vehicle consumes ACTUATOR commands only (ESC duties and surface angles),
// exactly what an autopilot drives, so the same core can be flown by the
// built-in ArduPilot-style controller or (later) by real ArduPilot SITL.

import {
  G0, DEG, clamp, cross, quatMul, quatNormalize, quatToDcm, mat3MulVec, mat3TMulVec,
  mat3Inverse, eulerToQuat, makeRng,
} from './math.js';
import { AIRCRAFT } from './params.js';
import { AeroModel } from './aero.js';
import {
  Motor, LiftRotorAero, PusherAero, calibrateLiftRotor, calibratePusher, RPM_PER_RADS,
} from './propulsion.js';
import { Battery } from './battery.js';
import { surfaceHeight, isWater, TERRAIN } from './terrain.js';

const N_STATE = 18; // pos3 vel3 quat4 omega3 rotor4 pusher1

const calibrated = new WeakSet();
export function ensureCalibrated(ac) {
  if (calibrated.has(ac)) return ac.calibration;
  const lift = calibrateLiftRotor(ac);
  const pusher = calibratePusher(ac);
  ac.calibration = { lift, pusher };
  calibrated.add(ac);
  return ac.calibration;
}

export class Vehicle {
  constructor(ac = AIRCRAFT, opts = {}) {
    this.ac = ac;
    ensureCalibrated(ac);
    this.aero = new AeroModel(ac);
    this.liftAero = new LiftRotorAero(ac.liftRotor);
    this.liftMotor = new Motor(ac.liftMotor);
    this.pusherAero = new PusherAero(ac.pusher);
    this.pusherMotor = new Motor({ ...ac.pusherMotor, brakeCurrent: 0 }); // ESC brake off: prop freewheels
    this.battery = new Battery(ac.battery);
    this.I = [ac.Ixx, 0, -ac.Ixz, 0, ac.Iyy, 0, -ac.Ixz, 0, ac.Izz];
    this.Iinv = mat3Inverse(this.I);
    this.rng = makeRng(opts.seed ?? 11);
    this.options = { vrs: true, ...opts };
    // optional test hook: replace the pusher by a constant body-x force
    this.fixedThrust = null;
    this.reset();
  }

  reset({ north = 0, east = 0, yaw = 0, height = null, speed = 0, soc = 1.0 } = {}) {
    const ground = surfaceHeight(north, east);
    const restZ = -(ground + 0.28 - 0.0046); // skids just touching, gear compressed by its static load
    const z = height == null ? restZ : -(ground + height);
    const q = eulerToQuat(0, 0, yaw);
    const v = [speed * Math.cos(yaw), speed * Math.sin(yaw), 0];
    this.s = new Float64Array(N_STATE);
    this.s.set([north, east, z], 0);
    this.s.set(v, 3);
    this.s.set(q, 6);
    this.time = 0;
    this.payloadAttached = true;
    this.cmd = { lift: [0, 0, 0, 0], pusher: 0, ailL: 0, ailR: 0, rvL: 0, rvR: 0 };
    this.servo = { ailL: 0, ailR: 0, rvL: 0, rvR: 0 };
    this.rotorState = [{}, {}, {}, {}];
    this.vrsNoise = [0, 0, 0, 0];
    this.battery.reset(soc);
    this.diag = null;
    this.env = { rho: 1.18, wind: [0, 0, 0], vBus: this.battery.voltage };
    this.evaluate();
  }

  get mass() {
    return this.ac.mass - (this.payloadAttached ? 0 : this.ac.payloadMass);
  }
  get pos() { return [this.s[0], this.s[1], this.s[2]]; }
  get vel() { return [this.s[3], this.s[4], this.s[5]]; }
  get quat() { return [this.s[6], this.s[7], this.s[8], this.s[9]]; }
  get omega() { return [this.s[10], this.s[11], this.s[12]]; }
  get rotorSpeeds() { return [this.s[13], this.s[14], this.s[15], this.s[16]]; }
  get pusherSpeed() { return this.s[17]; }

  /** Effective control deflections from the four servo positions. */
  surfaces() {
    const sv = this.servo;
    return {
      da: (sv.ailL - sv.ailR) / 2,
      de: (sv.rvL + sv.rvR) / 2,
      dr: (sv.rvL - sv.rvR) / 2,
    };
  }

  /**
   * State derivative. env = {rho, wind (NED), vBus}.
   * When diag is an object it is filled with forces, rotor data and contacts.
   */
  derivatives(s, env, diag = null) {
    const ac = this.ac;
    const d = new Float64Array(N_STATE);
    const q = [s[6], s[7], s[8], s[9]];
    const R = quatToDcm(q);
    const w = [s[10], s[11], s[12]];
    const vel = [s[3], s[4], s[5]];
    const m = this.mass;
    const vAirN = [vel[0] - env.wind[0], vel[1] - env.wind[1], vel[2] - env.wind[2]];
    const vAirB = mat3TMulVec(R, vAirN);

    // ---------- aerodynamics
    const aeroOut = { F: [0, 0, 0], M: [0, 0, 0], info: {} };
    this.aero.compute(vAirB, w, this.surfaces(), env.rho, aeroOut);
    const F = aeroOut.F.slice();
    const M = aeroOut.M.slice();
    const H = [0, 0, 0]; // spinning-rotor angular momentum (body)

    // ---------- lift rotors
    const surfH = surfaceHeight(s[0], s[1]);
    const rotorDiag = diag ? [] : null;
    let battI = 0;
    const lr = ac.liftRotors;
    const J = ac.liftRotor.inertia;
    for (let i = 0; i < lr.length; i++) {
      const r = lr[i].pos;
      const wr = cross(w, r);
      const vHub = [vAirB[0] + wr[0], vAirB[1] + wr[1], vAirB[2] + wr[2]];
      const Om = s[13 + i];
      const hubZ = s[2] + R[6] * r[0] + R[7] * r[1] + R[8] * r[2];
      const hGround = -hubZ - surfH;
      const ra = this.liftAero.compute(Om, vHub, env.rho, hGround, this.rotorState[i]);
      const rough = this.options.vrs ? 1 + 0.18 * ra.vrs * this.vrsNoise[i] : 1;
      const T = ra.T * rough;
      const f = [ra.Hx, ra.Hy, -T];
      F[0] += f[0]; F[1] += f[1]; F[2] += f[2];
      const mr = cross(r, f);
      M[0] += mr[0]; M[1] += mr[1]; M[2] += mr[2];
      const duty = this.cmd.lift[i];
      const I = this.liftMotor.current(duty, env.vBus, Om);
      const Qm = this.liftMotor.shaftTorque(I, Om);
      let dOm = (Qm - ra.Q) / J;
      if (Om <= 0 && dOm < 0) dOm = 0;
      d[13 + i] = dOm;
      M[2] += lr[i].spin * Qm; // reaction torque on the airframe
      H[2] += -lr[i].spin * J * Om;
      if (diag) {
        const bi = this.liftMotor.batteryCurrent(duty, I);
        battI += bi;
        rotorDiag.push({ T, Q: ra.Q, I, battI: bi, rpm: Om * RPM_PER_RADS, vrs: ra.vrs, vi: ra.vi, duty });
      }
    }

    // ---------- pusher
    const pp = ac.pusher;
    const Omp = s[17];
    let pusherT, pusherQ, pusherJ = 0;
    if (this.fixedThrust != null) {
      pusherT = this.fixedThrust;
      pusherQ = 0;
      d[17] = 0;
    } else {
      const pa = this.pusherAero.compute(Omp, vAirB[0], env.rho, {});
      pusherT = pa.T;
      pusherQ = pa.Q;
      pusherJ = pa.J;
      const Ip = this.pusherMotor.current(this.cmd.pusher, env.vBus, Omp);
      const Qmp = this.pusherMotor.shaftTorque(Ip, Omp);
      let dOm = (Qmp - pa.Q) / pp.inertia;
      if (Omp <= 0 && dOm < 0) dOm = 0;
      d[17] = dOm;
      M[0] += pp.spin * Qmp;
      H[0] += -pp.spin * pp.inertia * Omp;
      if (diag) {
        const bi = this.pusherMotor.batteryCurrent(this.cmd.pusher, Ip);
        battI += bi;
        diag.pusher = { T: pusherT, Q: pusherQ, I: Ip, battI: bi, rpm: Omp * RPM_PER_RADS, J: pusherJ, duty: this.cmd.pusher };
      }
    }
    {
      const f = [pusherT, 0, 0];
      F[0] += f[0];
      const mr = cross(pp.pos, f);
      M[0] += mr[0]; M[1] += mr[1]; M[2] += mr[2];
    }

    // ---------- contact with ground / water
    const Fc = [0, 0, 0];
    const contacts = diag ? [] : null;
    for (const cp of ac.contactPoints) {
      const r = cp.pos;
      const pw = [
        s[0] + R[0] * r[0] + R[1] * r[1] + R[2] * r[2],
        s[1] + R[3] * r[0] + R[4] * r[1] + R[5] * r[2],
        s[2] + R[6] * r[0] + R[7] * r[1] + R[8] * r[2],
      ];
      const sh = surfaceHeight(pw[0], pw[1]);
      const depth = pw[2] + sh;
      if (depth <= 0) {
        if (contacts) contacts.push(null);
        continue;
      }
      const wr = cross(w, r);
      const vw = [
        vel[0] + R[0] * wr[0] + R[1] * wr[1] + R[2] * wr[2],
        vel[1] + R[3] * wr[0] + R[4] * wr[1] + R[5] * wr[2],
        vel[2] + R[6] * wr[0] + R[7] * wr[1] + R[8] * wr[2],
      ];
      const water = isWater(pw[0], pw[1]);
      const k = water ? 1500 : 4000;
      const c = water ? 200 : 150;
      const mu = water ? 0.3 : 0.6;
      let Fn = k * depth + c * vw[2];
      if (Fn < 0) Fn = 0;
      const vt = Math.hypot(vw[0], vw[1]);
      const ft = (-mu * Fn) / Math.sqrt(vt * vt + 0.02 * 0.02);
      const fw = [ft * vw[0], ft * vw[1], -Fn];
      Fc[0] += fw[0]; Fc[1] += fw[1]; Fc[2] += fw[2];
      const fb = mat3TMulVec(R, fw);
      const mr = cross(r, fb);
      M[0] += mr[0]; M[1] += mr[1]; M[2] += mr[2];
      if (contacts) contacts.push({ name: cp.name, type: cp.type, depth, vDown: vw[2], vTan: vt, water, Fn });
    }

    // ---------- equations of motion
    const Fn = mat3MulVec(R, F);
    d[0] = vel[0];
    d[1] = vel[1];
    d[2] = vel[2];
    d[3] = (Fn[0] + Fc[0]) / m;
    d[4] = (Fn[1] + Fc[1]) / m;
    d[5] = (Fn[2] + Fc[2]) / m + G0;
    const qd = quatMul(q, [0, w[0], w[1], w[2]]);
    d[6] = 0.5 * qd[0];
    d[7] = 0.5 * qd[1];
    d[8] = 0.5 * qd[2];
    d[9] = 0.5 * qd[3];
    const Iw = mat3MulVec(this.I, w);
    const Htot = [Iw[0] + H[0], Iw[1] + H[1], Iw[2] + H[2]];
    const gyro = cross(w, Htot);
    const wd = mat3MulVec(this.Iinv, [M[0] - gyro[0], M[1] - gyro[1], M[2] - gyro[2]]);
    d[10] = wd[0];
    d[11] = wd[1];
    d[12] = wd[2];

    if (diag) {
      const FcB = mat3TMulVec(R, Fc);
      diag.aero = aeroOut.info;
      diag.aeroF = aeroOut.F;
      diag.aeroM = aeroOut.M;
      diag.F_body = F;
      diag.M_body = M;
      diag.rotors = rotorDiag;
      diag.contacts = contacts;
      diag.contactForce = Fc;
      diag.specificForce = [(F[0] + FcB[0]) / m, (F[1] + FcB[1]) / m, (F[2] + FcB[2]) / m];
      diag.motorBatteryCurrent = battI;
      diag.vAirBody = vAirB;
      diag.dcm = R;
      diag.accelNed = [d[3], d[4], d[5]];
    }
    return d;
  }

  /** Update servo positions toward their commands (rate limit + first-order lag). */
  updateServos(dt) {
    const lim = this.ac.aero.maxDeflection;
    const rate = this.ac.servo.rateLimit * dt;
    const k = Math.min(1, dt / this.ac.servo.tau);
    for (const key of ['ailL', 'ailR', 'rvL', 'rvR']) {
      const target = clamp(this.cmd[key], -lim, lim);
      const cur = this.servo[key];
      this.servo[key] = cur + clamp((target - cur) * k, -rate, rate);
    }
  }

  /** Recompute diagnostics for the current state. */
  evaluate() {
    const diag = {};
    this.derivatives(this.s, this.env, diag);
    this.diag = diag;
    return diag;
  }

  /**
   * Advance one fixed step.
   * @param dt  step (s)
   * @param env {rho, wind:[3]}  (bus voltage is taken from the battery)
   */
  step(dt, env) {
    this.env = { rho: env.rho, wind: env.wind, vBus: this.battery.voltage };
    this.updateServos(dt);
    // VRS roughness noise (low-pass Gaussian per rotor)
    const a = Math.exp(-dt / 0.15);
    for (let i = 0; i < 4; i++) {
      this.vrsNoise[i] = a * this.vrsNoise[i] + Math.sqrt(1 - a * a) * this.rng.gauss();
    }
    const s0 = this.s;
    const e = this.env;
    const k1 = this.derivatives(s0, e);
    const tmp = new Float64Array(N_STATE);
    for (let i = 0; i < N_STATE; i++) tmp[i] = s0[i] + 0.5 * dt * k1[i];
    const k2 = this.derivatives(tmp, e);
    for (let i = 0; i < N_STATE; i++) tmp[i] = s0[i] + 0.5 * dt * k2[i];
    const k3 = this.derivatives(tmp, e);
    for (let i = 0; i < N_STATE; i++) tmp[i] = s0[i] + dt * k3[i];
    const k4 = this.derivatives(tmp, e);
    const s1 = new Float64Array(N_STATE);
    for (let i = 0; i < N_STATE; i++) {
      s1[i] = s0[i] + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    const qn = quatNormalize([s1[6], s1[7], s1[8], s1[9]]);
    s1.set(qn, 6);
    for (let i = 13; i < 18; i++) if (s1[i] < 0) s1[i] = 0;
    this.s = s1;
    this.time += dt;
    const diag = this.evaluate();
    const I = diag.motorBatteryCurrent + this.ac.battery.avionicsPower / Math.max(this.battery.voltage, 10);
    this.battery.update(I, dt);
    diag.batteryCurrent = I;
    return diag;
  }

  /** Height of the CG above the local surface (ground or water). */
  heightAboveSurface() {
    return -this.s[2] - surfaceHeight(this.s[0], this.s[1]);
  }

  /** Altitude relative to the launch pad (ArduPilot "relative altitude"). */
  altitudeRelHome() {
    return -this.s[2] - (TERRAIN.padHeight + 0.28);
  }
}

export { N_STATE, DEG };
