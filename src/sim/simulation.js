// Simulation orchestrator (no DOM): vehicle + environment + flight controller
// + payload + rescue scenario + crash detection, stepped at a fixed rate.

import { G0, DEG, quatToEuler, quatToDcm, mat3MulVec, norm, makeRng } from '../core/math.js';
import { AIRCRAFT } from '../core/params.js';
import { Vehicle } from '../core/vehicle.js';
import { Wind, airDensity } from '../core/atmosphere.js';
import { surfaceHeight, isWater } from '../core/terrain.js';
import { FlightController } from '../fc/flightcontroller.js';
import { DEFAULT_FC_PARAMS } from '../fc/params.js';
import { RescueScenario, buildRescueMission, PAYLOAD_OFFSET, DEFAULT_SCENARIO } from './scenario.js';

export const DEFAULT_ENV = {
  windSpeed: 4, // m/s at 10 m
  windFromDeg: 340, // from the sea, slightly west
  gustAmp: 2,
  turbulence: 'light',
  temperatureC: 25,
  useDesignDensity: false, // true -> 1.083 kg/m^3 (owner's conservative design case)
  dragMultiplier: 1.0,
  vrs: true,
  sensorNoise: true, // GPS / baro / airspeed errors seen by the flight controller
};

export class Simulation {
  constructor({ env = {}, fcParams = {}, scenario = {}, seed = 3 } = {}) {
    this.ac = AIRCRAFT;
    this.envSettings = { ...DEFAULT_ENV, ...env };
    this.fcParams = { ...DEFAULT_FC_PARAMS, ...fcParams };
    this.scenarioSettings = { ...DEFAULT_SCENARIO, ...scenario };
    this.seed = seed;
    this.dt = 1 / 400;
    this.vehicle = new Vehicle(this.ac, { seed });
    this.wind = new Wind({}, seed + 1);
    this.fc = new FlightController(this.ac, this.fcParams);
    this.scenario = new RescueScenario(this.scenarioSettings);
    this.timeScale = 1;
    this.sensorRng = makeRng(seed + 5);
    this.reset();
  }

  get rho() {
    return this.envSettings.useDesignDensity ? this.ac.aero.rhoAnalysis : airDensity(this.envSettings.temperatureC, 0);
  }

  applyEnv() {
    const e = this.envSettings;
    Object.assign(this.wind.settings, {
      speed: e.windSpeed,
      fromDeg: e.windFromDeg,
      gustAmp: e.gustAmp,
      turbulence: e.turbulence,
    });
    this.ac.aero.dragMultiplier = e.dragMultiplier;
    this.vehicle.options.vrs = e.vrs;
  }

  setEnv(env) {
    Object.assign(this.envSettings, env);
    this.applyEnv();
  }

  setFcParams(params) {
    Object.assign(this.fcParams, params);
    this.fc.setParams(params);
    this.rebuildMission();
  }

  setScenario(sc) {
    Object.assign(this.scenarioSettings, sc);
    this.scenario.settings = { ...this.scenarioSettings };
    this.rebuildMission();
  }

  rebuildMission() {
    this.fc.setMission(buildRescueMission(this.scenarioSettings, this.fc.p));
  }

  reset({ yaw = 0, mode = null } = {}) {
    this.applyEnv();
    this.time = 0;
    this.crash = null;
    this.landingNote = null;
    this.vehicle.reset({ yaw });
    this.wind.reset();
    const keepMode = mode || this.fc.mode || 'QLOITER';
    this.fc.reset();
    this.fc.mode = keepMode;
    this.rebuildMission();
    this.scenario.reset();
    this.payload = null;
    this.contactPrev = {};
    this.est = this.estimate();
    this.maxG = 1;
    this.windNow = [0, 0, 0];
    this.sensErr = { gps: [0, 0], baro: 0, aspd: 0 };
  }

  /** Slowly wandering GPS / barometer errors and airspeed-sensor noise. */
  updateSensorErrors(dt) {
    const e = this.sensErr;
    if (!this.envSettings.sensorNoise) {
      e.gps = [0, 0];
      e.baro = 0;
      e.aspd = 0;
      return;
    }
    const g = this.sensorRng.gauss;
    const gm = (x, sigma, tau) => {
      const a = Math.exp(-dt / tau);
      return a * x + sigma * Math.sqrt(1 - a * a) * g();
    };
    e.gps[0] = gm(e.gps[0], 0.5, 25);
    e.gps[1] = gm(e.gps[1], 0.5, 25);
    e.baro = gm(e.baro, 0.3, 12);
    e.aspd = gm(e.aspd, 0.4, 0.3);
  }

  /** Ideal state estimate handed to the flight controller. */
  estimate() {
    const v = this.vehicle;
    const d = v.diag;
    const q = v.quat;
    const e = quatToEuler(q);
    const uAir = d.vAirBody[0];
    const onGround = (d.contacts || []).some((c) => c && c.type === 'gear' && c.Fn > 1);
    const err = this.sensErr || { gps: [0, 0], baro: 0, aspd: 0 };
    const pos = v.pos;
    pos[0] += err.gps[0];
    pos[1] += err.gps[1];
    pos[2] -= err.baro;
    // pitot: reads the forward airflow; noisy and clipped at zero
    const aspd = uAir > 2 ? Math.max(0, uAir + err.aspd) : Math.max(0, uAir);
    return {
      pos,
      vel: v.vel,
      q,
      roll: e.roll,
      pitch: e.pitch,
      yaw: e.yaw,
      omega: v.omega,
      airspeed: aspd,
      alt: v.altitudeRelHome() + err.baro,
      hagl: v.heightAboveSurface(),
      vBus: v.battery.voltage,
      onGround,
      rho: this.rho,
      accel: d.specificForce,
    };
  }

  /** One fixed physics step. pilot = {roll, pitch, yaw, throttle, rcOk, events} */
  step(pilot) {
    if (this.crash) return;
    const dt = this.dt;
    const v = this.vehicle;
    this.updateSensorErrors(dt);
    this.est = this.estimate();
    const out = this.fc.update(dt, this.est, pilot);
    const c = v.cmd;
    for (let i = 0; i < 4; i++) c.lift[i] = out.lift[i];
    c.pusher = out.pusher;
    c.ailL = out.ailL;
    c.ailR = out.ailR;
    c.rvL = out.rvL;
    c.rvR = out.rvR;
    if (this.fc.releaseRequest && v.payloadAttached) this.releasePayload();

    const hgt = Math.max(v.heightAboveSurface(), 0.3);
    this.wind.update(dt, hgt, this.est.airspeed);
    this.windNow = this.wind.velocityAt(hgt);
    const diag = v.step(dt, { rho: this.rho, wind: this.windNow });
    this.time += dt;
    this.updatePayload(dt);
    this.checkCrash(diag);
    this.scenario.update(this.time, this);
  }

  /** Advance by real elapsed time (s), running as many fixed steps as needed. */
  advance(realDt, pilot) {
    const simDt = Math.min(realDt, 0.1) * this.timeScale;
    this.acc = (this.acc || 0) + simDt;
    let n = 0;
    let first = true;
    while (this.acc >= this.dt && n < 200) {
      const p = first ? pilot : { ...pilot, events: [] };
      this.step(p);
      first = false;
      this.acc -= this.dt;
      n++;
    }
    if (first && pilot.events && pilot.events.length) {
      // no step this frame: keep events for the next one
      this.pendingEvents = (this.pendingEvents || []).concat(pilot.events);
    }
    return n;
  }

  // ---------------------------------------------------------------- payload
  releasePayload() {
    const v = this.vehicle;
    const R = quatToDcm(v.quat);
    const off = mat3MulVec(R, [0, 0, PAYLOAD_OFFSET]);
    const pos = [v.s[0] + off[0], v.s[1] + off[1], v.s[2] + off[2]];
    v.payloadAttached = false;
    const height = -pos[2] - surfaceHeight(pos[0], pos[1]);
    const vel = v.vel;
    this.payload = { pos, vel: vel.slice(), inAir: true, t0: this.time };
    this.scenario.onRelease(this.time, { height, speed: norm(vel), pos: pos.slice() });
  }

  /** One integration step of a falling payload {pos, vel}; returns true on impact. */
  stepPayloadBody(pl, dt) {
    const m = this.ac.payloadMass;
    const CdA = 0.035; // tumbling ring buoy / rescue tube
    const h = Math.max(-pl.pos[2] - surfaceHeight(pl.pos[0], pl.pos[1]), 0.3);
    const w = this.wind.velocityAt(h);
    const va = [pl.vel[0] - w[0], pl.vel[1] - w[1], pl.vel[2] - w[2]];
    const V = norm(va);
    const k = (-0.5 * this.rho * CdA * V) / m;
    pl.vel[0] += k * va[0] * dt;
    pl.vel[1] += k * va[1] * dt;
    pl.vel[2] += (k * va[2] + G0) * dt;
    for (let i = 0; i < 3; i++) pl.pos[i] += pl.vel[i] * dt;
    const surf = surfaceHeight(pl.pos[0], pl.pos[1]);
    if (-pl.pos[2] <= surf) {
      pl.pos[2] = -surf;
      return true;
    }
    return false;
  }

  updatePayload(dt) {
    const pl = this.payload;
    if (!pl || !pl.inAir) return;
    if (this.stepPayloadBody(pl, dt)) {
      pl.inAir = false;
      const onLand = !isWater(pl.pos[0], pl.pos[1]);
      this.scenario.onSplash(this.time, { pos: pl.pos.slice(), onLand });
    }
  }

  /** Where the flotation device would land if released right now (NED), or null. */
  predictPayloadImpact() {
    const v = this.vehicle;
    if (!v.payloadAttached) return null;
    const R = quatToDcm(v.quat);
    const off = mat3MulVec(R, [0, 0, PAYLOAD_OFFSET]);
    const pl = { pos: [v.s[0] + off[0], v.s[1] + off[1], v.s[2] + off[2]], vel: v.vel.slice() };
    for (let t = 0; t < 20; t += 0.02) if (this.stepPayloadBody(pl, 0.02)) return { pos: pl.pos, time: t };
    return null;
  }

  // ---------------------------------------------------------------- crash detection
  checkCrash(diag) {
    const v = this.vehicle;
    const est = this.est;
    const prev = this.contactPrev;
    const cur = {};
    // structural load factor from aerodynamic + propulsive forces (ground
    // contact loads are judged by the landing checks below)
    const g = norm(diag.F_body) / (v.mass * G0);
    this.maxG = Math.max(this.maxG, g);
    const airspeed = diag.aero.Va || 0;
    const e = quatToEuler(v.quat);
    const tilt = Math.acos(Math.max(-1, Math.min(1, Math.cos(e.roll) * Math.cos(e.pitch))));
    const crash = (reason, detail = '') => {
      if (this.crash) return;
      this.crash = { reason, detail, time: this.time };
      this.fc.disarm('crash');
      this.fc.crashed = true;
      this.fc.msg(`CRASH: ${reason}`, 'error');
    };
    for (const c of diag.contacts || []) {
      if (!c) continue;
      cur[c.name] = true;
      const first = !prev[c.name];
      if (c.water) {
        crash('Hit the water', `${c.name} touched the sea at ${Math.abs(c.vDown).toFixed(1)} m/s descent, ${c.vTan.toFixed(1)} m/s horizontal`);
        break;
      }
      if (c.type === 'gear' && first) {
        if (c.vDown > 2.5) crash('Hard landing', `touchdown at ${c.vDown.toFixed(1)} m/s vertical (limit 2.5 m/s)`);
        else if (c.vTan > 3.0) crash('Tipped over on touchdown', `skid touched down sliding at ${c.vTan.toFixed(1)} m/s (limit 3 m/s)`);
        else if (tilt > 25 * DEG) crash('Tipped over on touchdown', `touched down tilted ${(tilt / DEG).toFixed(0)} deg (limit 25 deg)`);
        else if (c.vDown > 1.5) this.landingNote = `Firm landing: ${c.vDown.toFixed(1)} m/s`;
      }
      if (c.type === 'structure' && (first || c.vTan > 0.5)) {
        if (c.vDown > 1.0 || c.vTan > 2.0) crash('Struck the ground', `${c.name} hit at ${Math.hypot(c.vDown, c.vTan).toFixed(1)} m/s`);
        else if (tilt > 50 * DEG) crash('Tipped over', `${c.name} on the ground`);
      }
      if (c.type === 'prop') {
        const spinning = c.name.startsWith('pusher') ? v.pusherSpeed > 50 : v.rotorSpeeds.some((w) => w > 50);
        if (spinning) crash('Prop strike', `${c.name} hit the ground`);
      }
    }
    this.contactPrev = cur;
    if (g > 8) crash('Structural failure', `load factor ${g.toFixed(1)} g`);
    if (airspeed > 42) crash('Structural failure', `overspeed ${airspeed.toFixed(0)} m/s`);
  }

  /** Telemetry snapshot for the HUD. */
  telemetry() {
    const v = this.vehicle;
    const d = v.diag;
    const est = this.est;
    const b = v.battery;
    const gs = Math.hypot(est.vel[0], est.vel[1]);
    return {
      time: this.time,
      est,
      airspeed: est.airspeed,
      trueAirspeed: d.aero.Va || 0,
      groundspeed: gs,
      climb: -est.vel[2],
      alt: est.alt,
      hagl: est.hagl,
      heading: ((est.yaw / DEG) + 360) % 360,
      alpha: d.aero.alpha || 0,
      beta: d.aero.beta || 0,
      stalled: (d.aero.sigma || 0) > 0.3 && (d.aero.Va || 0) > 5,
      battery: {
        voltage: b.voltage,
        cell: b.cellVoltage,
        soc: b.soc,
        current: b.current,
        usedAh: b.usedAh,
        usedWh: b.usedWh,
        power: b.power,
      },
      rotors: d.rotors,
      pusher: d.pusher,
      wind: this.windNow,
      gust: this.wind.gust ? this.wind.gust.amp : 0,
      fc: this.fc.status,
      messages: this.fc.messages,
      crash: this.crash,
      landingNote: this.landingNote,
      payload: this.payload,
      payloadAttached: v.payloadAttached,
      scenario: this.scenario.summary(this.time),
      g: norm(d.specificForce) / G0,
    };
  }
}
