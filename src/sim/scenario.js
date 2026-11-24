// Lifeguard rescue scenario: swimmer target, 60 s timer, payload drop scoring,
// and the AUTO mission that flies it.

import { TERRAIN } from '../core/terrain.js';

export const HOME_CG_ALT = TERRAIN.padHeight + 0.28; // CG height above sea level on the pad
export const PAYLOAD_OFFSET = 0.12; // payload hangs this far below the CG (m)

export const DEFAULT_SCENARIO = {
  enabled: true,
  swimmerNorth: 800, // [USER] 800 m offshore from the launch point
  swimmerEast: 0,
  timeLimit: 60, // [USER] must reach the swimmer within 60 s
  dropHeight: 5, // [USER] release height above the water (m)
  cruiseAlt: 20, // m above home for the dash
  reachRadius: 10, // "reached" when within this horizontal distance
};

/** Mission altitude (relative to home) that puts the payload at h metres above the water. */
export function altRelForPayloadHeight(h) {
  return h + PAYLOAD_OFFSET - HOME_CG_ALT;
}

/** The rescue mission, in the simulator's ArduPilot-like mission items. */
export function buildRescueMission(sc, fcParams) {
  const n = sc.swimmerNorth, e = sc.swimmerEast;
  const dropAlt = altRelForPayloadHeight(sc.dropHeight);
  return [
    { type: 'VTOL_TAKEOFF', alt: sc.cruiseAlt, label: `VTOL takeoff to ${sc.cruiseAlt} m` },
    { type: 'CHANGE_SPEED', speed: fcParams.DASH_SPEED, label: `Speed ${fcParams.DASH_SPEED} m/s` },
    { type: 'VTOL_APPROACH', n, e, alt: sc.cruiseAlt, label: 'Dash to swimmer, back-transition' },
    { type: 'VTOL_MOVE', n, e, alt: dropAlt, label: `Descend to ${sc.dropHeight} m above water` },
    { type: 'LOITER_TIME', seconds: 2, label: 'Steady hover 2 s' },
    { type: 'PAYLOAD_RELEASE', label: 'Release flotation device' },
    { type: 'LOITER_TIME', seconds: 1, label: 'Hold 1 s' },
    { type: 'VTOL_MOVE', n, e, alt: sc.cruiseAlt, label: `Climb to ${sc.cruiseAlt} m` },
    { type: 'CHANGE_SPEED', speed: fcParams.AIRSPEED_CRUISE, label: `Speed ${fcParams.AIRSPEED_CRUISE} m/s` },
    { type: 'VTOL_LAND', n: 0, e: 0, alt: sc.cruiseAlt, label: 'Return and VTOL land' },
  ];
}

export class RescueScenario {
  constructor(settings = {}) {
    this.settings = { ...DEFAULT_SCENARIO, ...settings };
    this.reset();
  }

  reset() {
    this.startTime = null;
    this.reachTime = null;
    this.release = null; // {time, height, speed, pos}
    this.splash = null; // {time, miss, onLand}
    this.returnTime = null;
    this.energyAtStart = null;
    this.events = [];
  }

  get swimmer() {
    return [this.settings.swimmerNorth, this.settings.swimmerEast];
  }

  log(t, text) {
    this.events.push({ t, text });
  }

  /** Call every physics step. */
  update(t, sim) {
    if (!this.settings.enabled) return;
    const v = sim.vehicle;
    const est = sim.est;
    if (this.startTime == null && sim.fc.armed && est && !est.onGround && est.hagl > 0.5) {
      this.startTime = t;
      this.energyAtStart = v.battery.usedWh;
      this.log(t, 'Lift-off: rescue timer started');
    }
    if (this.startTime == null) return;
    const [sn, se] = this.swimmer;
    const dist = Math.hypot(v.s[0] - sn, v.s[1] - se);
    if (this.reachTime == null && dist < this.settings.reachRadius) {
      this.reachTime = t - this.startTime;
      const ok = this.reachTime <= this.settings.timeLimit;
      this.log(t, `Reached swimmer in ${this.reachTime.toFixed(1)} s ${ok ? '(within limit)' : '(OVER the time limit)'}`);
    }
    if (this.release && this.splash && this.returnTime == null && !sim.fc.armed && est.onGround) {
      const dHome = Math.hypot(v.s[0], v.s[1]);
      if (dHome < 30) {
        this.returnTime = t - this.startTime;
        this.log(t, `Landed back at the beach after ${this.returnTime.toFixed(1)} s`);
      }
    }
  }

  onRelease(t, info) {
    if (this.release) return;
    this.release = { time: this.startTime != null ? t - this.startTime : null, ...info };
    this.log(t, `Released at ${info.height.toFixed(1)} m above the surface, ${info.speed.toFixed(1)} m/s`);
  }

  onSplash(t, info) {
    if (this.splash) return;
    const [sn, se] = this.swimmer;
    const miss = Math.hypot(info.pos[0] - sn, info.pos[1] - se);
    this.splash = { time: t, miss, onLand: info.onLand };
    this.log(t, info.onLand ? 'Flotation device landed on the beach!' : `Flotation device in the water ${miss.toFixed(1)} m from the swimmer`);
  }

  /** Summary for the HUD / results panel. */
  summary(t) {
    const s = this.settings;
    const elapsed = this.startTime != null ? (this.reachTime ?? t - this.startTime) : 0;
    const res = {
      running: this.startTime != null,
      elapsed: this.startTime != null ? t - this.startTime : 0,
      reachTime: this.reachTime,
      timeLimit: s.timeLimit,
      onTime: this.reachTime != null ? this.reachTime <= s.timeLimit : elapsed <= s.timeLimit,
      release: this.release,
      splash: this.splash,
      returnTime: this.returnTime,
    };
    if (this.release && this.splash) {
      const hErr = Math.abs(this.release.height - s.dropHeight);
      const missScore = Math.max(0, 50 - this.splash.miss * 5); // 10 m miss -> 0
      const hScore = Math.max(0, 25 - hErr * 12.5); // 2 m error -> 0
      const tScore = this.reachTime != null ? Math.max(0, Math.min(25, 25 - (this.reachTime - s.timeLimit))) : 0;
      res.score = Math.round(missScore + hScore + tScore);
    }
    return res;
  }
}
