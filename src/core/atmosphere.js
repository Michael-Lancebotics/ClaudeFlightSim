// Atmosphere and wind: air density, mean wind with height shear, Dryden
// turbulence (MIL-F-8785C low-altitude model) and discrete 1-cosine gusts.

import { DEG, clamp, makeRng } from './math.js';

const R_AIR = 287.05;

/** Air density from temperature (deg C), pressure altitude (m) and sea-level pressure (Pa). */
export function airDensity(tempC = 25, altitude = 0, p0 = 101325) {
  const p = p0 * Math.pow(1 - 2.25577e-5 * altitude, 5.25588);
  return p / (R_AIR * (tempC + 273.15));
}

export const TURBULENCE_LEVELS = {
  off: 0,
  light: 1,
  moderate: 2,
  severe: 3,
};

/**
 * Wind field. Settings:
 *   speed      mean wind at 10 m above the surface (m/s)
 *   fromDeg    direction the wind blows FROM (deg true, 0 = from the north / sea)
 *   gustAmp    peak amplitude of discrete gusts (m/s)
 *   turbulence 'off' | 'light' | 'moderate' | 'severe'
 *   shearExp   power-law exponent (0.11 over open sea)
 */
export class Wind {
  constructor(settings = {}, seed = 7) {
    this.settings = {
      speed: 4,
      fromDeg: 0,
      gustAmp: 2,
      turbulence: 'light',
      shearExp: 0.11,
      ...settings,
    };
    this.rng = makeRng(seed);
    this.reset();
  }

  reset() {
    // Dryden filter states (along-wind u, cross-wind v, vertical w)
    this.u = 0;
    this.v1 = 0;
    this.v2 = 0;
    this.w1 = 0;
    this.w2 = 0;
    this.turb = [0, 0, 0];
    this.gust = null;
    this.gustTimer = 4 + 6 * this.rng.uniform();
    this.gustVec = [0, 0, 0];
    this.time = 0;
  }

  /** Mean wind speed at height h above the surface (m). */
  meanSpeedAt(h) {
    const s = this.settings;
    if (s.speed <= 0) return 0;
    return s.speed * Math.pow(Math.max(h, 0.5) / 10, s.shearExp);
  }

  /** Unit vector (NED) of the direction the wind blows TOWARD. */
  get downwind() {
    const a = this.settings.fromDeg * DEG;
    return [-Math.cos(a), -Math.sin(a), 0];
  }

  /**
   * Advance turbulence and gust states.
   * @param dt     time step (s)
   * @param h      height above surface (m)
   * @param airspeed  aircraft airspeed (m/s), sets the Dryden time scale
   */
  update(dt, h, airspeed) {
    this.time += dt;
    const s = this.settings;
    const level = TURBULENCE_LEVELS[s.turbulence] ?? 0;
    const hFt = clamp(h, 3, 300) / 0.3048;
    // MIL-F-8785C low altitude: sigma_w = 0.1 W20; light/moderate/severe W20
    // scale with the mean wind (W20 = wind at 20 ft) times a severity factor.
    const W20 = this.meanSpeedAt(6.1);
    const severity = [0, 1.0, 1.6, 2.4][level];
    const sigmaW = 0.1 * W20 * severity;
    const sigmaUV = sigmaW / Math.pow(0.177 + 0.000823 * hFt, 0.4);
    const Lw = hFt * 0.3048;
    const Luv = (hFt / Math.pow(0.177 + 0.000823 * hFt, 1.2)) * 0.3048;
    const V = Math.max(airspeed, 3, this.meanSpeedAt(h));
    const g = this.rng.gauss;

    if (sigmaW > 0) {
      // u: first-order filter, exact discretisation, stationary std = sigmaUV
      const Tu = Luv / V;
      const au = Math.exp(-dt / Tu);
      this.u = au * this.u + sigmaUV * Math.sqrt(1 - au * au) * g();
      // v, w: Dryden second-order filters (1 + sqrt3 T s)/(1 + T s)^2,
      // driven so that the output variance equals sigma^2 (see docs/MODEL.md)
      const step2 = (state1, state2, sigma, T) => {
        const a = Math.exp(-dt / T);
        // y1 = 1/(1+Ts) w with intensity Q = sigma^2 T; Var(y1) = sigma^2/2
        const n1 = state1 * a + sigma * Math.sqrt((1 - a * a) / 2) * g();
        const n2 = state2 * a + (1 - a) * n1;
        const out = Math.sqrt(3) * n1 + (1 - Math.sqrt(3)) * n2;
        return [n1, n2, out];
      };
      let out;
      [this.v1, this.v2, out] = step2(this.v1, this.v2, sigmaUV, Luv / V);
      const vOut = out;
      [this.w1, this.w2, out] = step2(this.w1, this.w2, sigmaW, Lw / V);
      const wOut = out;
      this.turb = [this.u, vOut, wOut];
    } else {
      this.turb = [0, 0, 0];
    }

    // Discrete 1-cosine gusts at random intervals
    if (s.gustAmp > 0 && s.speed > 0) {
      if (this.gust) {
        this.gust.t += dt;
        const { t, dur, amp, dir } = this.gust;
        if (t >= dur) {
          this.gust = null;
          this.gustVec = [0, 0, 0];
          this.gustTimer = 3 + 9 * this.rng.uniform();
        } else {
          const m = 0.5 * amp * (1 - Math.cos((2 * Math.PI * t) / dur));
          this.gustVec = [dir[0] * m, dir[1] * m, dir[2] * m];
        }
      } else {
        this.gustTimer -= dt;
        if (this.gustTimer <= 0) {
          const amp = s.gustAmp * (0.5 + 0.5 * this.rng.uniform());
          const dw = this.downwind;
          const off = (this.rng.uniform() - 0.5) * 60 * DEG;
          const c = Math.cos(off), sn = Math.sin(off);
          const hx = dw[0] * c - dw[1] * sn;
          const hy = dw[0] * sn + dw[1] * c;
          const vert = (this.rng.uniform() - 0.5) * 0.4;
          this.gust = {
            t: 0,
            dur: 1.5 + 3 * this.rng.uniform(),
            amp,
            dir: [hx, hy, vert],
          };
        }
      }
    } else {
      this.gust = null;
      this.gustVec = [0, 0, 0];
    }
  }

  /** Wind velocity (NED, m/s) at height h above the surface. */
  velocityAt(h) {
    const mean = this.meanSpeedAt(h);
    const dw = this.downwind;
    // turbulence axes: along-wind, cross-wind (to the right of downwind), vertical
    const cw = [-dw[1], dw[0], 0];
    const t = this.turb;
    // vertical turbulence fades out close to the surface
    const wFade = clamp(h / 3, 0, 1);
    return [
      dw[0] * (mean + t[0]) + cw[0] * t[1] + this.gustVec[0],
      dw[1] * (mean + t[0]) + cw[1] * t[1] + this.gustVec[1],
      t[2] * wFade + this.gustVec[2] * wFade,
    ];
  }
}
