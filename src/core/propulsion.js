// Propulsion: brushless motor + ESC, lift-rotor aerodynamics (blade-element /
// momentum model with vortex-ring-state and ground effect), and the pusher
// propeller (owner's generic Ct/Cp model), plus start-up calibration so the
// model reproduces the supplied thrust and power figures.

import { bisect, clamp, smoothstep, G0 } from './math.js';

const RPM_PER_RADS = 60 / (2 * Math.PI);

// ====================================================================== motor + ESC
export class Motor {
  constructor({ kv, resistance, noLoadCurrent, escEfficiency, brakeCurrent = 15 }) {
    this.kvRad = kv / RPM_PER_RADS; // rad/s per volt
    this.kt = 1 / this.kvRad; // N m / A
    this.R = resistance;
    this.I0 = noLoadCurrent;
    this.eta = escEfficiency;
    this.brakeCurrent = brakeCurrent;
  }

  /** Winding current for ESC duty d (0..1), bus voltage and shaft speed (rad/s). */
  current(d, vBus, omega) {
    const I = (d * vBus - omega / this.kvRad) / this.R;
    return Math.max(I, -this.brakeCurrent);
  }

  /** Net torque delivered to the shaft (electromagnetic minus friction/iron loss). */
  shaftTorque(I, omega) {
    return this.kt * I - this.kt * this.I0 * Math.tanh(omega / 5);
  }

  /** Battery-side current drawn through the ESC. */
  batteryCurrent(d, I) {
    const p = d * I;
    return p >= 0 ? p / this.eta : p * this.eta;
  }

  /** Steady-state operating point for a required shaft torque at a given speed. */
  operatingPoint(omega, torque) {
    const I = torque / this.kt + this.I0;
    const V = I * this.R + omega / this.kvRad;
    return { I, V, Pin: (V * I) / this.eta };
  }
}

// ====================================================================== lift rotor
/** Induced-velocity ratio v_i / v_h for climb ratio x = Vc/vh and edgewise ratio y = Ve/vh. */
function glauertRatio(x, y) {
  // Solve nu * sqrt((x+nu)^2 + y^2) = 1, normal working state root (x + nu > 0)
  let nu = Math.max(1, -x + 1);
  for (let i = 0; i < 20; i++) {
    const s = Math.sqrt((x + nu) * (x + nu) + y * y);
    const f = nu * s - 1;
    const df = s + (nu * (x + nu)) / s;
    const step = f / df;
    nu -= step;
    if (nu < 1e-4) nu = 1e-4;
    if (Math.abs(step) < 1e-9) break;
  }
  return nu;
}

function axialRatio(x) {
  if (x >= 0) return (-x + Math.sqrt(x * x + 4)) / 2;
  if (x <= -2) return (-x - Math.sqrt(x * x - 4)) / 2;
  // Empirical vortex-ring-state curve (Leishman, Principles of Helicopter Aerodynamics)
  return 1 - 1.125 * x - 1.372 * x * x - 1.718 * x * x * x - 0.655 * x * x * x * x;
}

export function inducedRatio(x, y) {
  if (x >= 0) return glauertRatio(x, y);
  const w = Math.exp(-(y / 0.6) * (y / 0.6));
  return w * axialRatio(x) + (1 - w) * glauertRatio(x, y);
}

export class LiftRotorAero {
  constructor(p) {
    this.R = p.radius;
    this.A = Math.PI * p.radius * p.radius;
    this.sigma = p.solidity;
    this.a = p.liftSlope;
    this.kappa = p.kappa;
    this.theta75 = p.theta75 ?? 0.15;
    this.Cd0 = p.Cd0 ?? 0.01;
  }

  /**
   * @param omega rotor speed (rad/s, >= 0)
   * @param vHub  velocity of the hub relative to the air, body axes
   * @param rho   air density
   * @param hGround height of the rotor above the surface (m) for ground effect
   * @param st    per-rotor scratch state {Ct} for warm starting
   * @returns {T, Q, Hx, Hy, vrs, vi}
   */
  compute(omega, vHub, rho, hGround, st) {
    const out = st.out || (st.out = { T: 0, Q: 0, Hx: 0, Hy: 0, vrs: 0, vi: 0 });
    const Vt = omega * this.R;
    if (Vt < 2) {
      out.T = out.Q = out.Hx = out.Hy = out.vrs = out.vi = 0;
      return out;
    }
    const Vc = -vHub[2];
    const Ve = Math.hypot(vHub[0], vHub[1]);
    const mu = Ve / Vt;
    const lc = Vc / Vt;
    const sa2 = (this.sigma * this.a) / 2;
    const thetaTerm = this.theta75 * (1 / 3 + 0.5 * mu * mu);
    let Ct = st.Ct ?? 0.005;
    let li = 0, nu = 1, lh = 0.05;
    for (let i = 0; i < 12; i++) {
      lh = Math.sqrt(Math.max(Ct, 1e-7) / 2);
      nu = inducedRatio(lc / lh, mu / lh);
      li = nu * lh;
      const CtNew = sa2 * (thetaTerm - (lc + li) / 2);
      const dCt = CtNew - Ct;
      Ct += 0.6 * dCt;
      if (Math.abs(dCt) < 1e-8) break;
    }
    st.Ct = Ct;
    // Ground effect (Cheeseman-Bennett with forward-speed correction)
    let ge = 1;
    if (hGround < 4 * this.R) {
      const z = Math.max(hGround, 0.6 * this.R);
      const r4 = this.R / (4 * z);
      const fwd = li > 1e-6 ? (mu / li) * (mu / li) : 0;
      ge = 1 / (1 - (r4 * r4) / (1 + fwd));
    }
    const rhoA = rho * this.A;
    const T = Ct * rhoA * Vt * Vt * ge;
    const Cp = Ct * (lc + this.kappa * li) + (this.sigma * this.Cd0 / 8) * (1 + 4.65 * mu * mu);
    const Q = Cp * rhoA * Vt * Vt * this.R;
    // In-plane H-force (rotor drag) opposing the edgewise flow
    let Hx = 0, Hy = 0;
    if (Ve > 1e-3) {
      const CH = this.sigma * mu * (this.Cd0 / 4 + (this.a * (lc + li) * this.theta75) / 8);
      const H = CH * rhoA * Vt * Vt;
      Hx = (-H * vHub[0]) / Ve;
      Hy = (-H * vHub[1]) / Ve;
    }
    // Vortex-ring-state intensity 0..1 (used for thrust roughness)
    const x = lc / lh;
    const y = mu / lh;
    const vrs = smoothstep(-0.2, -0.5, x) * smoothstep(-1.8, -1.3, x) * Math.exp(-(y / 0.6) * (y / 0.6));
    out.T = T;
    out.Q = Q;
    out.Hx = Hx;
    out.Hy = Hy;
    out.vrs = vrs;
    out.vi = li * Vt;
    return out;
  }
}

// ====================================================================== pusher prop
export class PusherAero {
  constructor(p) {
    this.D = p.diameter;
    this.Ct0 = p.Ct0;
    this.Cp0 = p.Cp0;
    this.J0 = p.J0;
    this.Jp = p.Jp ?? 0.8;
  }

  /** Thrust (N, along +x) and shaft torque (N m) for speed omega (rad/s) and axial inflow V (m/s). */
  compute(omega, V, rho, out = {}) {
    const n = omega / (2 * Math.PI);
    if (n < 0.5) {
      out.T = 0;
      out.Q = 0;
      out.J = 0;
      return out;
    }
    const J = clamp(V / (n * this.D), -0.5, 1.6);
    const Ct = this.Ct0 * (1 - J / this.J0);
    const Cp = Math.max(this.Cp0 * (1 - (J / this.Jp) * (J / this.Jp)), -0.5 * this.Cp0);
    const D4 = this.D ** 4;
    out.T = Ct * rho * n * n * D4;
    out.Q = (Cp * rho * n * n * D4 * this.D) / (2 * Math.PI);
    out.J = J;
    return out;
  }
}

// ====================================================================== calibration
/** Steady speed of a motor+rotor at duty d and bus voltage for a torque function Q(omega). */
export function steadySpeed(motor, d, vBus, torqueFn) {
  const wMax = d * vBus * motor.kvRad;
  if (wMax < 1) return 0;
  const f = (w) => motor.shaftTorque(motor.current(d, vBus, w), w) - torqueFn(w);
  if (f(1) <= 0) return 0;
  return bisect(f, 1, wMax, 1e-6, 80);
}

/**
 * Calibrate the lift-rotor blade pitch (theta75) and profile drag (Cd0) so the
 * rotor+motor reproduces the owner's figures:
 *   - max static thrust per rotor at full throttle (32 N at 22.2 V)
 *   - total hover battery power (1205 W) at design weight and density
 */
export function calibrateLiftRotor(ac) {
  const p = ac.liftRotor;
  const c = p.calib;
  const motor = new Motor(ac.liftMotor);
  const rotor = new LiftRotorAero(p);
  const zero = [0, 0, 0];
  const st = {};
  const thrustAt = (w, rho) => rotor.compute(w, zero, rho, 100, st).T;
  const torqueAt = (w, rho) => rotor.compute(w, zero, rho, 100, st).Q;
  const hoverThrust = (ac.mass * G0) / ac.liftRotors.length;

  const maxThrust = () => {
    const w = steadySpeed(motor, 1, c.maxThrustVoltage, (x) => torqueAt(x, c.maxThrustRho));
    return thrustAt(w, c.maxThrustRho);
  };
  const hoverPower = () => {
    const w = bisect((x) => thrustAt(x, c.hoverRho) - hoverThrust, 10, 3000, 1e-6, 80);
    const Q = torqueAt(w, c.hoverRho);
    const op = motor.operatingPoint(w, Q);
    return { total: op.Pin * ac.liftRotors.length + ac.battery.avionicsPower, w, Q, op };
  };
  const solveTheta = () => {
    rotor.theta75 = bisect(
      (th) => {
        rotor.theta75 = th;
        return maxThrust() - c.maxStaticThrust;
      },
      0.02,
      0.6,
      1e-7,
      60,
    );
  };
  rotor.Cd0 = bisect(
    (cd) => {
      rotor.Cd0 = cd;
      solveTheta();
      return hoverPower().total - c.hoverPowerTotal;
    },
    0.001,
    0.08,
    1e-7,
    50,
  );
  solveTheta();
  p.theta75 = rotor.theta75;
  p.Cd0 = rotor.Cd0;
  const hp = hoverPower();
  return {
    theta75: rotor.theta75,
    Cd0: rotor.Cd0,
    maxThrust: maxThrust(),
    hoverPowerTotal: hp.total,
    hoverRpm: hp.w * RPM_PER_RADS,
    hoverCurrentPerMotor: hp.op.I,
  };
}

/**
 * Calibrate the pusher Cp shape (Jp) so level cruise at the design speed draws
 * the supplied total power (365 W at 20.8 m/s).
 */
export function calibratePusher(ac) {
  const pp = ac.pusher;
  const cc = ac.cruiseCalib;
  const motor = new Motor(ac.pusherMotor);
  const prop = new PusherAero(pp);
  const a = ac.aero;
  const qS = 0.5 * cc.rho * cc.V * cc.V * ac.ref.S;
  const CL = (ac.mass * G0) / qS;
  const drag = qS * (a.CD0 + a.k * CL * CL) * a.dragMultiplier;
  const tmp = {};
  const omegaForThrust = () =>
    bisect((w) => prop.compute(w, cc.V, cc.rho, tmp).T - drag, 50, 3000, 1e-6, 80);
  const power = () => {
    const w = omegaForThrust();
    const Q = prop.compute(w, cc.V, cc.rho, tmp).Q;
    const op = motor.operatingPoint(w, Q);
    return op.Pin + ac.battery.avionicsPower;
  };
  pp.Jp = bisect(
    (jp) => {
      prop.Jp = jp;
      return power() - cc.power;
    },
    0.66,
    3.0,
    1e-7,
    60,
  );
  prop.Jp = pp.Jp;
  const w = omegaForThrust();
  return { Jp: pp.Jp, cruiseDrag: drag, cruiseRpm: w * RPM_PER_RADS, cruisePower: power() };
}

export { RPM_PER_RADS };
