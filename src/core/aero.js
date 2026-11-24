// Aerodynamic model.
//
// Two models are blended:
//  1. ATTACHED FLOW - the flow5 stability-derivative model supplied by the
//     owner, used unchanged near the trim condition (blend factor exactly 0
//     for |alpha| below the stall onset and |beta| below 12 deg, so the linear
//     dynamics are exactly those of the flow5 analysis).
//  2. SEPARATED FLOW - a component flat-plate / bluff-body model valid for any
//     flow direction (stall, hover in gusty wind, flying backwards). It uses
//     the local velocity at each component, so it also supplies rotational
//     damping at zero airspeed.
// The stall onset is calibrated at start-up so the blended lift curve peaks at
// exactly CLmax = 1.08 (stall at 16.7 m/s at design weight and density).

import { clamp, cross, dot, norm, smoothstep, DEG } from './math.js';

function makeComponents(ac) {
  const wingX = ac.wing.leX - ac.wing.chord / 2;
  const halfArea = ac.ref.S / 2;
  const tailHalf = ac.tail.area / 2;
  const panelSpan = tailHalf / ac.tail.chord;
  const ga = ac.tail.panelAngle;
  const tailX = -ac.tail.arm - 0.25 * ac.tail.chord;
  const tailCy = 0.5 * panelSpan * Math.cos(ga);
  const tailCz = ac.tail.z - 0.5 * panelSpan * Math.sin(ga);
  const cn = ac.aero.flatPlateCN;
  const plates = [
    { name: 'wingL', r: [wingX, -ac.ref.b / 4, ac.wing.z], A: halfArea, n: [0, 0, 1], CN: cn, Cf: 0.012 },
    { name: 'wingR', r: [wingX, ac.ref.b / 4, ac.wing.z], A: halfArea, n: [0, 0, 1], CN: cn, Cf: 0.012 },
    { name: 'tailL', r: [tailX, -tailCy, tailCz], A: tailHalf, n: [0, -Math.sin(ga), -Math.cos(ga)], CN: 1.2, Cf: 0.012 },
    { name: 'tailR', r: [tailX, tailCy, tailCz], A: tailHalf, n: [0, Math.sin(ga), -Math.cos(ga)], CN: 1.2, Cf: 0.012 },
  ];
  const fusX = ac.noseToCg - ac.fuselage.length / 2;
  const d = ac.fuselage.diameter;
  const L = ac.fuselage.length;
  const bluffs = [
    { name: 'fuselage', r: [fusX, 0, 0], CdA: [0.4 * Math.PI * d * d / 4, 0.9 * L * d, 0.9 * L * d] },
    { name: 'boomL', r: [-0.2, -ac.boomY, 0], CdA: [0.0005, 0.04, 0.04] },
    { name: 'boomR', r: [-0.2, ac.boomY, 0], CdA: [0.0005, 0.04, 0.04] },
  ];
  return { plates, bluffs };
}

/**
 * Separated-flow forces and moments (body axes, N and N m).
 * vAir: velocity of the aircraft relative to the air, body axes.
 */
function separatedFlow(comp, vAir, omega, rho, outF, outM) {
  outF[0] = outF[1] = outF[2] = 0;
  outM[0] = outM[1] = outM[2] = 0;
  const half = 0.5 * rho;
  for (const p of comp.plates) {
    const w = cross(omega, p.r);
    const vx = vAir[0] + w[0], vy = vAir[1] + w[1], vz = vAir[2] + w[2];
    const V = Math.hypot(vx, vy, vz);
    if (V < 1e-6) continue;
    const vn = vx * p.n[0] + vy * p.n[1] + vz * p.n[2];
    const kN = -half * p.A * p.CN * V * vn;
    const tx = vx - vn * p.n[0], ty = vy - vn * p.n[1], tz = vz - vn * p.n[2];
    const kT = -half * p.A * p.Cf * V;
    const f = [kN * p.n[0] + kT * tx, kN * p.n[1] + kT * ty, kN * p.n[2] + kT * tz];
    outF[0] += f[0]; outF[1] += f[1]; outF[2] += f[2];
    const m = cross(p.r, f);
    outM[0] += m[0]; outM[1] += m[1]; outM[2] += m[2];
  }
  for (const b of comp.bluffs) {
    const w = cross(omega, b.r);
    const vx = vAir[0] + w[0], vy = vAir[1] + w[1], vz = vAir[2] + w[2];
    const V = Math.hypot(vx, vy, vz);
    const f = [-half * b.CdA[0] * V * vx, -half * b.CdA[1] * V * vy, -half * b.CdA[2] * V * vz];
    outF[0] += f[0]; outF[1] += f[1]; outF[2] += f[2];
    const m = cross(b.r, f);
    outM[0] += m[0]; outM[1] += m[1]; outM[2] += m[2];
  }
}

export class AeroModel {
  constructor(ac) {
    this.ac = ac;
    const a = ac.aero;
    this.a = a;
    this.S = ac.ref.S;
    this.b = ac.ref.b;
    this.c = ac.ref.c;
    // Zero-alpha intercepts from the trim point (Cm = 0 at trim with zero elevator)
    this.CL0 = a.trim.CL - a.CLa * a.trim.alpha;
    this.Cm0 = -a.Cma * a.trim.alpha;
    this.comp = makeComponents(ac);
    this._F = [0, 0, 0];
    this._M = [0, 0, 0];
    this.calibrateStall();
  }

  /** Blend factor 0 (attached) .. 1 (separated). */
  sigma(alpha, beta) {
    const a = this.a;
    const sp = smoothstep(this.alphaOnsetPos, this.alphaOnsetPos + a.stallBand, alpha);
    const sn = smoothstep(this.alphaOnsetNeg, this.alphaOnsetNeg - a.stallBand, alpha);
    const sa = 1 - (1 - sp) * (1 - sn);
    const sb = smoothstep(a.betaSepStart, a.betaSepFull, Math.abs(beta));
    return 1 - (1 - sa) * (1 - sb);
  }

  /** Lift coefficient of the separated model alone at angle alpha (beta 0, no rotation). */
  separatedCL(alpha) {
    const v = [Math.cos(alpha), 0, Math.sin(alpha)];
    separatedFlow(this.comp, v, [0, 0, 0], 2.0, this._F, this._M); // rho=2 -> qbar=V^2/... =1
    // lift direction (sin a, 0, -cos a)
    return (this._F[0] * Math.sin(alpha) - this._F[2] * Math.cos(alpha)) / this.S;
  }

  /** Static (no rates, no controls) blended lift coefficient. */
  staticCL(alpha) {
    const s = this.sigma(alpha, 0);
    return (1 - s) * (this.CL0 + this.a.CLa * alpha) + s * this.separatedCL(alpha);
  }

  calibrateStall() {
    const a = this.a;
    const peak = (onset, dir) => {
      this.alphaOnsetPos = dir > 0 ? onset : 40 * DEG;
      this.alphaOnsetNeg = dir < 0 ? onset : -40 * DEG;
      let best = dir > 0 ? -1e9 : 1e9;
      for (let al = -30 * DEG; al <= 40 * DEG; al += 0.05 * DEG) {
        const cl = this.staticCL(al);
        best = dir > 0 ? Math.max(best, cl) : Math.min(best, cl);
      }
      return best;
    };
    // positive stall: find onset so the CL peak equals CLmax
    let lo = -2 * DEG, hi = 20 * DEG;
    for (let i = 0; i < 40; i++) {
      const m = 0.5 * (lo + hi);
      if (peak(m, 1) > a.CLmax) hi = m;
      else lo = m;
    }
    const pos = 0.5 * (lo + hi);
    // Negative stall: the flat-plate branch keeps growing in magnitude past
    // stall, so there is no peak to match. Start the blend where the linear
    // lift reaches 65 % of CLminNeg so separation is complete just past the
    // linear model's CLminNeg angle.
    const alphaNegLin = (a.CLminNeg - this.CL0) / a.CLa;
    const neg = alphaNegLin + 0.3 * a.stallBand;
    this.alphaOnsetPos = pos;
    this.alphaOnsetNeg = neg;
    // record the stall angle (alpha at CLmax) for display
    let best = -1, bestA = 0;
    for (let al = 0; al <= 30 * DEG; al += 0.02 * DEG) {
      const cl = this.staticCL(al);
      if (cl > best) {
        best = cl;
        bestA = al;
      }
    }
    this.alphaStall = bestA;
    this.CLmaxAchieved = best;
  }

  /**
   * Compute aerodynamic force and moment about the CG in body axes.
   * @param vAir   aircraft velocity relative to the air mass, body axes (m/s)
   * @param omega  body rates [p, q, r] (rad/s)
   * @param surf   {da, de, dr} effective aileron (per side), elevator, rudder (rad)
   * @param rho    air density
   * @param out    {F:[3], M:[3]} accumulators (overwritten)
   */
  compute(vAir, omega, surf, rho, out) {
    const a = this.a;
    const S = this.S, b = this.b, c = this.c;
    const F = out.F, M = out.M;
    const Va = norm(vAir);
    const info = out.info;
    if (Va < 0.05) {
      separatedFlow(this.comp, vAir, omega, rho, F, M);
      info.Va = Va;
      info.alpha = 0;
      info.beta = 0;
      info.sigma = 1;
      info.CL = 0;
      info.CD = 0;
      info.qbar = 0;
      return;
    }
    const alpha = Math.atan2(vAir[2], vAir[0]);
    const beta = Math.asin(clamp(vAir[1] / Va, -1, 1));
    const qbar = 0.5 * rho * Va * Va;
    const sig = this.sigma(alpha, beta);
    const ca = Math.cos(alpha), sa = Math.sin(alpha);
    const vhat = [vAir[0] / Va, vAir[1] / Va, vAir[2] / Va];
    const liftDir = [sa, 0, -ca];

    let Fx = 0, Fy = 0, Fz = 0, Mx = 0, My = 0, Mz = 0;
    let CLatt = 0, CDatt = 0;

    if (sig < 1) {
      // stability-axis rates
      const [p, q, r] = omega;
      const ps = p * ca + r * sa;
      const rs = -p * sa + r * ca;
      const k2 = 1 / (2 * Va);
      const ph = clamp(ps * b * k2, -1, 1);
      const qh = clamp(q * c * k2, -1, 1);
      const rh = clamp(rs * b * k2, -1, 1);
      // limit the derivative model's alpha/beta to its region of validity
      const al = clamp(alpha, -25 * DEG, 25 * DEG);
      const be = clamp(beta, -35 * DEG, 35 * DEG);
      const CLaoa = this.CL0 + a.CLa * al;
      const CL = CLaoa + a.CLq * qh;
      const CD = (a.CD0 + a.k * CLaoa * CLaoa) * a.dragMultiplier;
      const CY = a.CYb * be + a.CYp * ph + a.CYr * rh;
      const Cl = a.Clb * be + a.Clp * ph + a.Clr * rh;
      const Cm = this.Cm0 + a.Cma * al + a.Cmq * qh;
      const Cn = a.Cnb * be + a.Cnp * ph + a.Cnr * rh;
      CLatt = CL;
      CDatt = CD;
      const qS = qbar * S;
      const w = 1 - sig;
      Fx += w * qS * (CL * liftDir[0] - CD * vhat[0]);
      Fy += w * qS * (CY - CD * vhat[1]);
      Fz += w * qS * (CL * liftDir[2] - CD * vhat[2]);
      // stability -> body moments
      const Ls = qS * b * Cl, Ns = qS * b * Cn;
      Mx += w * (Ls * ca - Ns * sa);
      My += w * qS * c * Cm;
      Mz += w * (Ls * sa + Ns * ca);
    }
    if (sig > 0) {
      separatedFlow(this.comp, vAir, omega, rho, this._F, this._M);
      Fx += sig * this._F[0];
      Fy += sig * this._F[1];
      Fz += sig * this._F[2];
      Mx += sig * this._M[0];
      My += sig * this._M[1];
      Mz += sig * this._M[2];
    }

    // Control surfaces: effective while the flow over the tail/wing is forward.
    const fwd = clamp(vAir[0] / Va, 0, 1);
    const eta = (1 - 0.5 * sig) * fwd * fwd;
    if (eta > 0) {
      const { da, de, dr } = surf;
      const qS = qbar * S * eta;
      const dCL = a.CLde * de;
      const dCY = a.CYdr * dr;
      const dCl = a.Clda * da + a.Cldr * dr;
      const dCm = a.Cmde * de;
      const dCn = a.Cnda * da + a.Cndr * dr;
      Fx += qS * dCL * liftDir[0];
      Fy += qS * dCY;
      Fz += qS * dCL * liftDir[2];
      const Ls = qS * b * dCl, Ns = qS * b * dCn;
      Mx += Ls * ca - Ns * sa;
      My += qS * c * dCm;
      Mz += Ls * sa + Ns * ca;
    }

    F[0] = Fx; F[1] = Fy; F[2] = Fz;
    M[0] = Mx; M[1] = My; M[2] = Mz;
    info.Va = Va;
    info.alpha = alpha;
    info.beta = beta;
    info.sigma = sig;
    info.qbar = qbar;
    const qS = qbar * S;
    info.CL = qS > 1e-6 ? (Fx * liftDir[0] + Fz * liftDir[2]) / qS : 0;
    info.CD = qS > 1e-6 ? -(Fx * vhat[0] + Fy * vhat[1] + Fz * vhat[2]) / qS : 0;
    info.CLatt = CLatt;
    info.CDatt = CDatt;
  }
}

export { separatedFlow, dot };
