// Trim, numerical linearisation and eigen-analysis of the nonlinear vehicle
// model. Used by the validation tests to check the 6-DOF implementation
// against the owner's flow5 stability results.

import { G0, DEG, cross, eulerToQuat, quatToDcm, mat3MulVec, mat3TMulVec } from '../core/math.js';

// ------------------------------------------------------------ complex polynomial roots
const cadd = (a, b) => [a[0] + b[0], a[1] + b[1]];
const csub = (a, b) => [a[0] - b[0], a[1] - b[1]];
const cmul = (a, b) => [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]];
const cdiv = (a, b) => {
  const d = b[0] * b[0] + b[1] * b[1];
  return [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d];
};

/** Characteristic polynomial coefficients (monic, highest power first) by Faddeev-LeVerrier. */
export function charPoly(A) {
  const n = A.length;
  let Mk = A.map((row, i) => row.map((_, j) => (i === j ? 1 : 0)));
  const c = [1];
  let ck = 1;
  for (let k = 1; k <= n; k++) {
    // AM = A * Mk
    const AM = A.map((row) => Mk[0].map((_, j) => row.reduce((s, a, l) => s + a * Mk[l][j], 0)));
    let tr = 0;
    for (let i = 0; i < n; i++) tr += AM[i][i];
    ck = -tr / k;
    c.push(ck);
    Mk = AM.map((row, i) => row.map((v, j) => v + (i === j ? ck : 0)));
  }
  return c;
}

/** Roots of a monic polynomial (Durand-Kerner). */
export function polyRoots(c) {
  const n = c.length - 1;
  const evalP = (z) => {
    let r = [1, 0];
    for (let i = 1; i <= n; i++) r = cadd(cmul(r, z), [c[i], 0]);
    return r;
  };
  let roots = [];
  for (let i = 0; i < n; i++) {
    const ang = (2 * Math.PI * i) / n + 0.4;
    roots.push([0.9 * Math.cos(ang) * 3, 0.9 * Math.sin(ang) * 3]);
  }
  for (let it = 0; it < 2000; it++) {
    let maxStep = 0;
    roots = roots.map((zi, i) => {
      let den = [1, 0];
      roots.forEach((zj, j) => {
        if (i !== j) den = cmul(den, csub(zi, zj));
      });
      const step = cdiv(evalP(zi), den);
      maxStep = Math.max(maxStep, Math.hypot(step[0], step[1]));
      return csub(zi, step);
    });
    if (maxStep < 1e-13) break;
  }
  return roots.map(([re, im]) => ({ re, im: Math.abs(im) < 1e-9 ? 0 : im }));
}

export function eigenvalues(A) {
  return polyRoots(charPoly(A));
}

/** Describe an eigenvalue as a mode. */
export function modeInfo(ev) {
  const wn = Math.hypot(ev.re, ev.im);
  const zeta = wn > 0 ? -ev.re / wn : 0;
  return {
    re: ev.re,
    im: ev.im,
    wn,
    zeta,
    freqHz: wn / (2 * Math.PI),
    periodS: ev.im !== 0 ? (2 * Math.PI) / Math.abs(ev.im) : Infinity,
    timeConstant: ev.im === 0 && ev.re < 0 ? -1 / ev.re : null,
    timeToDouble: ev.re > 0 ? Math.log(2) / ev.re : null,
    timeToHalf: ev.re < 0 ? Math.log(2) / -ev.re : null,
  };
}

// ------------------------------------------------------------ state mapping
/**
 * Build a full vehicle state from rigid-body variables
 * x = [u, v, w, p, q, r, phi, theta, psi] (body velocities, rates, Euler angles).
 */
function setRigidState(veh, x, altitude) {
  const [u, v, w, p, q, r, phi, theta, psi] = x;
  const quat = eulerToQuat(phi, theta, psi);
  const R = quatToDcm(quat);
  const vn = mat3MulVec(R, [u, v, w]);
  const s = veh.s;
  s[0] = 0;
  s[1] = 0;
  s[2] = -altitude;
  s[3] = vn[0];
  s[4] = vn[1];
  s[5] = vn[2];
  s.set(quat, 6);
  s[10] = p;
  s[11] = q;
  s[12] = r;
  return R;
}

/** Time derivative of x = [u v w p q r phi theta psi] for the current vehicle settings. */
export function rigidDerivative(veh, x, env, altitude = 300) {
  const R = setRigidState(veh, x, altitude);
  const d = veh.derivatives(veh.s, env);
  const [u, v, w, p, q, r, phi, theta] = x;
  const aN = [d[3], d[4], d[5]];
  const aB = mat3TMulVec(R, aN);
  const wxv = cross([p, q, r], [u, v, w]);
  const sphi = Math.sin(phi), cphi = Math.cos(phi);
  const tth = Math.tan(theta), cth = Math.cos(theta);
  return [
    aB[0] - wxv[0],
    aB[1] - wxv[1],
    aB[2] - wxv[2],
    d[10],
    d[11],
    d[12],
    p + (q * sphi + r * cphi) * tth,
    q * cphi - r * sphi,
    (q * sphi + r * cphi) / cth,
  ];
}

/**
 * Trim for steady, wings-level flight at airspeed V with the given flight-path
 * angle. Unknowns: alpha, elevator, thrust (thrust supplied by veh.fixedThrust
 * if useFixedThrust, otherwise by the pusher duty).
 */
export function trimLevel(veh, V, env, { gamma = 0, useFixedThrust = true, altitude = 300 } = {}) {
  let alpha = 3 * DEG, de = 0, thr = useFixedThrust ? 7 : 0.4;
  const apply = (al, el, th) => {
    veh.servo.rvL = veh.servo.rvR = el;
    veh.cmd.rvL = veh.cmd.rvR = el;
    if (useFixedThrust) veh.fixedThrust = th;
    else {
      veh.fixedThrust = null;
      veh.cmd.pusher = th;
    }
    return [V * Math.cos(al), 0, V * Math.sin(al), 0, 0, 0, 0, al + gamma, 0];
  };
  const resid = (al, el, th) => {
    const x = apply(al, el, th);
    if (!useFixedThrust) {
      // put the pusher at its steady speed for this duty/airspeed
      settlePusher(veh, x, env, altitude);
    }
    const dx = rigidDerivative(veh, x, env, altitude);
    return [dx[0], dx[2], dx[4]];
  };
  for (let it = 0; it < 60; it++) {
    const r0 = resid(alpha, de, thr);
    const err = Math.hypot(...r0);
    if (err < 1e-9) break;
    const h = [1e-5, 1e-5, useFixedThrust ? 1e-4 : 1e-5];
    const vars = [alpha, de, thr];
    const Jm = [[], [], []];
    for (let j = 0; j < 3; j++) {
      const vp = vars.slice();
      vp[j] += h[j];
      const r1 = resid(...vp);
      for (let i = 0; i < 3; i++) Jm[i][j] = (r1[i] - r0[i]) / h[j];
    }
    const dv = solve3(Jm, r0.map((v) => -v));
    alpha += dv[0];
    de += dv[1];
    thr += dv[2];
  }
  const x = apply(alpha, de, thr);
  if (!useFixedThrust) settlePusher(veh, x, env, altitude);
  return { alpha, de, thrust: thr, x };
}

function settlePusher(veh, x, env, altitude) {
  // integrate only the pusher speed state to steady state
  setRigidState(veh, x, altitude);
  for (let i = 0; i < 400; i++) {
    const d = veh.derivatives(veh.s, env);
    veh.s[17] = Math.max(0, veh.s[17] + d[17] * 0.002);
  }
}

function solve3(A, b) {
  const M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < 3; c++) {
    let p = c;
    for (let r = c + 1; r < 3; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < 3; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k < 4; k++) M[r][k] -= f * M[c][k];
    }
  }
  return [M[0][3] / M[0][0], M[1][3] / M[1][1], M[2][3] / M[2][2]];
}

/** Numerical Jacobian of the rigid-body derivative about x0. */
export function linearise(veh, x0, env, altitude = 300) {
  const n = 9;
  const A = [];
  for (let i = 0; i < n; i++) A.push(new Array(n).fill(0));
  const steps = [1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5];
  for (let j = 0; j < n; j++) {
    const xp = x0.slice(), xm = x0.slice();
    xp[j] += steps[j];
    xm[j] -= steps[j];
    const fp = rigidDerivative(veh, xp, env, altitude);
    const fm = rigidDerivative(veh, xm, env, altitude);
    for (let i = 0; i < n; i++) A[i][j] = (fp[i] - fm[i]) / (2 * steps[j]);
  }
  return A;
}

/** Longitudinal (u w q theta) and lateral (v p r phi) modes about a trimmed state. */
export function flightModes(A) {
  const pick = (idx) => idx.map((i) => idx.map((j) => A[i][j]));
  const lon = eigenvalues(pick([0, 2, 4, 7])).map(modeInfo);
  const lat = eigenvalues(pick([1, 3, 5, 6])).map(modeInfo);
  const lonOsc = lon.filter((m) => m.im > 0).sort((a, b) => b.wn - a.wn);
  const latOsc = lat.filter((m) => m.im > 0);
  const latReal = lat.filter((m) => m.im === 0).sort((a, b) => a.re - b.re);
  return {
    shortPeriod: lonOsc[0],
    phugoid: lonOsc[1],
    dutchRoll: latOsc[0],
    roll: latReal[0],
    spiral: latReal[latReal.length - 1],
    lonAll: lon,
    latAll: lat,
  };
}

export { G0 };
