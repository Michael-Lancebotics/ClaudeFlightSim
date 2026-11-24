// Small vector / quaternion / matrix helpers.
// Conventions used throughout the physics core:
//   World frame: NED (x = North, y = East, z = Down), origin on the launch pad
//                horizontally and at mean sea level vertically.
//   Body frame:  FRD (x = forward, y = right, z = down), origin at the CG.
//   Quaternion [w, x, y, z] rotates body-frame vectors into the NED frame.

export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;
export const G0 = 9.80665;

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a, b, t) => a + (b - a) * t;
export const sq = (v) => v * v;
export const sign = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);

/** C1-continuous 0..1 ramp between e0 and e1. */
export function smoothstep(e0, e1, x) {
  const t = clamp((x - e0) / (e1 - e0), 0, 1);
  return t * t * (3 - 2 * t);
}

/** Wrap an angle to (-pi, pi]. */
export function wrapPi(a) {
  a = (a + Math.PI) % (2 * Math.PI);
  if (a < 0) a += 2 * Math.PI;
  return a - Math.PI;
}

/** Wrap an angle to [0, 2pi). */
export function wrap2Pi(a) {
  a %= 2 * Math.PI;
  return a < 0 ? a + 2 * Math.PI : a;
}

// ---------------------------------------------------------------- vec3
export const v3 = (x = 0, y = 0, z = 0) => [x, y, z];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export function normalize(a) {
  const n = norm(a);
  return n > 1e-12 ? scale(a, 1 / n) : [0, 0, 0];
}
export function addInPlace(a, b) {
  a[0] += b[0];
  a[1] += b[1];
  a[2] += b[2];
  return a;
}
export function addScaledInPlace(a, b, s) {
  a[0] += b[0] * s;
  a[1] += b[1] * s;
  a[2] += b[2] * s;
  return a;
}

// ---------------------------------------------------------------- mat3 (row-major arrays of 9)
export function mat3MulVec(m, v) {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
}
export function mat3TMulVec(m, v) {
  return [
    m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
    m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
    m[2] * v[0] + m[5] * v[1] + m[8] * v[2],
  ];
}
export function mat3Inverse(m) {
  const [a, b, c, d, e, f, g, h, i] = m;
  const A = e * i - f * h;
  const B = -(d * i - f * g);
  const C = d * h - e * g;
  const det = a * A + b * B + c * C;
  const s = 1 / det;
  return [
    A * s, -(b * i - c * h) * s, (b * f - c * e) * s,
    B * s, (a * i - c * g) * s, -(a * f - c * d) * s,
    C * s, -(a * h - b * g) * s, (a * e - b * d) * s,
  ];
}

// ---------------------------------------------------------------- quaternion [w,x,y,z]
export function quatMul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}
export function quatNormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
/** Rotation matrix (row-major) taking body vectors to NED. */
export function quatToDcm(q) {
  const [w, x, y, z] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
  ];
}
/** ZYX (yaw-pitch-roll) Euler angles to quaternion. */
export function eulerToQuat(roll, pitch, yaw) {
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}
/** Quaternion to ZYX Euler angles {roll, pitch, yaw} (rad). */
export function quatToEuler(q) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(clamp(2 * (w * y - z * x), -1, 1));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
}

// ---------------------------------------------------------------- random
/** Deterministic PRNG (mulberry32) so runs can be reproduced from a seed. */
export function makeRng(seed = 1) {
  let s = seed >>> 0;
  const uniform = () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  let spare = null;
  const gauss = () => {
    if (spare !== null) {
      const v = spare;
      spare = null;
      return v;
    }
    let u = 0, v = 0;
    while (u <= 1e-12) u = uniform();
    v = uniform();
    const r = Math.sqrt(-2 * Math.log(u));
    spare = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  };
  return { uniform, gauss };
}

// ---------------------------------------------------------------- 1-D numerics
/** Bisection root find of f on [a, b] (f(a), f(b) must bracket a root). */
export function bisect(f, a, b, tol = 1e-9, maxIter = 100) {
  let fa = f(a);
  for (let i = 0; i < maxIter; i++) {
    const m = 0.5 * (a + b);
    const fm = f(m);
    if (Math.abs(b - a) < tol) return m;
    if ((fm > 0) === (fa > 0)) {
      a = m;
      fa = fm;
    } else {
      b = m;
    }
  }
  return 0.5 * (a + b);
}

/** Linear interpolation in a table of [x, y] pairs sorted by x. */
export function interpTable(table, x) {
  if (x <= table[0][0]) return table[0][1];
  const n = table.length;
  if (x >= table[n - 1][0]) return table[n - 1][1];
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (table[mid][0] <= x) lo = mid;
    else hi = mid;
  }
  const [x0, y0] = table[lo];
  const [x1, y1] = table[hi];
  return y0 + ((y1 - y0) * (x - x0)) / (x1 - x0);
}
