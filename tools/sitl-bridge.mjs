#!/usr/bin/env node
// ArduPilot SITL "JSON" physics backend for the rescue QuadPlane.
//
// ArduPilot (sim_vehicle.py -v ArduPlane -f JSON) sends servo PWM packets over
// UDP to port 9002; for each packet this bridge advances the same physics core
// the browser simulator uses and replies with the vehicle state as JSON.
// The flight controller is then the REAL ArduPilot instead of the built-in
// approximation.
//
//   node tools/sitl-bridge.mjs [--port 9002] [--wind 5] [--wind-from 0] [--gust 2]
//                              [--turbulence light] [--temp 25] [--map ch=func,...]
//
// See docs/SITL.md for the matching ArduPilot parameters and channel map.

import dgram from 'node:dgram';
import { fileURLToPath } from 'node:url';
import { Vehicle } from '../src/core/vehicle.js';
import { AIRCRAFT } from '../src/core/params.js';
import { Wind, airDensity } from '../src/core/atmosphere.js';
import { surfaceHeight } from '../src/core/terrain.js';
import { clamp, quatToEuler } from '../src/core/math.js';

const MAGIC_16 = 18458;
const MAGIC_32 = 29569;

/**
 * Default output map (1-based servo channel -> function), matching
 * sitl/quadplane-rescue.parm:
 *   1 aileron, 2 V-tail left, 3 throttle (pusher), 4 V-tail right,
 *   5-8 lift motors 1-4, 9 payload release (gripper)
 */
export const DEFAULT_CHANNEL_MAP = {
  1: 'aileron',
  2: 'vtailLeft',
  3: 'throttle',
  4: 'vtailRight',
  5: 'motor1',
  6: 'motor2',
  7: 'motor3',
  8: 'motor4',
  9: 'gripper',
};

export function createBridge({
  port = 9002,
  channelMap = DEFAULT_CHANNEL_MAP,
  reverse = {},
  env = {},
  log = console.log,
} = {}) {
  const veh = new Vehicle(AIRCRAFT, { seed: 21 });
  const wind = new Wind({ speed: env.wind ?? 0, fromDeg: env.windFrom ?? 0, gustAmp: env.gust ?? 0, turbulence: env.turbulence ?? 'off' });
  const rho = airDensity(env.temp ?? 25, 0);
  const maxDefl = AIRCRAFT.aero.maxDeflection;
  const state = { lastFrame: null, lastReply: null, time: 0, origin: null, packets: 0, released: false };

  const reset = () => {
    veh.reset({ yaw: env.yaw ?? 0 });
    wind.reset();
    state.time = 0;
    state.origin = veh.pos;
    state.released = false;
  };
  reset();

  const surf = (pwm, ch) => {
    const v = clamp((pwm - 1500) / 500, -1, 1);
    return (reverse[ch] ? -v : v) * maxDefl;
  };
  const thr = (pwm) => clamp((pwm - 1000) / 1000, 0, 1);

  function applyServos(pwm) {
    const c = veh.cmd;
    for (const [chStr, fn] of Object.entries(channelMap)) {
      const ch = Number(chStr);
      const v = pwm[ch - 1];
      if (v == null || v === 0) continue;
      switch (fn) {
        case 'aileron': {
          const a = surf(v, ch);
          c.ailL = a;
          c.ailR = -a;
          break;
        }
        case 'aileronLeft': c.ailL = surf(v, ch); break;
        case 'aileronRight': c.ailR = surf(v, ch); break;
        case 'vtailLeft': c.rvL = surf(v, ch); break;
        case 'vtailRight': c.rvR = surf(v, ch); break;
        case 'throttle': c.pusher = thr(v); break;
        case 'motor1': c.lift[0] = thr(v); break;
        case 'motor2': c.lift[1] = thr(v); break;
        case 'motor3': c.lift[2] = thr(v); break;
        case 'motor4': c.lift[3] = thr(v); break;
        case 'gripper':
          if (v > 1700 && veh.payloadAttached) {
            veh.payloadAttached = false;
            state.released = true;
            log(`[bridge] payload released at t=${state.time.toFixed(2)} s`);
          }
          break;
        default: break;
      }
    }
  }

  function stepPhysics(dt) {
    const sub = Math.max(1, Math.ceil(dt / (1 / 400)));
    const h = dt / sub;
    for (let i = 0; i < sub; i++) {
      const hgt = Math.max(veh.heightAboveSurface(), 0.3);
      wind.update(h, hgt, veh.diag.aero.Va || 0);
      veh.step(h, { rho, wind: wind.velocityAt(hgt) });
    }
    state.time += dt;
  }

  function reply() {
    const d = veh.diag;
    const p = veh.pos;
    const o = state.origin;
    const q = veh.quat;
    const e = quatToEuler(q);
    const hagl = veh.heightAboveSurface();
    const tilt = Math.cos(e.roll) * Math.cos(e.pitch);
    const msg = {
      timestamp: state.time,
      imu: { gyro: veh.omega, accel_body: d.specificForce },
      position: [p[0] - o[0], p[1] - o[1], p[2] - o[2]],
      quaternion: q,
      velocity: veh.vel,
      airspeed: Math.max(0, d.vAirBody[0]),
      rng_1: tilt > 0.5 ? Math.max(hagl - 0.28, 0) / tilt : 0,
    };
    return JSON.stringify(msg) + '\n';
  }

  const sock = dgram.createSocket('udp4');
  sock.on('message', (buf, rinfo) => {
    if (buf.length < 8) return;
    const magic = buf.readUInt16LE(0);
    if (magic !== MAGIC_16 && magic !== MAGIC_32) return;
    const n = magic === MAGIC_16 ? 16 : 32;
    if (buf.length < 8 + 2 * n) return;
    const frameRate = buf.readUInt16LE(2) || 400;
    const frame = buf.readUInt32LE(4);
    const pwm = [];
    for (let i = 0; i < n; i++) pwm.push(buf.readUInt16LE(8 + 2 * i));
    if (state.lastFrame != null && frame === state.lastFrame && state.lastReply) {
      sock.send(state.lastReply, rinfo.port, rinfo.address); // duplicate: resend
      return;
    }
    if (state.lastFrame != null && frame < state.lastFrame) {
      log('[bridge] frame counter went backwards - ArduPilot restarted, resetting physics');
      reset();
    }
    state.lastFrame = frame;
    state.packets++;
    applyServos(pwm);
    stepPhysics(1 / frameRate);
    state.lastReply = reply();
    sock.send(state.lastReply, rinfo.port, rinfo.address);
  });
  return new Promise((resolve) => {
    sock.bind(port, () => {
      log(`[bridge] ArduPilot JSON backend listening on UDP ${sock.address().port}`);
      resolve({ sock, vehicle: veh, state, port: sock.address().port, close: () => sock.close() });
    });
  });
}

// ------------------------------------------------------------------ CLI
function parseArgs(argv) {
  const out = { env: {}, channelMap: { ...DEFAULT_CHANNEL_MAP }, reverse: {} };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i], v = argv[i + 1];
    if (a === '--port') out.port = Number(v), i++;
    else if (a === '--wind') out.env.wind = Number(v), i++;
    else if (a === '--wind-from') out.env.windFrom = Number(v), i++;
    else if (a === '--gust') out.env.gust = Number(v), i++;
    else if (a === '--turbulence') out.env.turbulence = v, i++;
    else if (a === '--temp') out.env.temp = Number(v), i++;
    else if (a === '--yaw') out.env.yaw = (Number(v) * Math.PI) / 180, i++;
    else if (a === '--map') {
      for (const pair of v.split(',')) {
        const [ch, fn] = pair.split('=');
        out.channelMap[Number(ch)] = fn;
      }
      i++;
    } else if (a === '--reverse') {
      for (const ch of v.split(',')) out.reverse[Number(ch)] = true;
      i++;
    }
  }
  return out;
}

if (process.argv[1] && fileURLToPath(import.meta.url) === process.argv[1]) {
  const args = parseArgs(process.argv.slice(2));
  const br = await createBridge(args);
  setInterval(() => {
    const v = br.vehicle;
    const e = quatToEuler(v.quat);
    const alt = -v.s[2] - surfaceHeight(v.s[0], v.s[1]) - 0.28;
    console.log(`[bridge] t=${br.state.time.toFixed(1)}s pkts=${br.state.packets} alt=${alt.toFixed(1)}m as=${Math.max(0, v.diag.vAirBody[0]).toFixed(1)}m/s roll=${(e.roll * 57.3).toFixed(0)} pitch=${(e.pitch * 57.3).toFixed(0)} hdg=${((e.yaw * 57.3 + 360) % 360).toFixed(0)} batt=${v.battery.voltage.toFixed(1)}V`);
  }, 2000);
}
