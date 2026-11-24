// Closed-loop tests: the built-in flight controller flying the physics model.
import test from 'node:test';
import assert from 'node:assert/strict';
import { Simulation } from '../src/sim/simulation.js';

function fly(sim, seconds, pilot = {}, onStep = null) {
  const n = Math.round(seconds / sim.dt);
  for (let i = 0; i < n; i++) {
    sim.step({ roll: 0, pitch: 0, yaw: 0, throttle: 0, rcOk: true, ...pilot, events: i === 0 ? pilot.events || [] : [] });
    if (onStep) onStep(sim);
    if (sim.crash) return false;
  }
  return true;
}

function runMission(env, maxTime = 260) {
  const sim = new Simulation({ env });
  sim.reset({ mode: 'AUTO' });
  fly(sim, 0.1, { events: [{ type: 'arm' }] });
  let t = 0;
  while (t < maxTime && !sim.crash) {
    fly(sim, 1);
    t += 1;
    if (!sim.fc.armed && t > 30) break;
  }
  return sim;
}

test('QLOITER holds position within 3 m in 10 m/s gusty wind', () => {
  const sim = new Simulation({ env: { windSpeed: 10, windFromDeg: 0, gustAmp: 3, turbulence: 'moderate' } });
  sim.reset({ mode: 'QLOITER' });
  fly(sim, 0.1, { events: [{ type: 'arm' }] });
  assert.ok(fly(sim, 10, { throttle: 0.9 }), 'climb');
  let maxErr = 0;
  assert.ok(fly(sim, 40, { throttle: 0.5 }, (s) => {
    const p = s.vehicle.pos;
    maxErr = Math.max(maxErr, Math.hypot(p[0], p[1]));
  }), 'hover');
  assert.ok(maxErr < 3, `drift ${maxErr.toFixed(2)} m`);
  assert.ok(sim.vehicle.altitudeRelHome() > 10, 'kept altitude');
});

test('forward transition in FBWA completes and lift motors stop', () => {
  const sim = new Simulation({ env: { windSpeed: 0, gustAmp: 0, turbulence: 'off' } });
  sim.reset({ mode: 'QLOITER' });
  fly(sim, 0.1, { events: [{ type: 'arm' }] });
  fly(sim, 12, { throttle: 0.9 });
  const t0 = sim.time;
  let tDone = null;
  fly(sim, 12, { throttle: 1, events: [{ type: 'mode', mode: 'FBWA' }] }, (s) => {
    if (tDone == null && s.fc.phase === 'fw') tDone = s.time;
  });
  assert.ok(tDone != null, 'transition finished');
  assert.ok(tDone - t0 < 8, `transition took ${(tDone - t0).toFixed(1)} s`);
  assert.ok(sim.vehicle.rotorSpeeds.every((w) => w < 100), 'lift rotors spun down');
  assert.ok(sim.est.airspeed > 20);
});

test('AUTO rescue mission in calm air: reach < 60 s, accurate drop, lands', () => {
  const sim = runMission({ windSpeed: 0, gustAmp: 0, turbulence: 'off' });
  assert.equal(sim.crash, null);
  const s = sim.scenario.summary(sim.time);
  assert.ok(s.reachTime != null && s.reachTime < 60, `reach ${s.reachTime}`);
  assert.ok(Math.abs(s.release.height - 5) < 0.6, `release height ${s.release.height}`);
  assert.ok(s.splash && s.splash.miss < 3, `miss ${s.splash && s.splash.miss}`);
  assert.ok(s.returnTime != null, 'landed back on the beach');
  assert.equal(sim.fc.armed, false);
});

test('AUTO rescue mission in the 10 m/s gusty design condition completes safely', () => {
  const sim = runMission({ windSpeed: 10, windFromDeg: 0, gustAmp: 3, turbulence: 'moderate' }, 300);
  assert.equal(sim.crash, null, sim.crash && sim.crash.reason);
  const s = sim.scenario.summary(sim.time);
  assert.ok(s.release && s.splash, 'payload delivered');
  assert.ok(s.splash.miss < 8, `miss ${s.splash.miss}`);
  assert.ok(s.returnTime != null, 'returned and landed');
});

test('disarming in flight is detected as a crash', () => {
  const sim = new Simulation({ env: { windSpeed: 0, gustAmp: 0, turbulence: 'off' } });
  sim.reset({ mode: 'QHOVER' });
  fly(sim, 0.1, { events: [{ type: 'arm' }] });
  fly(sim, 6, { throttle: 0.9 });
  fly(sim, 10, { throttle: 0.5, events: [{ type: 'disarm' }] });
  assert.ok(sim.crash, 'crash detected');
});

test('QLAND descends and disarms on touchdown', () => {
  const sim = new Simulation({ env: { windSpeed: 3, gustAmp: 1, turbulence: 'light' } });
  sim.reset({ mode: 'QLOITER' });
  fly(sim, 0.1, { events: [{ type: 'arm' }] });
  fly(sim, 8, { throttle: 0.9 });
  fly(sim, 40, { throttle: 0.5, events: [{ type: 'mode', mode: 'QLAND' }] });
  assert.equal(sim.crash, null);
  assert.equal(sim.fc.armed, false);
});

test('after a crash, reset allows arming again', () => {
  const sim = new Simulation({ env: { windSpeed: 0, gustAmp: 0, turbulence: 'off' } });
  sim.reset({ mode: 'QHOVER' });
  fly(sim, 0.1, { events: [{ type: 'arm' }] });
  fly(sim, 6, { throttle: 0.9 });
  fly(sim, 10, { throttle: 0.5, events: [{ type: 'disarm' }] });
  assert.ok(sim.crash, 'crashed');
  sim.reset({ mode: 'QLOITER' });
  assert.equal(sim.crash, null);
  fly(sim, 0.1, { events: [{ type: 'arm' }] });
  assert.equal(sim.fc.armed, true, 'armed after reset');
});
