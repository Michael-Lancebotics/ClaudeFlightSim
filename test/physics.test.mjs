// Validation of the physics core against the owner's design data.
import test from 'node:test';
import assert from 'node:assert/strict';
import { AIRCRAFT } from '../src/core/params.js';
import { Vehicle } from '../src/core/vehicle.js';
import { DEG, G0 } from '../src/core/math.js';
import { trimLevel, linearise, flightModes } from '../src/sim/analysis.js';
import { Motor, PusherAero, steadySpeed } from '../src/core/propulsion.js';

const near = (actual, expected, relTol, label) => {
  const err = Math.abs(actual - expected) / Math.abs(expected);
  assert.ok(err <= relTol, `${label}: got ${actual.toFixed(4)}, expected ${expected} (+-${relTol * 100}%)`);
};

const veh = new Vehicle(AIRCRAFT);
const rho = AIRCRAFT.aero.rhoAnalysis;
const env = { rho, wind: [0, 0, 0], vBus: 22.2 };
veh.env = env;

test('trim at 22.8 m/s matches the flow5 trim point', () => {
  const t = trimLevel(veh, 22.8, env, { useFixedThrust: true });
  // flow5 trim excludes the 11 mm thrust-line offset; allow a small difference
  assert.ok(Math.abs(t.alpha - 2.63 * DEG) < 0.1 * DEG, `alpha ${t.alpha / DEG}`);
  assert.ok(Math.abs(t.de) < 0.3 * DEG, `elevator ${t.de / DEG}`);
  // thrust = drag = qbar S CD with CD ~ 0.053
  const qS = 0.5 * rho * 22.8 * 22.8 * AIRCRAFT.ref.S;
  near(t.thrust / qS, 0.053, 0.06, 'trim CD');
});

test('dynamic modes match the flow5 stability analysis', () => {
  const t = trimLevel(veh, 22.8, env, { useFixedThrust: true });
  const m = flightModes(linearise(veh, t.x, env));
  near(m.shortPeriod.freqHz, 0.93, 0.05, 'short-period frequency (Hz)');
  near(m.shortPeriod.zeta, 0.46, 0.08, 'short-period damping');
  near(m.phugoid.periodS, 11.4, 0.06, 'phugoid period (s)');
  assert.ok(m.phugoid.zeta > 0 && m.phugoid.zeta < 0.1, `phugoid lightly damped (zeta ${m.phugoid.zeta})`);
  near(m.roll.timeConstant, 0.23, 0.08, 'roll time constant (s)');
  near(m.dutchRoll.freqHz, 0.53, 0.05, 'Dutch-roll frequency (Hz)');
  assert.ok(Math.abs(m.dutchRoll.zeta) < 0.03, `Dutch roll ~neutral (zeta ${m.dutchRoll.zeta})`);
  assert.ok(m.spiral.re > 0, 'spiral divergent');
  near(m.spiral.timeToDouble, 27, 0.1, 'spiral time to double (s)');
});

test('stall: CLmax 1.08 gives 16.7 m/s at design weight and density', () => {
  const aero = veh.aero;
  near(aero.CLmaxAchieved, 1.08, 0.002, 'CLmax');
  const vs = Math.sqrt((2 * AIRCRAFT.mass * G0) / (rho * AIRCRAFT.ref.S * aero.CLmaxAchieved));
  near(vs, 16.7, 0.01, 'stall speed');
  // attached-flow model untouched at trim
  assert.equal(aero.sigma(2.63 * DEG, 0), 0);
});

test('lift rotors reproduce 32 N max thrust and 1205 W hover', () => {
  const c = AIRCRAFT.calibration.lift;
  near(c.maxThrust, 32, 0.002, 'max static thrust per rotor (N)');
  near(c.hoverPowerTotal, 1205, 0.002, 'hover power (W)');
  // figure of merit should be physically plausible
  const T = (AIRCRAFT.mass * G0) / 4;
  const A = Math.PI * AIRCRAFT.liftRotor.radius ** 2;
  const Pideal = T ** 1.5 / Math.sqrt(2 * 1.083 * A);
  assert.ok(Pideal * 4 / c.hoverPowerTotal > 0.45, 'overall hover efficiency plausible');
});

test('level cruise at 20.8 m/s with the real pusher draws ~365 W', () => {
  const v2 = new Vehicle(AIRCRAFT);
  const e2 = { rho, wind: [0, 0, 0], vBus: 22.2 };
  v2.env = e2;
  const t = trimLevel(v2, 20.8, e2, { useFixedThrust: false });
  v2.env = e2;
  const d = v2.evaluate();
  const P = (d.motorBatteryCurrent + AIRCRAFT.battery.avionicsPower / 22.2) * 22.2;
  near(P, 365, 0.05, 'cruise battery power (W)');
  assert.ok(t.thrust > 0 && t.thrust < 1, 'pusher duty in range');
});

test('endurance roughly matches 11 min hover / 36 min cruise (80 % usable)', () => {
  const usableWh = 0.8 * 13.0 * 3.83 * 6; // avg cell voltage ~3.83 V
  const hoverMin = (usableWh / 1205) * 60;
  const cruiseMin = (usableWh / 365) * 60;
  assert.ok(hoverMin > 10 && hoverMin < 13, `hover ${hoverMin}`);
  assert.ok(cruiseMin > 33 && cruiseMin < 42, `cruise ${cruiseMin}`);
});

test('pusher static full-throttle current exceeds the 85 A motor rating (flagged)', () => {
  const m = new Motor(AIRCRAFT.pusherMotor);
  const p = new PusherAero(AIRCRAFT.pusher);
  const w = steadySpeed(m, 1, 22.2, (x) => p.compute(x, 0, 1.18).Q);
  const I = m.current(1, 22.2, w);
  assert.ok(I > 85, `static current ${I}`);
});
