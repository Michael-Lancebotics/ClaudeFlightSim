// Prints the validation report: trim and dynamic modes vs. the flow5 analysis,
// plus propulsion/performance checks. Run: npm run validate
import { AIRCRAFT } from '../src/core/params.js';
import { Vehicle } from '../src/core/vehicle.js';
import { DEG, RAD } from '../src/core/math.js';
import { trimLevel, linearise, flightModes } from '../src/sim/analysis.js';

const veh = new Vehicle(AIRCRAFT);
const rho = AIRCRAFT.aero.rhoAnalysis;
const env = { rho, wind: [0, 0, 0], vBus: 22.2 };
veh.env = env;
const t = trimLevel(veh, 22.8, env, { useFixedThrust: true });
console.log(`Trim @22.8 m/s rho=${rho}: alpha=${(t.alpha * RAD).toFixed(3)} deg, elevator=${(t.de * RAD).toFixed(3)} deg, thrust=${t.thrust.toFixed(3)} N`);
const A = linearise(veh, t.x, env);
const m = flightModes(A);
const f = (x, d = 3) => x.toFixed(d);
console.log(`Short period: ${f(m.shortPeriod.freqHz)} Hz, zeta ${f(m.shortPeriod.zeta)}   (flow5: 0.93 Hz, 0.46)`);
console.log(`Phugoid: period ${f(m.phugoid.periodS, 2)} s, zeta ${f(m.phugoid.zeta)}   (flow5: 11.4 s, lightly damped)`);
console.log(`Roll: time constant ${f(m.roll.timeConstant)} s   (flow5: 0.23 s)`);
console.log(`Dutch roll: ${f(m.dutchRoll.freqHz)} Hz, zeta ${f(m.dutchRoll.zeta)}   (flow5: 0.53 Hz, ~0)`);
console.log(`Spiral: ${m.spiral.re > 0 ? 'time to double ' + f(m.spiral.timeToDouble, 1) : 'time to half ' + f(m.spiral.timeToHalf, 1)} s   (flow5: doubles in ~27 s)`);
console.log('lon eig', m.lonAll.map(e => `${f(e.re,4)}${e.im? '±'+f(Math.abs(e.im),4)+'i':''}`).join(', '));
console.log('lat eig', m.latAll.map(e => `${f(e.re,4)}${e.im? '±'+f(Math.abs(e.im),4)+'i':''}`).join(', '));
console.log('calibration', JSON.stringify(AIRCRAFT.calibration, null, 1));
