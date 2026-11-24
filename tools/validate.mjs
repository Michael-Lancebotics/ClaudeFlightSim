// Validation report: trim and dynamic modes vs. the flow5 analysis,
// propulsion calibration, performance and the rescue mission in several winds.
// Run: npm run validate   (add --quick to skip the mission runs)
import { AIRCRAFT } from '../src/core/params.js';
import { Vehicle } from '../src/core/vehicle.js';
import { RAD, G0 } from '../src/core/math.js';
import { trimLevel, linearise, flightModes } from '../src/sim/analysis.js';
import { Motor, PusherAero, steadySpeed } from '../src/core/propulsion.js';
import { Simulation } from '../src/sim/simulation.js';

const f = (x, d = 2) => (x == null ? '—' : x.toFixed(d));
const veh = new Vehicle(AIRCRAFT);
const rho = AIRCRAFT.aero.rhoAnalysis;
const env = { rho, wind: [0, 0, 0], vBus: 22.2 };
veh.env = env;
const t = trimLevel(veh, 22.8, env, { useFixedThrust: true });
const m = flightModes(linearise(veh, t.x, env));
console.log('## Trim and dynamic modes (V = 22.8 m/s, rho = 1.083, m = 7.481 kg)\n');
console.log('| Quantity | Simulator | flow5 |\n|---|---|---|');
console.log(`| Trim alpha | ${f(t.alpha * RAD)}° | 2.63° |`);
console.log(`| Trim elevator | ${f(t.de * RAD)}° | 0° |`);
console.log(`| Short period | ${f(m.shortPeriod.freqHz, 3)} Hz, ζ ${f(m.shortPeriod.zeta, 3)} | 0.93 Hz, ζ 0.46 |`);
console.log(`| Phugoid | ${f(m.phugoid.periodS)} s, ζ ${f(m.phugoid.zeta, 3)} | 11.4 s, lightly damped |`);
console.log(`| Roll mode | τ ${f(m.roll.timeConstant, 3)} s | 0.23 s |`);
console.log(`| Dutch roll | ${f(m.dutchRoll.freqHz, 3)} Hz, ζ ${f(m.dutchRoll.zeta, 3)} | 0.53 Hz, ζ ≈ 0 |`);
console.log(`| Spiral | doubles in ${f(m.spiral.timeToDouble, 1)} s | ~27 s |`);
console.log(`| Stall | CLmax ${f(veh.aero.CLmaxAchieved, 3)} at α ${f(veh.aero.alphaStall * RAD, 1)}° → ${f(Math.sqrt((2 * AIRCRAFT.mass * G0) / (rho * AIRCRAFT.ref.S * veh.aero.CLmaxAchieved)))} m/s | 1.08 → 16.7 m/s |`);

const c = AIRCRAFT.calibration;
console.log('\n## Propulsion calibration\n');
console.log(`- Lift rotor: effective blade pitch θ75 = ${f(c.lift.theta75 * RAD, 2)}°, profile drag Cd0 = ${f(c.lift.Cd0, 4)}`);
console.log(`- Max static thrust ${f(c.lift.maxThrust)} N per rotor at 22.2 V; hover ${f(c.lift.hoverPowerTotal, 0)} W total at ${f(c.lift.hoverRpm, 0)} rpm, ${f(c.lift.hoverCurrentPerMotor, 1)} A per motor (rho 1.083)`);
console.log(`- Pusher Cp shape: Jp = ${f(c.pusher.Jp, 3)} → cruise 20.8 m/s: drag ${f(c.pusher.cruiseDrag)} N, ${f(c.pusher.cruiseRpm, 0)} rpm, ${f(c.pusher.cruisePower, 0)} W`);
const pm = new Motor(AIRCRAFT.pusherMotor);
const pp = new PusherAero(AIRCRAFT.pusher);
for (const d of [0.8, 1.0]) {
  const w = steadySpeed(pm, d, 22.2, (x) => pp.compute(x, 0, 1.18).Q);
  console.log(`- Pusher static at ${d * 100}% throttle (22.2 V, rho 1.18): ${f(pp.compute(w, 0, 1.18).T, 1)} N, ${f(pm.current(d, 22.2, w), 0)} A, ${f((w * 60) / (2 * Math.PI), 0)} rpm`);
}

if (!process.argv.includes('--quick')) {
  console.log('\n## AUTO rescue mission (800 m, drop at 5 m, gusts 3 m/s + moderate turbulence when windy)\n');
  console.log('| Wind | Reached swimmer | Released | Release height | Miss | Landed back | Energy |\n|---|---|---|---|---|---|---|');
  const cases = [['calm', 0, 0], ['10 m/s onshore (headwind out)', 10, 0], ['10 m/s offshore (tailwind out)', 10, 180], ['10 m/s crosswind', 10, 90]];
  for (const [label, ws, from] of cases) {
    const sim = new Simulation({ env: ws ? { windSpeed: ws, windFromDeg: from, gustAmp: 3, turbulence: 'moderate' } : { windSpeed: 0, gustAmp: 0, turbulence: 'off' } });
    sim.reset({ mode: 'AUTO' });
    sim.step({ roll: 0, pitch: 0, yaw: 0, throttle: 0, rcOk: true, events: [{ type: 'arm' }] });
    for (let i = 0; i < 300 * 400 && !sim.crash; i++) {
      sim.step({ roll: 0, pitch: 0, yaw: 0, throttle: 0, rcOk: true, events: [] });
      if (!sim.fc.armed && sim.time > 30) break;
    }
    const s = sim.scenario.summary(sim.time);
    console.log(`| ${label} | ${f(s.reachTime, 1)} s | ${f(s.release && s.release.time, 1)} s | ${f(s.release && s.release.height)} m | ${f(s.splash && s.splash.miss, 1)} m | ${f(s.returnTime, 0)} s | ${f(sim.vehicle.battery.usedWh, 0)} Wh${sim.crash ? ' CRASH ' + sim.crash.reason : ''} |`);
  }
}
