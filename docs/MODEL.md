# Physics and flight-controller model

This page documents what the simulator computes, where every number comes
from, and how it was checked. `[USER]` marks your design data; `[EST]` marks
engineering estimates I made because the value wasn't supplied. All values live in
`src/core/params.js` (aircraft) and `src/fc/params.js` (autopilot).

## Frames and integration

- World frame NED (north = out to sea), origin on the launch pad, sea level at z = 0.
- Body frame FRD with the origin at the CG (562 mm from the nose, 11 mm above the fuselage centreline).
- 18 states: position, velocity, attitude quaternion, body rates, 4 lift-rotor speeds and the pusher speed.
- Fixed-step RK4 at 400 Hz, the same rate as ArduPilot's main loop. The battery, servos, wind and sensor errors update once per step.
- Inertia [USER]: Ixx 0.334, Iyy 1.110, Izz 1.426 kg·m², tensor element Ixz = −0.0346 (Σm·x·z = +0.0346).
  Euler's equations include the spinning rotors' angular momentum.

## Aerodynamics (`src/core/aero.js`)

Two models are blended.

**Attached flow:** your flow5 Type-7 derivatives are used unchanged. They are about the CG,
in stability axes rotated by the current α, with AVL-style rate normalisation (q̂ = qc/2V,
p̂ = pb/2V, r̂ = rb/2V, c = 0.338 m, b = 1.340 m).
- CL = CL0 + CLα·α + CLq·q̂ + CLδe·δe, where CL0 = 0.389 and Cm0 = 0.0332 are back-solved from the trim point
  (CL 0.574 and Cm 0 at α 2.63° with zero elevator).
- CD = 0.026 + 0.081·CL².
- Side force, roll and yaw use all nine lateral derivatives plus the aileron and rudder derivatives.
- Lift acts perpendicular to the relative wind and drag acts along it, so CXu and CXα come out of the nonlinear model
  instead of being added separately.
- Rudder sign as you specified: positive = nose right (Cnδr = +0.00197/°). Ailerons are per side, (δL − δR)/2.
  Elevator is trailing edge down positive.

**Separated flow** [EST] is a component model valid for any flow direction (stall, flying backwards,
hovering in gusts):
- wing halves and V-tail panels are flat plates (normal-force coefficient 1.25 and 1.2 at 90°);
- the fuselage and booms are bluff bodies;
- each component uses its own local velocity, including ω × r, so the model gives rotational damping even at zero airspeed.

**Blend:**
- The blend factor is exactly zero below the stall onset (α < 8.98°, |β| < 12°), so the trim and the linear modes are
  pure flow5.
- It ramps over 10° of α (or β from 12° to 35°).
- The onset is calibrated at start-up so the blended lift curve peaks at exactly CLmax = 1.08 (α 10.7°), which gives
  16.7 m/s at 7.481 kg and ρ 1.083.
- Negative stall is set near CL −0.6 [EST].
- Control surfaces keep partial effect when separated, and none in reversed flow.

## Lift rotors (`src/core/propulsion.js`)

- Blade-element/momentum rotor (σ 0.094, a 5.7, κ 1.15 [EST]). Thrust and torque depend on climb, descent and
  edgewise inflow, so the model includes translational lift in forward flight and in wind.
- Glauert momentum inflow, plus Leishman's empirical curve in the vortex-ring state. VRS also adds up to ±18 %
  low-frequency thrust roughness.
- Cheeseman–Bennett ground effect and an in-plane H-force (rotor drag).
- Motor and ESC per rotor:
  - Kv 520 [USER], R 0.034 Ω, I0 1.0 A [EST];
  - ESC 96 % efficient with a 60 A current limit and damped-light braking [EST];
  - rotor inertia 4.6e-4 kg·m² [EST].
  - The spin-up time constant (~45 ms) comes out of the motor equations; it isn't a separate parameter.
- Calibrated at start-up to your figures:
  - blade pitch and profile drag are solved so one rotor gives **32.0 N** at full throttle on 22.2 V
    (ρ 1.225, datasheet conditions [EST]);
  - the aircraft hovers on **1205 W** total at 7.481 kg and ρ 1.083.
  - Result: θ75 = 7.2°, Cd0 = 0.0098, hover 8900 rpm, 16.3 A per motor, figure of merit ≈ 0.61.
- Reaction torques use the motor's electromagnetic torque, so yaw transients from spinning rotors up and down are
  included.

## Pusher

- Your generic model: Ct = 0.101·(1 − J/0.64), Cp = 0.041·(1 − (J/Jp)²).
- Jp = 0.775 is calibrated so that level cruise at 20.8 m/s and ρ 1.083 draws **365 W** including avionics.
- X4130-520KV motor with your R 0.0129 Ω and I0 2.5 A, behind the 120 A ESC with braking off (the prop freewheels).
- The thrust line runs along the fuselage centreline, 11 mm below the CG.

## Battery

- 6S2P 13 Ah [USER], with a typical LiPo open-circuit-voltage curve.
- 9 mΩ ohmic + 2 mΩ wiring, plus one RC branch of 6 mΩ with a 20 s time constant [EST].
  This gives about 3.5 V/cell under hover load near empty, matching your note.
- Avionics draw 8 W [EST].

## Environment

- Air density from temperature (default 25 °C → 1.184 kg/m³), with an option for your 1.083 design case.
- Wind: mean wind at 10 m with a power-law profile (exponent 0.11, open sea).
- MIL-F-8785C low-altitude Dryden turbulence: first-order filter along-wind, second-order across-wind and vertical,
  with intensity and scale length set by height and wind.
- Random discrete 1-cosine gusts.
- Beach: the waterline is 40 m north of the pad and the pad is 1.0 m above sea level. The swimmer is 800 m north.

## Ground, water and crash rules

- Spring-damper contact on 4 skids plus the nose, tail, boom ends, wingtips and prop tips. The skids hold the CG
  0.28 m above the ground [EST].
- It counts as a crash when:
  - anything touches the water;
  - a skid touches down faster than 2.5 m/s vertically or 3 m/s sliding, or tilted more than 25°;
  - a structure point hits the ground faster than 1 m/s, or the aircraft tips over;
  - a spinning prop touches the ground;
  - the aerodynamic load exceeds 8 g;
  - the airspeed exceeds 42 m/s.

## Flight controller (`src/fc/`)

- It reads an EKF-like estimate with GPS error (0.5 m, 25 s correlation), baro error (0.3 m) and airspeed noise.
- Structure follows ArduPilot:
  - multicopter angle → rate → torque with input shaping;
  - a matrix mixer with priority roll/pitch > thrust > yaw (with yaw headroom) and thrust linearisation with
    voltage compensation;
  - AC_PosControl-style altitude and position loops;
  - fixed-wing roll, pitch and yaw controllers with turn coordination and a yaw damper;
  - TECS for speed and height, and L1 for lateral navigation.
- Gains are scheduled from the aircraft model, so it flies like a well-tuned (autotuned) aircraft.
- VTOL modes also use the control surfaces whenever there is airflow.
- Transitions:
  - **forward**: the lift motors hold height and attitude until AIRSPEED_MIN (20 m/s), then ramp down over
    Q_TRANSITION_MS;
  - **back**: the lift motors take over at once, braking is limited by Q_BCK_PIT_LIM while the wing is flying, and a
    balloon limiter eases the nose down when the aircraft climbs more than 5 m above target.
- Q_ASSIST triggers on airspeed or attitude error. THR_SLEWRATE limits the pusher.
- AUTO uses ArduPlane-like mission items. The VTOL approach plans the back-transition from Q_TRANS_DECEL and the
  ground speed.

## Validation (`npm run validate`, `npm test`)

| Quantity | Simulator | flow5 / design data |
|---|---|---|
| Trim α / elevator | 2.60° / 0.09° | 2.63° / 0° (flow5 ignores the 11 mm thrust offset) |
| Short period | 0.930 Hz, ζ 0.466 | 0.93 Hz, ζ 0.46 |
| Phugoid | 11.5 s, ζ 0.035 | 11.4 s, lightly damped |
| Roll mode | τ 0.226 s | 0.23 s |
| Dutch roll | 0.528 Hz, ζ −0.001 | 0.53 Hz, ζ ≈ 0 |
| Spiral | doubles in 27.2 s | ~27 s |
| Stall | CLmax 1.080 → 16.6 m/s | 1.08 → 16.7 m/s |
| Max lift thrust | 32.0 N per rotor | 32 N |
| Hover power | 1205 W | 1205 W |
| Cruise power, 20.8 m/s | 365 W (full nonlinear trim within 5 %) | 365 W |

The modes come from numerically linearising the full nonlinear simulator about trim, with the propulsion frozen as
flow5 assumes, and then taking eigenvalues. Closed-loop tests fly position hold in a 10 m/s gusty wind, a forward
transition, the full AUTO rescue in calm air and in the gusty design condition, QLAND, and in-flight disarm crash
detection.
