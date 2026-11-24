// Aircraft definition for the lifeguard-rescue QuadPlane.
//
// Everything marked [USER] comes straight from the design data supplied by the
// owner (flow5 panel analysis, mass properties, propulsion choices).
// Everything marked [EST] is an engineering estimate made for the simulator
// because the value was not supplied; each is documented in docs/MODEL.md and
// can be changed here.
//
// Body axes: x forward, y right, z down, origin at the CG.
// Positions below are measured from the CG unless stated otherwise.

import { DEG } from './math.js';

const NOSE_TO_CG = 0.562; // [USER] m
const NOSE_TO_WING_LE = 0.471; // [USER] m

export const AIRCRAFT = {
  name: 'Lifeguard rescue QuadPlane',

  // ------------------------------------------------------------ mass
  mass: 7.481, // [USER] kg all-up, with flotation payload
  payloadMass: 0.34, // [USER] kg flotation device (dropped)
  // [USER] inertia about the CG, kg m^2. Product sum(m x z) = +0.0346 (same
  // sign in FRD since both axes flip); tensor element is therefore -0.0346.
  Ixx: 0.334,
  Iyy: 1.11,
  Izz: 1.426,
  Ixz: 0.0346,
  noseToCg: NOSE_TO_CG,
  cgAboveCentreline: 0.011, // [USER] m

  // ------------------------------------------------------------ reference geometry
  ref: {
    S: 0.453, // [USER] m^2
    b: 1.34, // [USER] m
    c: 0.338, // [USER] m (flow5 reference chord)
  },
  wing: {
    span: 1.34,
    chord: 0.339,
    leX: NOSE_TO_CG - NOSE_TO_WING_LE, // +0.091 m: leading edge is ahead of CG
    z: -0.05, // [EST] wing sits on top of the fuselage
    aileronY: [0.37, 0.64], // [USER]
  },
  tail: {
    // [USER] inverted-V (Lambda) tail, ruddervators
    area: 0.106,
    chord: 0.162,
    panelAngle: 29 * DEG,
    arm: 0.85, // CG to tail aerodynamic centre
    z: 0.0, // [EST] panel roots sit on the booms, apex above
  },
  fuselage: {
    length: 1.0, // [USER] approx
    diameter: 0.14, // [EST]
  },
  boomY: 0.285, // [USER]

  // ------------------------------------------------------------ aerodynamics
  // [USER] flow5 Type-7 stability derivatives, about the CG, stability axes at
  // trim alpha 2.63 deg; AVL-style rate normalisation (q c/2V, p b/2V, r b/2V).
  aero: {
    rhoAnalysis: 1.083,
    trim: { V: 22.8, alpha: 2.63 * DEG, CL: 0.574, CD: 0.053 },
    CD0: 0.026, // polar CD = CD0 + k CL^2
    k: 0.081,
    CLmax: 1.08, // 3D
    CLminNeg: -0.6, // [EST] negative stall (cambered RG-15)
    CLa: 4.04,
    Cma: -0.723,
    CLq: 7.4,
    Cmq: -8.16,
    CYb: -0.081,
    Clb: -0.062,
    Cnb: 0.072,
    CYp: 0.035,
    Clp: -0.26,
    Cnp: -0.006,
    CYr: 0.114,
    Clr: 0.11,
    Cnr: -0.102,
    // control derivatives per RADIAN (converted from the per-degree data)
    Cmde: -0.0219 / DEG, // elevator: both ruddervators together, TE down +
    CLde: 0.0094 / DEG,
    Cndr: 0.00197 / DEG, // rudder: + = nose right (non-standard sign, as supplied)
    CYdr: -0.00217 / DEG,
    Cldr: 0.00052 / DEG,
    Clda: 0.00369 / DEG, // aileron: per side, (dL - dR)/2 > 0 = right wing down
    Cnda: -0.0005 / DEG,
    maxDeflection: 20 * DEG, // [USER] per surface
    // [EST] stall blending (post-stall flat-plate model) - see aero.js
    stallBand: 10 * DEG, // width of the attached->separated blend
    betaSepStart: 12 * DEG,
    betaSepFull: 35 * DEG,
    flatPlateCN: 1.25, // normal-force coefficient of the wing/tail at 90 deg
    dragMultiplier: 1.0, // sensitivity knob for the optimistic panel-method polar
  },

  // ------------------------------------------------------------ lift rotors
  // [USER] 4 x SunnySky X3525-520KV, 15x5.5 2-blade, H layout on two booms.
  // ArduPilot Quad-X motor order: 1 front-right CCW, 2 rear-left CCW,
  // 3 front-left CW, 4 rear-right CW. spin = +1 for CCW seen from above.
  liftRotors: [
    { pos: [0.55, 0.285, -0.12], spin: +1, label: '1 FR (CCW)' },
    { pos: [-0.55, -0.285, -0.12], spin: +1, label: '2 RL (CCW)' },
    { pos: [0.55, -0.285, -0.12], spin: -1, label: '3 FL (CW)' },
    { pos: [-0.55, 0.285, -0.12], spin: -1, label: '4 RR (CW)' },
  ],
  liftRotor: {
    radius: 0.1905, // 15 in
    blades: 2,
    solidity: 0.0936, // [EST] mean chord ~28 mm
    liftSlope: 5.7, // [EST] blade section lift-curve slope per rad
    kappa: 1.15, // [EST] induced power factor
    inertia: 4.6e-4, // [EST] kg m^2 prop + motor bell
    // Calibration targets [USER]: these fix blade pitch and profile drag.
    calib: {
      maxStaticThrust: 32, // N per rotor at full throttle
      maxThrustVoltage: 22.2, // V at the ESC (6S nominal, loaded)
      maxThrustRho: 1.225, // [EST] datasheet / eCalc conditions
      hoverPowerTotal: 1205, // W from the battery, whole aircraft
      hoverRho: 1.083, // design density used for the 1205 W figure
    },
    // Filled in by calibrateLiftRotor() at start-up:
    theta75: null,
    Cd0: null,
  },
  liftMotor: {
    kv: 520, // [USER] rpm/V
    resistance: 0.034, // [EST] ohm (not supplied for the X3525)
    noLoadCurrent: 1.0, // [EST] A
    escEfficiency: 0.96, // [EST]
    escMaxCurrent: 60, // [EST] A, ESC current limit (start-up inrush)
  },

  // ------------------------------------------------------------ pusher
  // [USER] SunnySky X4130-520KV, Hobbywing 120 A ESC, APC 15x8E.
  pusher: {
    pos: [-0.45, 0, 0.011], // thrust line on the fuselage centreline, 11 mm below CG
    spin: +1, // [EST] reaction torque sign about +x
    diameter: 0.381,
    Ct0: 0.101, // [USER]
    Cp0: 0.041, // [USER]
    J0: 0.64, // [USER] zero-thrust advance ratio: Ct = Ct0 (1 - J/J0)
    // Cp = Cp0 (1 - (J/Jp)^2). Jp is calibrated so that level cruise at
    // 20.8 m/s draws the supplied 365 W.
    Jp: null,
    inertia: 8.0e-4, // [EST]
    maxRpmApc: 145000 / 15, // APC published limit for E props
  },
  pusherMotor: {
    kv: 520, // [USER]
    resistance: 0.0129, // [USER]
    noLoadCurrent: 2.5, // [USER]
    maxCurrent: 85, // [USER] motor rating (warning threshold)
    escEfficiency: 0.96, // [EST]
    escMaxCurrent: 120, // [USER] Hobbywing 120 A ESC
  },
  cruiseCalib: { V: 20.8, power: 365, rho: 1.083 }, // [USER] design point

  // ------------------------------------------------------------ battery
  battery: {
    cells: 6, // [USER] 6S2P, 2 x 6500 mAh
    capacityAh: 13.0,
    massKg: 1.658,
    r0: 0.009, // [EST] ohm pack ohmic resistance (6S2P)
    r1: 0.006, // [EST] ohm polarisation resistance
    tau1: 20, // [EST] s polarisation time constant
    rWiring: 0.002, // [EST]
    avionicsPower: 8, // [EST] W (FC, GPS, receiver, servos, telemetry)
  },

  // ------------------------------------------------------------ landing gear / contact
  // [EST] skids under the booms; CG 0.28 m above the ground when parked.
  contactPoints: [
    { name: 'skid FR', pos: [0.45, 0.285, 0.28], type: 'gear' },
    { name: 'skid FL', pos: [0.45, -0.285, 0.28], type: 'gear' },
    { name: 'skid RR', pos: [-0.45, 0.285, 0.28], type: 'gear' },
    { name: 'skid RL', pos: [-0.45, -0.285, 0.28], type: 'gear' },
    { name: 'nose', pos: [0.56, 0, 0.07], type: 'structure' },
    { name: 'tail apex', pos: [-0.89, 0, -0.16], type: 'structure' },
    { name: 'right boom end', pos: [-0.97, 0.285, 0.02], type: 'structure' },
    { name: 'left boom end', pos: [-0.97, -0.285, 0.02], type: 'structure' },
    { name: 'right wingtip', pos: [-0.08, 0.67, -0.05], type: 'structure' },
    { name: 'left wingtip', pos: [-0.08, -0.67, -0.05], type: 'structure' },
    { name: 'pusher prop', pos: [-0.45, 0, 0.2], type: 'prop' },
    { name: 'rotor 1 tip', pos: [0.74, 0.285, -0.12], type: 'prop' },
    { name: 'rotor 2 tip', pos: [-0.74, -0.285, -0.12], type: 'prop' },
    { name: 'rotor 3 tip', pos: [0.74, -0.285, -0.12], type: 'prop' },
    { name: 'rotor 4 tip', pos: [-0.74, 0.285, -0.12], type: 'prop' },
  ],
  servo: { rateLimit: 400 * DEG, tau: 0.02 }, // [EST] digital servo
};

export const GEOMETRY_NOTES = {
  wingMidChordX: AIRCRAFT.wing.leX - AIRCRAFT.wing.chord / 2, // -0.078 m
  tailMidChordX: -AIRCRAFT.tail.arm - 0.25 * AIRCRAFT.tail.chord,
};
