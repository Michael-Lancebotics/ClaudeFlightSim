// LiPo battery: open-circuit voltage vs state of charge, ohmic resistance and
// one RC polarisation branch (gives the slow extra sag under sustained load).

import { clamp, interpTable } from './math.js';

// Typical LiPo resting voltage per cell vs state of charge
const OCV_TABLE = [
  [0.0, 3.27], [0.05, 3.61], [0.1, 3.69], [0.15, 3.71], [0.2, 3.73],
  [0.25, 3.75], [0.3, 3.77], [0.35, 3.79], [0.4, 3.8], [0.45, 3.82],
  [0.5, 3.84], [0.55, 3.85], [0.6, 3.87], [0.65, 3.91], [0.7, 3.95],
  [0.75, 3.98], [0.8, 4.02], [0.85, 4.08], [0.9, 4.11], [0.95, 4.15],
  [1.0, 4.2],
];

export class Battery {
  constructor(p, initialSoc = 1.0) {
    this.p = p;
    this.reset(initialSoc);
  }

  reset(soc = 1.0) {
    this.usedAh = this.p.capacityAh * (1 - soc);
    this.usedWh = 0;
    this.v1 = 0; // polarisation voltage
    this.current = 0;
    this.voltage = this.ocv();
    this.power = 0;
  }

  get soc() {
    return clamp(1 - this.usedAh / this.p.capacityAh, 0, 1);
  }

  ocv() {
    return this.p.cells * interpTable(OCV_TABLE, this.soc);
  }

  /** Terminal voltage that would result from drawing current I now. */
  terminalVoltage(I) {
    return this.ocv() - I * (this.p.r0 + this.p.rWiring) - this.v1;
  }

  /** Advance the battery state with total current I (A) over dt (s). */
  update(I, dt) {
    const p = this.p;
    this.current = I;
    const a = Math.exp(-dt / p.tau1);
    this.v1 = a * this.v1 + (1 - a) * I * p.r1;
    this.voltage = Math.max(this.terminalVoltage(I), 0);
    this.power = this.voltage * I;
    this.usedAh += (I * dt) / 3600;
    this.usedWh += (this.power * dt) / 3600;
  }

  get cellVoltage() {
    return this.voltage / this.p.cells;
  }
}
