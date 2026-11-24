// Pilot input: RadioMaster Pocket (EdgeTX "USB Joystick (HID)" mode) through
// the browser Gamepad API, with keyboard fallback and swappable mode bindings.

import { ALL_MODES } from '../fc/flightcontroller.js';

const STORAGE_KEY = 'cfs.input.v1';

export const FUNCTIONS = ['roll', 'pitch', 'throttle', 'yaw'];
export const SWITCHES = ['arm', 'mode', 'rtl', 'payload'];

/** Defaults for EdgeTX classic joystick mode with an AETR channel order. */
export function defaultInputConfig() {
  return {
    gamepadIndex: null, // null = first connected
    axes: {
      roll: { index: 0, min: -1, center: 0, max: 1, invert: false },
      pitch: { index: 1, min: -1, center: 0, max: 1, invert: true }, // + = stick back (nose up)
      throttle: { index: 2, min: -1, center: 0, max: 1, invert: false },
      yaw: { index: 3, min: -1, center: 0, max: 1, invert: false },
    },
    switches: {
      arm: { source: 'axis', index: 4, values: [-1, 1] }, // [disarmed, armed]
      mode: { source: 'axis', index: 5, values: [-1, 0, 1] },
      rtl: { source: 'axis', index: 6, values: [-1, 0, 1] },
      payload: { source: 'axis', index: 7, values: [-1, 1] }, // [hold, release]
    },
    modeSlots: ['QLOITER', 'FBWA', 'AUTO'], // [USER] 3-position flight-mode switch
    rtlSlots: [null, 'RTL', 'QLAND'], // [USER] second switch: off / RTL / QLAND
    keyModes: {
      KeyH: 'QHOVER', // [USER] modes not on the radio go on the keyboard
      KeyC: 'CRUISE',
      KeyM: 'MANUAL',
      Digit1: 'QLOITER',
      Digit2: 'FBWA',
      Digit3: 'AUTO',
      Digit4: 'RTL',
      Digit5: 'QLAND',
      Digit6: 'QRTL',
    },
    deadband: 0.02,
    expo: 0.0,
  };
}

export function loadInputConfig() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const cfg = JSON.parse(raw);
      const def = defaultInputConfig();
      return { ...def, ...cfg, axes: { ...def.axes, ...cfg.axes }, switches: { ...def.switches, ...cfg.switches } };
    }
  } catch (e) {
    /* storage unavailable */
  }
  return defaultInputConfig();
}

export function saveInputConfig(cfg) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(cfg));
  } catch (e) {
    /* storage unavailable */
  }
}

function normAxis(raw, a, centred) {
  let v;
  if (centred) {
    v = raw >= a.center ? (raw - a.center) / Math.max(a.max - a.center, 1e-3) : (raw - a.center) / Math.max(a.center - a.min, 1e-3);
    v = Math.max(-1, Math.min(1, v));
  } else {
    v = (raw - a.min) / Math.max(a.max - a.min, 1e-3);
    v = Math.max(0, Math.min(1, v));
  }
  if (a.invert) v = centred ? -v : 1 - v;
  return v;
}

function nearestIndex(values, v) {
  let best = 0, bd = Infinity;
  values.forEach((x, i) => {
    if (x == null) return;
    const d = Math.abs(x - v);
    if (d < bd) {
      bd = d;
      best = i;
    }
  });
  return best;
}

export class InputManager {
  constructor() {
    this.cfg = loadInputConfig();
    this.keys = new Set();
    this.keyThrottle = 0;
    this.keyArmed = false;
    this.pending = [];
    this.uiActions = [];
    this.prevSw = null;
    this.gamepad = null;
    this.gamepadSeen = false;
    this.gamepadError = null;
    this.source = 'keyboard';
    this.lastPilot = { roll: 0, pitch: 0, yaw: 0, throttle: 0 };
    this.captureKey = null; // callback used by the key-binding UI
    this.enabled = true;
    window.addEventListener('keydown', (e) => this.onKey(e, true));
    window.addEventListener('keyup', (e) => this.onKey(e, false));
    window.addEventListener('gamepadconnected', () => {
      this.gamepadSeen = true;
    });
  }

  onKey(e, down) {
    if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA')) return;
    if (down && this.captureKey) {
      e.preventDefault();
      const cb = this.captureKey;
      this.captureKey = null;
      cb(e.code);
      return;
    }
    if (!this.enabled) return;
    if (down) this.keys.add(e.code);
    else this.keys.delete(e.code);
    if (!down || e.repeat) return;
    const code = e.code;
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space', 'Tab'].includes(code)) e.preventDefault();
    const mode = this.cfg.keyModes[code];
    if (mode) {
      this.pending.push({ type: 'mode', mode, source: 'keyboard' });
      return;
    }
    switch (code) {
      case 'Space':
        this.keyArmed = !this.keyArmed;
        this.pending.push({ type: this.keyArmed ? 'arm' : 'disarm' });
        break;
      case 'KeyB':
        this.pending.push({ type: 'release' });
        break;
      case 'KeyX':
        this.keyThrottle = 0.5;
        break;
      case 'KeyZ':
        this.keyThrottle = 0;
        break;
      default:
        this.uiActions.push(code);
    }
  }

  /** Read the selected gamepad, or null. */
  readGamepad() {
    let pads;
    try {
      pads = navigator.getGamepads ? navigator.getGamepads() : [];
      this.gamepadError = null;
    } catch (e) {
      this.gamepadError = String(e && e.message ? e.message : e);
      return null;
    }
    const list = Array.from(pads || []).filter(Boolean);
    if (list.length === 0) return null;
    const idx = this.cfg.gamepadIndex;
    const gp = idx != null ? list.find((g) => g.index === idx) || list[0] : list[0];
    return gp;
  }

  rawAxes() {
    const gp = this.readGamepad();
    return gp ? { id: gp.id, index: gp.index, axes: Array.from(gp.axes), buttons: gp.buttons.map((b) => b.value) } : null;
  }

  readSwitch(sw, gp) {
    if (!sw || sw.index == null) return null;
    if (sw.source === 'button') {
      const b = gp.buttons[sw.index];
      if (!b) return null;
      return b.pressed || b.value > 0.5 ? 1 : 0;
    }
    const v = gp.axes[sw.index];
    if (v == null) return null;
    return nearestIndex(sw.values, v);
  }

  /** Poll inputs; returns the pilot command for the flight controller. */
  poll(dt, fcMode) {
    const events = this.pending;
    this.pending = [];
    const gp = this.readGamepad();
    const cfg = this.cfg;
    let pilot;
    if (gp) {
      this.gamepad = gp;
      this.source = 'radio';
      const a = cfg.axes;
      const get = (f, centred) => {
        const ax = a[f];
        const raw = gp.axes[ax.index];
        if (raw == null) return centred ? 0 : 0;
        let v = normAxis(raw, ax, centred);
        if (centred) {
          const db = cfg.deadband;
          v = Math.abs(v) < db ? 0 : (v - Math.sign(v) * db) / (1 - db);
          if (cfg.expo > 0) v = (1 - cfg.expo) * v + cfg.expo * v * v * v;
        }
        return v;
      };
      pilot = { roll: get('roll', true), pitch: get('pitch', true), yaw: get('yaw', true), throttle: get('throttle', false) };
      // switches -> events on change
      const sw = {};
      for (const s of SWITCHES) sw[s] = this.readSwitch(cfg.switches[s], gp);
      if (this.prevSw) {
        if (sw.arm != null && sw.arm !== this.prevSw.arm) events.push({ type: sw.arm === 1 ? 'arm' : 'disarm' });
        if (sw.mode != null && sw.mode !== this.prevSw.mode) {
          const m = cfg.modeSlots[sw.mode];
          const rtlActive = sw.rtl != null && cfg.rtlSlots[sw.rtl];
          if (m && !rtlActive) events.push({ type: 'mode', mode: m, source: 'mode switch' });
        }
        if (sw.rtl != null && sw.rtl !== this.prevSw.rtl) {
          const m = cfg.rtlSlots[sw.rtl];
          if (m) events.push({ type: 'mode', mode: m, source: 'RTL/land switch' });
          else if (sw.mode != null && cfg.modeSlots[sw.mode]) events.push({ type: 'mode', mode: cfg.modeSlots[sw.mode], source: 'mode switch' });
        }
        if (sw.payload != null && sw.payload !== this.prevSw.payload && sw.payload === 1) events.push({ type: 'release' });
      } else {
        // first read: adopt the mode switch position; arming needs a fresh toggle
        if (sw.mode != null && cfg.modeSlots[sw.mode]) events.push({ type: 'mode', mode: cfg.modeSlots[sw.mode], source: 'mode switch' });
        if (sw.arm === 1) this.armSwitchStartedOn = true;
      }
      this.prevSw = sw;
      this.switchState = sw;
      pilot.rcOk = true;
    } else {
      const kbFlying = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'KeyW', 'KeyS', 'KeyA', 'KeyD'].some((k) => this.keys.has(k));
      if (this.source === 'radio' && kbFlying) this.forgetRadio(); // pilot took over on the keyboard
      if (this.source === 'radio') {
        // radio was in use and disappeared -> radio failsafe
        pilot = { ...this.lastPilot, rcOk: false };
        this.prevSw = null;
      } else {
        this.source = 'keyboard';
        pilot = this.keyboardPilot(dt);
        pilot.rcOk = true;
      }
      this.gamepad = null;
    }
    this.lastPilot = { roll: pilot.roll, pitch: pilot.pitch, yaw: pilot.yaw, throttle: pilot.throttle };
    pilot.events = events;
    return pilot;
  }

  keyboardPilot(dt) {
    const k = this.keys;
    const axis = (neg, pos) => (k.has(pos) ? 1 : 0) - (k.has(neg) ? 1 : 0);
    if (k.has('KeyW')) this.keyThrottle = Math.min(1, this.keyThrottle + 0.5 * dt);
    if (k.has('KeyS')) this.keyThrottle = Math.max(0, this.keyThrottle - 0.5 * dt);
    const s = k.has('ShiftLeft') || k.has('ShiftRight') ? 1 : 0.5;
    return {
      roll: axis('ArrowLeft', 'ArrowRight') * s,
      pitch: axis('ArrowUp', 'ArrowDown') * s, // up arrow = stick forward = nose down
      yaw: axis('KeyA', 'KeyD') * s,
      throttle: this.keyThrottle,
    };
  }

  /** Drop the radio and return to keyboard control (after an intentional unplug). */
  forgetRadio() {
    this.source = 'keyboard';
    this.prevSw = null;
  }

  takeUiActions() {
    const a = this.uiActions;
    this.uiActions = [];
    return a;
  }

  setConfig(cfg) {
    this.cfg = cfg;
    this.prevSw = null;
    saveInputConfig(cfg);
  }
}

export { ALL_MODES };
