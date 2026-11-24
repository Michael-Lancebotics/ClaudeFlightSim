// Menus: settings (environment, scenario, ArduPilot parameters, display),
// controller setup / stick calibration wizard, and help.

import { ALL_MODES } from '../fc/flightcontroller.js';
import { PARAM_INFO } from '../fc/params.js';
import { FUNCTIONS, SWITCHES, defaultInputConfig } from './input.js';

const SETTINGS_KEY = 'cfs.settings.v1';

export function loadSettings() {
  try {
    const raw = localStorage.getItem(SETTINGS_KEY);
    if (raw) return JSON.parse(raw);
  } catch (e) {
    /* storage unavailable */
  }
  return {};
}

export function saveSettings(s) {
  try {
    localStorage.setItem(SETTINGS_KEY, JSON.stringify(s));
  } catch (e) {
    /* storage unavailable */
  }
}

const h = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v;
    else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
    else if (k === 'html') e.innerHTML = v;
    else e.setAttribute(k, v);
  }
  for (const k of kids) if (k != null) e.append(k);
  return e;
};

function modal(title, onClose) {
  const back = h('div', { class: 'modal-back' });
  const box = h('div', { class: 'modal' });
  const head = h('div', { class: 'modal-head' }, h('div', { class: 'modal-title' }, title), h('button', { class: 'btn', onclick: () => close() }, 'Close ✕'));
  const body = h('div', { class: 'modal-body' });
  box.append(head, body);
  back.append(box);
  document.body.append(back);
  const close = () => {
    back.remove();
    if (onClose) onClose();
  };
  back.addEventListener('mousedown', (e) => {
    if (e.target === back) close();
  });
  return { body, close, box };
}

function field(label, input, hint) {
  return h('label', { class: 'field' }, h('span', { class: 'field-label' }, label), input, hint ? h('span', { class: 'field-hint' }, hint) : null);
}

function num(value, step = 'any', min, max) {
  const i = h('input', { type: 'number', step: String(step), value: String(value) });
  if (min != null) i.min = String(min);
  if (max != null) i.max = String(max);
  return i;
}

function select(options, value) {
  const s = h('select');
  for (const o of options) {
    const [v, l] = Array.isArray(o) ? o : [o, o];
    const opt = h('option', { value: v == null ? '' : String(v) }, l);
    if (String(v) === String(value) || (v == null && (value == null || value === ''))) opt.selected = true;
    s.append(opt);
  }
  return s;
}

function check(value) {
  const c = h('input', { type: 'checkbox' });
  c.checked = !!value;
  return c;
}

// ====================================================================== settings
export function openSettings(app) {
  const sim = app.sim;
  app.setPaused(true, 'menu');
  const m = modal('Settings', () => app.setPaused(false, 'menu'));
  const tabs = h('div', { class: 'tabs' });
  const pages = h('div', { class: 'tab-pages' });
  m.body.append(tabs, pages);
  const addTab = (name, page) => {
    const b = h('button', { class: 'tab', onclick: () => show() }, name);
    const show = () => {
      for (const t of tabs.children) t.classList.remove('active');
      for (const p of pages.children) p.style.display = 'none';
      b.classList.add('active');
      page.style.display = 'block';
    };
    tabs.append(b);
    page.style.display = 'none';
    pages.append(page);
    if (tabs.children.length === 1) show();
  };

  // --- environment
  const env = sim.envSettings;
  const e = {
    windSpeed: num(env.windSpeed, 0.5, 0, 20),
    windFromDeg: num(env.windFromDeg, 5, 0, 359),
    gustAmp: num(env.gustAmp, 0.5, 0, 10),
    turbulence: select(['off', 'light', 'moderate', 'severe'], env.turbulence),
    temperatureC: num(env.temperatureC, 1, -10, 50),
    useDesignDensity: check(env.useDesignDensity),
    dragMultiplier: num(env.dragMultiplier, 0.05, 0.8, 1.6),
    vrs: check(env.vrs),
    sensorNoise: check(env.sensorNoise),
  };
  const envPage = h('div', { class: 'page' },
    h('p', { class: 'note' }, 'Design condition: steady wind up to 10 m/s plus gusts. Wind direction is where it blows FROM (0° = from the sea, 180° = offshore).'),
    field('Wind speed at 10 m (m/s)', e.windSpeed),
    field('Wind from (deg)', e.windFromDeg, '0 = onshore from the sea, 90 = from the east'),
    field('Gust amplitude (m/s)', e.gustAmp, 'discrete 1-cosine gusts'),
    field('Turbulence', e.turbulence, 'Dryden model, scales with wind'),
    field('Air temperature (°C)', e.temperatureC, `density ${sim.rho.toFixed(3)} kg/m³ now`),
    field('Use design density 1.083 kg/m³', e.useDesignDensity, 'the conservative flow5 case'),
    field('Drag multiplier', e.dragMultiplier, 'panel methods under-predict drag; try 1.2'),
    field('Vortex-ring-state model', e.vrs, 'thrust loss/roughness in fast vertical descents'),
    field('Sensor errors (GPS / baro / airspeed)', e.sensorNoise),
  );
  addTab('Environment', envPage);

  // --- scenario
  const sc = sim.scenarioSettings;
  const s = {
    swimmerNorth: num(sc.swimmerNorth, 10, 100, 2000),
    swimmerEast: num(sc.swimmerEast, 10, -1000, 1000),
    timeLimit: num(sc.timeLimit, 5, 10, 300),
    dropHeight: num(sc.dropHeight, 0.5, 1, 20),
    cruiseAlt: num(sc.cruiseAlt, 1, 10, 100),
  };
  const scPage = h('div', { class: 'page' },
    h('p', { class: 'note' }, 'Rescue scenario: the swimmer is offshore; reach them within the time limit, release the flotation device at the drop height, then return and land. AUTO flies it with the mission below.'),
    field('Swimmer distance offshore (m)', s.swimmerNorth),
    field('Swimmer offset east (m)', s.swimmerEast),
    h('button', { class: 'btn', onclick: () => {
      s.swimmerNorth.value = String(Math.round(500 + Math.random() * 600));
      s.swimmerEast.value = String(Math.round(-300 + Math.random() * 600));
    } }, 'Random swimmer position'),
    field('Time limit (s)', s.timeLimit),
    field('Drop height above water (m)', s.dropHeight),
    field('Cruise / dash altitude (m)', s.cruiseAlt),
    h('div', { class: 'mission', html: missionHtml(sim) }),
  );
  addTab('Rescue scenario', scPage);

  // --- parameters
  const pInputs = {};
  const pPage = h('div', { class: 'page' }, h('p', { class: 'note' }, 'ArduPilot-style parameters used by the built-in flight controller (approximation of ArduPlane/QuadPlane behaviour).'));
  for (const [k, label] of Object.entries(PARAM_INFO)) {
    pInputs[k] = num(sim.fc.p[k], 'any');
    pPage.append(field(`${k}`, pInputs[k], label));
  }
  addTab('ArduPilot parameters', pPage);

  // --- display
  const cam = app.cam;
  const d = {
    losFov: num(cam.losFov, 1, 3, 75),
    autoZoom: check(cam.autoZoom),
    marker: check(app.showMarker),
    fpvTilt: num(cam.fpvTilt, 1, -30, 45),
    fpvFov: num(cam.fpvFov, 1, 60, 130),
    pilotN: num(cam.pilotNed[0], 1, -60, 30),
    pilotE: num(cam.pilotNed[1], 1, -100, 100),
    yaw: num(app.startYaw, 5, 0, 359),
    mode: select(ALL_MODES, app.startMode),
  };
  const dPage = h('div', { class: 'page' },
    field('Line-of-sight field of view (deg)', d.losFov, 'a person sees roughly 50-60°'),
    field('Line-of-sight auto zoom', d.autoZoom, 'keeps the aircraft from shrinking to a dot'),
    field('Aircraft / swimmer markers', d.marker, 'boxes around the aircraft and swimmer (O key)'),
    field('FPV camera tilt (deg)', d.fpvTilt),
    field('FPV field of view (deg)', d.fpvFov),
    field('Pilot position north of pad (m)', d.pilotN),
    field('Pilot position east of pad (m)', d.pilotE),
    field('Start heading (deg)', d.yaw, '0 = facing the sea'),
    field('Start flight mode', d.mode),
  );
  addTab('Display & start', dPage);

  const val = (i) => (i.type === 'checkbox' ? i.checked : i.tagName === 'SELECT' ? i.value : parseFloat(i.value));
  const apply = (reset) => {
    const envNew = {};
    for (const [k, i] of Object.entries(e)) envNew[k] = val(i);
    sim.setEnv(envNew);
    const scNew = {};
    for (const [k, i] of Object.entries(s)) scNew[k] = val(i);
    sim.setScenario(scNew);
    const pNew = {};
    for (const [k, i] of Object.entries(pInputs)) {
      const v = val(i);
      if (isFinite(v)) pNew[k] = v;
    }
    sim.setFcParams(pNew);
    cam.losFov = val(d.losFov);
    cam.autoZoom = val(d.autoZoom);
    app.showMarker = val(d.marker);
    cam.fpvTilt = val(d.fpvTilt);
    cam.fpvFov = val(d.fpvFov);
    cam.pilotNed = [val(d.pilotN), val(d.pilotE)];
    app.startYaw = val(d.yaw);
    app.startMode = val(d.mode);
    app.persist();
    if (reset) app.reset();
    m.close();
  };
  const foot = h('div', { class: 'modal-foot' },
    h('button', { class: 'btn', onclick: () => {
      if (confirm('Restore all settings to defaults?')) {
        app.resetDefaults();
        m.close();
      }
    } }, 'Defaults'),
    h('div', { class: 'spacer' }),
    h('button', { class: 'btn', onclick: () => apply(false) }, 'Apply'),
    h('button', { class: 'btn primary', onclick: () => apply(true) }, 'Apply & reset flight'),
  );
  m.box.append(foot);
}

function missionHtml(sim) {
  const items = sim.fc.mission || [];
  return '<div class="mission-title">AUTO mission (rebuilt from these settings)</div><ol>' + items.map((it) => `<li>${it.label || it.type}</li>`).join('') + '</ol>';
}

// ====================================================================== controller setup
export function openController(app) {
  const input = app.input;
  app.setPaused(true, 'menu');
  let cfg = JSON.parse(JSON.stringify(input.cfg));
  const m = modal('Controller setup — RadioMaster Pocket (ELRS / EdgeTX)', () => {
    cancelAnimationFrame(raf);
    input.enabled = true;
    app.setPaused(false, 'menu');
  });
  input.enabled = false;
  const status = h('div', { class: 'ctl-status' });
  const help = h('div', { class: 'note', html: `
    <b>Connect the radio:</b> power it on, plug it into the PC with a USB-C data cable and choose
    <b>"USB Joystick (HID)"</b> on the radio's pop-up. Then move a stick — browsers only report a joystick after it moves.
    Windows: it should also appear as a game controller in <i>joy.cpl</i>.<br>
    In the radio's model, keep the ArduPilot channel order (AETR): CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw,
    then the arm, flight-mode, RTL/QLAND and payload switches on CH5–CH8 (EdgeTX: SYS → USB Joystick, classic mode maps CH1–8 to axes).
    Use the wizard to detect everything automatically.` });
  const live = h('div', { class: 'ctl-live' });
  const mapped = h('div', { class: 'ctl-mapped' });
  const wizard = h('div', { class: 'ctl-wizard' });
  const manual = h('div', { class: 'ctl-manual' });
  const modes = h('div', { class: 'ctl-modes' });
  m.body.append(status, help, h('h3', {}, 'Live joystick axes'), live, h('h3', {}, 'Mapped controls'), mapped, wizard, h('h3', {}, 'Manual mapping'), manual, h('h3', {}, 'Flight-mode assignments (swappable)'), modes);

  // ---------------- live view
  const liveRows = [];
  const mappedRows = {};
  const buildLive = (n) => {
    live.innerHTML = '';
    liveRows.length = 0;
    for (let i = 0; i < n; i++) {
      const bar = h('div', { class: 'bar' }, h('div', { class: 'bar-fill' }));
      const v = h('span', { class: 'bar-val' }, '0');
      live.append(h('div', { class: 'bar-row' }, h('span', { class: 'bar-lbl' }, `Axis ${i}`), bar, v));
      liveRows.push({ bar, v });
    }
  };
  for (const f of [...FUNCTIONS, ...SWITCHES]) {
    const bar = h('div', { class: 'bar' }, h('div', { class: 'bar-fill' }));
    const v = h('span', { class: 'bar-val' }, '');
    mapped.append(h('div', { class: 'bar-row' }, h('span', { class: 'bar-lbl' }, f), bar, v));
    mappedRows[f] = { bar, v };
  }

  // ---------------- wizard
  let wiz = null;
  const steps = [
    { id: 'center', text: 'Centre the roll, pitch and yaw sticks and put the THROTTLE fully DOWN. Put all switches in their normal (disarmed / first) position. Then press Next.' },
    { id: 'extents', text: 'Move BOTH sticks slowly around their full range (all corners) a few times, then press Next.' },
    { id: 'roll', text: 'Hold the ROLL stick fully RIGHT, then press Next.' },
    { id: 'pitch', text: 'Hold the PITCH stick fully FORWARD (away from you, nose down), then press Next.' },
    { id: 'throttle', text: 'Push the THROTTLE fully UP, then press Next.' },
    { id: 'yaw', text: 'Hold the YAW stick fully RIGHT, then press Next.' },
    { id: 'arm', text: 'Flip the ARM switch to ARMED, then press Next. (Flip it back afterwards.)' },
    { id: 'mode1', text: 'Put the FLIGHT-MODE switch in position 1 (QLOITER), then press Next.' },
    { id: 'mode2', text: 'Flight-mode switch to position 2 (FBWA), then press Next.' },
    { id: 'mode3', text: 'Flight-mode switch to position 3 (AUTO), then press Next.' },
    { id: 'rtl1', text: 'RTL/QLAND switch in its NORMAL (off) position, then press Next.' },
    { id: 'rtl2', text: 'RTL/QLAND switch to RTL, then press Next.' },
    { id: 'rtl3', text: 'RTL/QLAND switch to QLAND (press Next again without moving it if it only has 2 positions).' },
    { id: 'payload', text: 'Flip the PAYLOAD RELEASE switch to RELEASE, then press Next (flip it back afterwards).' },
  ];
  const startWizard = () => {
    wiz = { i: 0, snap: {}, min: [], max: [] };
    renderWizard();
  };
  const snapshot = () => {
    const r = input.rawAxes();
    return r ? { axes: r.axes.slice(), buttons: r.buttons.slice() } : null;
  };
  const biggestChange = (a, b) => {
    let best = null, bd = 0;
    a.axes.forEach((v, i) => {
      const d = Math.abs(b.axes[i] - v);
      if (d > bd) {
        bd = d;
        best = { source: 'axis', index: i, from: v, to: b.axes[i], d };
      }
    });
    a.buttons.forEach((v, i) => {
      const d = Math.abs(b.buttons[i] - v);
      if (d > 0.5 && d > bd) {
        bd = d;
        best = { source: 'button', index: i, from: v, to: b.buttons[i], d };
      }
    });
    return best;
  };
  const renderWizard = () => {
    wizard.innerHTML = '';
    if (!wiz) {
      wizard.append(h('button', { class: 'btn primary', onclick: startWizard }, 'Start calibration wizard'));
      return;
    }
    const st = steps[wiz.i];
    wizard.append(
      h('div', { class: 'wiz-step' }, `Step ${wiz.i + 1} of ${steps.length}: `, h('b', {}, st.text)),
      h('div', { class: 'wiz-msg' }, wiz.msg || ''),
      h('button', { class: 'btn', onclick: () => {
        wiz = null;
        renderWizard();
      } }, 'Cancel'),
      h('button', { class: 'btn primary', onclick: next }, wiz.i === steps.length - 1 ? 'Finish' : 'Next'),
    );
  };
  const next = () => {
    const snap = snapshot();
    if (!snap) {
      wiz.msg = 'No joystick detected yet - plug in the radio and move a stick.';
      renderWizard();
      return;
    }
    const st = steps[wiz.i];
    const base = wiz.snap.center;
    wiz.msg = '';
    switch (st.id) {
      case 'center':
        wiz.snap.center = snap;
        wiz.min = snap.axes.slice();
        wiz.max = snap.axes.slice();
        break;
      case 'extents':
        break;
      case 'roll':
      case 'pitch':
      case 'yaw':
      case 'throttle': {
        const ch = biggestChange(base, snap);
        if (!ch || ch.source !== 'axis' || ch.d < 0.3) {
          wiz.msg = 'Did not see a stick move - try again.';
          renderWizard();
          return;
        }
        const i = ch.index;
        const lo = Math.min(wiz.min[i], snap.axes[i], base.axes[i]);
        const hi = Math.max(wiz.max[i], snap.axes[i], base.axes[i]);
        if (st.id === 'throttle') {
          cfg.axes.throttle = { index: i, min: base.axes[i], max: snap.axes[i], center: (lo + hi) / 2, invert: false };
          if (snap.axes[i] < base.axes[i]) cfg.axes.throttle = { index: i, min: snap.axes[i], max: base.axes[i], center: 0, invert: true };
        } else {
          const positive = snap.axes[i] > base.axes[i];
          // roll/yaw: right = +1. pitch: forward = -1 (nose down)
          const invert = st.id === 'pitch' ? positive : !positive;
          cfg.axes[st.id] = { index: i, min: lo, center: base.axes[i], max: hi, invert };
        }
        break;
      }
      case 'arm':
      case 'payload': {
        const ch = biggestChange(base, snap);
        if (!ch) {
          wiz.msg = 'Did not see a switch change - try again.';
          renderWizard();
          return;
        }
        cfg.switches[st.id] = { source: ch.source, index: ch.index, values: [ch.from, ch.to] };
        break;
      }
      case 'mode1':
      case 'rtl1':
        wiz.snap[st.id] = snap;
        break;
      case 'mode2':
      case 'mode3':
      case 'rtl2':
      case 'rtl3': {
        const group = st.id.startsWith('mode') ? 'mode' : 'rtl';
        const first = wiz.snap[group + '1'];
        if (st.id.endsWith('2')) {
          const ch = biggestChange(first, snap);
          if (!ch) {
            wiz.msg = 'Did not see the switch change - try again.';
            renderWizard();
            return;
          }
          wiz.snap[group + 'ch'] = ch;
          cfg.switches[group] = { source: ch.source, index: ch.index, values: [ch.from, ch.to] };
        } else {
          const ch = wiz.snap[group + 'ch'];
          const v = ch.source === 'axis' ? snap.axes[ch.index] : snap.buttons[ch.index];
          const vals = cfg.switches[group].values;
          if (Math.abs(v - vals[1]) > 0.2) vals.push(v);
        }
        break;
      }
    }
    if (wiz.i === steps.length - 1) {
      wiz = null;
      wizard.innerHTML = '';
      wizard.append(h('div', { class: 'wiz-done' }, 'Calibration captured. Check the mapped bars below, then Save.'));
      buildManual();
      return;
    }
    wiz.i++;
    renderWizard();
  };

  // ---------------- manual mapping
  const buildManual = () => {
    manual.innerHTML = '';
    const r = input.rawAxes();
    const nAx = r ? r.axes.length : 8;
    const nBt = r ? r.buttons.length : 0;
    const axisOpts = Array.from({ length: Math.max(nAx, 8) }, (_, i) => [i, `Axis ${i}`]);
    const t = h('table', { class: 'map-table' }, h('tr', {}, h('th', {}, 'Control'), h('th', {}, 'Axis'), h('th', {}, 'Reverse')));
    for (const f of FUNCTIONS) {
      const a = cfg.axes[f];
      const sel = select(axisOpts, a.index);
      sel.onchange = () => (a.index = parseInt(sel.value, 10));
      const inv = check(a.invert);
      inv.onchange = () => (a.invert = inv.checked);
      t.append(h('tr', {}, h('td', {}, f), h('td', {}, sel), h('td', {}, inv)));
    }
    for (const sw of SWITCHES) {
      const s = cfg.switches[sw];
      const opts = [...axisOpts.map(([i, l]) => [`axis:${i}`, l]), ...Array.from({ length: nBt }, (_, i) => [`button:${i}`, `Button ${i}`])];
      const sel = select(opts, `${s.source}:${s.index}`);
      sel.onchange = () => {
        const [src, idx] = sel.value.split(':');
        s.source = src;
        s.index = parseInt(idx, 10);
        if (src === 'button') s.values = sw === 'mode' || sw === 'rtl' ? [0, 1] : [0, 1];
      };
      t.append(h('tr', {}, h('td', {}, `${sw} switch`), h('td', {}, sel), h('td', {}, h('span', { class: 'field-hint' }, `positions: ${s.values.map((v) => (+v).toFixed(2)).join(' / ')}`))));
    }
    manual.append(t);
  };

  // ---------------- mode assignments
  const buildModes = () => {
    modes.innerHTML = '';
    const modeOpts = ALL_MODES.map((x) => [x, x]);
    const row = (label, arr, i, allowNone) => {
      const sel = select(allowNone ? [[null, '(no change)'], ...modeOpts] : modeOpts, arr[i]);
      sel.onchange = () => (arr[i] = sel.value || null);
      return h('div', { class: 'mode-row' }, h('span', { class: 'field-label' }, label), sel);
    };
    modes.append(h('div', { class: 'note' }, 'Flight-mode switch positions and the RTL/QLAND switch. Modes not on a switch can be bound to keys below - any mode can be moved between switch and keyboard.'));
    for (let i = 0; i < 3; i++) modes.append(row(`Mode switch position ${i + 1}`, cfg.modeSlots, i, false));
    for (let i = 0; i < 3; i++) modes.append(row(`RTL/QLAND switch position ${i + 1}`, cfg.rtlSlots, i, true));
    const keyTable = h('table', { class: 'map-table' }, h('tr', {}, h('th', {}, 'Mode'), h('th', {}, 'Keyboard key'), h('th', {}, '')));
    for (const mode of ALL_MODES) {
      const key = Object.keys(cfg.keyModes).find((k) => cfg.keyModes[k] === mode);
      const keyLbl = h('span', { class: 'kbd' }, key ? key.replace('Key', '').replace('Digit', '') : '—');
      const bindBtn = h('button', { class: 'btn small', onclick: () => {
        keyLbl.textContent = 'press a key…';
        input.captureKey = (code) => {
          for (const k of Object.keys(cfg.keyModes)) if (cfg.keyModes[k] === mode || k === code) delete cfg.keyModes[k];
          if (code !== 'Escape' && code !== 'Backspace') cfg.keyModes[code] = mode;
          buildModes();
        };
      } }, 'Bind');
      const clearBtn = h('button', { class: 'btn small', onclick: () => {
        for (const k of Object.keys(cfg.keyModes)) if (cfg.keyModes[k] === mode) delete cfg.keyModes[k];
        buildModes();
      } }, 'Clear');
      keyTable.append(h('tr', {}, h('td', {}, mode), h('td', {}, keyLbl), h('td', {}, bindBtn, clearBtn)));
    }
    modes.append(keyTable);
  };

  buildManual();
  buildModes();
  renderWizard();

  const foot = h('div', { class: 'modal-foot' },
    h('button', { class: 'btn', onclick: () => {
      cfg = defaultInputConfig();
      buildManual();
      buildModes();
    } }, 'Defaults'),
    h('button', { class: 'btn', onclick: () => {
      const blob = new Blob([JSON.stringify(cfg, null, 2)], { type: 'application/json' });
      const a = h('a', { href: URL.createObjectURL(blob), download: 'radiomaster-pocket-calibration.json' });
      a.click();
    } }, 'Export'),
    h('button', { class: 'btn', onclick: () => {
      const f = h('input', { type: 'file', accept: '.json' });
      f.onchange = async () => {
        try {
          cfg = { ...defaultInputConfig(), ...JSON.parse(await f.files[0].text()) };
          buildManual();
          buildModes();
        } catch (err) {
          alert('Could not read that file');
        }
      };
      f.click();
    } }, 'Import'),
    h('div', { class: 'spacer' }),
    h('button', { class: 'btn primary', onclick: () => {
      input.setConfig(cfg);
      m.close();
    } }, 'Save'),
  );
  m.box.append(foot);

  // ---------------- live refresh
  let raf = 0;
  let nLive = -1;
  const tick = () => {
    const r = input.rawAxes();
    if (input.gamepadError) {
      status.innerHTML = `<span class="bad">This page cannot read game controllers (${input.gamepadError}). Open the downloaded ClaudeFlightSim.html file directly in Chrome or Edge instead.</span>`;
    } else if (!r) {
      status.innerHTML = '<span class="caution">No joystick detected.</span> Plug in the radio (USB Joystick mode) and move a stick.';
    } else {
      status.innerHTML = `<span class="good">Connected:</span> ${r.id} (index ${r.index}) — ${r.axes.length} axes, ${r.buttons.length} buttons`;
      if (r.axes.length !== nLive) {
        nLive = r.axes.length;
        buildLive(nLive);
        buildManual();
      }
      r.axes.forEach((v, i) => {
        const row = liveRows[i];
        if (!row) return;
        row.bar.firstChild.style.width = `${((v + 1) * 50).toFixed(0)}%`;
        row.v.textContent = v.toFixed(2);
        if (wiz && wiz.min.length) {
          wiz.min[i] = Math.min(wiz.min[i], v);
          wiz.max[i] = Math.max(wiz.max[i], v);
        }
      });
      // mapped preview with the working copy of the configuration
      const saved = input.cfg;
      input.cfg = cfg;
      const gp = input.readGamepad();
      for (const f of FUNCTIONS) {
        const a = cfg.axes[f];
        const raw = gp.axes[a.index] ?? 0;
        const centred = f !== 'throttle';
        let v;
        if (centred) {
          v = raw >= a.center ? (raw - a.center) / Math.max(a.max - a.center, 1e-3) : (raw - a.center) / Math.max(a.center - a.min, 1e-3);
          v = Math.max(-1, Math.min(1, v)) * (a.invert ? -1 : 1);
          mappedRows[f].bar.firstChild.style.width = `${((v + 1) * 50).toFixed(0)}%`;
        } else {
          v = Math.max(0, Math.min(1, (raw - a.min) / Math.max(a.max - a.min, 1e-3)));
          if (a.invert) v = 1 - v;
          mappedRows[f].bar.firstChild.style.width = `${(v * 100).toFixed(0)}%`;
        }
        mappedRows[f].v.textContent = v.toFixed(2);
      }
      for (const sw of SWITCHES) {
        const pos = input.readSwitch(cfg.switches[sw], gp);
        const n = cfg.switches[sw].values.length;
        mappedRows[sw].bar.firstChild.style.width = pos == null ? '0%' : `${(((pos + 1) / n) * 100).toFixed(0)}%`;
        let lbl = pos == null ? '-' : `pos ${pos + 1}`;
        if (sw === 'mode' && pos != null) lbl += ` = ${cfg.modeSlots[pos] || ''}`;
        if (sw === 'rtl' && pos != null) lbl += ` = ${cfg.rtlSlots[pos] || 'off'}`;
        if (sw === 'arm' && pos != null) lbl = pos === 1 ? 'ARMED' : 'disarmed';
        if (sw === 'payload' && pos != null) lbl = pos === 1 ? 'RELEASE' : 'hold';
        mappedRows[sw].v.textContent = lbl;
      }
      input.cfg = saved;
    }
    raf = requestAnimationFrame(tick);
  };
  tick();
}

// ====================================================================== help
export function openHelp(app) {
  app.setPaused(true, 'menu');
  const m = modal('How to fly', () => app.setPaused(false, 'menu'));
  m.body.innerHTML = `
  <div class="help">
  <h3>Radio (RadioMaster Pocket, Mode 2)</h3>
  <ul>
    <li><b>Arm switch</b> arms/disarms (throttle must be low to arm). Disarming in the air stops the motors.</li>
    <li><b>Flight-mode switch</b>: QLOITER / FBWA / AUTO. <b>RTL/QLAND switch</b> overrides it; back to off restores the mode switch.</li>
    <li><b>Payload switch</b> releases the flotation device. Target: 5 m above the water.</li>
    <li>Use <b>Controller setup</b> (G) to calibrate the sticks and change the mode assignments.</li>
  </ul>
  <h3>Keyboard</h3>
  <table class="keys">
    <tr><td>H / C / M</td><td>QHOVER / CRUISE / MANUAL (re-bindable)</td></tr>
    <tr><td>1-6</td><td>QLOITER, FBWA, AUTO, RTL, QLAND, QRTL</td></tr>
    <tr><td>Space</td><td>Arm / disarm (keyboard flying)</td></tr>
    <tr><td>W / S, X, Z</td><td>Throttle up / down, 50 % (hover hold), 0</td></tr>
    <tr><td>Arrows</td><td>Pitch / roll (Shift = full deflection)</td></tr>
    <tr><td>A / D</td><td>Yaw</td></tr>
    <tr><td>B</td><td>Release flotation device</td></tr>
    <tr><td>V</td><td>Cycle camera: line-of-sight, chase, FPV</td></tr>
    <tr><td>Mouse wheel or + / -</td><td>Zoom (LOS) / distance (chase) / tilt (FPV)</td></tr>
    <tr><td>O</td><td>Aircraft & swimmer markers on/off</td></tr>
    <tr><td>P</td><td>Pause · <b>T</b> time x0.5 / x1 / x2</td></tr>
    <tr><td>R</td><td>Reset to the launch pad</td></tr>
    <tr><td>G / Tab / F1</td><td>Controller setup / Settings / this help</td></tr>
  </table>
  <h3>Flying the quadplane</h3>
  <ul>
    <li><b>QLOITER</b>: GPS position hold. Throttle centre holds height, up/down climbs/descends; sticks move the aircraft.</li>
    <li><b>QHOVER</b>: height hold, sticks set the lean angle (no position hold — it drifts with the wind).</li>
    <li><b>To transition</b>: climb to a safe height in QLOITER, point into the wind if you can, switch to <b>FBWA</b> and push the throttle up.
      The lift motors hold height until the airspeed reaches ARSPD_FBW_MIN (20 m/s), then ramp down.</li>
    <li><b>FBWA</b>: sticks set bank and pitch, throttle is the pusher. Keep the airspeed above ~20 m/s; below 16 m/s Q_ASSIST adds lift-motor help (stall is 16.7 m/s).</li>
    <li><b>CRUISE</b>: throttle sets the airspeed, pitch stick climbs/descends, heading locks when the roll stick is centred.</li>
    <li><b>Back-transition</b>: slow down in FBWA first, then switch to QLOITER. Expect it to balloon upward and keep coasting for 100+ m from 20+ m/s.</li>
    <li><b>MANUAL</b> turns the lift motors off and removes all stabilisation (training only — the Dutch roll is undamped).</li>
    <li><b>AUTO</b> flies the rescue mission: VTOL take-off, dash, back-transition over the swimmer, drop at 5 m, return and land.</li>
  </ul>
  <h3>Crashes</h3>
  <p>Touching the water, a vertical touchdown faster than 2.5 m/s, touching down sliding faster than 3 m/s or tilted more than 25°, wing/tail strikes, prop strikes, more than 8 g, or more than 42 m/s airspeed.</p>
  </div>`;
}
