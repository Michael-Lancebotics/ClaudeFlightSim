// Browser entry point.
import * as THREE from 'three';
import { Simulation } from '../sim/simulation.js';
import { World, ned2three } from './scene.js';
import { AircraftModel } from './aircraftModel.js';
import { CameraRig } from './cameras.js';
import { Hud } from './hud.js';
import { InputManager } from './input.js';
import { openSettings, openController, openHelp, loadSettings, saveSettings } from './ui.js';
import './style.css';

class App {
  constructor() {
    const saved = loadSettings();
    this.saved = saved;
    this.sim = new Simulation({ env: saved.env || {}, fcParams: saved.fcParams || {}, scenario: saved.scenario || {} });
    this.startYaw = saved.startYaw ?? 0;
    this.startMode = saved.startMode ?? 'QLOITER';
    this.showMarker = saved.showMarker ?? true;
    this.canvas = document.getElementById('view');
    this.world = new World(this.canvas);
    this.model = new AircraftModel(this.world.scene);
    this.cam = new CameraRig(window.innerWidth / window.innerHeight);
    if (saved.cam) Object.assign(this.cam, saved.cam);
    this.hud = new Hud(document.getElementById('hud'));
    this.input = new InputManager();
    this.pauseReasons = new Set();
    this.timeScale = 1;
    this.mapScale = 6.2;
    this.reset();
    this.bindUi();
    this.onResize();
    window.addEventListener('resize', () => this.onResize());
    this.last = performance.now();
    requestAnimationFrame((t) => this.frame(t));
  }

  get paused() {
    return this.pauseReasons.size > 0;
  }

  setPaused(on, reason = 'user') {
    if (on) this.pauseReasons.add(reason);
    else this.pauseReasons.delete(reason);
  }

  reset() {
    this.sim.reset({ yaw: (this.startYaw * Math.PI) / 180, mode: this.startMode });
    this.hud.swimmer = this.sim.scenario.swimmer;
    this.input.keyArmed = false;
    this.input.keyThrottle = 0;
  }

  resetDefaults() {
    try {
      localStorage.removeItem('cfs.settings.v1');
    } catch (e) {
      /* ignore */
    }
    location.reload();
  }

  persist() {
    saveSettings({
      env: this.sim.envSettings,
      fcParams: this.sim.fcParams,
      scenario: this.sim.scenarioSettings,
      startYaw: this.startYaw,
      startMode: this.startMode,
      showMarker: this.showMarker,
      cam: { losFov: this.cam.losFov, autoZoom: this.cam.autoZoom, fpvTilt: this.cam.fpvTilt, fpvFov: this.cam.fpvFov, downFov: this.cam.downFov, downStabilized: this.cam.downStabilized, pilotNed: this.cam.pilotNed, mode: this.cam.mode },
    });
    this.hud.swimmer = this.sim.scenario.swimmer;
  }

  bindUi() {
    const bar = document.getElementById('menu');
    const btn = (label, title, fn) => {
      const b = document.createElement('button');
      b.className = 'btn';
      b.textContent = label;
      b.title = title;
      b.addEventListener('click', (e) => {
        fn();
        b.blur();
        e.stopPropagation();
      });
      bar.appendChild(b);
    };
    btn('Controller', 'Controller setup / calibration (G)', () => openController(this));
    btn('Settings', 'Wind, scenario, parameters, display (Tab)', () => openSettings(this));
    btn('Camera', 'Cycle camera (V)', () => this.cam.cycle());
    btn('Reset', 'Reset to the launch pad (R)', () => this.reset());
    btn('Help', 'Controls and tips (F1)', () => openHelp(this));
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.cam.zoom(e.deltaY < 0 ? 1 : -1);
    }, { passive: false });
    let drag = null;
    this.canvas.addEventListener('mousedown', (e) => (drag = e.clientX));
    window.addEventListener('mouseup', () => (drag = null));
    window.addEventListener('mousemove', (e) => {
      if (drag != null && this.cam.mode === 'chase') {
        this.cam.orbit += (e.clientX - drag) * 0.01;
        drag = e.clientX;
      }
    });
    this.canvas.addEventListener('dblclick', () => (this.cam.orbit = 0));
    window.addEventListener('keydown', (e) => {
      if (e.code === 'Tab' || e.code === 'F1') e.preventDefault();
    });
  }

  handleUiKeys() {
    for (const code of this.input.takeUiActions()) {
      switch (code) {
        case 'KeyV': this.cam.cycle(); break;
        case 'KeyR': this.reset(); break;
        case 'KeyP':
          if (this.pauseReasons.has('user')) this.setPaused(false, 'user');
          else this.setPaused(true, 'user');
          break;
        case 'KeyO': this.showMarker = !this.showMarker; break;
        case 'KeyK': this.cam.downStabilized = !this.cam.downStabilized; this.persist(); break;
        case 'KeyT': this.timeScale = this.timeScale === 1 ? 2 : this.timeScale === 2 ? 0.5 : 1; break;
        case 'Equal': case 'NumpadAdd': this.cam.zoom(1); break;
        case 'Minus': case 'NumpadSubtract': this.cam.zoom(-1); break;
        case 'KeyG': openController(this); break;
        case 'Tab': openSettings(this); break;
        case 'F1': case 'Slash': openHelp(this); break;
        default: break;
      }
    }
  }

  onResize() {
    const w = window.innerWidth, h = window.innerHeight;
    this.world.resize(w, h);
    this.cam.camera.aspect = w / h;
    this.cam.camera.updateProjectionMatrix();
  }

  frame(t) {
    const dt = Math.min((t - this.last) / 1000, 0.1);
    this.last = t;
    this.handleUiKeys();
    const pilot = this.input.poll(dt, this.sim.fc.mode);
    if (!this.paused) {
      this.sim.timeScale = this.timeScale;
      if (this.sim.pendingEvents) {
        pilot.events = this.sim.pendingEvents.concat(pilot.events);
        this.sim.pendingEvents = null;
      }
      this.sim.advance(dt, pilot);
    } else if (pilot.events.length) {
      // keep arm/mode events that happen while a menu is open
      this.sim.pendingEvents = (this.sim.pendingEvents || []).concat(pilot.events);
    }
    this.world.swimmerMarker.visible = this.showMarker && this.cam.mode !== 'down';
    this.world.update(this.paused ? 0 : dt * this.timeScale, this.sim);
    this.model.fpv = this.cam.mode === 'fpv';
    this.model.update(this.paused ? 0 : dt * this.timeScale, this.sim);
    this.cam.update(dt, this.model, this.sim);
    const target = this.model.position(new THREE.Vector3());
    this.world.followShadow(target);
    this.world.renderer.render(this.world.scene, this.cam.camera);

    // markers
    const w = window.innerWidth, h = window.innerHeight;
    let marker = null, swMarker = null;
    const camPos = this.cam.camera.position;
    if (this.cam.mode !== 'fpv' || true) {
      const p = this.cam.project(target, w, h);
      const dist = camPos.distanceTo(target);
      if (p && this.cam.mode !== 'fpv' && this.cam.mode !== 'down' && dist > 15) marker = { ...p, dist };
      const [sn, se] = this.sim.scenario.swimmer;
      const sp = ned2three([sn, se, 0]);
      const ps = this.cam.project(sp, w, h);
      if (ps && ps.x > 0 && ps.x < w && ps.y > 0 && ps.y < h) swMarker = { ...ps, dist: camPos.distanceTo(sp) };
    }
    // predicted landing point of the flotation device if released now
    let dropMarker = null;
    const imp = this.sim.predictPayloadImpact();
    if (imp) {
      const ip = this.cam.project(ned2three(imp.pos), w, h);
      if (ip && ip.x > 0 && ip.x < w && ip.y > 0 && ip.y < h) {
        const [sn2, se2] = this.sim.scenario.swimmer;
        dropMarker = { ...ip, miss: Math.hypot(imp.pos[0] - sn2, imp.pos[1] - se2), t: imp.time };
      }
    }
    const tel = this.sim.telemetry();
    const src = this.input.source;
    const inputLabel = src === 'radio'
      ? `RADIO: ${(this.input.gamepad && this.input.gamepad.id || '').slice(0, 40)}`
      : this.input.gamepadError ? 'KEYBOARD (controllers blocked on this page — open the local HTML file)' : 'KEYBOARD (plug in the radio: USB Joystick mode)';
    this.hud.update(tel, {
      pilot: this.input.lastPilot,
      cameraMode: this.cam.mode,
      fov: this.cam.camera.fov,
      marker,
      swMarker,
      dropMarker,
      downStabilized: this.cam.downStabilized,
      showMarker: this.showMarker,
      paused: this.paused,
      fbwMin: this.sim.fc.p.ARSPD_FBW_MIN,
      inputLabel,
      timeScale: this.timeScale,
      mapScale: this.mapScale,
    });
    requestAnimationFrame((tt) => this.frame(tt));
  }
}

window.addEventListener('DOMContentLoaded', () => {
  try {
    window.app = new App();
  } catch (e) {
    const d = document.createElement('pre');
    d.className = 'fatal';
    d.textContent = 'Failed to start the simulator:\n' + (e && e.stack ? e.stack : e);
    document.body.appendChild(d);
    throw e;
  }
});
