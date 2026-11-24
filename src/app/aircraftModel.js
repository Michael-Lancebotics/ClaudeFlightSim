// 3-D model of the quadplane, built in body axes (x forward, y right, z down)
// from the owner's geometry: 1.34 m wing, booms 0.57 m apart, lift rotors at
// x = +-0.55 m, inverted-V tail joining the boom ends, pusher at the rear.

import * as THREE from 'three';
import { AIRCRAFT } from '../core/params.js';

const C_NED2THREE = new THREE.Matrix4().set(
  0, 1, 0, 0,
  0, 0, -1, 0,
  -1, 0, 0, 0,
  0, 0, 0, 1,
);

function mat(color, opts = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.05, ...opts });
}

/** Box with dimensions along body x, y, z. */
function box(lx, ly, lz, material) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(lx, ly, lz), material);
  m.castShadow = true;
  return m;
}

/** Cylinder along the body x axis. */
function cylX(r1, r2, len, material, seg = 20) {
  const g = new THREE.CylinderGeometry(r2, r1, len, seg);
  g.rotateZ(-Math.PI / 2); // +y -> +x  (r1 at -x end, r2 at +x end)
  const m = new THREE.Mesh(g, material);
  m.castShadow = true;
  return m;
}

/** Cylinder along the body z axis. */
function cylZ(r, len, material, seg = 16) {
  const g = new THREE.CylinderGeometry(r, r, len, seg);
  g.rotateX(Math.PI / 2);
  const m = new THREE.Mesh(g, material);
  m.castShadow = true;
  return m;
}

function glowTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grd.addColorStop(0, 'rgba(255,255,255,1)');
  grd.addColorStop(0.25, 'rgba(255,255,255,0.8)');
  grd.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grd;
  g.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
}

export class AircraftModel {
  constructor(scene) {
    const ac = AIRCRAFT;
    this.root = new THREE.Group();
    this.root.matrixAutoUpdate = false;
    scene.add(this.root);
    const body = this.root;
    const white = mat(0xf4f4f2);
    const orange = mat(0xff6a13);
    const dark = mat(0x2b2f36);
    const black = mat(0x111111, { roughness: 0.4 });
    const zc = ac.cgAboveCentreline; // fuselage centreline is below the CG

    // fuselage: nose cone, body, tail cone
    const L = ac.fuselage.length, r = ac.fuselage.diameter / 2;
    const xNose = ac.noseToCg, xTail = ac.noseToCg - L;
    const main = cylX(r, r, L * 0.62, white);
    main.position.set(xNose - 0.17 - (L * 0.62) / 2, 0, zc);
    body.add(main);
    const nose = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14, 0, Math.PI * 2, 0, Math.PI / 2), white);
    nose.geometry.rotateZ(-Math.PI / 2);
    nose.scale.set(2.4, 1, 1);
    nose.position.set(xNose - 0.17 + 0.0, 0, zc);
    nose.castShadow = true;
    body.add(nose);
    const tailCone = cylX(r * 0.45, r, L - 0.17 - L * 0.62, white);
    tailCone.position.set(xTail + (L - 0.17 - L * 0.62) / 2, 0, zc);
    body.add(tailCone);
    // orange rescue band and canopy
    const band = cylX(r * 1.02, r * 1.02, 0.06, orange);
    band.position.set(0.25, 0, zc);
    body.add(band);
    const canopy = new THREE.Mesh(new THREE.SphereGeometry(0.055, 16, 10), mat(0x223344, { roughness: 0.1, metalness: 0.4 }));
    canopy.scale.set(2.2, 0.9, 0.7);
    canopy.position.set(0.33, 0, zc - r * 0.8);
    body.add(canopy);

    // wing
    const w = ac.wing;
    const wingX = w.leX - w.chord / 2;
    const ailChord = 0.08;
    const wingMain = box(w.chord - ailChord, w.span, 0.03, white);
    wingMain.position.set(wingX + ailChord / 2, 0, w.z);
    body.add(wingMain);
    // trailing edge between the ailerons and outboard of them
    const yIn = w.aileronY[0], yOut = w.aileronY[1];
    const teInner = box(ailChord, 2 * yIn, 0.02, white);
    teInner.position.set(w.leX - w.chord + ailChord / 2, 0, w.z);
    body.add(teInner);
    for (const s of [-1, 1]) {
      const tip = box(ailChord, w.span / 2 - yOut, 0.02, orange);
      tip.position.set(w.leX - w.chord + ailChord / 2, s * (yOut + (w.span / 2 - yOut) / 2), w.z);
      body.add(tip);
      const tipCap = box(w.chord - ailChord, 0.12, 0.031, orange);
      tipCap.position.set(wingX + ailChord / 2, s * (w.span / 2 - 0.06), w.z);
      body.add(tipCap);
    }
    // ailerons (hinge pivots)
    this.ailerons = {};
    for (const [key, s] of [['L', -1], ['R', 1]]) {
      const pivot = new THREE.Group();
      pivot.position.set(w.leX - w.chord + ailChord, s * (yIn + yOut) / 2, w.z);
      const surf = box(ailChord, yOut - yIn, 0.018, mat(0xe8e8e4));
      surf.position.set(-ailChord / 2, 0, 0);
      pivot.add(surf);
      body.add(pivot);
      this.ailerons[key] = pivot;
    }

    // booms, motors and lift rotors
    const by = ac.boomY, bz = -0.02;
    const tailTE = -ac.tail.arm - 0.25 * ac.tail.chord - 0.75 * ac.tail.chord + 0.01;
    for (const s of [-1, 1]) {
      const len = 0.72 - tailTE;
      const boom = cylX(0.016, 0.016, len, dark, 12);
      boom.position.set((0.72 + tailTE) / 2, s * by, bz);
      body.add(boom);
    }
    this.props = [];
    this.discs = [];
    const propMat = mat(0x1a1a1a, { roughness: 0.3 });
    const discMat = new THREE.MeshBasicMaterial({ color: 0x222222, transparent: true, opacity: 0.0, depthWrite: false, side: THREE.DoubleSide });
    for (const rot of ac.liftRotors) {
      const [x, y] = rot.pos;
      const motor = cylZ(0.022, 0.035, black);
      motor.position.set(x, y, bz - 0.035);
      body.add(motor);
      const mount = cylZ(0.012, 0.03, dark);
      mount.position.set(x, y, bz - 0.012);
      body.add(mount);
      const hub = new THREE.Group();
      hub.position.set(x, y, bz - 0.06);
      const blade = box(0.381, 0.025, 0.004, propMat);
      hub.add(blade);
      body.add(hub);
      const disc = new THREE.Mesh(new THREE.CircleGeometry(0.1905, 32), discMat.clone());
      disc.position.set(x, y, bz - 0.062);
      body.add(disc);
      this.props.push({ hub, spin: rot.spin });
      this.discs.push(disc);
    }

    // inverted-V tail: apex above the centreline, panels down to the boom ends
    const t = ac.tail;
    const half = t.area / 2;
    const panelSpan = half / t.chord;
    const ga = t.panelAngle;
    const ruddChord = 0.35 * t.chord;
    const tailLE = -t.arm + 0.25 * t.chord;
    this.ruddervators = {};
    for (const [key, s] of [['L', -1], ['R', 1]]) {
      const g = new THREE.Group();
      // panel frame: local y along the span (apex -> boom end), local x chord
      g.position.set(0, s * (panelSpan / 2) * Math.cos(ga), bz - (panelSpan / 2) * Math.sin(ga));
      g.rotation.x = s * ga;
      const fixed = box(t.chord - ruddChord, panelSpan, 0.012, white);
      fixed.position.set(tailLE - (t.chord - ruddChord) / 2, 0, 0);
      g.add(fixed);
      const pivot = new THREE.Group();
      pivot.position.set(tailLE - (t.chord - ruddChord), 0, 0);
      const surf = box(ruddChord, panelSpan * 0.95, 0.01, orange);
      surf.position.set(-ruddChord / 2, 0, 0);
      pivot.add(surf);
      g.add(pivot);
      body.add(g);
      this.ruddervators[key] = pivot;
    }

    // pusher
    const pp = ac.pusher;
    const pmot = cylX(0.03, 0.025, 0.06, black);
    pmot.position.set(xTail - 0.01, 0, zc);
    body.add(pmot);
    this.pusherHub = new THREE.Group();
    this.pusherHub.position.set(pp.pos[0], 0, zc);
    const pblade = box(0.006, 0.381, 0.03, propMat);
    this.pusherHub.add(pblade);
    body.add(this.pusherHub);
    const pdisc = new THREE.Mesh(new THREE.CircleGeometry(0.1905, 32), discMat.clone());
    pdisc.geometry.rotateY(Math.PI / 2);
    pdisc.position.set(pp.pos[0] - 0.003, 0, zc);
    body.add(pdisc);
    this.pusherDisc = pdisc;

    // skids
    for (const s of [-1, 1]) {
      const skid = cylX(0.008, 0.008, 1.0, dark, 8);
      skid.position.set(0, s * by, 0.28);
      body.add(skid);
      for (const x of [0.45, -0.45]) {
        const strut = cylZ(0.008, 0.34, dark, 8);
        strut.position.set(x, s * by, 0.11);
        body.add(strut);
      }
    }

    // payload: orange flotation ring under the fuselage
    this.payload = new THREE.Mesh(new THREE.TorusGeometry(0.16, 0.05, 10, 24), orange);
    this.payload.position.set(0.0, 0, 0.12);
    this.payload.castShadow = true;
    body.add(this.payload);

    // navigation lights (red left, green right, white tail) + strobes
    const tex = glowTexture();
    const light = (color, x, y, z, size) => {
      const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, color, blending: THREE.AdditiveBlending, depthWrite: false, transparent: true }));
      sp.position.set(x, y, z);
      sp.scale.set(size, size, size);
      body.add(sp);
      return sp;
    };
    light(0xff2020, wingX, -w.span / 2, w.z, 0.18);
    light(0x20ff40, wingX, w.span / 2, w.z, 0.18);
    light(0xffffff, xTail, 0, zc, 0.14);
    this.strobes = [light(0xffffff, wingX - 0.05, -w.span / 2, w.z, 0.5), light(0xffffff, wingX - 0.05, w.span / 2, w.z, 0.5)];
    this.time = 0;
    this._m = new THREE.Matrix4();
  }

  /** Update pose and animated parts from the simulation. */
  update(dt, sim) {
    this.time += dt;
    const v = sim.vehicle;
    const q = v.quat;
    const p = v.pos;
    // body->NED rotation, then NED->three
    const [w, x, y, z] = q;
    const R = new THREE.Matrix4().set(
      1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), p[0],
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), p[1],
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y), p[2],
      0, 0, 0, 1,
    );
    this.root.matrix.multiplyMatrices(C_NED2THREE, R);
    this.root.matrixWorldNeedsUpdate = true;
    // rotors
    const rs = v.rotorSpeeds;
    for (let i = 0; i < 4; i++) {
      const pr = this.props[i];
      // visual spin (stroboscopic): slow it down so it reads as rotation
      pr.hub.rotation.z += -pr.spin * Math.min(rs[i], 60) * dt;
      const blur = Math.min(rs[i] / 400, 1);
      this.discs[i].material.opacity = (this.fpv ? 0.1 : 0.35) * blur;
      pr.hub.visible = blur < 0.9;
    }
    const ps = v.pusherSpeed;
    this.pusherHub.rotation.x += Math.min(ps, 60) * dt;
    const pblur = Math.min(ps / 400, 1);
    this.pusherDisc.material.opacity = 0.35 * pblur;
    this.pusherHub.visible = pblur < 0.9;
    // control surfaces (TE down positive -> rotate about +y)
    const sv = v.servo;
    this.ailerons.L.rotation.y = sv.ailL;
    this.ailerons.R.rotation.y = sv.ailR;
    this.ruddervators.L.rotation.y = sv.rvL;
    this.ruddervators.R.rotation.y = sv.rvR;
    this.payload.visible = v.payloadAttached;
    // strobes: double flash every second
    const ph = this.time % 1.0;
    const on = ph < 0.05 || (ph > 0.12 && ph < 0.17);
    for (const s of this.strobes) s.visible = on && sim.fc.armed;
  }

  /** Aircraft position in three.js coordinates. */
  position(out = new THREE.Vector3()) {
    return out.setFromMatrixPosition(this.root.matrix);
  }
}

export { C_NED2THREE };
