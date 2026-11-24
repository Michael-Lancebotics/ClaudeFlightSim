// Camera views: line-of-sight from the pilot's spot on the beach (default),
// chase camera, and FPV from the nose.

import * as THREE from 'three';
import { groundHeight } from '../core/terrain.js';

export const CAMERA_MODES = ['los', 'chase', 'fpv'];
export const CAMERA_LABELS = { los: 'LINE OF SIGHT (pilot on the beach)', chase: 'CHASE', fpv: 'FPV (nose camera)' };

// body (FRD) -> three camera frame (camera looks down its -Z, +Y up)
const B_CAM = new THREE.Matrix4().set(
  0, 0, -1, 0,
  1, 0, 0, 0,
  0, -1, 0, 0,
  0, 0, 0, 1,
);

export class CameraRig {
  constructor(aspect) {
    this.camera = new THREE.PerspectiveCamera(55, aspect, 0.05, 20000);
    this.mode = 'los';
    this.losFov = 55;
    this.autoZoom = false;
    this.fpvTilt = 0; // deg, positive = camera tilted up
    this.fpvFov = 100;
    this.chaseDist = 7;
    this.chaseHeight = 2.2;
    this.orbit = 0; // chase-camera orbit offset (rad)
    this.pilotNed = [-10, -8];
    this.chasePos = null;
    this.tmp = new THREE.Vector3();
  }

  setMode(m) {
    this.mode = m;
    this.chasePos = null;
  }

  cycle() {
    const i = CAMERA_MODES.indexOf(this.mode);
    this.setMode(CAMERA_MODES[(i + 1) % CAMERA_MODES.length]);
  }

  zoom(dir) {
    if (this.mode === 'los') this.losFov = THREE.MathUtils.clamp(this.losFov * (dir > 0 ? 0.85 : 1 / 0.85), 3, 75);
    else if (this.mode === 'chase') this.chaseDist = THREE.MathUtils.clamp(this.chaseDist * (dir > 0 ? 0.85 : 1 / 0.85), 2.5, 60);
    else this.fpvTilt = THREE.MathUtils.clamp(this.fpvTilt + (dir > 0 ? 5 : -5), -30, 45);
  }

  /** Pilot eye position in three.js coordinates. */
  pilotEye() {
    const [n, e] = this.pilotNed;
    return new THREE.Vector3(e, groundHeight(n, e) + 1.7, -n);
  }

  update(dt, model, sim) {
    const cam = this.camera;
    const target = model.position(this.tmp);
    if (this.mode === 'los') {
      const eye = this.pilotEye();
      cam.position.copy(eye);
      cam.up.set(0, 1, 0);
      cam.lookAt(target);
      let fov = this.losFov;
      if (this.autoZoom) {
        // keep the 1.34 m wingspan at least ~4 % of the view height
        const d = eye.distanceTo(target);
        const need = THREE.MathUtils.radToDeg(2 * Math.atan((1.34 / 0.04) / (2 * d)));
        fov = THREE.MathUtils.clamp(Math.min(fov, need), 3, 75);
      }
      if (Math.abs(cam.fov - fov) > 0.01) {
        cam.fov = fov;
        cam.updateProjectionMatrix();
      }
    } else if (this.mode === 'chase') {
      // follow behind along the heading (not rolling with the aircraft)
      const v = sim.vehicle;
      const e = sim.est;
      const yaw = e.yaw + this.orbit;
      const back = new THREE.Vector3(-Math.sin(yaw), 0, Math.cos(yaw)); // three: behind = -north_body
      const want = target.clone().addScaledVector(back, this.chaseDist);
      want.y += this.chaseHeight;
      const ground = groundHeight(-want.z, want.x);
      want.y = Math.max(want.y, Math.max(ground, 0) + 0.5);
      if (!this.chasePos) this.chasePos = want.clone();
      this.chasePos.lerp(want, 1 - Math.exp(-dt * 4));
      cam.position.copy(this.chasePos);
      cam.up.set(0, 1, 0);
      const look = target.clone();
      look.y += 0.3;
      cam.lookAt(look);
      if (cam.fov !== 60) {
        cam.fov = 60;
        cam.updateProjectionMatrix();
      }
      void v;
    } else {
      // FPV: camera rigidly on the nose
      const m = new THREE.Matrix4().multiplyMatrices(model.root.matrix, B_CAM);
      const tilt = new THREE.Matrix4().makeRotationX(THREE.MathUtils.degToRad(this.fpvTilt));
      m.multiply(tilt);
      const nose = new THREE.Vector3(0.6, 0, -0.02).applyMatrix4(model.root.matrix);
      cam.position.copy(nose);
      cam.quaternion.setFromRotationMatrix(m);
      if (cam.fov !== this.fpvFov) {
        cam.fov = this.fpvFov;
        cam.updateProjectionMatrix();
      }
    }
  }

  /** Project a world point to screen pixels (or null if behind the camera). */
  project(p, w, h) {
    const v = p.clone().project(this.camera);
    if (v.z > 1) return null;
    return { x: (v.x * 0.5 + 0.5) * w, y: (-v.y * 0.5 + 0.5) * h, behind: v.z > 1 };
  }
}
