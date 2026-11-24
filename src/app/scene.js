// Three.js world: sky, ocean, beach, props, swimmer, windsock.
// Coordinates: physics uses NED (x north, y east, z down); three.js uses
// x = east, y = up, z = south.

import * as THREE from 'three';
import { Sky } from 'three/examples/jsm/objects/Sky.js';
import { groundHeight, shoreNorth, TERRAIN } from '../core/terrain.js';

export const ned2three = (p) => new THREE.Vector3(p[1], -p[2], -p[0]);

const WATER_VERT = /* glsl */ `
  varying vec3 vWorld;
  void main() {
    vec4 w = modelMatrix * vec4(position, 1.0);
    vWorld = w.xyz;
    gl_Position = projectionMatrix * viewMatrix * w;
  }
`;

const WATER_FRAG = /* glsl */ `
  uniform float uTime;
  uniform vec3 uSunDir;
  uniform vec3 uSky;
  uniform vec3 uHorizon;
  uniform vec3 uDeep;
  uniform vec3 uShallow;
  uniform vec2 uWind;
  varying vec3 vWorld;
  float shoreNorth(float e) { return 40.0 + 8.0 * sin(e / 120.0) + 3.0 * sin(e / 37.0 + 1.3); }
  vec3 waveNormal(vec2 p, float dist) {
    vec2 g = vec2(0.0);
    float fade = clamp(1.0 - dist / 1500.0, 0.15, 1.0);
    vec2 d1 = normalize(uWind + vec2(0.001));
    vec2 d2 = normalize(vec2(d1.y, -d1.x) * 0.6 + d1);
    vec2 d3 = normalize(vec2(-d1.y, d1.x) * 0.5 + d1);
    float w1 = 0.22, w2 = 0.55, w3 = 1.3, w4 = 3.1;
    g += d1 * cos(dot(p, d1) * w1 + uTime * 0.9) * 0.08;
    g += d2 * cos(dot(p, d2) * w2 + uTime * 1.4) * 0.05;
    g += d3 * cos(dot(p, d3) * w3 + uTime * 2.1) * 0.035 * fade;
    g += vec2(0.7, 0.3) * cos(dot(p, vec2(0.7, 0.3)) * w4 + uTime * 3.3) * 0.02 * fade;
    return normalize(vec3(-g.x, 1.0, -g.y));
  }
  void main() {
    vec3 viewVec = cameraPosition - vWorld;
    float dist = length(viewVec);
    vec3 V = viewVec / dist;
    vec2 p = vWorld.xz;
    vec3 N = waveNormal(p, dist);
    float fres = 0.02 + 0.98 * pow(1.0 - max(dot(N, V), 0.0), 5.0);
    vec3 R = reflect(-V, N);
    vec3 skyCol = mix(uHorizon, uSky, clamp(R.y * 2.0, 0.0, 1.0));
    float north = -vWorld.z;
    float east = vWorld.x;
    float toShore = north - shoreNorth(east);
    vec3 base = mix(uShallow, uDeep, clamp(toShore / 60.0, 0.0, 1.0));
    vec3 col = mix(base, skyCol, fres);
    float spec = pow(max(dot(R, uSunDir), 0.0), 180.0) * 2.5;
    col += vec3(1.0, 0.95, 0.85) * spec;
    float foam = smoothstep(4.0, 0.0, toShore) * (0.55 + 0.45 * sin(toShore * 2.5 - uTime * 1.6 + east * 0.05));
    col = mix(col, vec3(0.93, 0.95, 0.95), clamp(foam, 0.0, 0.85));
    float haze = 1.0 - exp(-dist / 3500.0);
    col = mix(col, uHorizon, haze * 0.7);
    gl_FragColor = vec4(col, 1.0);
    #include <colorspace_fragment>
  }
`;

export class World {
  constructor(canvas) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 0.55;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(0xbfd4e6, 1500, 6000);
    this.sunDir = new THREE.Vector3();
    this.buildSky();
    this.buildLights();
    this.buildOcean();
    this.buildBeach();
    this.buildProps();
    this.buildSwimmer();
    this.buildWindsock();
    this.time = 0;
  }

  buildSky() {
    const sky = new Sky();
    sky.scale.setScalar(20000);
    const u = sky.material.uniforms;
    u.turbidity.value = 4;
    u.rayleigh.value = 1.2;
    u.mieCoefficient.value = 0.004;
    u.mieDirectionalG.value = 0.8;
    const phi = THREE.MathUtils.degToRad(90 - 38); // elevation 38 deg
    const theta = THREE.MathUtils.degToRad(330); // sun to the south-west, behind the pilot
    this.sunDir.setFromSphericalCoords(1, phi, theta);
    u.sunPosition.value.copy(this.sunDir);
    this.scene.add(sky);
    this.sky = sky;
  }

  buildLights() {
    const hemi = new THREE.HemisphereLight(0xcfe6ff, 0xd8c8a0, 1.4);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xfff2dd, 3.2);
    sun.position.copy(this.sunDir).multiplyScalar(100);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    const s = 25;
    Object.assign(sun.shadow.camera, { left: -s, right: s, top: s, bottom: -s, near: 1, far: 400 });
    sun.shadow.bias = -0.0005;
    this.scene.add(sun);
    this.scene.add(sun.target);
    this.sun = sun;
  }

  buildOcean() {
    const geo = new THREE.PlaneGeometry(16000, 16000, 1, 1);
    geo.rotateX(-Math.PI / 2);
    this.waterUniforms = {
      uTime: { value: 0 },
      uSunDir: { value: this.sunDir.clone() },
      uSky: { value: new THREE.Color(0x3d78b8) },
      uHorizon: { value: new THREE.Color(0xbfd4e6) },
      uDeep: { value: new THREE.Color(0x0b3d5c) },
      uShallow: { value: new THREE.Color(0x2e8f9a) },
      uWind: { value: new THREE.Vector2(0, 1) },
    };
    const mat = new THREE.ShaderMaterial({ vertexShader: WATER_VERT, fragmentShader: WATER_FRAG, uniforms: this.waterUniforms });
    const water = new THREE.Mesh(geo, mat);
    water.position.y = 0;
    water.position.z = -7000; // centred out to sea
    this.scene.add(water);
    this.water = water;
  }

  buildBeach() {
    // terrain grid: east -1500..1500, north -700..60 (extends under the water)
    const W = 3000, D = 760, nx = 300, nz = 152;
    const geo = new THREE.PlaneGeometry(W, D, nx, nz);
    geo.rotateX(-Math.PI / 2);
    const pos = geo.attributes.position;
    const colors = new Float32Array(pos.count * 3);
    const wet = new THREE.Color(0xb59f73), dry = new THREE.Color(0xe9d7a9), grass = new THREE.Color(0x8f9a5a);
    const c = new THREE.Color();
    for (let i = 0; i < pos.count; i++) {
      const east = pos.getX(i);
      const north = -(pos.getZ(i) - (-D / 2 + 60)) ; // remap below
      pos.setZ(i, -north);
      const h = groundHeight(north, east);
      pos.setY(i, h);
      const inland = shoreNorth(east) - north;
      if (inland < 6) c.copy(wet);
      else if (inland < 120) c.copy(wet).lerp(dry, Math.min(1, (inland - 6) / 15));
      else c.copy(dry).lerp(grass, Math.min(1, (inland - 120) / 60));
      const n = (Math.sin(east * 0.37) * Math.sin(north * 0.29) + Math.sin(east * 1.7 + north * 1.1)) * 0.025;
      c.offsetHSL(0, 0, n);
      colors[i * 3] = c.r;
      colors[i * 3 + 1] = c.g;
      colors[i * 3 + 2] = c.b;
    }
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.computeVertexNormals();
    const mat = new THREE.MeshLambertMaterial({ vertexColors: true });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true;
    this.scene.add(mesh);
  }

  addBox(w, h, d, color, x, y, z, parent = this.scene, cast = true) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), new THREE.MeshLambertMaterial({ color }));
    m.position.set(x, y, z);
    m.castShadow = cast;
    m.receiveShadow = true;
    parent.add(m);
    return m;
  }

  buildProps() {
    const padH = TERRAIN.padHeight;
    // launch pad with H marking
    const pad = new THREE.Mesh(new THREE.CylinderGeometry(2.5, 2.5, 0.05, 40), new THREE.MeshLambertMaterial({ color: 0x3a3f45 }));
    pad.position.set(0, padH + 0.02, 0);
    pad.receiveShadow = true;
    this.scene.add(pad);
    const ring = new THREE.Mesh(new THREE.RingGeometry(2.0, 2.2, 40), new THREE.MeshBasicMaterial({ color: 0xffd23f }));
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(0, padH + 0.05, 0);
    this.scene.add(ring);
    const hm = new THREE.MeshBasicMaterial({ color: 0xffffff });
    for (const [w, d, x, z] of [[0.25, 1.6, -0.5, 0], [0.25, 1.6, 0.5, 0], [0.8, 0.25, 0, 0]]) {
      const b = new THREE.Mesh(new THREE.PlaneGeometry(w, d), hm);
      b.rotation.x = -Math.PI / 2;
      b.position.set(x, padH + 0.06, z);
      this.scene.add(b);
    }
    // lifeguard tower 25 m east of the pad
    const tower = new THREE.Group();
    const wood = 0xd9d2c0;
    for (const [x, z] of [[-1, -1], [1, -1], [-1, 1], [1, 1]]) this.addBox(0.15, 3, 0.15, wood, x, 1.5, z, tower);
    this.addBox(2.6, 0.15, 2.6, wood, 0, 3, 0, tower);
    this.addBox(2.4, 1.6, 2.4, 0xf4f1e8, 0, 3.9, 0, tower);
    const roof = new THREE.Mesh(new THREE.ConeGeometry(2.2, 0.9, 4), new THREE.MeshLambertMaterial({ color: 0xc8262c }));
    roof.position.set(0, 5.15, 0);
    roof.rotation.y = Math.PI / 4;
    roof.castShadow = true;
    tower.add(roof);
    tower.position.set(25, groundHeight(-2, 25), 2);
    this.scene.add(tower);
    // beach umbrellas and towels
    const cols = [0xe63946, 0x1d3557, 0xffb703, 0x2a9d8f, 0xf77f00];
    for (let i = 0; i < 14; i++) {
      const e = -320 + i * 47 + ((i * 37) % 17);
      if (Math.abs(e) < 40) continue;
      const n = 8 + ((i * 53) % 25);
      const h = groundHeight(n, e);
      const g = new THREE.Group();
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 2.2), new THREE.MeshLambertMaterial({ color: 0xeeeeee }));
      pole.position.y = 1.1;
      g.add(pole);
      const top = new THREE.Mesh(new THREE.ConeGeometry(1.2, 0.5, 10, 1, true), new THREE.MeshLambertMaterial({ color: cols[i % cols.length], side: THREE.DoubleSide }));
      top.position.y = 2.1;
      top.castShadow = true;
      g.add(top);
      g.position.set(e, h, -n);
      this.scene.add(g);
      this.addBox(0.8, 0.02, 1.8, cols[(i + 2) % cols.length], e + 1.2, h + 0.01, -n + 0.5, this.scene, false);
    }
    // swim-zone buoys out to sea
    const buoyMat = new THREE.MeshLambertMaterial({ color: 0xff7b00 });
    for (let i = 0; i < 12; i++) {
      const b = new THREE.Mesh(new THREE.SphereGeometry(0.35, 12, 8), buoyMat);
      b.position.set(-120 + i * 22, 0.1, -(shoreNorth(-120 + i * 22) + 90));
      this.scene.add(b);
    }
    // a few distant boats
    for (const [e, n] of [[-900, 2300], [1300, 3400], [300, 4800]]) {
      const g = new THREE.Group();
      this.addBox(6, 1.2, 2.2, 0xffffff, 0, 0.6, 0, g, false);
      this.addBox(2.2, 1.2, 1.8, 0xdddddd, -0.8, 1.6, 0, g, false);
      g.position.set(e, 0, -n);
      g.rotation.y = e * 0.001;
      this.scene.add(g);
    }
  }

  buildSwimmer() {
    const g = new THREE.Group();
    const skin = new THREE.MeshLambertMaterial({ color: 0xd8a27a });
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.13, 16, 12), skin);
    head.position.y = 0.12;
    g.add(head);
    const cap = new THREE.Mesh(new THREE.SphereGeometry(0.135, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2), new THREE.MeshLambertMaterial({ color: 0xffd400 }));
    cap.position.y = 0.13;
    g.add(cap);
    for (const s of [-1, 1]) {
      const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.6), skin);
      arm.position.set(s * 0.3, 0.25, 0);
      arm.rotation.z = s * 0.6;
      g.add(arm);
      this[s < 0 ? 'armL' : 'armR'] = arm;
    }
    // splash ring
    const ripple = new THREE.Mesh(new THREE.RingGeometry(0.3, 0.45, 24), new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.6 }));
    ripple.rotation.x = -Math.PI / 2;
    ripple.position.y = 0.02;
    g.add(ripple);
    this.ripple = ripple;
    // tall marker pole (toggleable target marker)
    const marker = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 30), new THREE.MeshBasicMaterial({ color: 0xff3355, transparent: true, opacity: 0.35 }));
    marker.position.y = 15;
    g.add(marker);
    this.swimmerMarker = marker;
    this.swimmer = g;
    this.scene.add(g);
    // payload (dropped flotation device) - visual only, positioned from the sim
    const payload = new THREE.Mesh(new THREE.TorusGeometry(0.22, 0.07, 10, 24), new THREE.MeshLambertMaterial({ color: 0xff6a00 }));
    payload.castShadow = true;
    payload.visible = false;
    this.scene.add(payload);
    this.payloadMesh = payload;
  }

  buildWindsock() {
    const g = new THREE.Group();
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.05, 5), new THREE.MeshLambertMaterial({ color: 0xcccccc }));
    pole.position.y = 2.5;
    pole.castShadow = true;
    g.add(pole);
    const sockGeo = new THREE.CylinderGeometry(0.25, 0.12, 1.6, 12, 1, true);
    sockGeo.rotateZ(Math.PI / 2);
    sockGeo.translate(0.8, 0, 0);
    const sock = new THREE.Mesh(sockGeo, new THREE.MeshLambertMaterial({ color: 0xff5a1f, side: THREE.DoubleSide }));
    sock.position.y = 4.9;
    sock.castShadow = true;
    g.add(sock);
    g.position.set(9, groundHeight(-3, 9), 3); // right of the pad, clear of the pilot's view
    this.scene.add(g);
    this.sock = sock;
  }

  /** Per-frame world animation. */
  update(dt, sim) {
    this.time += dt;
    this.waterUniforms.uTime.value = this.time;
    const w = sim.wind.settings;
    const a = (w.fromDeg * Math.PI) / 180;
    // waves travel downwind
    this.waterUniforms.uWind.value.set(-Math.sin(a), Math.cos(a));
    // swimmer
    const [sn, se] = sim.scenario.swimmer;
    this.swimmer.position.set(se, Math.sin(this.time * 1.3) * 0.06, -sn);
    if (this.armL) {
      this.armL.rotation.z = -0.6 - 0.5 * Math.sin(this.time * 3);
      this.armR.rotation.z = 0.6 + 0.5 * Math.sin(this.time * 3 + 1);
    }
    const r = 1 + ((this.time * 0.6) % 1) * 1.5;
    this.ripple.scale.set(r, r, r);
    this.ripple.material.opacity = 0.6 * (1 - ((this.time * 0.6) % 1));
    // windsock: points downwind, droops in light wind
    const wv = sim.windNow || [0, 0, 0];
    const ws = Math.hypot(wv[0], wv[1]);
    const yawW = Math.atan2(wv[0], wv[1]); // sock axis (+x) points downwind
    const droop = -Math.max(0, 1 - ws / 8) * 1.2;
    this.sock.rotation.set(0, yawW, droop, 'YXZ');
    // payload
    const pl = sim.payload;
    if (pl) {
      this.payloadMesh.visible = true;
      const p = ned2three(pl.pos);
      if (!pl.inAir) p.y = Math.sin(this.time * 1.4) * 0.05 + 0.03;
      this.payloadMesh.position.copy(p);
      this.payloadMesh.rotation.x += pl.inAir ? dt * 6 : 0;
      if (!pl.inAir) this.payloadMesh.rotation.set(-Math.PI / 2, 0, 0);
    } else {
      this.payloadMesh.visible = false;
    }
  }

  /** Keep the shadow camera centred on the aircraft. */
  followShadow(target) {
    this.sun.position.copy(target).addScaledVector(this.sunDir, 100);
    this.sun.target.position.copy(target);
  }

  resize(w, h) {
    this.renderer.setSize(w, h, false);
  }
}
