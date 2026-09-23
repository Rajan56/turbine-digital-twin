// scene.js
// Three.js scene: the physical turbine (left) and its digital twin (right) share one model.
// Geometry follows the NREL 5-MW reference turbine: 126 m rotor with the published chord and
// twist distribution, 87.6 m tapered tower, 2.5° precone and 5° shaft tilt.

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { Sky } from 'three/examples/jsm/objects/Sky.js';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import { CSS2DRenderer, CSS2DObject } from 'three/examples/jsm/renderers/CSS2DRenderer.js';
import { T5 } from './physics.js';

const D2R = Math.PI / 180;

// ------------------------------------------------------------------ blade loft
// NREL 5-MW blade structural/aero stations (Jonkman et al., 2009, Table 2-1)
const BLADE = [
  // r [m], chord [m], twist [deg], relative thickness
  [1.5, 3.542, 13.308, 1.00], [2.8667, 3.542, 13.308, 1.00], [5.6, 3.854, 13.308, 0.93], [8.3333, 4.167, 13.308, 0.70],
  [11.75, 4.557, 13.308, 0.405], [15.85, 4.652, 11.480, 0.35], [19.95, 4.458, 10.162, 0.30], [24.05, 4.249, 9.011, 0.25],
  [28.15, 4.007, 7.795, 0.25], [32.25, 3.748, 6.544, 0.21], [36.35, 3.502, 5.361, 0.21], [40.45, 3.256, 4.188, 0.21],
  [44.55, 3.010, 3.125, 0.18], [48.65, 2.764, 2.319, 0.18], [52.75, 2.518, 1.526, 0.18], [56.1667, 2.313, 0.863, 0.18],
  [58.9, 2.086, 0.370, 0.18], [61.6333, 1.419, 0.106, 0.18], [63.0, 0.5, 0.0, 0.18],
];
function interpBlade(r) {
  for (let i = 1; i < BLADE.length; i++) if (r <= BLADE[i][0]) {
    const a = BLADE[i - 1], b = BLADE[i], f = (r - a[0]) / (b[0] - a[0]);
    return a.map((v, k) => v + (b[k] - v) * f);
  }
  return BLADE[BLADE.length - 1];
}
function airfoil(xc, t, upper) { // NACA 4-digit, 2 % camber at 40 %
  const m = 0.02, p = 0.4;
  const yt = 5 * t * (0.2969 * Math.sqrt(xc) - 0.126 * xc - 0.3516 * xc * xc + 0.2843 * xc ** 3 - 0.1036 * xc ** 4);
  const yc = xc < p ? m / (p * p) * (2 * p * xc - xc * xc) : m / ((1 - p) ** 2) * ((1 - 2 * p) + 2 * p * xc - xc * xc);
  return upper ? yc + yt : yc - yt;
}
export function makeBladeGeometry(nSpan = 46, nChord = 22) {
  const pos = [], rr = [], idx = [];
  const ring = 2 * nChord;
  for (let s = 0; s < nSpan; s++) {
    const u = s / (nSpan - 1);
    const r = T5.hubR + (T5.R - T5.hubR) * (1 - Math.cos(u * Math.PI / 2 * 1.0)) ** 0.85; // denser at root and tip
    const [, c, tw, th] = interpBlade(r);
    const circ = Math.max(0, Math.min(1, (th - 0.4) / 0.6));   // 1 = cylinder, 0 = airfoil
    const xAxis = 0.3 * (1 - circ) + 0.5 * circ;                // pitch axis position along chord
    const rot = -tw * D2R;
    for (let k = 0; k < ring; k++) {
      const upper = k < nChord;
      const j = upper ? k : ring - 1 - k;
      const xc = 0.5 - 0.5 * Math.cos(Math.PI * j / (nChord - 1));
      const yAf = airfoil(xc, Math.min(th, 0.45), upper);
      const yCi = (upper ? 1 : -1) * Math.sqrt(Math.max(0, 0.25 - (xc - 0.5) ** 2));
      const yN = yAf * (1 - circ) + yCi * circ;
      const lz = (xc - xAxis) * c;          // chordwise, leading edge towards -z
      const lx = -yN * c;                   // suction side downwind (-x)
      const x = lx * Math.cos(rot) + lz * Math.sin(rot);
      const z = -lx * Math.sin(rot) + lz * Math.cos(rot);
      pos.push(x, r - 0, z); rr.push(r / T5.R, xc);
    }
  }
  for (let s = 0; s < nSpan - 1; s++) for (let k = 0; k < ring; k++) {
    const a = s * ring + k, b = s * ring + (k + 1) % ring, c2 = (s + 1) * ring + k, d = (s + 1) * ring + (k + 1) % ring;
    idx.push(a, c2, b, b, c2, d);
  }
  // tip cap
  const tipC = pos.length / 3; const last = (nSpan - 1) * ring;
  let cx = 0, cz = 0; for (let k = 0; k < ring; k++) { cx += pos[(last + k) * 3]; cz += pos[(last + k) * 3 + 2]; }
  pos.push(cx / ring, T5.R + 0.05, cz / ring); rr.push(1, 0.5);
  for (let k = 0; k < ring; k++) idx.push(last + k, tipC, last + (k + 1) % ring);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('span', new THREE.Float32BufferAttribute(rr, 2));
  g.setIndex(idx); g.computeVertexNormals();
  g.setAttribute('color', new THREE.Float32BufferAttribute(new Float32Array(pos.length), 3));
  return g;
}

// ---------------------------------------------------------------- materials
const M = {
  paint: new THREE.MeshStandardMaterial({ color: 0xf1f3f4, roughness: 0.42, metalness: 0.05 }),
  blade: new THREE.MeshStandardMaterial({ color: 0xf4f5f6, roughness: 0.38, metalness: 0.02 }),
  steelDark: new THREE.MeshStandardMaterial({ color: 0x5d6770, roughness: 0.5, metalness: 0.6 }),
  concrete: new THREE.MeshStandardMaterial({ color: 0xa9a59c, roughness: 0.95 }),
  gravel: new THREE.MeshStandardMaterial({ color: 0x9c948a, roughness: 1 }),
  red: new THREE.MeshStandardMaterial({ color: 0xff2a1a, emissive: 0xff1a0a, emissiveIntensity: 2 }),
  twinBody: new THREE.MeshStandardMaterial({ color: 0x0f3b52, emissive: 0x0b8fb3, emissiveIntensity: 0.25, transparent: true, opacity: 0.32, depthWrite: false, roughness: 0.6 }),
  twinBlade: new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.92 }),
  twinEdge: new THREE.LineBasicMaterial({ color: 0x47d7ff, transparent: true, opacity: 0.55 }),
  twinGround: new THREE.MeshBasicMaterial({ color: 0x07131c }),
};

// colour map for loads (blue → cyan → green → yellow → red)
function heat(v, out) {
  v = Math.max(0, Math.min(1, v));
  const stops = [[0, 0.10, 0.25, 0.75], [0.3, 0.05, 0.75, 0.95], [0.55, 0.25, 0.9, 0.45], [0.78, 1.0, 0.85, 0.2], [1, 1.0, 0.25, 0.15]];
  for (let i = 1; i < stops.length; i++) if (v <= stops[i][0]) {
    const a = stops[i - 1], b = stops[i], f = (v - a[0]) / (b[0] - a[0]);
    out.setRGB(a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f, a[3] + (b[3] - a[3]) * f); return out;
  }
  return out.setRGB(1, 0.25, 0.15);
}

// ----------------------------------------------------------------- turbine
function edges(mesh, thr = 25) {
  const e = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, thr), M.twinEdge);
  e.visible = false; e.userData.twinOnly = true; mesh.add(e); return e;
}
function buildTurbine({ detailed = true, bladeGeo }) {
  const root = new THREE.Group();
  const parts = { root, edges: [], blades: [], physMeshes: [] };
  const reg = (m, twinMat = M.twinBody) => { m.userData.phys = m.material; m.userData.twin = twinMat; parts.physMeshes.push(m); m.castShadow = true; m.receiveShadow = true; if (detailed) parts.edges.push(edges(m)); return m; };
  // foundation + tower
  const found = new THREE.Mesh(new THREE.CylinderGeometry(4.2, 4.6, 1.2, 40), M.concrete); found.position.y = 0.3; root.add(reg(found));
  const Ht = T5.towerH;
  const tg = new THREE.CylinderGeometry(T5.towerDTop / 2, T5.towerDBase / 2, Ht, 48, 24, true); tg.translate(0, Ht / 2 + 0.9, 0);
  tg.setAttribute('color', new THREE.Float32BufferAttribute(new Float32Array(tg.attributes.position.count * 3), 3));
  const towerTwin = detailed ? new THREE.MeshBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.8 }) : M.twinBody;
  const tower = new THREE.Mesh(tg, M.paint); root.add(reg(tower, towerTwin));
  parts.tower = tower; parts.towerBase = Float32Array.from(tg.attributes.position.array);
  if (detailed) {
    const door = new THREE.Mesh(new THREE.BoxGeometry(0.2, 2.3, 1.1), M.steelDark); door.position.set(3.0, 2.2, 0); root.add(door);
    const tr = new THREE.Mesh(new RoundedBoxGeometry(2.6, 2.4, 3.2, 2, 0.12), new THREE.MeshStandardMaterial({ color: 0x7c8a78, roughness: 0.8 }));
    tr.position.set(7.5, 1.2, 4); root.add(reg(tr));
  }
  // nacelle (yaws)
  const yaw = new THREE.Group(); yaw.position.y = Ht + 0.9; root.add(yaw); parts.yaw = yaw;
  const tilt = new THREE.Group(); tilt.rotation.z = T5.tilt * D2R; yaw.add(tilt); parts.tilt = tilt;
  const body = new THREE.Mesh(new RoundedBoxGeometry(14.5, 4.4, 4.3, 4, 0.9), M.paint); body.position.set(-2.4, 2.4, 0); tilt.add(reg(body));
  if (detailed) {
    const cool = new THREE.Mesh(new RoundedBoxGeometry(3.2, 1.3, 3.8, 2, 0.2), M.paint); cool.position.set(-8.3, 4.9, 0); tilt.add(reg(cool));
    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 2.4, 8), M.steelDark); mast.position.set(-6.4, 5.8, 0.9); tilt.add(mast);
    const cup = new THREE.Mesh(new THREE.SphereGeometry(0.22, 12, 8), M.steelDark); cup.position.set(-6.4, 7.0, 0.9); tilt.add(cup);
    const vane = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.35, 0.04), M.steelDark); vane.position.set(-6.9, 6.7, -0.9); tilt.add(vane);
    parts.lights = [];
    for (const zz of [-1.3, 1.3]) { const l = new THREE.Mesh(new THREE.SphereGeometry(0.22, 10, 8), M.red.clone()); l.position.set(-8.8, 5.75, zz); tilt.add(l); parts.lights.push(l); }
    // drivetrain inside (visible in the twin)
    const dt = new THREE.Group(); dt.userData.twinOnly = true; dt.visible = false; tilt.add(dt); parts.drivetrain = dt;
    const mk = (geo, x, y, col) => { const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: col, emissive: col, emissiveIntensity: 0.35, roughness: 0.4 })); m.position.set(x, y, 0); dt.add(m); return m; };
    parts.mainShaft = mk(new THREE.CylinderGeometry(0.45, 0.45, 5.2, 16).rotateZ(Math.PI / 2), 1.6, 2.4, 0x7cc8e8);
    parts.gearbox = mk(new THREE.CylinderGeometry(1.35, 1.35, 2.6, 24).rotateZ(Math.PI / 2), -2.2, 2.4, 0x3fb5d9);
    parts.generator = mk(new THREE.CylinderGeometry(1.1, 1.1, 3.0, 24).rotateZ(Math.PI / 2), -6.0, 2.4, 0x3aa0c8);
    parts.hssBearing = mk(new THREE.SphereGeometry(0.55, 16, 12), -3.8, 2.4, 0x55e0a0);
  }
  // rotor
  const rotor = new THREE.Group(); rotor.position.set(5.0, 2.4, 0); tilt.add(rotor); parts.rotor = rotor;
  const spinProfile = []; for (let i = 0; i <= 16; i++) { const t = i / 16; spinProfile.push(new THREE.Vector2(1.9 * Math.sqrt(Math.sin(t * Math.PI / 2)), -1.2 + 5.2 * (1 - t) - 0.0)); }
  const spinner = new THREE.Mesh(new THREE.LatheGeometry(spinProfile.reverse(), 40), M.paint); spinner.rotation.z = -Math.PI / 2; spinner.position.x = -1.2; rotor.add(reg(spinner));
  for (let b = 0; b < 3; b++) {
    const pivot = new THREE.Group(); pivot.rotation.x = b * 2 * Math.PI / 3; rotor.add(pivot);
    const cone = new THREE.Group(); cone.rotation.z = -T5.precone * D2R; pivot.add(cone);
    const blade = new THREE.Mesh(bladeGeo, M.blade); blade.castShadow = true; blade.userData.phys = M.blade; blade.userData.twin = M.twinBlade;
    parts.physMeshes.push(blade); cone.add(blade); parts.blades.push(blade);
    if (detailed) parts.edges.push(edges(blade, 40));
  }
  return parts;
}

// ------------------------------------------------------------------ terrain
function hash(x, y) { const s = Math.sin(x * 127.1 + y * 311.7) * 43758.5453; return s - Math.floor(s); }
function noise(x, y) {
  const xi = Math.floor(x), yi = Math.floor(y), xf = x - xi, yf = y - yi;
  const u = xf * xf * (3 - 2 * xf), v = yf * yf * (3 - 2 * yf);
  const a = hash(xi, yi), b = hash(xi + 1, yi), c = hash(xi, yi + 1), d = hash(xi + 1, yi + 1);
  return a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v;
}
function terrainH(x, z) {
  const d = Math.hypot(x, z);
  let h = 0, amp = 22, f = 1 / 700;
  for (let o = 0; o < 4; o++) { h += (noise(x * f + 11, z * f - 7) - 0.5) * amp; amp *= 0.45; f *= 2.1; }
  const pad = Math.min(1, Math.max(0, (d - 70) / 250));
  return h * pad;
}

export class TurbineScene {
  constructor(container) {
    this.container = container;
    this.mode = 'split';                      // 'physical' | 'twin' | 'split'
    this.exag = 15; this.showFlow = true; this.snow = 0; this.t = 0;
    const renderer = this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.75));
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 0.55;
    container.appendChild(renderer.domElement);
    this.labels = new CSS2DRenderer(); this.labels.domElement.className = 'labels'; container.appendChild(this.labels.domElement);

    const scene = this.scene = new THREE.Scene();
    const cam = this.camera = new THREE.PerspectiveCamera(40, 1, 1, 20000);
    cam.position.set(-235, 60, 300);
    const ctr = this.controls = new OrbitControls(cam, renderer.domElement);
    ctr.target.set(0, 74, 0); ctr.enableDamping = true; ctr.maxPolarAngle = Math.PI * 0.495; ctr.minDistance = 20; ctr.maxDistance = 3000;
    ctr.update();

    // sky + sun
    const sky = this.sky = new Sky(); sky.scale.setScalar(15000); scene.add(sky);
    const su = sky.material.uniforms;
    su.turbidity.value = 3.2; su.rayleigh.value = 2.2; su.mieCoefficient.value = 0.003; su.mieDirectionalG.value = 0.8;
    this.sunDir = new THREE.Vector3().setFromSphericalCoords(1, THREE.MathUtils.degToRad(90 - 24), THREE.MathUtils.degToRad(200));
    su.sunPosition.value.copy(this.sunDir);
    const pmrem = new THREE.PMREMGenerator(renderer);
    const envScene = new THREE.Scene(); const sky2 = new Sky(); sky2.scale.setScalar(1000); Object.assign(sky2.material.uniforms.sunPosition.value, this.sunDir);
    for (const k of ['turbidity', 'rayleigh', 'mieCoefficient', 'mieDirectionalG']) sky2.material.uniforms[k].value = su[k].value;
    envScene.add(sky2);
    this.env = pmrem.fromScene(envScene).texture; scene.environment = this.env;
    this.fogPhys = new THREE.Fog(0xbfd0dc, 900, 7000); this.fogTwin = new THREE.Fog(0x061019, 700, 5000);
    scene.fog = this.fogPhys;

    const sun = this.sun = new THREE.DirectionalLight(0xfff1dc, 3.2);
    sun.position.copy(this.sunDir).multiplyScalar(400); sun.target.position.set(0, 40, 0);
    sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
    const sc = sun.shadow.camera; sc.left = -120; sc.right = 120; sc.top = 160; sc.bottom = -60; sc.near = 50; sc.far = 900; sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.4;
    scene.add(sun, sun.target);
    this.hemi = new THREE.HemisphereLight(0xcfe4ff, 0x5a6b4a, 0.9); scene.add(this.hemi);

    this.buildGround();
    // turbines
    const bladeGeo = this.bladeGeo = makeBladeGeometry();
    this.main = buildTurbine({ detailed: true, bladeGeo });
    scene.add(this.main.root);
    const simpleBlade = makeBladeGeometry(18, 8);
    { const cA = simpleBlade.attributes.color; for (let i = 0; i < cA.count; i++) cA.setXYZ(i, 0.12, 0.55, 0.72); }
    this.fleet = [];
    const spots = [[-700, -420], [-80, -820], [640, -560], [980, 120], [-980, 260], [520, 760], [-520, 820], [1350, -380]];
    for (const [x, z] of spots) {
      const t = buildTurbine({ detailed: false, bladeGeo: simpleBlade });
      t.root.position.set(x, terrainH(x, z) - 1, z); t.phase = Math.random() * 6; t.speedF = 0.9 + Math.random() * 0.2;
      scene.add(t.root); this.fleet.push(t);
    }
    // upwind neighbour for the wake scenario (appears on demand), at 225° and 5 rotor diameters
    const up = buildTurbine({ detailed: false, bladeGeo: simpleBlade });
    const a = 225 * D2R, dist = 5 * 126;
    const ux = Math.sin(a) * dist, uz = -Math.cos(a) * dist;
    up.root.position.set(ux, terrainH(ux, uz) - 1, uz); up.root.visible = false; up.phase = 1; up.speedF = 1;
    scene.add(up.root); this.upwind = up;

    this.buildFlow();
    this.buildTwinOverlays();
    this.resize();
    window.addEventListener('resize', () => this.resize());
    this.camGoal = null;
  }

  buildGround() {
    const size = 9000, seg = 220;
    const g = new THREE.PlaneGeometry(size, size, seg, seg); g.rotateX(-Math.PI / 2);
    const p = g.attributes.position, col = new Float32Array(p.count * 3);
    const c = new THREE.Color();
    this.groundBase = new Float32Array(p.count * 3);
    for (let i = 0; i < p.count; i++) {
      const x = p.getX(i), z = p.getZ(i);
      const h = terrainH(x, z); p.setY(i, h);
      const n = noise(x / 90, z / 90), n2 = noise(x / 23 + 5, z / 23);
      c.setRGB(0.20 + 0.10 * n + 0.05 * n2, 0.30 + 0.10 * n, 0.14 + 0.05 * n2);
      const d = Math.hypot(x, z); if (d < 55) c.setRGB(0.40, 0.38, 0.34); // crane pad
      col[i * 3] = c.r; col[i * 3 + 1] = c.g; col[i * 3 + 2] = c.b;
    }
    this.groundBase.set(col);
    g.setAttribute('color', new THREE.BufferAttribute(col, 3)); g.computeVertexNormals();
    this.groundMat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 1, metalness: 0 });
    const ground = this.ground = new THREE.Mesh(g, this.groundMat); ground.receiveShadow = true; this.scene.add(ground);
    // access road
    const road = new THREE.Mesh(new THREE.PlaneGeometry(7, 900), M.gravel); road.rotation.x = -Math.PI / 2; road.rotation.z = 0.5; road.position.set(210, 0.25, 360); road.receiveShadow = true;
    this.scene.add(road); this.road = road;
    // boreal forest (instanced spruce)
    const cone = new THREE.ConeGeometry(2.6, 13, 7); cone.translate(0, 8.5, 0);
    const trunk = new THREE.CylinderGeometry(0.3, 0.4, 3, 5); trunk.translate(0, 1.5, 0);
    const n = 5200;
    const treeMat = new THREE.MeshStandardMaterial({ color: 0x1f3a26, roughness: 0.95 });
    this.treeMat = treeMat;
    const trees = new THREE.InstancedMesh(cone, treeMat, n), trunks = new THREE.InstancedMesh(trunk, new THREE.MeshStandardMaterial({ color: 0x3b2c20 }), n);
    const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(), v = new THREE.Vector3();
    let k = 0, guard = 0;
    while (k < n && guard++ < 60000) {
      const x = (Math.random() - 0.5) * 3600, z = (Math.random() - 0.5) * 3600;
      const d = Math.hypot(x, z);
      if (d < 120) continue;
      if (Math.abs((x - 210) * Math.cos(0.5) - (z - 360) * Math.sin(0.5)) < 14 && Math.hypot(x - 210, z - 360) < 460) continue;
      if (noise(x / 260, z / 260) < 0.38) continue; // clearings
      const sc = 0.7 + Math.random() * 0.8;
      s.set(sc, sc * (0.8 + Math.random() * 0.5), sc); q.setFromAxisAngle(v.set(0, 1, 0), Math.random() * 6);
      m4.compose(v.set(x, terrainH(x, z) - 0.3, z), q, s); trees.setMatrixAt(k, m4); trunks.setMatrixAt(k, m4); k++;
    }
    trees.count = trunks.count = k; trees.castShadow = true; trees.receiveShadow = true;
    this.scene.add(trees, trunks); this.trees = trees; this.trunks = trunks;
    // twin grid
    const grid = new THREE.GridHelper(4000, 80, 0x1b6f8f, 0x0d3446); grid.position.y = 0.4; grid.visible = false; this.scene.add(grid); this.grid = grid;
  }

  // ----------------------------------------------------------- wind streaks
  buildFlow() {
    const n = this.nP = 1600;
    this.pPos = new Float32Array(n * 3); this.pAge = new Float32Array(n);
    const g = new THREE.BufferGeometry();
    this.lPos = new Float32Array(n * 6); this.lCol = new Float32Array(n * 6);
    g.setAttribute('position', new THREE.BufferAttribute(this.lPos, 3));
    g.setAttribute('color', new THREE.BufferAttribute(this.lCol, 3));
    this.flowMat = new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.55, depthWrite: false });
    this.flow = new THREE.LineSegments(g, this.flowMat); this.flow.frustumCulled = false;
    this.flowGroup = new THREE.Group(); this.flowGroup.add(this.flow); this.scene.add(this.flowGroup);
    for (let i = 0; i < n; i++) this.spawn(i, true);
  }
  spawn(i, anywhere) {
    const x = anywhere ? -350 + Math.random() * 1100 : -350 - Math.random() * 30;
    const r = Math.sqrt(Math.random()) * 105, th = Math.random() * Math.PI * 2;
    this.pPos[i * 3] = x; this.pPos[i * 3 + 1] = Math.max(8, T5.hubH + r * Math.cos(th) * 0.95); this.pPos[i * 3 + 2] = r * Math.sin(th) * 1.4;
    this.pAge[i] = Math.random() * 0.3;
  }
  // wind speed at a point in the wind frame (x downwind, rotor at x = 0): shear + induction + Jensen wake
  windAt(x, y, z, U, a) {
    const shear = Math.pow(Math.max(y, 2) / T5.hubH, 0.2);
    const rr = Math.hypot(y - T5.hubH, z);
    let f = 1;
    if (x < 0 && rr < T5.R * 1.1) { const xi = x / T5.R; f = 1 - a * (1 + xi / Math.sqrt(1 + xi * xi)); }
    else if (x >= 0) { const Rw = T5.R + 0.075 * x; if (rr < Rw) f = 1 - 2 * a * (T5.R / Rw) ** 2; }
    return U * shear * f;
  }

  buildTwinOverlays() {
    const mkLabel = (cls, dx = 0, dy = 0, near = 0) => {
      const outer = document.createElement('div'); const d = document.createElement('div'); d.className = 'tag ' + (cls || '');
      d.style.transform = `translate(${dx}px, ${dy}px)`; outer.appendChild(d);
      const o = new CSS2DObject(outer); o.visible = false; o.userData.near = near; o.userData.inner = d; return o;
    };
    const t = this.main;
    this.tags = {};
    const add = (key, parent, pos, cls, dx, dy, near) => { const o = mkLabel(cls, dx, dy, near); o.position.copy(pos); parent.add(o); this.tags[key] = o; return o; };
    add('anemo', t.tilt, new THREE.Vector3(-6.4, 7.4, 0.9), '', 0, -44, 1);
    add('bearing', t.tilt, new THREE.Vector3(-3.8, 2.4, 0), '', -120, 40, 1);
    add('gen', t.tilt, new THREE.Vector3(-6.0, 2.4, 0), '', 110, 56, 1);
    add('yaw', t.yaw, new THREE.Vector3(0, -2, 0), '', 0, 96, 1);
    add('pitch', t.rotor, new THREE.Vector3(0, 0, 0), '', 96, -46, 0);
    add('nac', t.tilt, new THREE.Vector3(-4, 4, 0), '', -118, -52, -1);
    add('tower', t.root, new THREE.Vector3(0, 45, 0), '', -96, 0, 0);
    add('base', t.root, new THREE.Vector3(0, 3, 0), '', 92, -18, 0);
    // sensor dots
    this.dots = [];
    const dotMat = new THREE.MeshBasicMaterial({ color: 0x7ff3ff });
    for (const [parent, p] of [[t.tilt, [-6.4, 7.0, 0.9]], [t.tilt, [-3.8, 2.4, 0]], [t.tilt, [-6.0, 2.4, 0]], [t.root, [0, 30, 2.8]], [t.root, [0, 3, 3.2]], [t.yaw, [0, 0, 1.8]]]) {
      const d = new THREE.Mesh(new THREE.SphereGeometry(0.45, 12, 8), dotMat.clone()); d.position.set(...p); d.visible = false; d.userData.twinOnly = true; parent.add(d); this.dots.push(d);
    }
    // yaw misalignment indicator
    const arcG = new THREE.BufferGeometry();
    this.arc = new THREE.Line(arcG, new THREE.LineBasicMaterial({ color: 0xffb84d })); this.arc.visible = false; this.arc.userData.twinOnly = true;
    this.windLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(60, 0, 0)]), new THREE.LineDashedMaterial({ color: 0x7ff3ff, dashSize: 3, gapSize: 2 }));
    this.windLine.computeLineDistances(); this.windLine.visible = false; this.windLine.userData.twinOnly = true;
    this.axisLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(60, 0, 0)]), new THREE.LineBasicMaterial({ color: 0xffb84d }));
    this.axisLine.visible = false; this.axisLine.userData.twinOnly = true;
    this.scene.add(this.arc, this.windLine, this.axisLine);
    this.tags.misalign = mkLabel('warn'); this.scene.add(this.tags.misalign);
    this.flowMatPhys = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.22, depthWrite: false });
    // wake cone from the upwind neighbour (twin only)
    const wc = new THREE.Mesh(new THREE.CylinderGeometry(63, 63 + 0.075 * 640, 640, 32, 1, true), new THREE.MeshBasicMaterial({ color: 0xffb84d, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
    wc.rotation.z = Math.PI / 2; this.wakeCone = new THREE.Group(); this.wakeCone.add(wc); wc.position.x = 320; this.wakeCone.visible = false; this.wakeCone.userData.twinOnly = true;
    this.scene.add(this.wakeCone);
  }

  // switch every mesh between the physical look and the twin look
  applyLook(twin) {
    const all = [this.main, this.upwind, ...this.fleet];
    for (const t of all) {
      for (const m of t.physMeshes) {
        m.material = twin ? m.userData.twin : m.userData.phys;
      }
      for (const e of t.edges) e.visible = twin;
      if (t.drivetrain) t.drivetrain.visible = twin;
      if (t.lights) t.lights.forEach(l => l.visible = !twin);
    }
    this.sky.visible = !twin;
    this.scene.background = twin ? new THREE.Color(0x061019) : null;
    this.scene.fog = twin ? this.fogTwin : this.fogPhys;
    this.ground.material = twin ? M.twinGround : this.groundMat;
    this.grid.visible = twin; this.trees.visible = this.trunks.visible = !twin; this.road.visible = !twin;
    this.hemi.intensity = twin ? 0.5 : 0.9;
    for (const d of this.dots) d.visible = twin;
    this.arc.visible = this.windLine.visible = this.axisLine.visible = twin && this.misalign > 1.5;
    this.wakeCone.visible = twin && this.wakeOn;
    this.flow.material = twin ? this.flowMat : this.flowMatPhys; this.flowMat.opacity = 0.75;
    this.twinPass = twin;
  }

  setMode(m) { this.mode = m; this.resize(); }
  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    this.W = w; this.H = h;
    this.renderer.setSize(w, h);
    const split = this.mode === 'split';
    this.camera.aspect = (split ? w / 2 : w) / h; this.camera.updateProjectionMatrix();
    const lw = this.mode === 'physical' ? 0 : (split ? w / 2 : w);
    this.labels.setSize(Math.max(1, lw), h);
    this.labels.domElement.style.left = split ? `${w / 2}px` : '0px';
    this.labels.domElement.style.display = this.mode === 'physical' ? 'none' : 'block';
  }

  goTo(preset) {
    const P = {
      overview: [[-235, 60, 300], [0, 74, 0]],
      nacelle: [[46, 107, 34], [1, 92, 0]],
      rotor: [[-210, 95, 30], [0, 84, 0]],
      farm: [[-1300, 520, 1500], [0, 60, 0]],
      ground: [[60, 4, 70], [0, 60, 0]],
    }[preset];
    if (!P) return;
    // presets are defined for wind from the west; rotate with the current wind direction
    const a = ((270 - (this.windDirDeg ?? 270)) * D2R), c = Math.cos(a), s = Math.sin(a);
    const rot = ([x, y, z]) => new THREE.Vector3(x * c + z * s, y, -x * s + z * c);
    this.camGoal = { pos: rot(P[0]), tgt: rot(P[1]) };
  }

  // ------------------------------------------------------------ per frame
  update(dt, st) {
    this.t += dt;
    const t = this.main;
    // wind frame: meteorological direction (from), north = -z, east = +x
    const th = st.windDir * D2R;
    const psiRotor = (90 - (st.windDir + st.yawErr)) * D2R;
    const psiWind = Math.atan2(-Math.cos(th), -Math.sin(th));
    this.windDirDeg = st.windDir;
    // smooth yaw drive (0.3°/s)
    if (this.yawNow == null) this.yawNow = psiRotor;
    let dy = Math.atan2(Math.sin(psiRotor - this.yawNow), Math.cos(psiRotor - this.yawNow));
    this.yawNow += Math.max(-0.3 * D2R * dt * 8, Math.min(0.3 * D2R * dt * 8, dy));
    t.yaw.rotation.y = this.yawNow;
    // rotor: clockwise seen from upwind
    t.rotor.rotation.x = -st.azimuth;
    for (const b of t.blades) b.rotation.y = -st.pitch * D2R;
    // tower bending (first mode shape ~ (z/H)^2), displacement downwind
    const xTop = st.towerX * this.exag;
    const dwx = -Math.cos(this.yawNow), dwz = Math.sin(this.yawNow); // downwind in world = -rotor axis
    {
      const pa = t.tower.geometry.attributes.position, b = t.towerBase, ca = t.tower.geometry.attributes.color, cc = new THREE.Color();
      const inW = st.inWake ? 0.18 : 0;
      for (let i = 0; i < pa.count; i++) {
        const y = b[i * 3 + 1] - 0.9, f = Math.pow(Math.max(0, y) / T5.towerH, 2);
        pa.array[i * 3] = b[i * 3] + dwx * xTop * f; pa.array[i * 3 + 2] = b[i * 3 + 2] + dwz * xTop * f;
        heat(Math.min(1, st.thrust * (T5.hubH - y) / (800000 * T5.hubH) * 0.9 + inW), cc); ca.setXYZ(i, cc.r, cc.g, cc.b);
      }
      pa.needsUpdate = true; ca.needsUpdate = true; t.tower.geometry.computeBoundingSphere();
    }
    t.yaw.position.x = dwx * xTop; t.yaw.position.z = dwz * xTop;
    if (t.lights) { const on = (this.t % 2) < 1; t.lights.forEach(l => l.material.emissiveIntensity = on ? 3 : 0.1); }
    // fleet and neighbour
    for (const f of this.fleet.concat([this.upwind])) {
      f.yaw.rotation.y = (90 - st.windDir) * D2R;
      f.rotor.rotation.x = -(st.azimuth * f.speedF + f.phase);
      for (const b of f.blades) b.rotation.y = -st.pitch * D2R;
    }
    this.wakeOn = st.wakeOn; this.upwind.root.visible = st.wakeOn;
    this.wakeCone.rotation.y = psiWind; this.wakeCone.position.copy(this.upwind.root.position).setY(T5.hubH);
    this.wakeCone.children[0].material.opacity = st.inWake ? 0.14 : 0.04;

    // ---- twin colouring: blade flapwise bending (uniform thrust per unit span → M(r) ∝ (1 - r/R)^2 (2 + r/R)), tower bending
    const load = Math.min(1.15, st.thrust / 800000);
    const col = this.bladeGeo.attributes.color, sp = this.bladeGeo.attributes.span, c = new THREE.Color();
    for (let i = 0; i < col.count; i++) {
      const r = sp.getX(i);
      let v = load * Math.pow(1 - r, 2) * (2 + r) / 2 * 0.95;
      if (st.ice > 0.02) { const x = sp.getY(i); v = v * (1 - st.ice) + st.ice * (0.35 + 0.15 * (x < 0.15)); }
      heat(v, c);
      if (st.ice > 0.02) c.lerp(new THREE.Color(0.85, 0.55, 1.0), st.ice * 0.6);
      if (st.heating) { const x = sp.getY(i); if (x < 0.2) c.lerp(new THREE.Color(1, 0.45, 0.1), 0.6); }
      col.setXYZ(i, c.r, c.g, c.b);
    }
    col.needsUpdate = true;
    const Mbase = st.thrust * T5.hubH;
    // drivetrain temperatures
    if (t.hssBearing) {
      const exc = Math.max(0, st.bearingResid);
      heat(0.3 + exc / 10, c); t.hssBearing.material.color.copy(c); t.hssBearing.material.emissive.copy(c);
      t.hssBearing.material.emissiveIntensity = 0.5 + (exc > 2 ? 0.8 * (0.5 + 0.5 * Math.sin(this.t * 6)) : 0);
      t.hssBearing.scale.setScalar(1 + Math.min(0.8, exc / 10));
      heat(0.25 + st.load * 0.35, c); t.gearbox.material.color.copy(c); t.gearbox.material.emissive.copy(c).multiplyScalar(0.5);
      heat(0.2 + st.load * 0.4, c); t.generator.material.color.copy(c); t.generator.material.emissive.copy(c).multiplyScalar(0.5);
      t.mainShaft.rotation.x = -st.azimuth;
    }
    // physical look: ice on blades, snow on ground
    const iceCol = new THREE.Color(0xf4f5f6).lerp(new THREE.Color(0xc9dde9), Math.min(1, st.ice * 1.4));
    M.blade.color.copy(iceCol); M.blade.roughness = 0.38 - 0.2 * st.ice;
    const snowTarget = st.snow;
    if (Math.abs(snowTarget - this.snow) > 0.01) {
      this.snow += Math.sign(snowTarget - this.snow) * Math.min(Math.abs(snowTarget - this.snow), dt * 0.25);
      const cc = this.ground.geometry.attributes.color, b = this.groundBase;
      for (let i = 0; i < cc.count; i++) {
        cc.array[i * 3] = b[i * 3] + (0.9 - b[i * 3]) * this.snow; cc.array[i * 3 + 1] = b[i * 3 + 1] + (0.93 - b[i * 3 + 1]) * this.snow; cc.array[i * 3 + 2] = b[i * 3 + 2] + (0.97 - b[i * 3 + 2]) * this.snow;
      }
      cc.needsUpdate = true;
      this.treeMat.color.setRGB(0.12 + 0.5 * this.snow, 0.23 + 0.45 * this.snow, 0.15 + 0.55 * this.snow);
      this.fogPhys.near = 900 - 650 * this.snow; this.fogPhys.far = 7000 - 5200 * this.snow;
      this.fogPhys.color.setRGB(0.75 + 0.1 * this.snow, 0.82 + 0.06 * this.snow, 0.86 + 0.06 * this.snow);
    }
    // yaw misalignment indicator
    this.misalign = Math.abs(st.yawErrTrue);
    const hub = new THREE.Vector3(dwx * xTop, T5.hubH + 4, dwz * xTop);
    const up = new THREE.Vector3(Math.cos(th - Math.PI / 2 + Math.PI / 2) * 0, 0, 0);
    const Lw = 55;
    const wdir = new THREE.Vector3(Math.sin(th), 0, -Math.cos(th));           // towards where wind comes from
    const rdir = new THREE.Vector3(Math.cos(this.yawNow), 0, -Math.sin(this.yawNow)); // rotor axis (upwind)
    this.windLine.geometry.setFromPoints([hub, hub.clone().addScaledVector(wdir, Lw)]); this.windLine.computeLineDistances();
    this.axisLine.geometry.setFromPoints([hub, hub.clone().addScaledVector(rdir, Lw)]);
    const pts = []; for (let k = 0; k <= 20; k++) { const v = wdir.clone().lerp(rdir, k / 20).normalize(); pts.push(hub.clone().addScaledVector(v, 40)); }
    this.arc.geometry.setFromPoints(pts);
    this.tags.misalign.position.copy(hub.clone().addScaledVector(wdir.clone().add(rdir).normalize(), 46));

    // ---- wind streaks
    this.flowGroup.rotation.y = psiWind;
    this.flowGroup.position.set(dwx * xTop * 0, 0, 0);
    const U = st.windSpeed, a = st.induction, n = this.nP, p = this.pPos, L = this.lPos, C = this.lCol;
    const tail = 0.9, twin = this.twinPass;
    for (let i = 0; i < n; i++) {
      let x = p[i * 3], y = p[i * 3 + 1], z = p[i * 3 + 2];
      const u = this.windAt(x, y, z, U, a) * (1 + 0.15 * Math.sin(this.t * 1.3 + i));
      x += u * dt; y += Math.sin(this.t * 0.7 + i * 1.7) * 0.4 * dt * (1 + st.ti * 10); z += Math.cos(this.t * 0.6 + i) * 0.4 * dt * (1 + st.ti * 10);
      if (x > 800) { this.spawn(i, false); x = p[i * 3]; y = p[i * 3 + 1]; z = p[i * 3 + 2]; }
      p[i * 3] = x; p[i * 3 + 1] = y; p[i * 3 + 2] = z;
      L[i * 6] = x; L[i * 6 + 1] = y; L[i * 6 + 2] = z; L[i * 6 + 3] = x - u * tail; L[i * 6 + 4] = y; L[i * 6 + 5] = z;
      heat(U > 0.5 ? u / (U * 1.25) * 0.8 : 0, c);
      const pr = 0.85, pg = 0.9, pb = 0.95;
      C[i * 6] = C[i * 6 + 3] = c.r; C[i * 6 + 1] = C[i * 6 + 4] = c.g; C[i * 6 + 2] = C[i * 6 + 5] = c.b;
      this._phys = [pr, pg, pb];
    }
    this.flow.geometry.attributes.position.needsUpdate = true; this.flow.geometry.attributes.color.needsUpdate = true;
    this.flow.visible = this.showFlow;
    // labels
    const T = this.tags;
    const set = (k, html) => { T[k].userData.inner.innerHTML = html; };
    set('anemo', `<b>Nacelle anemometer</b>${st.windSpeed.toFixed(1)} m/s · ${Math.round(st.windDir)}°`);
    set('bearing', `<b>HSS bearing</b>${st.bearingT.toFixed(1)} °C <span class="${st.bearingResid > 3 ? 'bad' : ''}">(${st.bearingResid >= 0 ? '+' : ''}${st.bearingResid.toFixed(1)} vs twin)</span>`);
    set('gen', `<b>Generator</b>${Math.round(st.genRpm)} rpm · ${(st.power / 1e6).toFixed(2)} MW`);
    set('pitch', `<b>Rotor</b>${st.rpm.toFixed(1)} rpm · pitch ${st.pitch.toFixed(1)}°`);
    set('tower', `<b>Tower top</b>${(st.towerX * 100).toFixed(0)} cm sway${this.exag > 1 ? ` <i>(shown ×${this.exag})</i>` : ''}`);
    set('base', `<b>Tower-base strain</b>${(Mbase / 1e6).toFixed(0)} MN·m`);
    set('yaw', `<b>Yaw</b>${Math.round(((90 - this.yawNow / D2R) % 360 + 360) % 360)}°${st.ice > 0.05 ? ` · <span class="ice">ice ${(st.ice * 100).toFixed(0)} %</span>` : ''}`);
    set('misalign', `yaw error ${this.misalign.toFixed(0)}°`);
    set('nac', `<b>Nacelle</b>${(st.power / 1e6).toFixed(2)} MW · bearing <span class="${st.bearingResid > 3 ? 'bad' : ''}">${st.bearingT.toFixed(0)} °C</span>`);
    const camD = this.camera.position.distanceTo(new THREE.Vector3(0, T5.hubH, 0));
    const isNear = camD < 150;
    for (const k in T) { const nr = T[k].userData.near; T[k].visible = this.mode !== 'physical' && (nr === 0 || (nr === 1 && isNear) || (nr === -1 && !isNear)); }
    T.misalign.visible = this.mode !== 'physical' && this.misalign > 1.5;
    for (const d of this.dots) { d.material.color.setHSL(0.52, 1, 0.6 + 0.2 * Math.sin(this.t * 4)); }
    // camera animation
    if (this.camGoal) {
      const k = 1 - Math.exp(-dt * 2.2);
      this.camera.position.lerp(this.camGoal.pos, k); this.controls.target.lerp(this.camGoal.tgt, k);
      if (this.camera.position.distanceTo(this.camGoal.pos) < 1) this.camGoal = null;
    }
    this.controls.update();
  }

  render() {
    const r = this.renderer, w = this.W, h = this.H;
    r.shadowMap.autoUpdate = true;
    if (this.mode === 'split') {
      r.setScissorTest(true);
      this.applyLook(false); r.setViewport(0, 0, w / 2, h); r.setScissor(0, 0, w / 2, h); r.render(this.scene, this.camera);
      r.shadowMap.autoUpdate = false;
      this.applyLook(true); r.setViewport(w / 2, 0, w / 2, h); r.setScissor(w / 2, 0, w / 2, h); r.render(this.scene, this.camera);
      r.setScissorTest(false);
      this.labels.render(this.scene, this.camera);
    } else {
      const twin = this.mode === 'twin';
      this.applyLook(twin); r.setViewport(0, 0, w, h); r.render(this.scene, this.camera);
      if (twin) this.labels.render(this.scene, this.camera);
    }
  }
}
