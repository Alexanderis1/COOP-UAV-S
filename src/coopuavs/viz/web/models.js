// models.js — procedural low-poly airframes at TRUE 1:1 metre scale
// (SRS HMI-MAP-007). One factory per airframe; each returns
// { group, accent, mats } where `accent` is a per-instance material the
// caller recolours for class/mode semantics and `mats` is every cloned
// material (for opacity fades). Models are built nose along +Z, up +Y, so
// callers orient with group.lookAt(worldPos + velocityDir).
//
// Every model carries an invisible-but-raycastable pick proxy sphere so a
// 0.3 m FPV stays clickable at any magnification.
import * as THREE from 'three';

// Shared geometries (never disposed; materials are per-instance clones).
const G = {
  proxy: new THREE.SphereGeometry(2.5, 8, 6),
  prop: new THREE.CylinderGeometry(0.13, 0.13, 0.012, 10),
  propBig: new THREE.CylinderGeometry(0.45, 0.45, 0.03, 12),
  ball: new THREE.SphereGeometry(0.12, 10, 8),
  chunk: new THREE.DodecahedronGeometry(0.2, 0),
};

const BODY_DARK = 0x3c4250;
const BODY_GREY = 0x6b7280;
const BODY_LIGHT = 0x9aa4b0;

function baseMats(bodyColor, accentColor = 0xffffff) {
  const body = new THREE.MeshLambertMaterial({ color: bodyColor, transparent: true });
  const accent = new THREE.MeshLambertMaterial({
    color: accentColor, emissive: accentColor, emissiveIntensity: 0.55, transparent: true,
  });
  return { body, accent, mats: [body, accent] };
}

function finish(group, parts) {
  const proxy = new THREE.Mesh(G.proxy, new THREE.MeshBasicMaterial({ visible: false }));
  group.add(proxy);
  return { group, accent: parts.accent, mats: parts.mats };
}

function box(w, h, d, mat, x = 0, y = 0, z = 0, ry = 0) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  if (ry) m.rotation.y = ry;
  return m;
}

// ---- threat airframes -------------------------------------------------------

// Shahed-136-type strategic OWA: 3.5 m delta span, 2.6 m long.
export function makeShahed() {
  const p = baseMats(BODY_DARK);
  const g = new THREE.Group();
  // fuselage: flattened blended body
  const fus = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.32, 2.4, 8), p.body);
  fus.rotation.x = Math.PI / 2;
  g.add(fus);
  // delta wing (triangular extrusion, thin)
  const shape = new THREE.Shape();
  shape.moveTo(0, 1.1); shape.lineTo(-1.75, -1.2); shape.lineTo(1.75, -1.2); shape.closePath();
  const wing = new THREE.Mesh(
    new THREE.ExtrudeGeometry(shape, { depth: 0.06, bevelEnabled: false }), p.body);
  wing.rotation.x = -Math.PI / 2;
  wing.position.y = -0.03;
  g.add(wing);
  // canted wingtip fins
  for (const s of [-1, 1]) {
    const fin = box(0.04, 0.4, 0.5, p.body, s * 1.7, 0.12, -1.0);
    fin.rotation.z = -s * 0.35;
    g.add(fin);
  }
  // nose accent + pusher prop disc at the tail
  const nose = new THREE.Mesh(G.ball, p.accent); nose.position.z = 1.25; g.add(nose);
  const prop = new THREE.Mesh(G.propBig, p.body);
  prop.rotation.x = Math.PI / 2; prop.position.z = -1.3;
  g.add(prop);
  return finish(g, p);
}

// Geran-3-type jet OWA: slimmer, swept wings, ~3 m.
export function makeJet() {
  const p = baseMats(BODY_GREY);
  const g = new THREE.Group();
  const fus = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.24, 2.9, 8), p.body);
  fus.rotation.x = Math.PI / 2;
  g.add(fus);
  for (const s of [-1, 1]) {
    const wing = box(1.3, 0.05, 0.55, p.body, s * 0.75, 0, -0.45);
    wing.rotation.y = s * 0.55;            // sweep back
    g.add(wing);
  }
  g.add(box(0.05, 0.5, 0.45, p.body, 0, 0.3, -1.25));   // vertical tail
  g.add(box(0.26, 0.2, 0.5, p.body, 0, -0.2, -1.0));    // intake pod
  const nose = new THREE.Mesh(G.ball, p.accent); nose.position.z = 1.5; g.add(nose);
  return finish(g, p);
}

// FPV kamikaze quad: 0.30 m across.
export function makeFPV() {
  const p = baseMats(BODY_DARK);
  const g = new THREE.Group();
  g.add(box(0.11, 0.05, 0.13, p.body));
  for (const [sx, sz] of [[-1, -1], [-1, 1], [1, -1], [1, 1]]) {
    g.add(box(0.16, 0.015, 0.025, p.body, sx * 0.08, 0, sz * 0.08, sx * sz > 0 ? 0.785 : -0.785));
    const prop = new THREE.Mesh(G.prop, p.body);
    prop.scale.setScalar(0.55);
    prop.position.set(sx * 0.14, 0.02, sz * 0.14);
    g.add(prop);
  }
  const cam = new THREE.Mesh(G.ball, p.accent);
  cam.scale.setScalar(0.45); cam.position.set(0, 0.02, 0.09);
  g.add(cam);
  return finish(g, p);
}

// Lancet-type loitering munition: ~1.2 m, X-form twin wings.
export function makeLoiterer() {
  const p = baseMats(BODY_GREY);
  const g = new THREE.Group();
  const fus = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.1, 1.15, 8), p.body);
  fus.rotation.x = Math.PI / 2;
  g.add(fus);
  for (const roll of [0.785, -0.785]) {                 // two X-wing pairs
    for (const off of [0.25, -0.35]) {
      const wing = box(1.0, 0.03, 0.16, p.body, 0, 0, off);
      wing.rotation.z = roll;
      g.add(wing);
    }
  }
  const nose = new THREE.Mesh(G.ball, p.accent); nose.scale.setScalar(0.7);
  nose.position.z = 0.6; g.add(nose);
  return finish(g, p);
}

// ---- friendly airframes ------------------------------------------------------

function quadFrame(p, size) {
  const g = new THREE.Group();
  g.add(box(size * 0.34, size * 0.14, size * 0.4, p.body));
  for (const [sx, sz] of [[-1, -1], [-1, 1], [1, -1], [1, 1]]) {
    g.add(box(size * 0.5, size * 0.04, size * 0.07, p.body,
      sx * size * 0.26, 0, sz * size * 0.26, sx * sz > 0 ? 0.785 : -0.785));
    const prop = new THREE.Mesh(G.prop, p.body);
    prop.scale.setScalar(size * 1.6);
    prop.position.set(sx * size * 0.42, size * 0.08, sz * size * 0.42);
    g.add(prop);
  }
  // mode beacon on top
  const beacon = new THREE.Mesh(G.ball, p.accent);
  beacon.scale.setScalar(size * 0.7);
  beacon.position.y = size * 0.16;
  g.add(beacon);
  return g;
}

// Interceptor quad (~1.0 m): gun barrel vs slung net frame.
export function makeInterceptor(weapon) {
  const p = baseMats(BODY_LIGHT);
  const g = quadFrame(p, 1.0);
  if (weapon === 'net') {
    // square net frame slung under the belly
    for (const [w, d, x, z] of [[0.7, 0.05, 0, 0.33], [0.7, 0.05, 0, -0.33],
                                [0.05, 0.7, 0.33, 0], [0.05, 0.7, -0.33, 0]]) {
      g.add(box(w, 0.03, d, p.accent, x, -0.18, z));
    }
  } else {
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.045, 0.55, 8), p.accent);
    barrel.rotation.x = Math.PI / 2;
    barrel.position.set(0, -0.12, 0.3);
    g.add(barrel);
  }
  return finish(g, p);
}

// Sentinel quad (~1.0 m): white livery, gimballed sensor ball, no effector.
export function makeSentinel() {
  const p = baseMats(0xd8dee8);
  const g = quadFrame(p, 1.0);
  const ball = new THREE.Mesh(new THREE.SphereGeometry(0.16, 12, 10),
    new THREE.MeshPhongMaterial({ color: 0x18222e, shininess: 90, transparent: true }));
  p.mats.push(ball.material);
  ball.position.set(0, -0.18, 0.12);
  g.add(ball);
  return finish(g, p);
}

// ---- debris + infrastructure ---------------------------------------------------

export function makeDebrisChunk() {
  const mat = new THREE.MeshLambertMaterial({ color: 0x2a2e36, transparent: true });
  const g = new THREE.Group();
  const m = new THREE.Mesh(G.chunk, mat);
  m.scale.set(1.4, 0.8, 1.1);
  g.add(m);
  return { group: g, accent: mat, mats: [mat] };
}

// Charging pad: 3 m disc + emissive occupancy ring + lightning icon.
export function makeChargingPad() {
  const g = new THREE.Group();
  const cnv = document.createElement('canvas');
  cnv.width = cnv.height = 64;
  const ctx = cnv.getContext('2d');
  ctx.fillStyle = '#1c2733'; ctx.fillRect(0, 0, 64, 64);
  ctx.strokeStyle = '#39d2ff'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(32, 32, 26, 0, Math.PI * 2); ctx.stroke();
  ctx.fillStyle = '#ffd166';
  ctx.beginPath();
  ctx.moveTo(36, 12); ctx.lineTo(22, 36); ctx.lineTo(31, 36);
  ctx.lineTo(28, 52); ctx.lineTo(43, 28); ctx.lineTo(33, 28); ctx.closePath();
  ctx.fill();
  const tex = new THREE.CanvasTexture(cnv);
  const pad = new THREE.Mesh(
    new THREE.CylinderGeometry(1.6, 1.8, 0.3, 20),
    [new THREE.MeshLambertMaterial({ color: 0x2a3848 }),
     new THREE.MeshLambertMaterial({ map: tex }),
     new THREE.MeshLambertMaterial({ color: 0x1c2733 })]);
  pad.position.y = 0.15;
  g.add(pad);
  const ringMat = new THREE.MeshBasicMaterial({
    color: 0x49c97c, transparent: true, opacity: 0.85, side: THREE.DoubleSide, depthWrite: false,
  });
  const ring = new THREE.Mesh(new THREE.RingGeometry(1.9, 2.3, 24), ringMat);
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.32;
  g.add(ring);
  return { group: g, ringMat };
}

// ---- registry -------------------------------------------------------------------

export const MODEL_FOR_CLASS = {
  owa_strategic: makeShahed,
  owa_jet: makeJet,
  fpv: makeFPV,
  loitering: makeLoiterer,
  decoy: makeShahed,       // decoys mimic the Shahed airframe by design
};

export function makeThreatModel(cls) {
  return (MODEL_FOR_CLASS[cls] || makeShahed)();
}

export function makeUavModel(kind, effector) {
  return kind === 'sentinel' ? makeSentinel() : makeInterceptor(effector || 'projectile');
}

// Grey wireframe variant for evaluation ghosts (HMI-EVAL-001).
export function makeGhostModel(cls) {
  const { group } = makeThreatModel(cls);
  const wire = new THREE.MeshBasicMaterial({
    color: 0x9aa4b0, wireframe: true, transparent: true, opacity: 0.4,
  });
  group.traverse((o) => {
    if (o.isMesh && o.material?.visible !== false) o.material = wire;
  });
  return { group, accent: wire, mats: [wire] };
}

export function disposeModel(model) {
  for (const m of model.mats || []) m.dispose?.();
}
