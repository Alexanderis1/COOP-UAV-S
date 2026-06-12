// scene.js — three.js scaffolding, static scene (terrain/buildings/assets/
// sensors/turrets/homes), environment-driven lighting & fog, picking.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ZONE_COLORS, SENSOR_COLOR, clamp, disposeGroup } from './util.js';
import { City } from './city.js';
import { makeChargingPad } from './models.js';

// map ENU (x east, y north, z up) -> three (x, y=up, z=-north)
export const W = (p) => new THREE.Vector3(p?.[0] || 0, p?.[2] || 0, -(p?.[1] || 0));
export const setW = (v, p) => v.set(p?.[0] || 0, p?.[2] || 0, -(p?.[1] || 0));

const NIGHT_BG = new THREE.Color(0x05070c);
const DAY_BG = new THREE.Color(0x1a2533);
const SUN_LOW = new THREE.Color(0xffc78f);   // warm low sun
const SUN_HIGH = new THREE.Color(0xfff4e2);  // near-white noon sun

const TURRET_STATE_COLOR = {
  idle: 0x6b7a8f, slewing: 0xffd166, tracking: 0xff9a3d,
  firing: 0xff5050, empty: 0x3a4254,
};

export class SceneView {
  constructor(container, onPick) {
    this.onPick = onPick;
    this.scene = new THREE.Scene();
    this.scene.background = NIGHT_BG.clone();
    this.scene.fog = new THREE.Fog(0x0b0e13, 9000, 26000);

    this.camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 1, 80000);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(innerWidth, innerHeight);
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    // filmic tone mapping: realistic light falloff at no measurable cost
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    container.appendChild(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.maxPolarAngle = Math.PI / 2.05;

    this.amb = new THREE.AmbientLight(0xffffff, 0.4);
    this.hemi = new THREE.HemisphereLight(0x9db8d6, 0x3a4047, 0.5);
    this.sun = new THREE.DirectionalLight(0xfff2d9, 0.9);
    this.sun.position.set(3000, 6000, 2000);
    this.scene.add(this.amb, this.hemi, this.sun);

    // gradient sky dome (HMI-MAP-006): redrawn only when daylight/precip move
    this._sky = makeSkyDome();
    this._skyKey = '';
    this.scene.add(this._sky.mesh);
    this._drawSky(0.35, 0);

    this.staticRoot = new THREE.Group();
    this.coverageGroup = new THREE.Group();   // sensor domes (layer)
    this.ringsGroup = new THREE.Group();      // turret range rings (layer)
    this.scene.add(this.staticRoot, this.coverageGroup, this.ringsGroup);

    this.turrets = new Map();   // id -> { group, yaw, barrel, body }
    this.stations = new Map();  // id -> ringMat (occupancy colour)
    this.pickables = [];        // meshes with userData.pick (static side: turrets)
    this.ground = null;
    this.zoneRaster = null;     // translucent civilian-presence overlay
    this.city = null;

    addEventListener('resize', () => {
      this.camera.aspect = innerWidth / innerHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(innerWidth, innerHeight);
    });

    // click-to-pick (ignore drags)
    this._ray = new THREE.Raycaster();
    this._down = null;
    const dom = this.renderer.domElement;
    dom.addEventListener('pointerdown', (e) => { this._down = [e.clientX, e.clientY]; });
    dom.addEventListener('pointerup', (e) => {
      if (!this._down) return;
      const moved = Math.hypot(e.clientX - this._down[0], e.clientY - this._down[1]);
      this._down = null;
      if (moved > 5) return;
      this._pick(e);
    });
  }

  _pick(e) {
    const ndc = new THREE.Vector2(
      (e.clientX / innerWidth) * 2 - 1, -(e.clientY / innerHeight) * 2 + 1);
    this._ray.setFromCamera(ndc, this.camera);
    const all = this.pickables.concat(this.extraPickables ? this.extraPickables() : []);
    const hits = this._ray.intersectObjects(all, true);
    for (const h of hits) {
      let o = h.object;
      while (o && !o.userData.pick) o = o.parent;
      if (o && o.userData.pick) { this.onPick?.(o.userData.pick); return; }
    }
    this.onPick?.(null);
  }

  // ----- static scene -------------------------------------------------------
  buildStatic(sc) {
    // wipe previous — every static resource is rebuilt per scene, so free
    // the GPU side too (textures/materials/geometries leak across runs
    // under Group.clear() alone)
    for (const g of [this.staticRoot, this.coverageGroup, this.ringsGroup]) {
      disposeGroup(g);
      g.clear();
    }
    this.turrets.clear();
    this.stations.clear();
    this.city = null;
    this.ground = null;
    this.zoneRaster = null;
    this.pickables.length = 0;
    if (!sc || !sc.bounds) return;

    const [x0, y0, x1, y1] = sc.bounds;

    // terrain: streets and lots baked from the building fabric, soil/scrub
    // noise outside the urban area
    this.ground = new THREE.Mesh(
      new THREE.PlaneGeometry(x1 - x0, y1 - y0),
      new THREE.MeshLambertMaterial({ map: terrainTexture(sc) }));
    this.ground.rotation.x = -Math.PI / 2;
    this.ground.position.set((x0 + x1) / 2, -1, -(y0 + y1) / 2);
    this.staticRoot.add(this.ground);

    // zone raster (SIM-ENV-005): translucent overlay above the terrain so
    // the civilian-presence map stays readable over realistic ground
    if (sc.grid && sc.grid.length) {
      const ny = sc.grid.length, nx = sc.grid[0].length;
      const cnv = document.createElement('canvas');
      cnv.width = nx; cnv.height = ny;
      const ctx = cnv.getContext('2d');
      for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
        ctx.fillStyle = ZONE_COLORS[sc.grid[j][i]] || ZONE_COLORS[1];
        ctx.fillRect(i, ny - 1 - j, 1, 1);            // grid row j = south -> north
      }
      const tex = new THREE.CanvasTexture(cnv);
      tex.magFilter = THREE.NearestFilter;
      tex.colorSpace = THREE.SRGBColorSpace;
      this.zoneRaster = new THREE.Mesh(
        new THREE.PlaneGeometry(x1 - x0, y1 - y0),
        new THREE.MeshBasicMaterial({
          map: tex, transparent: true, opacity: 0.42, depthWrite: false,
        }));
      this.zoneRaster.rotation.x = -Math.PI / 2;
      this.zoneRaster.position.set((x0 + x1) / 2, -0.85, -(y0 + y1) / 2);
      this.staticRoot.add(this.zoneRaster);
    }

    // city fabric: kind-textured instanced buildings, parks/water, zone
    // roof tints and zone border outlines (HMI-MAP-007)
    this.city = new City(this.staticRoot, sc);

    // protected assets
    const ageo = new THREE.CylinderGeometry(28, 28, 60, 16);
    const amat = new THREE.MeshLambertMaterial({ color: 0x4fc3f7, emissive: 0x10405c });
    for (const a of sc.assets || []) {
      const m = new THREE.Mesh(ageo, amat);
      m.position.copy(W(a.pos)); m.position.y = 30;
      this.staticRoot.add(m);
      this.staticRoot.add(makeGroundRing(W(a.pos), 90, 0x4fc3f7, 0.35));
    }

    // sensors: mast + translucent coverage dome + ground circle
    for (const s of sc.sensors || []) {
      const col = SENSOR_COLOR[s.type] ?? 0x4fc3f7;
      const mast = new THREE.Mesh(
        new THREE.CylinderGeometry(8, 12, 90, 8),
        new THREE.MeshLambertMaterial({ color: col, emissive: col, emissiveIntensity: 0.25 }));
      mast.position.copy(W(s.pos)); mast.position.y += 45;
      this.staticRoot.add(mast);

      const r = Math.max(1, s.range || 0);
      const dome = new THREE.Mesh(
        new THREE.SphereGeometry(r, 28, 14, 0, Math.PI * 2, 0, Math.PI / 2),
        new THREE.MeshBasicMaterial({
          color: col, transparent: true, opacity: 0.05,
          depthWrite: false, side: THREE.DoubleSide,
        }));
      dome.position.copy(W(s.pos)); dome.position.y = 0;
      this.coverageGroup.add(dome);
      this.coverageGroup.add(makeGroundRing(W(s.pos), r, col, 0.3));
    }

    // turrets: base box + yaw pivot + barrel, range ring
    for (const t of sc.turrets || []) {
      const group = new THREE.Group();
      group.position.copy(W(t.pos));
      const plinth = new THREE.Mesh(
        new THREE.CylinderGeometry(30, 34, 8, 16),
        new THREE.MeshLambertMaterial({ color: 0x474e58 }));
      plinth.position.y = 4;
      group.add(plinth);
      const body = new THREE.Mesh(
        new THREE.BoxGeometry(40, 26, 40),
        new THREE.MeshLambertMaterial({ color: 0x6b7a8f }));
      body.position.y = 13;
      const yaw = new THREE.Group(); yaw.position.y = 30;
      const pitch = new THREE.Group();
      const barrel = new THREE.Mesh(
        new THREE.CylinderGeometry(4, 5, 70, 8),
        new THREE.MeshLambertMaterial({ color: 0x9aa4b0 }));
      barrel.rotation.z = -Math.PI / 2;     // along +x of pitch group
      barrel.position.x = 35;
      pitch.add(barrel);
      yaw.add(pitch);
      group.add(body, yaw);
      group.userData.pick = { kind: 'turret', id: t.id };
      this.staticRoot.add(group);
      this.pickables.push(group);
      this.ringsGroup.add(makeGroundRing(W(t.pos), Math.max(1, t.range || 0), 0xff9a3d, 0.35));
      this.turrets.set(t.id, { group, yaw, pitch, body });
    }

    // charging stations (PHY-CHG-001): authoritative when present;
    // legacy recordings fall back to abstract home pads.
    if ((sc.stations || []).length) {
      for (const st of sc.stations) {
        const { group, ringMat } = makeChargingPad();
        group.position.copy(W(st.pos));
        // pad model is ~3 m; keep it visible at city scale
        group.scale.setScalar(8);
        group.userData.pick = { kind: 'station', id: st.id };
        this.staticRoot.add(group);
        this.pickables.push(group);          // station inspector (HMI-MAP-004)
        this.staticRoot.add(makeGroundRing(
          new THREE.Vector3(group.position.x, 0, group.position.z), 46, 0x39d2ff, 0.25));
        this.stations.set(st.id, ringMat);
      }
    } else {
      const pgeo = new THREE.CylinderGeometry(34, 34, 4, 20);
      const pmat = new THREE.MeshLambertMaterial({ color: 0x1f6f9c, emissive: 0x0a2a3c });
      for (const h of sc.homes || []) {
        const pad = new THREE.Mesh(pgeo, pmat);
        pad.position.copy(W(h.pos)); pad.position.y = 2;
        this.staticRoot.add(pad);
        this.staticRoot.add(makeGroundRing(W(h.pos), 46, 0x39d2ff, 0.3));
      }
    }

    // camera framing
    const cx = (x0 + x1) / 2, cz = -(y0 + y1) / 2;
    const span = Math.max(x1 - x0, y1 - y0);
    this.camera.position.set(cx, span * 0.38, cz + span * 0.46);
    this.controls.target.set(cx, 0, cz);
  }

  // dynamic turret state from frame.turrets
  updateTurret(tu) {
    const rec = this.turrets.get(tu.id);
    if (!rec) return;
    // az: compass degrees (0 = north, cw). barrel along +x of pitch group:
    // yaw theta about Y st direction = (sin az, -cos az) in three xz.
    rec.yaw.rotation.y = THREE.MathUtils.degToRad(90 - (tu.az || 0));
    rec.pitch.rotation.z = THREE.MathUtils.degToRad(clamp(tu.el || 0, -10, 89));
    const col = TURRET_STATE_COLOR[tu.state] ?? 0x6b7a8f;
    rec.body.material.color.setHex(col);
  }

  setLayer(name, on) {
    if (name === 'coverage') this.coverageGroup.visible = on;
    else if (name === 'rings') this.ringsGroup.visible = on;
    else if (name === 'grid' && this.zoneRaster) this.zoneRaster.visible = on;
    else if (name === 'zoneborders') this.city?.setBorders(on);
    else if (name === 'rooftints') this.city?.setRoofTints(on);
  }

  // charging-pad occupancy from frame.stations
  updateStation(st) {
    const ringMat = this.stations.get(st.id);
    if (!ringMat) return;
    ringMat.color.setHex(st.occupied > 0 ? 0xff9a3d : 0x49c97c);
  }

  // SRS HMI-MAP-006: lighting, sky and fog respond to frame.env
  applyEnv(env) {
    if (!env) return;
    const d = clamp(env.daylight ?? 0.35, 0, 1);
    const p = clamp(env.precip ?? 0, 0, 1);
    this.amb.intensity = 0.16 + 0.5 * d * (1 - 0.3 * p);
    this.hemi.intensity = 0.22 + 0.55 * d * (1 - 0.35 * p);
    this.sun.intensity = 0.12 + 1.15 * d * (1 - 0.45 * p);
    this.sun.color.copy(SUN_LOW).lerp(SUN_HIGH, d);
    this._drawSky(d, p);
    const bg = NIGHT_BG.clone().lerp(DAY_BG, d);
    this.scene.background.copy(bg);
    const f = clamp(env.fog ?? 0, 0, 1);
    this.scene.fog.color.copy(this._sky.horizon);
    this.scene.fog.near = 9000 * (1 - f) + 220 * f;
    this.scene.fog.far = 26000 * (1 - f) + 2800 * f;
    this.city?.applyDaylight(d);     // windows glow at night (HMI-MAP-007)
  }

  // Vertical sky gradient with a warm dusk band; cheap 2x256 canvas, only
  // redrawn when daylight/precip actually move.
  _drawSky(d, p) {
    const key = d.toFixed(2) + '|' + p.toFixed(2);
    if (key === this._skyKey) return;
    this._skyKey = key;
    const grey = new THREE.Color(0x6a737b);
    const zen = new THREE.Color(0x070d1c).lerp(new THREE.Color(0x6fa3d8), d).lerp(grey, p * 0.45);
    const hor = new THREE.Color(0x141b2a).lerp(new THREE.Color(0xcfdde8), d).lerp(grey, p * 0.45);
    const gnd = new THREE.Color(0x0a0d12).lerp(new THREE.Color(0x6e7a80), d).lerp(grey, p * 0.45);
    // golden-hour glow around low sun (the default urban_raid dusk)
    const warm = Math.exp(-(((d - 0.3) / 0.14) ** 2));
    hor.lerp(new THREE.Color(0xe09a5a), warm * 0.45);
    this._sky.horizon = hor;
    const ctx = this._sky.cnv.getContext('2d');
    const g = ctx.createLinearGradient(0, 0, 0, 256);
    g.addColorStop(0, '#' + zen.getHexString());
    g.addColorStop(0.46, '#' + zen.clone().lerp(hor, 0.7).getHexString());
    g.addColorStop(0.52, '#' + hor.getHexString());
    g.addColorStop(0.56, '#' + gnd.getHexString());
    g.addColorStop(1, '#' + gnd.clone().multiplyScalar(0.6).getHexString());
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, 2, 256);
    this._sky.tex.needsUpdate = true;
  }

  focus(pos3) {
    if (!pos3) return;
    const off = this.camera.position.clone().sub(this.controls.target);
    if (off.length() > 6500) off.setLength(6500);
    this.controls.target.copy(pos3).setY(0);
    this.camera.position.copy(this.controls.target).add(off);
  }

  start(beforeRender) {
    this.renderer.setAnimationLoop(() => {
      const now = performance.now();
      beforeRender?.(now);
      this.city?.tick(now);          // river ripple drift
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }
}

// Inward-facing sphere with a vertical gradient texture; fog-exempt so the
// horizon stays crisp.
function makeSkyDome() {
  const cnv = document.createElement('canvas');
  cnv.width = 2; cnv.height = 256;
  const tex = new THREE.CanvasTexture(cnv);
  tex.colorSpace = THREE.SRGBColorSpace;
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(42000, 24, 12),
    new THREE.MeshBasicMaterial({
      map: tex, side: THREE.BackSide, fog: false, depthWrite: false,
    }));
  mesh.renderOrder = -1000;
  mesh.frustumCulled = false;
  return { mesh, cnv, tex, horizon: new THREE.Color(0x141b2a) };
}

// Street/lot ground plate generated from the building fabric: asphalt
// corridors emerge as the gaps between expanded lots, scrubland outside.
// One 2048^2 canvas built once per scene (canvas row 0 = north edge).
function terrainTexture(sc) {
  const [x0, y0, x1, y1] = sc.bounds;
  const S = 2048;
  const sx = S / (x1 - x0), sy = S / (y1 - y0);
  const px = (x) => (x - x0) * sx;
  const py = (y) => (y1 - y) * sy;
  const cnv = document.createElement('canvas');
  cnv.width = cnv.height = S;
  const ctx = cnv.getContext('2d');
  const frand = (i, s) =>
    (((Math.imul(i + 1, 2654435761) ^ Math.imul(s + 7, 40503)) >>> 9) % 65536) / 65536;

  // scrub/soil base with deterministic mottling
  ctx.fillStyle = '#232c23';
  ctx.fillRect(0, 0, S, S);
  for (let i = 0; i < 9000; i++) {
    ctx.globalAlpha = 0.2 + 0.3 * frand(i, 1);
    ctx.fillStyle = ['#1d251d', '#2a352a', '#262e22', '#2e3a2e'][Math.floor(frand(i, 2) * 4)];
    const r = 2 + frand(i, 3) * 7;
    ctx.fillRect(frand(i, 4) * S, frand(i, 5) * S, r, r);
  }
  ctx.globalAlpha = 1;

  const buildings = (sc.buildings || []).filter(
    (b) => b.kind !== 'water' && b.rect);
  // asphalt street corridors: union of lot rects expanded by ~half a street
  ctx.fillStyle = '#24272c';
  for (const b of buildings) {
    const [bx0, by0, bx1, by1] = b.rect;
    ctx.fillRect(px(bx0 - 55), py(by1 + 55),
      (bx1 - bx0 + 110) * sx, (by1 - by0 + 110) * sy);
  }
  // lots: slightly lighter pavement so blocks read against the streets
  ctx.fillStyle = '#2e3238';
  for (const b of buildings) {
    if (!(b.height > 0)) continue;
    const [bx0, by0, bx1, by1] = b.rect;
    ctx.fillRect(px(bx0 - 8), py(by1 + 8),
      (bx1 - bx0 + 16) * sx, (by1 - by0 + 16) * sy);
  }
  // unifying grime pass
  for (let i = 0; i < 4000; i++) {
    ctx.globalAlpha = 0.05;
    ctx.fillStyle = frand(i, 6) < 0.5 ? '#000' : '#fff';
    ctx.fillRect(frand(i, 7) * S, frand(i, 8) * S, 2 + frand(i, 9) * 4, 2 + frand(i, 9) * 4);
  }
  ctx.globalAlpha = 1;

  const tex = new THREE.CanvasTexture(cnv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

export function makeGroundRing(center, radius, color, opacity = 0.35) {
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(Math.max(0.5, radius - radius * 0.012 - 2), radius, 64),
    new THREE.MeshBasicMaterial({
      color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false,
    }));
  ring.rotation.x = -Math.PI / 2;
  ring.position.set(center.x, 1.5, center.z);
  return ring;
}
