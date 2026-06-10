// entities.js — pooled dynamic 3D entities: fused tracks, friendly UAVs
// (interceptors + sentinels), live debris, wrecks, strays, predicted
// impacts, weapon tracers, kill markers, trails, labels, and the
// evaluation ghost overlay (SRS HMI-EVAL-001/002, HMI-MAP-007/008).
//
// Airframes are true-scale procedural models (models.js) with zoom-aware
// magnification: scale = max(1, magFactor · cameraDistance · K) — 1:1 when
// viewed close, visible from across the 12 km map when zoomed out.
// Meshes are indexed by id and reused across frames; never recreated.
import * as THREE from 'three';
import { setW, W } from './scene.js';
import {
  CLS_COLOR, MODE_COLOR, WEAPON_COLOR, ZONE_TINT, argmaxClass, clamp,
} from './util.js';
import {
  disposeModel, makeDebrisChunk, makeGhostModel, makeThreatModel, makeUavModel,
} from './models.js';

const TRUTH_GEO = new THREE.OctahedronGeometry(11, 0);
const WRECK_NET_GEO = new THREE.OctahedronGeometry(15, 0);
const WRECK_PROJ_GEO = new THREE.TetrahedronGeometry(18, 0);
const STRAY_GEO = new THREE.ConeGeometry(9, 22, 4);
const IMPACT_GEO = new THREE.RingGeometry(34, 46, 32);
const DEBRIS_IMPACT_GEO = new THREE.RingGeometry(20, 30, 24);
const FLASH_GEO = new THREE.SphereGeometry(40, 12, 8);
const TRACER_HEAD_GEO = new THREE.SphereGeometry(6, 8, 6);
const KILL_RING_GEO = new THREE.RingGeometry(30, 38, 28);

const TRAIL_MAX = 140;
// Zoom-aware magnification: scale = max(1, mag · cameraDistance · MAG_K).
// 1:1 inside ~250 m at mag 1; ×24 at the default 6 km framing.
const MAG_K = 0.004;
// Label sprites counter their parent's scale and track camera distance so
// they stay readable at any zoom.
const LABEL_K = 0.047;

class Trail {
  constructor(color) {
    this.buf = new Float32Array(TRAIL_MAX * 3);
    this.count = 0;
    this.geo = new THREE.BufferGeometry();
    this.attr = new THREE.BufferAttribute(this.buf, 3);
    this.attr.setUsage(THREE.DynamicDrawUsage);
    this.geo.setAttribute('position', this.attr);
    this.geo.setDrawRange(0, 0);
    this.line = new THREE.Line(this.geo,
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.4 }));
    this.line.frustumCulled = false;
  }
  push(v) {
    if (this.count === TRAIL_MAX) {
      this.buf.copyWithin(0, 3);
      this.count--;
    }
    const i = this.count * 3;
    this.buf[i] = v.x; this.buf[i + 1] = v.y; this.buf[i + 2] = v.z;
    this.count++;
    this.attr.needsUpdate = true;
    this.geo.setDrawRange(0, this.count);
  }
  setColor(c) { this.line.material.color.setHex(c); }
}

function makeLabel() {
  const cnv = document.createElement('canvas');
  cnv.width = 256; cnv.height = 48;
  const tex = new THREE.CanvasTexture(cnv);
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({
    map: tex, transparent: true, depthTest: false,
  }));
  spr.scale.set(560, 105, 1);
  spr.center.set(0.5, 0);
  return { spr, cnv, ctx: cnv.getContext('2d'), tex, text: null };
}
function setLabel(l, text, color) {
  if (l.text === text) return;
  l.text = text;
  l.ctx.clearRect(0, 0, 256, 48);
  l.ctx.font = 'bold 22px ui-monospace, monospace';
  l.ctx.fillStyle = color || '#cfd8e3';
  l.ctx.shadowColor = '#000'; l.ctx.shadowBlur = 4;
  l.ctx.fillText(text, 6, 32);
  l.tex.needsUpdate = true;
}

// orient a nose-+Z model group along an ENU velocity
const _dir = new THREE.Vector3();
const _aim = new THREE.Vector3();
function orient(group, vel) {
  const v = vel || [0, 0, 0];
  if (Math.hypot(v[0], v[1], v[2]) < 1) return;
  _dir.set(v[0], v[2], -v[1]).normalize();
  _aim.copy(group.position).add(_dir);
  group.lookAt(_aim);
}

export class Entities {
  constructor(view, { onAcquired } = {}) {
    this.view = view;
    this.onAcquired = onAcquired;
    this.root = new THREE.Group();
    view.scene.add(this.root);
    this.groups = {};
    for (const g of ['tracks', 'uavs', 'ghosts', 'truth', 'debris', 'wrecks',
                     'strays', 'impacts', 'trails', 'fx', 'tracers'])
      this.groups[g] = new THREE.Group(), this.root.add(this.groups[g]);

    this.tracks = new Map();   // id -> rec
    this.uavs = new Map();
    this.ghosts = new Map();
    this.truthMarks = new Map();
    this.debris = new Map();   // debris id -> rec
    this.wrecks = new Map();   // index -> rec
    this.strays = new Map();
    this.flashes = [];
    this.tracers = [];
    this.killMarks = [];
    this.ghostSeen = new Map();  // enemy id -> last acquired bool

    this.labelsVisible = true;
    this.recvWall = 0;
    this.effSpeed = 0;
    this.extrapolate = true;
    this.magFactor = 1;        // model magnification slider (0 = strict 1:1)

    // selection ring
    this.selection = null;       // {kind, id}
    this.selRing = new THREE.Mesh(
      new THREE.RingGeometry(40, 48, 36),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.8, side: THREE.DoubleSide, depthTest: false }));
    this.selRing.rotation.x = -Math.PI / 2;
    this.selRing.visible = false;
    this.root.add(this.selRing);

    view.extraPickables = () =>
      [this.groups.tracks, this.groups.uavs, this.groups.ghosts, this.groups.debris];
  }

  reset() {
    for (const m of [this.tracks, this.uavs, this.ghosts, this.truthMarks,
                     this.debris, this.wrecks, this.strays])
      m.clear();
    for (const k in this.groups) this.groups[k].clear();
    this.flashes.length = 0;
    this.tracers.length = 0;
    this.killMarks.length = 0;
    this.ghostSeen.clear();
    this.selRing.visible = false;
    this.selection = null;
  }

  setLayer(name, on) {
    if (name === 'impacts') this.groups.impacts.visible = on;
    else if (name === 'ghosts') {
      this.groups.ghosts.visible = on;
      this.groups.truth.visible = on;
    } else if (name === 'trails') this.groups.trails.visible = on;
    else if (name === 'debris') this.groups.debris.visible = on;
    else if (name === 'tracers') this.groups.tracers.visible = on;
    else if (name === 'labels') {
      this.labelsVisible = on;
      for (const m of [this.tracks, this.uavs])
        for (const rec of m.values()) if (rec.label) rec.label.spr.visible = on;
    }
  }

  setMagnification(f) { this.magFactor = f; }

  select(sel) { this.selection = sel; }

  entityPos(kind, id) {
    const map = {
      track: this.tracks, uav: this.uavs, ghost: this.ghosts,
      truth: this.truthMarks, debris: this.debris,
    }[kind];
    if (kind === 'turret') {
      const t = this.view.turrets.get(id);
      return t ? t.group.position.clone() : null;
    }
    const rec = map?.get(id);
    return rec ? rec.mesh.position.clone() : null;
  }

  // swap a record's airframe model in place (class belief changed)
  _swapModel(rec, group, make) {
    const old = rec.model;
    const newModel = make();
    newModel.group.position.copy(old.group.position);
    newModel.group.quaternion.copy(old.group.quaternion);
    newModel.group.scale.copy(old.group.scale);
    newModel.group.userData.pick = old.group.userData.pick;
    if (rec.label) newModel.group.add(rec.label.spr);
    group.remove(old.group);
    disposeModel(old);
    group.add(newModel.group);
    rec.model = newModel;
    rec.mesh = newModel.group;
  }

  // --------------------------------------------------------------- frame
  applyFrame(f) {
    this.recvWall = performance.now();
    const run = f.run || {};
    this.effSpeed = (this.extrapolate && run.status === 'running') ? (run.speed || 1) : 0;

    // ---- fused tracks (true-scale threat models, class-swapped on belief)
    const liveT = new Set();
    for (const t of f.tracks || []) {
      if (t.id == null) continue;
      liveT.add(t.id);
      let rec = this.tracks.get(t.id);
      const cls = argmaxClass(t.belief);
      if (!rec) {
        const model = makeThreatModel(cls);
        model.group.userData.pick = { kind: 'track', id: t.id };
        const label = makeLabel();
        label.spr.position.y = 2.6;
        label.spr.visible = this.labelsVisible;
        model.group.add(label.spr);
        const trail = new Trail(0xffe14d);
        this.groups.trails.add(trail.line);
        const impactRing = new THREE.Mesh(IMPACT_GEO, new THREE.MeshBasicMaterial({
          color: 0xff4040, transparent: true, opacity: 0.65, side: THREE.DoubleSide, depthWrite: false,
        }));
        impactRing.rotation.x = -Math.PI / 2;
        const lineGeo = new THREE.BufferGeometry().setFromPoints(
          [new THREE.Vector3(), new THREE.Vector3()]);
        const impactLine = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({
          color: 0xff4040, transparent: true, opacity: 0.45,
        }));
        impactLine.frustumCulled = false;
        this.groups.impacts.add(impactRing, impactLine);
        rec = { model, mesh: model.group, cls, label, trail, impactRing, impactLine,
                base: { pos: [0, 0, 0], vel: [0, 0, 0] }, data: null };
        this.groups.tracks.add(model.group);
        this.tracks.set(t.id, rec);
      } else if (rec.cls !== cls) {
        // classification swung to another airframe: swap the model
        rec.cls = cls;
        this._swapModel(rec, this.groups.tracks, () => makeThreatModel(cls));
      }
      rec.data = t;
      const col = CLS_COLOR[cls] ?? 0xffe14d;
      rec.model.accent.color.setHex(col);
      rec.model.accent.emissive.setHex(col);
      const op = clamp(0.95 - 0.65 * (t.p_decoy || 0), 0.2, 1);
      for (const m of rec.model.mats) m.opacity = op;
      setW(rec.mesh.position, t.pos);
      orient(rec.mesh, t.vel);
      rec.base.pos = t.pos || [0, 0, 0];
      rec.base.vel = t.vel || [0, 0, 0];
      rec.trail.setColor(col);
      rec.trail.push(rec.mesh.position);
      const tti = (t.tti != null) ? ` T-${Math.max(0, t.tti).toFixed(0)}s` : '';
      setLabel(rec.label, `#${t.id} ${cls}${tti}`, '#' + col.toString(16).padStart(6, '0'));
      // predicted impact
      const hasImpact = Array.isArray(t.impact);
      rec.impactRing.visible = hasImpact;
      rec.impactLine.visible = hasImpact;
      if (hasImpact) {
        rec.impactRing.position.set(t.impact[0] || 0, 2, -(t.impact[1] || 0));
        const p = rec.impactLine.geometry.attributes.position;
        p.setXYZ(0, rec.mesh.position.x, rec.mesh.position.y, rec.mesh.position.z);
        p.setXYZ(1, t.impact[0] || 0, 2, -(t.impact[1] || 0));
        p.needsUpdate = true;
      }
    }
    this._prune(this.tracks, liveT, (rec) => {
      this.groups.tracks.remove(rec.mesh);
      disposeModel(rec.model);
      this.groups.trails.remove(rec.trail.line);
      this.groups.impacts.remove(rec.impactRing, rec.impactLine);
    });

    // ---- friendly UAVs (interceptor gun/net, sentinel)
    const liveU = new Set();
    for (const u of f.uavs || []) {
      if (u.id == null) continue;
      liveU.add(u.id);
      let rec = this.uavs.get(u.id);
      if (!rec) {
        const model = makeUavModel(u.kind, u.effector);
        model.group.userData.pick = { kind: 'uav', id: u.id };
        const label = makeLabel();
        label.spr.position.y = 1.4;
        label.spr.visible = this.labelsVisible;
        model.group.add(label.spr);
        const trail = new Trail(0x39d2ff);
        this.groups.trails.add(trail.line);
        rec = { model, mesh: model.group, label, trail,
                base: { pos: [0, 0, 0], vel: [0, 0, 0] }, data: null };
        this.groups.uavs.add(model.group);
        this.uavs.set(u.id, rec);
      }
      rec.data = u;
      const mode = String(u.mode || 'IDLE').toUpperCase();
      const col = MODE_COLOR[mode] ?? 0x39d2ff;
      rec.model.accent.color.setHex(col);
      rec.model.accent.emissive.setHex(col);
      setW(rec.mesh.position, u.pos);
      orient(rec.mesh, u.vel);
      rec.base.pos = u.pos || [0, 0, 0];
      rec.base.vel = u.vel || [0, 0, 0];
      rec.trail.setColor(col);
      rec.trail.push(rec.mesh.position);
      const tag = u.kind === 'sentinel' ? '◉' : (u.effector === 'net' ? '⊞' : '╪');
      setLabel(rec.label, `${tag} ${u.id} ${mode}`, '#9adcff');
    }
    this._prune(this.uavs, liveU, (rec) => {
      this.groups.uavs.remove(rec.mesh);
      disposeModel(rec.model);
      this.groups.trails.remove(rec.trail.line);
    });

    // ---- turrets (static meshes owned by SceneView)
    for (const tu of f.turrets || []) this.view.updateTurret(tu);

    // ---- charging-station occupancy
    for (const st of f.stations || []) this.view.updateStation(st);

    // ---- live falling debris with zone-coloured predicted impact
    const liveD = new Set();
    for (const d of f.debris || []) {
      if (d.id == null) continue;
      liveD.add(d.id);
      let rec = this.debris.get(d.id);
      if (!rec) {
        const model = makeDebrisChunk();
        model.group.userData.pick = { kind: 'debris', id: d.id };
        const trail = new Trail(0x8b9099);
        this.groups.trails.add(trail.line);
        const ring = new THREE.Mesh(DEBRIS_IMPACT_GEO, new THREE.MeshBasicMaterial({
          transparent: true, opacity: 0.8, side: THREE.DoubleSide, depthWrite: false,
        }));
        ring.rotation.x = -Math.PI / 2;
        const lineGeo = new THREE.BufferGeometry().setFromPoints(
          [new THREE.Vector3(), new THREE.Vector3()]);
        const line = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({
          transparent: true, opacity: 0.5,
        }));
        line.frustumCulled = false;
        this.groups.debris.add(model.group, ring, line);
        rec = { model, mesh: model.group, trail, ring, line,
                spin: 1 + Math.abs((hash32(d.id) % 100) / 50),
                base: { pos: [0, 0, 0], vel: [0, 0, 0] }, data: null };
        this.debris.set(d.id, rec);
      }
      rec.data = d;
      setW(rec.mesh.position, d.pos);
      rec.base.pos = d.pos || [0, 0, 0];
      rec.base.vel = d.vel || [0, 0, 0];
      rec.trail.push(rec.mesh.position);
      const zoneCol = ZONE_TINT[d.zone] ?? 0xffb347;
      rec.ring.material.color.setHex(zoneCol);
      rec.line.material.color.setHex(zoneCol);
      rec.ring.position.set(d.impact?.[0] || 0, 2, -(d.impact?.[1] || 0));
      const p = rec.line.geometry.attributes.position;
      p.setXYZ(0, rec.mesh.position.x, rec.mesh.position.y, rec.mesh.position.z);
      p.setXYZ(1, d.impact?.[0] || 0, 2, -(d.impact?.[1] || 0));
      p.needsUpdate = true;
    }
    this._prune(this.debris, liveD, (rec) => {
      this.groups.debris.remove(rec.mesh, rec.ring, rec.line);
      disposeModel(rec.model);
      this.groups.trails.remove(rec.trail.line);
    });

    // ---- engagement attribution FX (HMI-MAP-008): weapon tracers + kill marks
    for (const ev of f.events || []) this._engagementFx(ev);

    // ---- wrecks (mechanism-distinct shape, zone-tinted)
    const liveW = new Set();
    (f.wrecks || []).forEach((wk, i) => {
      liveW.add(i);
      let rec = this.wrecks.get(i);
      if (!rec) {
        const geo = wk.mechanism === 'net' ? WRECK_NET_GEO : WRECK_PROJ_GEO;
        const mesh = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({}));
        this.groups.wrecks.add(mesh);
        rec = { mesh };
        this.wrecks.set(i, rec);
      }
      rec.mesh.material.color.setHex(ZONE_TINT[wk.zone] ?? 0x888888);
      rec.mesh.position.set(wk.pos?.[0] || 0, 8, -(wk.pos?.[1] || 0));
    });
    this._prune(this.wrecks, liveW, (rec) => this.groups.wrecks.remove(rec.mesh));

    // ---- stray-round impacts
    const liveS = new Set();
    (f.strays || []).forEach((s, i) => {
      liveS.add(i);
      let rec = this.strays.get(i);
      if (!rec) {
        const mesh = new THREE.Mesh(STRAY_GEO, new THREE.MeshBasicMaterial({
          color: 0xff5050, transparent: true, opacity: 0.85,
        }));
        mesh.rotation.x = Math.PI;          // tip down
        this.groups.strays.add(mesh);
        rec = { mesh };
        this.strays.set(i, rec);
      }
      rec.mesh.material.color.setHex(ZONE_TINT[s.zone] ?? 0xff5050);
      rec.mesh.position.set(s.pos?.[0] || 0, 11, -(s.pos?.[1] || 0));
    });
    this._prune(this.strays, liveS, (rec) => this.groups.strays.remove(rec.mesh));
  }

  // tracer + kill-mark effects from engagement events
  _engagementFx(ev) {
    const kinds = { kill: 1, miss: 1, debris_neutralized: 1, fire_blocked_los: 1 };
    if (!kinds[ev.kind] || !ev.uav_id) return;
    const from = this.entityPos('uav', ev.uav_id) || this.entityPos('turret', ev.uav_id);
    const to = Array.isArray(ev.pos) ? W(ev.pos)
      : (ev.debris_id ? this.entityPos('debris', ev.debris_id) : null);
    if (!from || !to) return;
    const weapon = this.view.turrets.has(ev.uav_id) ? 'turret_gun' : (ev.effector || 'projectile');
    this._tracer(from, to, WEAPON_COLOR[weapon] ?? 0xffd166,
      ev.kind === 'fire_blocked_los');
    if (ev.kind === 'kill' || ev.kind === 'debris_neutralized')
      this._killMark(to, ev.uav_id, ev.kind === 'debris_neutralized');
  }

  _tracer(from, to, color, blocked = false) {
    const geo = new THREE.BufferGeometry().setFromPoints([from, to]);
    const line = new THREE.Line(geo, new THREE.LineBasicMaterial({
      color, transparent: true, opacity: blocked ? 0.25 : 0.85,
      blending: THREE.AdditiveBlending, depthWrite: false,
    }));
    line.frustumCulled = false;
    const head = new THREE.Mesh(TRACER_HEAD_GEO, new THREE.MeshBasicMaterial({
      color: 0xffffff, transparent: true, opacity: blocked ? 0.3 : 1,
      blending: THREE.AdditiveBlending, depthWrite: false,
    }));
    head.position.copy(from);
    this.groups.tracers.add(line, head);
    this.tracers.push({ line, head, from, to, t0: performance.now(), dur: 550 });
  }

  _killMark(pos, shooter, isDebris) {
    const ring = new THREE.Mesh(KILL_RING_GEO, new THREE.MeshBasicMaterial({
      color: isDebris ? 0x7cd6c0 : 0xff5050, transparent: true, opacity: 0.9,
      side: THREE.DoubleSide, depthWrite: false,
    }));
    ring.position.copy(pos);
    ring.rotation.x = -Math.PI / 2;
    const label = makeLabel();
    setLabel(label, `${shooter} ✕${isDebris ? ' debris' : ''}`, isDebris ? '#7cd6c0' : '#ff8080');
    label.spr.position.copy(pos);
    label.spr.scale.set(420, 79, 1);
    this.groups.fx.add(ring, label.spr);
    this.killMarks.push({ ring, spr: label.spr, t0: performance.now(), dur: 3000 });
  }

  // --------------------------------------------------------------- truth
  // Ghost rule (ICD §4, SRS HMI-EVAL-001/002): enemies with acquired==false
  // are grey wireframe ghosts (true airframe shape) at true position; on
  // acquisition: flash + subtle grey truth marker remains while eval is on.
  applyTruth(tr) {
    const liveG = new Set(), liveM = new Set();
    for (const e of tr.enemies || []) {
      if (e.id == null) continue;
      const prev = this.ghostSeen.get(e.id);
      if (prev === false && e.acquired) {
        this._flash(W(e.pos));
        this.onAcquired?.(e, tr.t);
      }
      this.ghostSeen.set(e.id, !!e.acquired);
      if (!e.alive) continue;

      if (!e.acquired) {
        liveG.add(e.id);
        let rec = this.ghosts.get(e.id);
        if (!rec) {
          const model = makeGhostModel(e.cls);
          model.group.userData.pick = { kind: 'ghost', id: e.id };
          this.groups.ghosts.add(model.group);
          rec = { model, mesh: model.group, base: { pos: [0, 0, 0], vel: [0, 0, 0] }, data: null };
          this.ghosts.set(e.id, rec);
        }
        rec.data = e;
        setW(rec.mesh.position, e.pos);
        orient(rec.mesh, e.vel);
        rec.base.pos = e.pos || [0, 0, 0];
        rec.base.vel = e.vel || [0, 0, 0];
      } else {
        // acquired & alive: thin grey truth marker so truth-vs-track offset shows
        liveM.add(e.id);
        let rec = this.truthMarks.get(e.id);
        if (!rec) {
          const mesh = new THREE.Mesh(TRUTH_GEO, new THREE.MeshBasicMaterial({
            color: 0x9aa4b0, wireframe: true, transparent: true, opacity: 0.3,
          }));
          this.groups.truth.add(mesh);
          rec = { mesh, base: { pos: [0, 0, 0], vel: [0, 0, 0] }, data: null };
          this.truthMarks.set(e.id, rec);
        }
        rec.data = e;
        setW(rec.mesh.position, e.pos);
        rec.base.pos = e.pos || [0, 0, 0];
        rec.base.vel = e.vel || [0, 0, 0];
      }
    }
    this._prune(this.ghosts, liveG, (rec) => {
      this.groups.ghosts.remove(rec.mesh);
      disposeModel(rec.model);
    });
    this._prune(this.truthMarks, liveM, (rec) => this.groups.truth.remove(rec.mesh));
  }

  // production mode / eval drop: remove every eval-only visual
  clearEval() {
    this.ghosts.clear(); this.truthMarks.clear(); this.ghostSeen.clear();
    this.groups.ghosts.clear(); this.groups.truth.clear();
  }

  _flash(pos) {
    const m = new THREE.Mesh(FLASH_GEO, new THREE.MeshBasicMaterial({
      color: 0xffffff, wireframe: true, transparent: true, opacity: 0.9,
    }));
    m.position.copy(pos);
    this.groups.fx.add(m);
    this.flashes.push({ m, t0: performance.now() });
  }

  _prune(map, live, dispose) {
    for (const [id, rec] of map) {
      if (!live.has(id)) { dispose(rec); map.delete(id); }
    }
  }

  // ------------------------------------------------------------- render tick
  tick(now) {
    // smooth extrapolation between 5 Hz frames (live/mock running only)
    const dt = clamp((now - this.recvWall) / 1000, 0, 0.5) * this.effSpeed;
    if (dt > 0) {
      for (const m of [this.tracks, this.uavs, this.ghosts, this.truthMarks, this.debris]) {
        for (const rec of m.values()) {
          const b = rec.base;
          rec.mesh.position.set(
            b.pos[0] + b.vel[0] * dt,
            (b.pos[2] || 0) + (b.vel[2] || 0) * dt,
            -(b.pos[1] + b.vel[1] * dt));
        }
      }
    }
    // zoom-aware magnification (HMI-MAP-007): true scale near, visible far
    const cam = this.view.camera.position;
    for (const m of [this.tracks, this.uavs, this.ghosts, this.debris]) {
      for (const rec of m.values()) {
        const d = cam.distanceTo(rec.mesh.position);
        const s = Math.max(1, this.magFactor * d * MAG_K);
        rec.mesh.scale.setScalar(s);
        if (rec.label) {
          // labels stay screen-readable: world size ∝ camera distance,
          // divided by the parent's scale
          const lw = Math.max(40, LABEL_K * d);
          rec.label.spr.scale.set(lw / s, lw * 0.1875 / s, 1);
        }
        if (rec.spin) {        // tumbling debris
          rec.mesh.rotation.x += 0.016 * rec.spin;
          rec.mesh.rotation.z += 0.011 * rec.spin;
        }
      }
    }
    // acquisition flashes (1.1 s grow + fade)
    for (let i = this.flashes.length - 1; i >= 0; i--) {
      const f = this.flashes[i];
      const a = (now - f.t0) / 1100;
      if (a >= 1) {
        this.groups.fx.remove(f.m);
        f.m.material.dispose();
        this.flashes.splice(i, 1);
      } else {
        f.m.scale.setScalar(1 + a * 3);
        f.m.material.opacity = 0.9 * (1 - a);
      }
    }
    // weapon tracers: head flies from→to, then the line fades
    for (let i = this.tracers.length - 1; i >= 0; i--) {
      const tr = this.tracers[i];
      const a = (now - tr.t0) / tr.dur;
      if (a >= 2.2) {
        this.groups.tracers.remove(tr.line, tr.head);
        tr.line.material.dispose(); tr.line.geometry.dispose();
        tr.head.material.dispose();
        this.tracers.splice(i, 1);
      } else if (a <= 1) {
        tr.head.position.lerpVectors(tr.from, tr.to, a);
      } else {
        tr.head.visible = false;
        tr.line.material.opacity *= 0.94;
      }
    }
    // kill marks: ring expands, label floats up, both fade
    for (let i = this.killMarks.length - 1; i >= 0; i--) {
      const k = this.killMarks[i];
      const a = (now - k.t0) / k.dur;
      if (a >= 1) {
        this.groups.fx.remove(k.ring, k.spr);
        k.ring.material.dispose();
        k.spr.material.dispose();
        this.killMarks.splice(i, 1);
      } else {
        k.ring.scale.setScalar(1 + a * 2.5);
        k.ring.material.opacity = 0.9 * (1 - a);
        k.spr.position.y += 0.55;
        k.spr.material.opacity = 1 - a * a;
      }
    }
    // selection ring follows selected entity
    if (this.selection) {
      const p = this.entityPos(this.selection.kind, this.selection.id);
      if (p) {
        this.selRing.visible = true;
        this.selRing.position.set(p.x, 3, p.z);
        const s = 1 + 0.08 * Math.sin(now / 180);
        this.selRing.scale.setScalar(s);
      } else this.selRing.visible = false;
    } else this.selRing.visible = false;
  }
}

function hash32(s) {
  let h = 2166136261;
  for (let i = 0; i < s.length; i++) { h ^= s.charCodeAt(i); h = Math.imul(h, 16777619); }
  return h >>> 0;
}
