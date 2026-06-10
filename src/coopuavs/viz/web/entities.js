// entities.js — pooled dynamic 3D entities: fused tracks, interceptor UAVs,
// wrecks, strays, predicted impacts, trails, labels, and the evaluation
// ghost overlay (SRS HMI-EVAL-001/002). Meshes are indexed by id and reused
// across frames; never recreated per frame.
import * as THREE from 'three';
import { setW, W } from './scene.js';
import { CLS_COLOR, MODE_COLOR, ZONE_TINT, argmaxClass, clamp } from './util.js';

const TRACK_GEO = new THREE.OctahedronGeometry(26, 0);
const UAV_GEO = new THREE.ConeGeometry(16, 44, 4);
const GHOST_GEO = new THREE.ConeGeometry(22, 60, 6);
const TRUTH_GEO = new THREE.OctahedronGeometry(11, 0);
const WRECK_NET_GEO = new THREE.OctahedronGeometry(15, 0);
const WRECK_PROJ_GEO = new THREE.TetrahedronGeometry(18, 0);
const STRAY_GEO = new THREE.ConeGeometry(9, 22, 4);
const IMPACT_GEO = new THREE.RingGeometry(34, 46, 32);
const FLASH_GEO = new THREE.SphereGeometry(40, 12, 8);

const TRAIL_MAX = 140;

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

export class Entities {
  constructor(view, { onAcquired } = {}) {
    this.view = view;
    this.onAcquired = onAcquired;
    this.root = new THREE.Group();
    view.scene.add(this.root);
    this.groups = {};
    for (const g of ['tracks', 'uavs', 'ghosts', 'truth', 'wrecks', 'strays', 'impacts', 'trails', 'fx'])
      this.groups[g] = new THREE.Group(), this.root.add(this.groups[g]);

    this.tracks = new Map();   // id -> rec
    this.uavs = new Map();
    this.ghosts = new Map();
    this.truthMarks = new Map();
    this.wrecks = new Map();   // index -> rec
    this.strays = new Map();
    this.flashes = [];
    this.ghostSeen = new Map();  // enemy id -> last acquired bool

    this.labelsVisible = true;
    this.recvWall = 0;
    this.effSpeed = 0;
    this.extrapolate = true;

    // selection ring
    this.selection = null;       // {kind, id}
    this.selRing = new THREE.Mesh(
      new THREE.RingGeometry(40, 48, 36),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.8, side: THREE.DoubleSide, depthTest: false }));
    this.selRing.rotation.x = -Math.PI / 2;
    this.selRing.visible = false;
    this.root.add(this.selRing);

    view.extraPickables = () =>
      [this.groups.tracks, this.groups.uavs, this.groups.ghosts];
  }

  reset() {
    for (const m of [this.tracks, this.uavs, this.ghosts, this.truthMarks, this.wrecks, this.strays])
      m.clear();
    for (const k in this.groups) this.groups[k].clear();
    this.flashes.length = 0;
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
    else if (name === 'labels') {
      this.labelsVisible = on;
      for (const m of [this.tracks, this.uavs])
        for (const rec of m.values()) if (rec.label) rec.label.spr.visible = on;
    }
  }

  select(sel) { this.selection = sel; }

  entityPos(kind, id) {
    const map = { track: this.tracks, uav: this.uavs, ghost: this.ghosts, truth: this.truthMarks }[kind];
    if (kind === 'turret') {
      const t = this.view.turrets.get(id);
      return t ? t.group.position.clone() : null;
    }
    const rec = map?.get(id);
    return rec ? rec.mesh.position.clone() : null;
  }

  // --------------------------------------------------------------- frame
  applyFrame(f) {
    this.recvWall = performance.now();
    const run = f.run || {};
    this.effSpeed = (this.extrapolate && run.status === 'running') ? (run.speed || 1) : 0;

    // ---- fused tracks
    const liveT = new Set();
    for (const t of f.tracks || []) {
      if (t.id == null) continue;
      liveT.add(t.id);
      let rec = this.tracks.get(t.id);
      if (!rec) {
        const mesh = new THREE.Mesh(TRACK_GEO, new THREE.MeshLambertMaterial({
          transparent: true, emissiveIntensity: 0.5,
        }));
        mesh.userData.pick = { kind: 'track', id: t.id };
        const label = makeLabel();
        label.spr.position.y = 46;
        label.spr.visible = this.labelsVisible;
        mesh.add(label.spr);
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
        rec = { mesh, label, trail, impactRing, impactLine, base: { pos: [0, 0, 0], vel: [0, 0, 0] }, data: null };
        this.groups.tracks.add(mesh);
        this.tracks.set(t.id, rec);
      }
      rec.data = t;
      const cls = argmaxClass(t.belief);
      const col = CLS_COLOR[cls] ?? 0xffe14d;
      rec.mesh.material.color.setHex(col);
      rec.mesh.material.emissive.setHex(col);
      rec.mesh.material.opacity = clamp(0.95 - 0.65 * (t.p_decoy || 0), 0.2, 1);
      setW(rec.mesh.position, t.pos);
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
      this.groups.trails.remove(rec.trail.line);
      this.groups.impacts.remove(rec.impactRing, rec.impactLine);
    });

    // ---- interceptor UAVs
    const liveU = new Set();
    for (const u of f.uavs || []) {
      if (u.id == null) continue;
      liveU.add(u.id);
      let rec = this.uavs.get(u.id);
      if (!rec) {
        const mesh = new THREE.Mesh(UAV_GEO, new THREE.MeshLambertMaterial({ emissiveIntensity: 0.45 }));
        mesh.userData.pick = { kind: 'uav', id: u.id };
        const label = makeLabel();
        label.spr.position.y = 40;
        label.spr.visible = this.labelsVisible;
        mesh.add(label.spr);
        const trail = new Trail(0x39d2ff);
        this.groups.trails.add(trail.line);
        rec = { mesh, label, trail, base: { pos: [0, 0, 0], vel: [0, 0, 0] }, data: null };
        this.groups.uavs.add(mesh);
        this.uavs.set(u.id, rec);
      }
      rec.data = u;
      const mode = String(u.mode || 'IDLE').toUpperCase();
      const col = MODE_COLOR[mode] ?? 0x39d2ff;
      rec.mesh.material.color.setHex(col);
      rec.mesh.material.emissive.setHex(col);
      setW(rec.mesh.position, u.pos);
      rec.base.pos = u.pos || [0, 0, 0];
      rec.base.vel = u.vel || [0, 0, 0];
      // nose toward velocity
      const v = u.vel || [0, 0, 0];
      if (Math.hypot(v[0], v[1], v[2]) > 1) {
        const dir = new THREE.Vector3(v[0], v[2], -v[1]).normalize();
        rec.mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
      }
      rec.trail.setColor(col);
      rec.trail.push(rec.mesh.position);
      setLabel(rec.label, `${u.id} ${mode}`, '#9adcff');
    }
    this._prune(this.uavs, liveU, (rec) => {
      this.groups.uavs.remove(rec.mesh);
      this.groups.trails.remove(rec.trail.line);
    });

    // ---- turrets (static meshes owned by SceneView)
    for (const tu of f.turrets || []) this.view.updateTurret(tu);

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

  // --------------------------------------------------------------- truth
  // Ghost rule (ICD §4, SRS HMI-EVAL-001/002): enemies with acquired==false
  // are grey wireframe ghosts at true position; on acquisition: flash +
  // subtle grey truth marker remains while eval is on.
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
          const mesh = new THREE.Mesh(GHOST_GEO, new THREE.MeshBasicMaterial({
            color: 0x9aa4b0, wireframe: true, transparent: true, opacity: 0.4,
          }));
          mesh.userData.pick = { kind: 'ghost', id: e.id };
          this.groups.ghosts.add(mesh);
          rec = { mesh, base: { pos: [0, 0, 0], vel: [0, 0, 0] }, data: null };
          this.ghosts.set(e.id, rec);
        }
        rec.data = e;
        setW(rec.mesh.position, e.pos);
        rec.base.pos = e.pos || [0, 0, 0];
        rec.base.vel = e.vel || [0, 0, 0];
        const v = e.vel || [0, 0, 0];
        if (Math.hypot(v[0], v[1], v[2]) > 1) {
          const dir = new THREE.Vector3(v[0], v[2], -v[1]).normalize();
          rec.mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
        }
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
    this._prune(this.ghosts, liveG, (rec) => this.groups.ghosts.remove(rec.mesh));
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
      for (const m of [this.tracks, this.uavs, this.ghosts, this.truthMarks]) {
        for (const rec of m.values()) {
          const b = rec.base;
          rec.mesh.position.set(
            b.pos[0] + b.vel[0] * dt,
            (b.pos[2] || 0) + (b.vel[2] || 0) * dt,
            -(b.pos[1] + b.vel[1] * dt));
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
