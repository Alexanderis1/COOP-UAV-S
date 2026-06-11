// mock.js — ?mock=1 development mode: a deterministic, fully synthetic
// backend that fabricates a plausible scene plus ~3 minutes of frames,
// truth, auth requests, decisions and a summary, covering EVERY message
// type of docs/ICD_RUNTIME.md on both channels. No network needed.
import { CLASSES, zoneAt, clamp } from './util.js';

const STEP = 0.2;                 // 5 Hz sim-time record rate (ICD §2.2)

function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0; a = a + 0x6D2B79F5 | 0;
    let t = Math.imul(a ^ a >>> 15, 1 | a);
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}

const CLS = {
  owa_strategic: { speed: 60, alt: 900, warhead: true },
  owa_jet: { speed: 103, alt: 1500, warhead: true },
  fpv: { speed: 35, alt: 80, warhead: true },
  loitering: { speed: 80, alt: 300, warhead: true },
  decoy: { speed: 60, alt: 900, warhead: false },
};
const DEFAULT_AXIS = { owa_strategic: 350, owa_jet: 40, fpv: 95, loitering: 250, decoy: 5 };
const EFFECTOR = {
  net: { range: 40, p_kill: 0.72 },
  projectile: { range: 110, p_kill: 0.65 },
};
const ZCOST = { SAFE: 2, DANGEROUS: 25, CRITICAL: 120 };

function buildScene(seed, duration) {
  const bounds = [-6000, -6000, 6000, 6000], cell = 250, n = 48;
  const assets = [
    { name: 'HQ COMPLEX', pos: [0, 0, 0], value: 100 },
    { name: 'POWER PLANT', pos: [1800, 1200, 0], value: 80 },
    { name: 'FUEL DEPOT', pos: [-2200, -900, 0], value: 60 },
  ];
  // ---- city fabric: street-grid blocks with kinds (mirrors citygen)
  const rng = mulberry32(7);
  const buildings = [
    { rect: [-6000, -120, 6000, 120], height: 0, kind: 'water', name: 'river' },
    { rect: [520, 880, 820, 1120], height: 28, kind: 'hospital', name: 'hospital' },
    { rect: [-1120, -880, -880, -680], height: 12, kind: 'school', name: 'school-a' },
    { rect: [1380, 1880, 1620, 2080], height: 12, kind: 'school', name: 'school-b' },
    { rect: [-1750, 750, -1250, 1150], height: 0, kind: 'park', name: 'west-park' },
    { rect: [1550, -1350, 2050, -1000], height: 0, kind: 'park', name: 'east-park' },
  ];
  const step = 300, half = 112;
  for (let cy = -2550; cy <= 2550; cy += step) {
    if (Math.abs(cy) < 280) continue;            // river corridor
    for (let cx = -2550; cx <= 2550; cx += step) {
      const r = Math.hypot(cx, cy);
      if (r > 2700) continue;
      if (buildings.some((b) => b.rect && cx + half > b.rect[0] - 40 && cx - half < b.rect[2] + 40
          && cy + half > b.rect[1] - 40 && cy - half < b.rect[3] + 40)) continue;
      const roll = rng();
      let kind, h;
      if (roll < 0.07) { kind = 'park'; h = 0; }
      else if (Math.abs(cx) < 260) { kind = 'commercial'; h = 22 + rng() * 35; }
      else if (r < 1300) { kind = roll < 0.8 ? 'residential_high' : 'commercial'; h = kind === 'commercial' ? 22 + rng() * 35 : 30 + rng() * 42; }
      else { kind = roll < 0.9 ? 'residential_low' : 'commercial'; h = kind === 'commercial' ? 18 + rng() * 22 : 7 + rng() * 7; }
      const hw = half * (0.78 + rng() * 0.17), hh = half * (0.78 + rng() * 0.17);
      buildings.push({ rect: [cx - hw, cy - hh, cx + hw, cy + hh], height: +h.toFixed(1), kind });
    }
  }
  for (let iy = 0; iy < 3; iy++) {
    for (let ix = 0; ix < 4; ix++) {
      const cx = 3400 + ix * 520, cy = -4400 + iy * 480;
      buildings.push({ rect: [cx - 190, cy - 160, cx + 190, cy + 160],
        height: +(8 + rng() * 8).toFixed(1), kind: 'industrial' });
    }
  }
  // ---- zone grid DERIVED from building kinds (SIM-ENV-005 semantics)
  const grid = [];
  const overlaps = (x, y, b, m) => x > b.rect[0] - m && x < b.rect[2] + m
    && y > b.rect[1] - m && y < b.rect[3] + m;
  for (let j = 0; j < n; j++) {
    const row = [];
    for (let i = 0; i < n; i++) {
      const x = bounds[0] + (i + 0.5) * cell, y = bounds[1] + (j + 0.5) * cell;
      let z = 0;
      for (const b of buildings) {
        const k = b.kind;
        // buffers mirror risk/zones._DANGEROUS_BUFFER
        if (k === 'residential_low' && overlaps(x, y, b, 60)) z = Math.max(z, 1);
        if (k === 'commercial' && overlaps(x, y, b, 40)) z = Math.max(z, 1);
        if ((k === 'residential_high') && overlaps(x, y, b, 150)) z = Math.max(z, 1);
        if ((k === 'hospital' || k === 'school') && overlaps(x, y, b, 200)) z = Math.max(z, 1);
      }
      for (const b of buildings) {
        const k = b.kind;
        if ((k === 'hospital' || k === 'school') && overlaps(x, y, b, 100)) z = 2;
        if (k === 'residential_high' && overlaps(x, y, b, 50)) z = 2;
      }
      for (const b of buildings)
        if ((b.kind === 'park' || b.kind === 'water' || b.kind === 'industrial')
            && overlaps(x, y, b, 0) && z < 2) z = 0;
      row.push(z);
    }
    grid.push(row);
  }
  const stations = [
    { id: 'cs-base-1', pos: [-450, -2600, 0], rooftop: false },
    { id: 'cs-base-2', pos: [450, -2600, 0], rooftop: false },
    { id: 'cs-comm-1', pos: [60, -2250, 30], rooftop: true },
    { id: 'cs-ind-1', pos: [3400, -4400, 14], rooftop: true },
    { id: 'cs-north-1', pos: [-2400, 2400, 0], rooftop: false },
  ];
  const homes = [];
  for (let i = 0; i < 20; i++)
    homes.push({ uav_id: `hawk-${i + 1}`, pos: stations[i % stations.length].pos.slice() });
  for (let i = 0; i < 10; i++)
    homes.push({ uav_id: `sent-${i + 1}`, pos: stations[i % stations.length].pos.slice() });
  return {
    bounds, cell_size: cell, grid, assets, buildings, stations,
    sensors: [
      { name: 'radar_north', type: 'radar', pos: [0, 4200, 0], range: 5200 },
      { name: 'rf_west', type: 'rf', pos: [-4600, 500, 0], range: 6800 },
      { name: 'eoir_central', type: 'eo_ir', pos: [400, -300, 0], range: 2200 },
      { name: 'acoustic_south', type: 'acoustic', pos: [900, -3600, 0], range: 1600 },
    ],
    turrets: [
      { id: 'tur_east', pos: [2600, 200, 0], range: 1500 },
      { id: 'tur_west', pos: [-2300, 700, 0], range: 1500 },
    ],
    homes,
    run: { name: 'mock-raid', seed, duration, eval: true },
  };
}

function defaultRequest() {
  return {
    threats: {
      owa_strategic: { count: 3, target: 'auto', axis_deg: 350, first_time: 6, spacing: 9 },
      decoy: { count: 2, target: 'auto', axis_deg: 5, first_time: 10, spacing: 9 },
      fpv: { count: 2, target: 'HQ COMPLEX', axis_deg: 95, first_time: 28, spacing: 7 },
      owa_jet: { count: 1, target: 'POWER PLANT', axis_deg: 40, first_time: 45, spacing: 8 },
      loitering: { count: 2, target: 'FUEL DEPOT', axis_deg: 250, first_time: 60, spacing: 12 },
    },
    weather: { wind_speed: 6, wind_dir_deg: 230, fog: 0.15, precip: 0.05, daylight: 0.25 },
    duration: 180, speed: 1, seed: null, posture: 'human_confirm',
  };
}

export class MockServer {
  constructor({ onOps, onEval }) {
    this.onOps = onOps;
    this.onEval = onEval;
    this.runCount = 0;
    this.status = 'idle';
  }

  start() {
    this._beginRun(defaultRequest(), 12345);
    this.timer = setInterval(() => this._wallTick(), 50);
  }

  stop() { clearInterval(this.timer); }

  _ops(type, data) { this.onOps({ type, data }); }

  // -------------------------------------------------------- control channel
  send(msg) {
    const { type, data = {} } = msg || {};
    switch (type) {
      case 'start_run': {
        if (this.status === 'running' || this.status === 'paused') {
          this._ops('error', { message: 'start_run rejected: a run is already active — stop it first.' });
          return;
        }
        const assets = new Set(buildScene(0, 0).assets.map((a) => a.name));
        for (const [cls, spec] of Object.entries(data.threats || {})) {
          if (!CLASSES.includes(cls)) {
            this._ops('error', { message: `start_run rejected: unknown threat class "${cls}".` });
            return;
          }
          const tgt = spec?.target ?? 'auto';
          if (tgt !== 'auto' && !assets.has(tgt)) {
            this._ops('error', { message: `start_run rejected: unknown asset "${tgt}" for class ${cls}.` });
            return;
          }
        }
        const seed = (data.seed == null || data.seed === '') ? 20000 + ++this.runCount : (data.seed | 0);
        this._beginRun({ ...defaultRequest(), ...data }, seed);
        break;
      }
      case 'stop_run':
        if (this.status === 'running' || this.status === 'paused') this._finish('stopped by operator');
        break;
      case 'pause':
        if (this.status === 'running') { this.status = 'paused'; this._emitFrame(); }
        break;
      case 'resume':
        if (this.status === 'paused') { this.status = 'running'; this._emitFrame(); }
        break;
      case 'set_speed':
        this.speed = clamp(+data.speed || 1, 0.1, 10);
        this._emitFrame();
        break;
      case 'set_posture': {
        const p = data.posture;
        if (!['human_confirm', 'pre_authorized', 'weapons_hold'].includes(p)) return;
        this.posture = p;
        this._decide('operator', 'posture', `autonomy posture set to ${p}`);
        if (p === 'weapons_hold')
          for (const a of this.auths.values())
            if (!a.resolved) this._resolveAuth(a, false, 'posture');
        this._emitFrame();
        break;
      }
      case 'authorize': {
        const a = this.auths.get(data.id);
        if (a && !a.resolved) this._resolveAuth(a, !!data.approve, 'operator');
        break;
      }
      case 'uav_command': {
        const u = this.uavs.find((x) => x.id === data.uav_id);
        if (u && data.command === 'rtb') {
          u.mode = 'RTB'; u.task = null;
          this._decide('operator', 'rtb', `${u.id} ordered RTB by operator`, null, u.id);
        }
        break;
      }
      default: break;
    }
  }

  // ------------------------------------------------------------- run setup
  _beginRun(req, seed) {
    this.req = req;
    this.seed = seed;
    this.rng = mulberry32(seed);
    this.t = 0;
    this.acc = 0;
    this.speed = clamp(+req.speed || 1, 0.1, 10);
    this.posture = req.posture || 'human_confirm';
    this.duration = +req.duration || 180;
    this.status = 'running';
    this.summarySent = false;
    this.scene = buildScene(seed, this.duration);

    this.enemies = [];
    this.spawnQueue = [];
    let ai = 0, eid = 0;
    for (const cls of CLASSES) {
      const spec = (req.threats || {})[cls];
      if (!spec || !(spec.count > 0)) continue;
      for (let i = 0; i < spec.count; i++) {
        const target = (spec.target && spec.target !== 'auto')
          ? this.scene.assets.find((a) => a.name === spec.target)
          : this.scene.assets[ai++ % this.scene.assets.length];
        this.spawnQueue.push({
          t: (+spec.first_time || 5) + i * (+spec.spacing || 8),
          cls,
          axis: (spec.axis_deg == null) ? DEFAULT_AXIS[cls] : +spec.axis_deg,
          target,
          id: `${cls}-${++eid}`,
        });
      }
    }
    this.spawnQueue.sort((a, b) => a.t - b.t);

    this.uavs = this.scene.homes
      .filter((h) => h.uav_id.startsWith('hawk'))
      .map((h, i) => ({
        id: h.uav_id, home: h.pos.slice(), pos: [h.pos[0], h.pos[1], h.pos[2] || 0],
        vel: [0, 0, 0], mode: 'IDLE', ammo: 4, battery: 1, task: null, kind: 'interceptor',
        effector: i % 3 === 2 ? 'net' : 'projectile', link: 0.98,
        fireAt: null, lastAuthT: -99, authId: null,
      }));
    // sentinels: unarmed patrol orbits feeding the picture (PHY-SNT-*)
    this.sentinels = this.scene.homes
      .filter((h) => h.uav_id.startsWith('sent'))
      .map((h, i) => ({
        id: h.uav_id, home: h.pos.slice(), pos: [h.pos[0], h.pos[1], h.pos[2] || 0],
        vel: [0, 0, 0], mode: 'TRANSIT', battery: 1, link: 0.97, kind: 'sentinel',
        orbit: {
          cx: 3000 * Math.sin(i * 0.628), cy: 3000 * Math.cos(i * 0.628),
          r: 700, alt: 320, w: 25 / 700,
        },
        ang: i * 1.7,
      }));
    this.debris = [];          // live falling wreckage (SIM-DEB-001)
    this.debrisSeq = 0;
    this.m2 = { debrisInt: 0, debrisSaved: 0 };
    this.engage = {};          // shooter -> tally for metrics.engagements
    this.turrets = this.scene.turrets.map((t) => ({
      id: t.id, pos: t.pos.slice(), range: t.range, az: 0, el: 0, ammo: 480,
      state: 'idle', target: null, alignT: 0, burstT: 0, authId: null, lastAuthT: -99,
    }));
    this.auths = new Map();
    this.nextAuth = 1;
    this.nextTrack = 1;
    this.wrecks = [];
    this.strays = [];
    this.pendEvents = [];
    this.pendDecisions = [];
    this.m = { shots: 0, kills: 0, decoyShots: 0, authLat: [] };
    this.authStats = { requests: 0, approved: 0, denied: 0, expired: 0 };

    this._ops('scene', this.scene);
    this._ops('run_started', { name: this.scene.run.name, seed, eval: true });
    this._decide('c2', 'run', `run started — seed ${seed}, posture ${this.posture}`);
    this._emitFrame();
    this._emitTruth();
  }

  _finish(why) {
    this.status = 'done';
    this._decide('c2', 'run', `run finished: ${why}`);
    this._emitFrame();
    this._emitTruth();
    if (!this.summarySent) {
      this.summarySent = true;
      const wz = this._byZone(this.wrecks);
      this._ops('summary', {
        name: this.scene.run.name, seed: this.seed, t_end: +this.t.toFixed(1),
        enemies_total: this.enemies.length,
        kills: this.enemies.filter((e) => e.killed).length,
        armed_leakers: this.enemies.filter((e) => e.leaker && e.warhead).length,
        wrecks_by_zone: wz,
        metrics: this._metrics(),
      });
    }
  }

  // ---------------------------------------------------------------- ticking
  _wallTick() {
    if (this.status !== 'running') return;
    this.acc += 0.05 * this.speed;
    let steps = 0;
    while (this.acc >= STEP && steps < 60 && this.status === 'running') {
      this.acc -= STEP; steps++;
      this._step(STEP);
      this._emitFrame();
      this._emitTruth();
    }
  }

  _step(dt) {
    this.t += dt;
    const t = this.t;

    // spawns
    while (this.spawnQueue.length && this.spawnQueue[0].t <= t) {
      const s = this.spawnQueue.shift();
      const c = CLS[s.cls];
      const rad = (s.axis * Math.PI) / 180;
      const R = 5700;
      this.enemies.push({
        id: s.id, cls: s.cls, warhead: c.warhead, target: s.target,
        pos: [R * Math.sin(rad), R * Math.cos(rad), c.alt],
        vel: [0, 0, 0], speed: c.speed, alt: c.alt, spawn_t: t,
        alive: true, killed: false, leaker: false,
        acquired: false, acquired_t: null, track_id: null,
        discr: 0, assigned: null, phase: this.rng() * 6.28,
      });
    }

    // enemy kinematics
    for (const e of this.enemies) {
      if (!e.alive) continue;
      const dx = e.target.pos[0] - e.pos[0], dy = e.target.pos[1] - e.pos[1];
      const hd = Math.hypot(dx, dy);
      if (hd < 45) {
        e.alive = false; e.leaker = true;
        if (e.warhead)
          this._event({ kind: 'leaker', track_id: e.track_id, asset: e.target.name });
        else
          this._event({ kind: 'decoy_expended', track_id: e.track_id });
        if (e.assigned) this._unassign(e);
        continue;
      }
      const zT = hd < 1500 ? (e.alt * hd) / 1500 : e.alt;
      const vz = clamp((zT - e.pos[2]) / Math.max(dt, 1e-6), -e.speed, e.speed * 0.3);
      e.vel = [(dx / hd) * e.speed, (dy / hd) * e.speed, vz * 0.4];
      e.pos = [e.pos[0] + e.vel[0] * dt, e.pos[1] + e.vel[1] * dt,
        Math.max(4, e.pos[2] + e.vel[2] * dt)];

      // acquisition by the sensor net
      if (!e.acquired) {
        let by = null;
        for (const s of this.scene.sensors) {
          const d = Math.hypot(e.pos[0] - s.pos[0], e.pos[1] - s.pos[1], e.pos[2] - s.pos[2]);
          if (s.type === 'radar' && e.pos[2] > 60 && d < s.range) by = s.name;
          else if (s.type === 'rf' && e.cls !== 'fpv' && d < s.range) by = s.name;
          else if (s.type === 'eo_ir' && d < s.range * (0.5 + 0.5 * this._daylight())) by = s.name;
          else if (s.type === 'acoustic' && d < s.range) by = s.name;
          if (by) break;
        }
        if (by) {
          e.acquired = true; e.acquired_t = t; e.track_id = this.nextTrack++;
          this._event({ kind: 'acquired', track_id: e.track_id, sensor: by });
          this._decide('c2', 'track', `track #${e.track_id} acquired by ${by}`, e.track_id);
        }
      } else {
        // decoy discrimination near the EO/IR tower
        const eo = this.scene.sensors.find((s) => s.type === 'eo_ir');
        const d = Math.hypot(e.pos[0] - eo.pos[0], e.pos[1] - eo.pos[1]);
        if (d < eo.range) {
          e.discr += dt;
          if (e.cls === 'decoy' && e.discr > 3 && !e.decoyCalled) {
            e.decoyCalled = true;
            this._decide('c2', 'classify',
              `track #${e.track_id} assessed DECOY (p_decoy high) — withholding fire`, e.track_id);
            if (e.assigned) this._unassign(e);
          }
        }
      }
    }

    // C2 assignment (1 Hz TEWA cycle)
    this._assignT = (this._assignT || 0) + dt;
    if (this._assignT >= 1) {
      this._assignT = 0;
      for (const e of this.enemies) {
        if (!e.alive || !e.acquired || e.assigned) continue;
        if (this._pDecoy(e) > 0.6) continue;
        if (e.cls === 'owa_jet') {
          if (!e.jetCalled) {
            e.jetCalled = true;
            this._decide('c2', 'tewa',
              `track #${e.track_id} jet-OWA: outruns Tier-P — turret engagement only`, e.track_id);
          }
          continue;
        }
        const u = this.uavs.find((x) =>
          (x.mode === 'IDLE' || x.mode === 'RTB') && x.ammo > 0 && x.battery > 0.3);
        if (u) {
          u.mode = 'PURSUIT'; u.task = e.id; e.assigned = u.id;
          this._decide('c2', 'tewa', `${u.id} assigned to track #${e.track_id} (${u.effector})`,
            e.track_id, u.id);
        }
      }
    }

    // interceptor UAVs
    for (const u of this.uavs) {
      const moving = u.mode !== 'IDLE';
      u.battery = clamp(u.battery + (moving ? -0.0012 : 0.004) * dt, 0, 1);
      u.link = clamp(0.99 - Math.hypot(u.pos[0], u.pos[1]) / 22000, 0.3, 1);
      const eff = EFFECTOR[u.effector];

      if (u.mode === 'PURSUIT' || u.mode === 'ENGAGE') {
        const e = this.enemies.find((x) => x.id === u.task);
        if (!e || !e.alive) { this._uavIdle(u); continue; }
        const dx = e.pos[0] - u.pos[0], dy = e.pos[1] - u.pos[1], dz = e.pos[2] - u.pos[2];
        const d = Math.hypot(dx, dy, dz);
        const spd = 80;
        if (d > 1) {
          u.vel = [(dx / d) * spd, (dy / d) * spd, (dz / d) * spd];
          u.pos = [u.pos[0] + u.vel[0] * dt, u.pos[1] + u.vel[1] * dt,
            Math.max(5, u.pos[2] + u.vel[2] * dt)];
        }
        if (u.mode === 'PURSUIT' && d < eff.range * 0.9) u.mode = 'ENGAGE';
        if (u.mode === 'ENGAGE' && d > eff.range * 2.5) u.mode = 'PURSUIT';

        if (u.mode === 'ENGAGE') {
          const auth = u.authId != null ? this.auths.get(u.authId) : null;
          if (auth && auth.resolved && auth.approved && u.fireAt != null && t >= u.fireAt) {
            this._fire(u, e, eff, u.effector);
            u.fireAt = null; u.authId = null;
          } else if ((!auth || auth.resolved) && u.fireAt == null
                     && t - u.lastAuthT > 5 && d < eff.range && u.ammo > 0) {
            u.lastAuthT = t;
            u.authId = this._createAuth(u.id, e, u.effector, eff.p_kill);
          }
        }
      } else if (u.mode === 'RTB') {
        const dx = u.home[0] - u.pos[0], dy = u.home[1] - u.pos[1], dz = 0 - u.pos[2];
        const d = Math.hypot(dx, dy, dz);
        if (d < 30) { u.mode = 'IDLE'; u.vel = [0, 0, 0]; u.pos = [u.home[0], u.home[1], 0]; }
        else {
          const spd = 60;
          u.vel = [(dx / d) * spd, (dy / d) * spd, (dz / d) * spd];
          u.pos = [u.pos[0] + u.vel[0] * dt, u.pos[1] + u.vel[1] * dt,
            Math.max(0, u.pos[2] + u.vel[2] * dt)];
        }
      } else {
        u.vel = [0, 0, 0];
        u.rearmT = (u.rearmT || 0) + dt;       // rearm cycle at the pad
        if (u.ammo < 4 && u.rearmT > 8) { u.ammo++; u.rearmT = 0; }
      }

      if (u.mode !== 'RTB' && u.mode !== 'IDLE' && (u.battery < 0.22 || u.ammo <= 0)) {
        this._uavIdle(u); u.mode = 'RTB';
        this._decide('c2', 'rtb', `${u.id} RTB (${u.ammo <= 0 ? 'ammo out' : 'low battery'})`, null, u.id);
      }
    }

    // sentinels: climb to the orbit, then fly it
    for (const s of this.sentinels) {
      const o = s.orbit;
      const onSta = Math.abs(Math.hypot(s.pos[0] - o.cx, s.pos[1] - o.cy) - o.r) < 150
        && Math.abs(s.pos[2] - o.alt) < 60;
      if (onSta) { s.mode = 'PATROL'; s.ang += o.w * dt; }
      else s.mode = 'TRANSIT';
      const wp = [o.cx + o.r * Math.sin(s.ang + 0.15), o.cy + o.r * Math.cos(s.ang + 0.15), o.alt];
      const dx = wp[0] - s.pos[0], dy = wp[1] - s.pos[1], dz = wp[2] - s.pos[2];
      const d = Math.hypot(dx, dy, dz);
      const spd = Math.min(28, d);
      if (d > 1) {
        s.vel = [(dx / d) * spd, (dy / d) * spd, (dz / d) * spd];
        s.pos = [s.pos[0] + s.vel[0] * dt, s.pos[1] + s.vel[1] * dt, Math.max(0, s.pos[2] + s.vel[2] * dt)];
      }
      s.battery = clamp(s.battery - 0.0006 * dt, 0.2, 1);
    }

    // live debris: fall, get intercepted over populated ground, or land
    for (let i = this.debris.length - 1; i >= 0; i--) {
      const d = this.debris[i];
      d.vel[2] = Math.max(d.vel[2] - 9.81 * dt, -45);
      d.pos = [d.pos[0] + d.vel[0] * dt, d.pos[1] + d.vel[1] * dt, d.pos[2] + d.vel[2] * dt];
      d.t_impact = Math.max(0, d.pos[2] / 45);
      d.impact = [d.pos[0] + d.vel[0] * d.t_impact, d.pos[1] + d.vel[1] * d.t_impact, 0];
      d.zone = zoneAt(this.scene, d.impact[0], d.impact[1]);
      // a turret intercepts red/yellow-bound debris mid-fall (kinetic only,
      // in range, with rounds left — like the real adjudicator)
      if (d.zone !== 'SAFE' && d.pos[2] < 500 && d.pos[2] > 120 && this.rng() < 0.06) {
        const tu = this.turrets.find((x) => x.ammo > 0
          && Math.hypot(d.pos[0] - x.pos[0], d.pos[1] - x.pos[1], d.pos[2]) < x.range);
        if (tu) {
          tu.ammo = Math.max(0, tu.ammo - 12);
          this.m.shots++;
          this._tally(tu.id, 'turret_gun', 'debris');
          this._event({
            kind: 'debris_neutralized', uav_id: tu.id, debris_id: d.id,
            effector: 'turret_gun', saved_zone: d.zone, pk: +(0.5 + this.rng() * 0.3).toFixed(2),
            pos: d.pos.map((v) => +v.toFixed(1)), target_kind: 'debris',
          });
          this.m2.debrisInt++;
          this.m2.debrisSaved += d.zone === 'CRITICAL' ? 25 : 1;
          this.debris.splice(i, 1);
          continue;
        }
      }
      if (d.pos[2] <= 0) {
        this.wrecks.push({ pos: [d.pos[0], d.pos[1], 0], zone: d.zone, mechanism: d.mechanism });
        this._event({ kind: 'debris_impact', debris_id: d.id, zone: d.zone });
        this.debris.splice(i, 1);
      }
    }

    // pending authorisations: posture auto-clear + expiry
    for (const a of this.auths.values()) {
      if (a.resolved) continue;
      if (this.posture === 'pre_authorized' && a.roe.decision === 'authorized' && t >= a.t + 0.4)
        this._resolveAuth(a, true, 'orc');
      else if (t >= a.expires_t)
        this._resolveAuth(a, false, 'timeout');
    }

    // turrets
    for (const tu of this.turrets) {
      if (tu.ammo <= 0) { tu.state = 'empty'; tu.target = null; continue; }
      let best = null, bd = Infinity;
      for (const e of this.enemies) {
        if (!e.alive || !e.acquired) continue;
        const d = Math.hypot(e.pos[0] - tu.pos[0], e.pos[1] - tu.pos[1], e.pos[2] - tu.pos[2]);
        if (d < tu.range && d < bd && this._pDecoy(e) < 0.6) { best = e; bd = d; }
      }
      if (!best) { tu.state = 'idle'; tu.target = null; tu.authId = null; tu.alignT = 0; continue; }
      if (tu.target !== best.track_id) { tu.target = best.track_id; tu.authId = null; tu.alignT = 0; }
      const dx = best.pos[0] - tu.pos[0], dy = best.pos[1] - tu.pos[1];
      const azT = (Math.atan2(dx, dy) * 180) / Math.PI;
      const elT = (Math.atan2(best.pos[2], Math.hypot(dx, dy)) * 180) / Math.PI;
      const dAz = ((azT - tu.az + 540) % 360) - 180;
      tu.az += clamp(dAz, -90 * dt, 90 * dt);
      tu.el += clamp(elT - tu.el, -60 * dt, 60 * dt);
      const aligned = Math.abs(dAz) < 6 && Math.abs(elT - tu.el) < 6;
      if (!aligned) { tu.state = 'slewing'; tu.alignT = 0; continue; }
      tu.alignT += dt;
      const auth = tu.authId != null ? this.auths.get(tu.authId) : null;
      const needNew = !auth || (auth.resolved && !auth.approved);
      if (needNew && tu.alignT > 0.5 && t - tu.lastAuthT > 8) {
        tu.lastAuthT = t;
        tu.authId = this._createAuth(tu.id, best, 'projectile', 0.35);
        tu.state = 'tracking';
      } else if (auth && auth.resolved && auth.approved) {
        tu.state = 'firing';
        tu.burstT += dt;
        if (tu.burstT >= 0.8) {
          tu.burstT = 0;
          tu.ammo = Math.max(0, tu.ammo - 12);
          this.m.shots++;
          if (best.cls === 'decoy') this.m.decoyShots++;
          this._tally(tu.id, 'turret_gun');
          if (this.rng() < 0.3) this._kill(best, 'projectile', tu.id, 'turret_gun');
          else {
            this._event({
              kind: 'miss', uav_id: tu.id, enemy_id: best.id,
              effector: 'turret_gun', pk: +(0.15 + this.rng() * 0.25).toFixed(2),
              pos: best.pos.map((v) => +v.toFixed(1)), target_kind: 'track',
            });
            if (this.rng() < 0.55) {
              const over = 1.5 + this.rng();
              const sx = tu.pos[0] + dx * over, sy = tu.pos[1] + dy * over;
              const z = zoneAt(this.scene, sx, sy);
              this.strays.push({ pos: [sx, sy, 0], zone: z });
              this._event({ kind: 'stray', shooter: tu.id, zone: z });
            }
          }
        }
      } else tu.state = 'tracking';
    }

    // end of run?
    const allDone = this.spawnQueue.length === 0 && this.enemies.length > 0
      && this.enemies.every((e) => !e.alive);
    if (t >= this.duration) this._finish('duration reached');
    else if (allDone && this.enemies.length) this._finish('raid resolved');
  }

  _uavIdle(u) {
    u.task = null; u.fireAt = null;
    // if airborne away from the pad, fly home instead of hovering
    u.mode = Math.hypot(u.pos[0] - u.home[0], u.pos[1] - u.home[1], u.pos[2]) > 60 ? 'RTB' : 'IDLE';
  }

  _unassign(e) {
    const u = this.uavs.find((x) => x.id === e.assigned);
    if (u) this._uavIdle(u);
    e.assigned = null;
  }

  _fire(u, e, eff, mech) {
    u.ammo = Math.max(0, u.ammo - 1);
    this.m.shots++;
    if (e.cls === 'decoy') this.m.decoyShots++;
    this._tally(u.id, mech);
    if (this.rng() < eff.p_kill) {
      this._kill(e, mech, u.id);
      this._uavIdle(u);
    } else {
      // miss events match the real adjudicator: enemy_id, no track_id
      this._event({
        kind: 'miss', uav_id: u.id, enemy_id: e.id,
        effector: mech, pk: +(eff.p_kill * (0.4 + this.rng() * 0.5)).toFixed(2),
        pos: e.pos.map((v) => +v.toFixed(1)), target_kind: 'track',
      });
      if (mech === 'projectile' && this.rng() < 0.5) {
        const sx = e.pos[0] + (this.rng() - 0.5) * 700, sy = e.pos[1] + (this.rng() - 0.5) * 700;
        const z = zoneAt(this.scene, sx, sy);
        this.strays.push({ pos: [sx, sy, 0], zone: z });
        this._event({ kind: 'stray', shooter: u.id, zone: z });
      }
    }
  }

  _kill(e, mechanism, shooter, weapon = mechanism) {
    e.alive = false; e.killed = true;
    this.m.kills++;
    const r = this.engage[shooter];
    if (r) { r.hits++; r.kills++; }
    // the wreck becomes live falling debris (SIM-DEB-001)
    const drift = mechanism === 'net' ? 0.15 : 0.65;
    const deb = {
      id: `deb-${++this.debrisSeq}`, mechanism,
      pos: e.pos.slice(), vel: [e.vel[0] * drift, e.vel[1] * drift, 0],
      t_impact: e.pos[2] / 45, impact: [e.pos[0], e.pos[1], 0], zone: 'SAFE',
    };
    deb.impact = [e.pos[0] + deb.vel[0] * deb.t_impact, e.pos[1] + deb.vel[1] * deb.t_impact, 0];
    deb.zone = zoneAt(this.scene, deb.impact[0], deb.impact[1]);
    this.debris.push(deb);
    // kill events match the real adjudicator: enemy_id + ICD weapon name
    // (turret bursts log turret_gun), no track_id
    this._event({
      kind: 'kill', uav_id: shooter, enemy_id: e.id,
      effector: weapon, pk: +(0.4 + this.rng() * 0.4).toFixed(2),
      debris_zone: deb.zone, pos: e.pos.map((v) => +v.toFixed(1)),
      target_kind: 'track',
    });
    this._event({ kind: 'debris_spawn', debris_id: deb.id, zone: deb.zone,
      t_impact: +deb.t_impact.toFixed(1) });
    if (e.assigned) this._unassign(e);
  }

  _tally(shooter, weapon, kind = 'track') {
    let r = this.engage[shooter];
    if (!r) {
      r = { weapon, shots: 0, hits: 0, kills: 0, debris_kills: 0 };
      this.engage[shooter] = r;
    }
    r.shots++;
    if (kind === 'debris') { r.hits++; r.debris_kills++; }
  }

  // ------------------------------------------------------------- auth flow
  _createAuth(shooter, e, effector, p_kill) {
    const id = this.nextAuth++;
    const zone = zoneAt(this.scene, e.pos[0], e.pos[1]);
    // net wrecks fall nearly straight down (low debris throw) -> still
    // geometry_safe over DANGEROUS; projectile debris is not.
    const decision = zone === 'SAFE' ? 'authorized'
      : zone === 'DANGEROUS' ? (effector === 'net' ? 'authorized' : 'hold')
        : 'denied';
    const reason = decision === 'authorized'
      ? (zone === 'SAFE' ? 'geometry_safe' : 'geometry_safe (net capture, low debris throw)')
      : decision === 'hold' ? `debris footprint over ${zone} zone — operator judgement required`
        : `predicted debris in ${zone} zone`;
    const pd = this._pDecoy(e);
    const a = {
      id, t: this.t, shooter, track_id: e.track_id, effector,
      p_kill: +(p_kill * (1 - 0.2 * this.rng())).toFixed(2),
      roe: {
        decision, reason,
        expected_collateral: +({ SAFE: 0.02, DANGEROUS: 0.3, CRITICAL: 0.9 }[zone]).toFixed(2),
      },
      rationale:
        `Engage track #${e.track_id} with ${effector} from ${shooter}. ` +
        `P(decoy)=${pd.toFixed(2)}, predicted wreck zone ${zone}, ` +
        `tti ${(this._tti(e) ?? 0).toFixed(0)} s. ROE: ${decision} (${reason}).`,
      expires_t: this.t + 12,
      resolved: false, approved: null, by: null, enemyId: e.id,
    };
    this.auths.set(id, a);
    this.authStats.requests++;
    this._ops('auth_request', {
      id: a.id, t: a.t, shooter: a.shooter, track_id: a.track_id, effector: a.effector,
      p_kill: a.p_kill, roe: a.roe, rationale: a.rationale, expires_t: a.expires_t,
    });
    if (this.posture === 'weapons_hold') this._resolveAuth(a, false, 'posture');
    return id;
  }

  _resolveAuth(a, approved, by) {
    if (a.resolved) return;
    a.resolved = true; a.approved = approved; a.by = by;
    if (approved) this.authStats.approved++;
    else if (by === 'timeout') this.authStats.expired++;
    else this.authStats.denied++;
    if (by === 'operator' || by === 'orc') this.m.authLat.push(this.t - a.t);
    this._ops('auth_resolved', { id: a.id, approved, by });
    this._decide(by === 'operator' ? 'operator' : by === 'orc' ? 'orc' : 'c2', 'auth',
      `clearance #${a.id} (${a.shooter} → track #${a.track_id}) ` +
      `${approved ? 'APPROVED' : 'REFUSED'} by ${by}`, a.track_id);
    if (approved) {
      const u = this.uavs.find((x) => x.authId === a.id);
      if (u) u.fireAt = this.t + 1.0;
    }
  }

  // --------------------------------------------------------------- helpers
  _daylight() {
    const w = this.req.weather || {};
    return clamp((w.daylight ?? 0.3) + Math.min(0.25, this.t * 0.001), 0, 1);
  }

  _env() {
    const w = this.req.weather || {};
    const ws = w.wind_speed ?? 5, wd = ((w.wind_dir_deg ?? 230) * Math.PI) / 180;
    return {
      wind: [ws * Math.sin(wd), ws * Math.cos(wd), 0],
      fog: clamp((w.fog ?? 0.1) + 0.05 * Math.sin(this.t / 25), 0, 1),
      precip: clamp(w.precip ?? 0, 0, 1),
      daylight: this._daylight(),
    };
  }

  _pDecoy(e) {
    if (e.cls === 'decoy') return e.discr > 3 ? 0.85 : clamp(0.2 + e.discr * 0.2, 0, 0.85);
    return clamp(0.18 - Math.min(0.13, ((this.t - (e.acquired_t ?? this.t)) / 60) * 0.13), 0.04, 0.2);
  }

  _tti(e) {
    const hd = Math.hypot(e.target.pos[0] - e.pos[0], e.target.pos[1] - e.pos[1]);
    return hd / Math.max(1, e.speed);
  }

  _belief(e) {
    const age = clamp((this.t - (e.acquired_t ?? this.t)) / 15, 0, 1);
    const looksLike = (e.cls === 'decoy' && e.discr <= 3) ? 'owa_strategic' : e.cls;
    const b = {};
    for (const c of CLASSES) b[c] = 0.2 * (1 - age) + (c === looksLike ? 0.8 : 0.05) * age;
    const s = CLASSES.reduce((x, c) => x + b[c], 0);
    for (const c of CLASSES) b[c] = +(b[c] / s).toFixed(3);
    return b;
  }

  _event(ev) { this.pendEvents.push({ t: +this.t.toFixed(1), ...ev }); }
  _decide(actor, kind, text, track_id = null, uav_id = null) {
    this.pendDecisions.push({ t: +this.t.toFixed(1), actor, kind, text, track_id, uav_id });
  }

  _byZone(list) {
    const out = { SAFE: 0, DANGEROUS: 0, CRITICAL: 0 };
    for (const w of list) out[w.zone] = (out[w.zone] || 0) + 1;
    return out;
  }

  // --------------------------------------------------------------- emitters
  _emitFrame() {
    const tracks = [];
    for (const e of this.enemies) {
      if (!e.alive || !e.acquired) continue;
      const age = this.t - e.acquired_t;
      const err = 55 / (1 + age / 6);
      const n = (k) => Math.sin(this.t * 0.9 + e.phase * (k + 1)) * err;
      const pd = this._pDecoy(e);
      const tti = this._tti(e);
      tracks.push({
        id: e.track_id,
        pos: [e.pos[0] + n(0), e.pos[1] + n(1), Math.max(4, e.pos[2] + n(2) * 0.4)],
        vel: e.vel.slice(),
        p_decoy: +pd.toFixed(2),
        belief: this._belief(e),
        score: +((1 - pd) * clamp(1 - tti / 150, 0.05, 1)).toFixed(2),
        impact: e.warhead || pd < 0.5 ? [e.target.pos[0], e.target.pos[1], 0] : null,
        tti: +tti.toFixed(1),
      });
    }
    this._ops('frame', {
      t: +this.t.toFixed(1),
      run: { status: this.status, speed: this.speed, posture: this.posture },
      tracks,
      uavs: [
        ...this.uavs.map((u) => ({
          id: u.id, pos: u.pos.map((v) => +v.toFixed(1)), vel: u.vel.map((v) => +v.toFixed(1)),
          mode: u.mode, ammo: u.ammo, battery: +u.battery.toFixed(3),
          task_id: u.task ? (this.enemies.find((e) => e.id === u.task)?.track_id ?? null) : null,
          link: +u.link.toFixed(2), kind: 'interceptor', effector: u.effector,
        })),
        ...this.sentinels.map((s) => ({
          id: s.id, pos: s.pos.map((v) => +v.toFixed(1)), vel: s.vel.map((v) => +v.toFixed(1)),
          mode: s.mode, ammo: 0, battery: +s.battery.toFixed(3),
          task_id: null, link: +s.link.toFixed(2), kind: 'sentinel', effector: null,
        })),
      ],
      turrets: this.turrets.map((tu) => ({
        id: tu.id, az: +tu.az.toFixed(1), el: +tu.el.toFixed(1),
        ammo: tu.ammo, state: tu.state, target: tu.target,
      })),
      wrecks: this.wrecks,
      strays: this.strays,
      debris: this.debris.map((d) => ({
        id: d.id, pos: d.pos.map((v) => +v.toFixed(1)), vel: d.vel.map((v) => +v.toFixed(1)),
        impact: d.impact.map((v) => +v.toFixed(1)), zone: d.zone,
        t_impact: +d.t_impact.toFixed(1),
      })),
      stations: this.scene.stations.map((st) => ({
        id: st.id,
        occupied: [...this.uavs, ...this.sentinels].filter((u) =>
          Math.hypot(u.pos[0] - st.pos[0], u.pos[1] - st.pos[1]) < 30 && u.pos[2] < 40).length,
      })),
      env: this._env(),
      events: this.pendEvents.splice(0),
      decisions: this.pendDecisions.splice(0),
    });
  }

  _metrics() {
    const lat = this.enemies
      .map((e) => ({ id: e.id, cls: e.cls, latency: e.acquired ? +(e.acquired_t - e.spawn_t).toFixed(1) : null }));
    const got = lat.filter((l) => l.latency != null);
    const attr = {};
    for (const c of CLASSES) {
      const of = this.enemies.filter((e) => e.cls === c);
      if (!of.length) continue;
      attr[c] = {
        spawned: of.length,
        killed: of.filter((e) => e.killed).length,
        leaked: of.filter((e) => e.leaker).length,
      };
    }
    return {
      detection: {
        acquired: this.enemies.filter((e) => e.acquired).length,
        total: this.enemies.length,
        latencies: lat,
        mean_latency: got.length
          ? +(got.reduce((s, l) => s + l.latency, 0) / got.length).toFixed(1) : null,
      },
      attrition: attr,
      economics: {
        shots: this.m.shots, kills: this.m.kills,
        ammo_per_kill: this.m.kills ? +(this.m.shots / this.m.kills).toFixed(1) : null,
        decoy_shots: this.m.decoyShots,
      },
      collateral: {
        wrecks_by_zone: this._byZone(this.wrecks),
        strays_by_zone: this._byZone(this.strays),
        debris_cost: +(
          this.wrecks.reduce((s, w) => s + ZCOST[w.zone], 0) +
          this.strays.reduce((s, w) => s + ZCOST[w.zone] * 0.4, 0)).toFixed(0),
        debris_intercepts: this.m2.debrisInt,
        debris_saved_cost: +this.m2.debrisSaved.toFixed(1),
      },
      auth: {
        ...this.authStats,
        mean_latency: this.m.authLat.length
          ? +(this.m.authLat.reduce((a, b) => a + b, 0) / this.m.authLat.length).toFixed(1) : null,
      },
      engagements: {
        by_shooter: this.engage,
        // shooter rows already carry the ICD weapon name (turret_gun)
        by_weapon: Object.values(this.engage).reduce((acc, r) => {
          const row = acc[r.weapon]
            || (acc[r.weapon] = { shots: 0, hits: 0, kills: 0, debris_kills: 0 });
          row.shots += r.shots; row.hits += r.hits;
          row.kills += r.kills; row.debris_kills += r.debris_kills;
          return acc;
        }, {}),
      },
    };
  }

  _emitTruth() {
    this.onEval({
      type: 'truth',
      data: {
        t: +this.t.toFixed(1),
        enemies: this.enemies.map((e) => ({
          id: e.id, cls: e.cls, pos: e.pos.map((v) => +v.toFixed(1)),
          vel: e.vel.map((v) => +v.toFixed(1)),
          alive: e.alive, killed: e.killed, warhead: e.warhead,
          target: e.target.name,
          acquired: e.acquired, acquired_t: e.acquired_t, track_id: e.track_id,
        })),
        metrics: this._metrics(),
      },
    });
  }
}
