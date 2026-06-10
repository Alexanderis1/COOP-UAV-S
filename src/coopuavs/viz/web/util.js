// util.js — shared constants and helpers (no three.js dependency)

export const CLASSES = ['owa_strategic', 'owa_jet', 'fpv', 'loitering', 'decoy'];

export const CLS_COLOR = {
  owa_strategic: 0xff4040, owa_jet: 0xff7a29, fpv: 0xff3d8a,
  loitering: 0xc9342c, decoy: 0xb45cff,
};
export const CLS_CSS = {
  owa_strategic: '#ff4040', owa_jet: '#ff7a29', fpv: '#ff3d8a',
  loitering: '#c9342c', decoy: '#b45cff',
};
export const CLS_SHORT = {
  owa_strategic: 'OWA-A', owa_jet: 'OWA-A+', fpv: 'FPV-B',
  loitering: 'LOIT-C', decoy: 'DECOY-D',
};

export const MODE_COLOR = {
  IDLE: 0x6b7a8f, TRANSIT: 0x8fa3bd, PURSUIT: 0x39d2ff, ENGAGE: 0xff5050,
  BLOCKING: 0xffd166, HERDING: 0xb45cff, PATROL: 0x7cd6c0, RTB: 0x7cfc9a,
  REARM: 0x4f6f5a,
};

// Civilian-presence palette (SRS SIM-ENV-005 / HMI-MAP-007):
// green = civilian-free, yellow = civilians possible, red = civilians present.
export const ZONE_COLORS = ['#2c8a4a', '#d6a531', '#d23c3c'];   // SAFE / DANGEROUS / CRITICAL
export const ZONE_NAMES = ['SAFE', 'DANGEROUS', 'CRITICAL'];
export const ZONE_TINT = { SAFE: 0x49c97c, DANGEROUS: 0xffb347, CRITICAL: 0xff5050 };
export const ZONE_CSS = { SAFE: '#49c97c', DANGEROUS: '#ffb347', CRITICAL: '#ff5050' };
export const ZONE_LEGEND = {
  SAFE: 'civilian-free', DANGEROUS: 'civilians possible', CRITICAL: 'civilians present',
};

export const SENSOR_COLOR = { radar: 0x4fc3f7, rf: 0xb45cff, eo_ir: 0x7cfc9a, acoustic: 0xffd166 };

// Tracer colours per weapon (HMI-MAP-008 attribution).
export const WEAPON_COLOR = {
  projectile: 0xffd166, net: 0x39d2ff, turret_gun: 0xff9a3d,
};

export const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

export function argmaxClass(belief) {
  let best = 'owa_strategic', bp = -Infinity;
  for (const k in (belief || {})) { if (belief[k] > bp) { bp = belief[k]; best = k; } }
  return best;
}

export const fmtT = (t) => (t == null ? '—' : (+t).toFixed(1) + ' s');
export const fmtPct = (p) => (p == null ? '—' : Math.round(p * 100) + '%');
export const fmtNum = (v, d = 1) => (v == null || Number.isNaN(+v) ? '—' : (+v).toFixed(d));

export function esc(s) {
  return String(s ?? '').replace(/[&<>"]/g,
    (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

export function dist3(a, b) {
  const dx = (a[0] || 0) - (b[0] || 0), dy = (a[1] || 0) - (b[1] || 0), dz = (a[2] || 0) - (b[2] || 0);
  return Math.hypot(dx, dy, dz);
}

// Engagement event kinds rendered as "shooter → target [weapon] OUTCOME (pk)"
// in the log (SRS HMI-EVAL-006); everything else falls back to k=v dump.
export const ENGAGEMENT_EVENT = {
  kill: 'KILL', miss: 'MISS', debris_neutralized: 'DEBRIS KILL',
  fire_blocked_los: 'BLOCKED (LOS)', fire_no_target: 'NO TARGET',
};

export function fmtEngagement(ev) {
  const outcome = ENGAGEMENT_EVENT[ev.kind];
  if (!outcome || !ev.uav_id) return null;
  const target = ev.enemy_id ?? ev.debris_id
    ?? (ev.track_id != null ? `track #${ev.track_id}` : '?');
  const weapon = ev.effector ?? '?';
  const pk = ev.pk != null ? ` (pk ${(+ev.pk).toFixed(2)})` : '';
  const saved = ev.saved_zone ? ` saved ${ev.saved_zone}` : '';
  const zone = ev.debris_zone ? ` debris→${ev.debris_zone}` : '';
  return `${ev.uav_id} → ${target} [${weapon}] ${outcome}${pk}${saved}${zone}`;
}

// Zone name at world (x, y); grid row j runs south -> north (matches v0.1 raster).
export function zoneAt(sc, x, y) {
  if (!sc || !sc.grid || !sc.grid.length) return 'DANGEROUS';
  const [x0, y0, x1, y1] = sc.bounds;
  const g = sc.grid, ny = g.length, nx = g[0].length;
  const csx = (x1 - x0) / nx, csy = (y1 - y0) / ny;
  const i = clamp(Math.floor((x - x0) / csx), 0, nx - 1);
  const j = clamp(Math.floor((y - y0) / csy), 0, ny - 1);
  return ZONE_NAMES[g[j][i]] || 'DANGEROUS';
}
