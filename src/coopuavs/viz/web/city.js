// city.js — instanced city rendering (SRS HMI-MAP-007): one InstancedMesh
// per building kind (hundreds of buildings ≈ 8 draw calls), procedural
// facade canvas textures with night-lit windows, kind-distinct roofs
// (hospital red cross, school disc), zone-tinted roof overlays so the red
// civilian-presence zoning stays readable where buildings stand on it,
// zone border outlines, and parks/water as ground patches.
import * as THREE from 'three';
import { ZONE_COLORS, disposeGroup } from './util.js';

const KIND_STYLE = {
  residential_high: { wall: '#8e9099', winRows: 12, winCols: 6, accent: null },
  residential_low: { wall: '#9c7b66', winRows: 3, winCols: 4, accent: null },
  school: { wall: '#c9a35a', winRows: 3, winCols: 8, accent: '#e8c75a', roofMark: 'disc' },
  hospital: { wall: '#cfd4da', winRows: 6, winCols: 8, accent: '#d23c3c', roofMark: 'cross' },
  commercial: { wall: '#5d7d99', winRows: 10, winCols: 8, accent: null, glass: true },
  industrial: { wall: '#707a80', winRows: 1, winCols: 6, accent: null },
};
const FALLBACK_KIND = 'commercial';

function facadeTexture(style) {
  const cnv = document.createElement('canvas');
  cnv.width = 128; cnv.height = 256;
  const ctx = cnv.getContext('2d');
  ctx.fillStyle = style.wall;
  ctx.fillRect(0, 0, 128, 256);
  if (style.glass) {                       // curtain wall: full glass grid
    ctx.fillStyle = '#27384a';
    ctx.fillRect(2, 2, 124, 252);
  }
  // window grid (also drawn as the emissive "lit windows" map sibling)
  const lit = document.createElement('canvas');
  lit.width = 128; lit.height = 256;
  const lctx = lit.getContext('2d');
  lctx.fillStyle = '#000';
  lctx.fillRect(0, 0, 128, 256);
  const rows = style.winRows, cols = style.winCols;
  const cw = 128 / cols, ch = 256 / rows;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const x = c * cw + cw * 0.25, y = r * ch + ch * 0.22;
      const w = cw * 0.5, h = ch * 0.5;
      ctx.fillStyle = style.glass ? '#3d5870' : '#222d3a';
      ctx.fillRect(x, y, w, h);
      // deterministic sparse lighting pattern
      if ((r * 31 + c * 17) % 5 < 2) {
        lctx.fillStyle = '#ffd98a';
        lctx.fillRect(x, y, w, h);
      }
    }
  }
  if (style.accent) {                      // accent band above the ground floor
    ctx.fillStyle = style.accent;
    ctx.fillRect(0, 236, 128, 12);
  }
  const tex = new THREE.CanvasTexture(cnv);
  const litTex = new THREE.CanvasTexture(lit);
  tex.colorSpace = THREE.SRGBColorSpace;
  litTex.colorSpace = THREE.SRGBColorSpace;
  return { tex, litTex };
}

function roofTexture(style) {
  const cnv = document.createElement('canvas');
  cnv.width = cnv.height = 128;
  const ctx = cnv.getContext('2d');
  ctx.fillStyle = '#454c57';
  ctx.fillRect(0, 0, 128, 128);
  ctx.strokeStyle = '#383f49'; ctx.lineWidth = 3;
  ctx.strokeRect(6, 6, 116, 116);
  if (style.roofMark === 'cross') {        // hospital
    ctx.fillStyle = '#e9edf2';
    ctx.beginPath(); ctx.arc(64, 64, 44, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#d23c3c';
    ctx.fillRect(52, 28, 24, 72);
    ctx.fillRect(28, 52, 72, 24);
  } else if (style.roofMark === 'disc') {  // school
    ctx.fillStyle = '#e8c75a';
    ctx.beginPath(); ctx.arc(64, 64, 34, 0, Math.PI * 2); ctx.fill();
  }
  const tex = new THREE.CanvasTexture(cnv);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

export class City {
  constructor(parent, sc) {
    this.root = new THREE.Group();
    parent.add(this.root);
    this.facadeMats = [];                  // for night-window emissive drive
    this.zoneOverlay = null;
    this.borders = null;
    this._build(sc);
  }

  _build(sc) {
    const buildings = sc.buildings || [];
    const byKind = new Map();
    const flats = [];                      // parks/water ground patches
    for (const b of buildings) {
      const kind = b.kind || (b.height > 0 ? FALLBACK_KIND : 'park');
      if (kind === 'park' || kind === 'water' || !(b.height > 0)) {
        flats.push({ ...b, kind });
        continue;
      }
      if (!byKind.has(kind)) byKind.set(kind, []);
      byKind.get(kind).push(b);
    }

    // Unit box with its base at y=0 so per-instance scale = (w, h, d).
    const unit = new THREE.BoxGeometry(1, 1, 1);
    unit.translate(0, 0.5, 0);
    const m4 = new THREE.Matrix4();
    const tint = new THREE.Color();

    for (const [kind, list] of byKind) {
      const style = KIND_STYLE[kind] || KIND_STYLE[FALLBACK_KIND];
      const { tex, litTex } = facadeTexture(style);
      const side = new THREE.MeshLambertMaterial({
        map: tex, emissiveMap: litTex, emissive: 0xffd98a, emissiveIntensity: 0.0,
      });
      const roof = new THREE.MeshLambertMaterial({ map: roofTexture(style) });
      const bottom = new THREE.MeshLambertMaterial({ color: 0x20242c });
      this.facadeMats.push(side);
      // box material order: +x, -x, +y (roof), -y, +z, -z
      const mesh = new THREE.InstancedMesh(
        unit, [side, side, roof, bottom, side, side], list.length);
      list.forEach((b, i) => {
        const [x0, y0, x1, y1] = b.rect;
        m4.makeScale(Math.max(1, x1 - x0), Math.max(1, b.height), Math.max(1, y1 - y0));
        m4.setPosition((x0 + x1) / 2, 0, -((y0 + y1) / 2));
        mesh.setMatrixAt(i, m4);
        // subtle deterministic per-instance shade variance
        const v = 0.88 + 0.24 * (((i * 2654435761) >>> 16) % 100) / 100;
        mesh.setColorAt(i, tint.setScalar(v));
      });
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
      this.root.add(mesh);
    }

    // Parks and water: flat coloured patches just above the zone raster.
    for (const b of flats) {
      const [x0, y0, x1, y1] = b.rect;
      const col = b.kind === 'water' ? 0x274b6d : 0x2e6b3c;
      const m = new THREE.Mesh(
        new THREE.PlaneGeometry(x1 - x0, y1 - y0),
        new THREE.MeshLambertMaterial({ color: col }));
      m.rotation.x = -Math.PI / 2;
      m.position.set((x0 + x1) / 2, b.kind === 'water' ? -0.5 : -0.4, -((y0 + y1) / 2));
      this.root.add(m);
      if (b.kind === 'park') this._trees(b);
    }

    this._roofTints(sc, buildings);
    this._zoneBorders(sc);
  }

  _trees(b) {
    const [x0, y0, x1, y1] = b.rect;
    const w = x1 - x0, d = y1 - y0;
    const n = Math.max(3, Math.min(24, Math.floor((w * d) / 12000)));
    const geo = new THREE.ConeGeometry(3.5, 9, 6);
    geo.translate(0, 7, 0);
    const mesh = new THREE.InstancedMesh(
      geo, new THREE.MeshLambertMaterial({ color: 0x3a7d46 }), n);
    const m4 = new THREE.Matrix4();
    for (let i = 0; i < n; i++) {
      // deterministic scatter (no RNG: replay-stable)
      const fx = ((i * 2654435761) >>> 8) % 1000 / 1000;
      const fy = ((i * 40503 + 17) >>> 4) % 1000 / 1000;
      m4.makeTranslation(x0 + 8 + fx * (w - 16), 0, -(y0 + 8 + fy * (d - 16)));
      mesh.setMatrixAt(i, m4);
    }
    mesh.instanceMatrix.needsUpdate = true;
    this.root.add(mesh);
  }

  // Zone-tinted translucent roof plates on every building standing on
  // non-SAFE ground: fixes "the red zone is hidden under the buildings".
  _roofTints(sc, buildings) {
    const entries = [];
    for (const b of buildings) {
      if (!(b.height > 0)) continue;
      const [x0, y0, x1, y1] = b.rect;
      const zone = zoneIndexAt(sc, (x0 + x1) / 2, (y0 + y1) / 2);
      if (zone <= 0) continue;
      entries.push({ b, zone });
    }
    if (!entries.length) return;
    const unit = new THREE.PlaneGeometry(1, 1);
    unit.rotateX(-Math.PI / 2);
    const mesh = new THREE.InstancedMesh(unit, new THREE.MeshBasicMaterial({
      transparent: true, opacity: 0.55, depthWrite: false,
    }), entries.length);
    const m4 = new THREE.Matrix4();
    const col = new THREE.Color();
    entries.forEach(({ b, zone }, i) => {
      const [x0, y0, x1, y1] = b.rect;
      m4.makeScale(x1 - x0, 1, y1 - y0);
      m4.setPosition((x0 + x1) / 2, b.height + 0.8, -((y0 + y1) / 2));
      mesh.setMatrixAt(i, m4);
      mesh.setColorAt(i, col.set(ZONE_COLORS[zone]));
    });
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    this.root.add(mesh);
    this.zoneOverlay = mesh;
  }

  // Crisp outline segments wherever the zone class changes between cells.
  _zoneBorders(sc) {
    if (!sc.grid || !sc.grid.length) return;
    const [x0, y0, x1, y1] = sc.bounds;
    const g = sc.grid, ny = g.length, nx = g[0].length;
    const csx = (x1 - x0) / nx, csy = (y1 - y0) / ny;
    const pos = [], cols = [];
    const c = new THREE.Color();
    const push = (xa, ya, xb, yb, zone) => {
      c.set(ZONE_COLORS[zone]);
      pos.push(xa, 3, -ya, xb, 3, -yb);
      cols.push(c.r, c.g, c.b, c.r, c.g, c.b);
    };
    for (let j = 0; j < ny; j++) {
      for (let i = 0; i < nx; i++) {
        const z = g[j][i];
        if (i + 1 < nx && g[j][i + 1] !== z) {
          const xe = x0 + (i + 1) * csx, ys = y0 + j * csy;
          push(xe, ys, xe, ys + csy, Math.max(z, g[j][i + 1]));
        }
        if (j + 1 < ny && g[j + 1][i] !== z) {
          const ye = y0 + (j + 1) * csy, xs = x0 + i * csx;
          push(xs, ye, xs + csx, ye, Math.max(z, g[j + 1][i]));
        }
      }
    }
    if (!pos.length) return;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
    this.borders = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({
      vertexColors: true, transparent: true, opacity: 0.8,
    }));
    this.borders.frustumCulled = false;
    this.root.add(this.borders);
  }

  // Night windows glow as daylight falls (driven from SceneView.applyEnv).
  applyDaylight(daylight) {
    const glow = 0.6 * (1 - Math.min(Math.max(daylight ?? 0.35, 0), 1));
    for (const m of this.facadeMats) m.emissiveIntensity = glow;
  }

  setBorders(on) { if (this.borders) this.borders.visible = on; }
  setRoofTints(on) { if (this.zoneOverlay) this.zoneOverlay.visible = on; }

  dispose() {
    disposeGroup(this.root);                 // textures, materials, geometries
    this.root.parent?.remove(this.root);
  }
}

function zoneIndexAt(sc, x, y) {
  if (!sc.grid || !sc.grid.length) return 0;
  const [x0, y0, x1, y1] = sc.bounds;
  const g = sc.grid, ny = g.length, nx = g[0].length;
  const i = Math.min(nx - 1, Math.max(0, Math.floor((x - x0) / ((x1 - x0) / nx))));
  const j = Math.min(ny - 1, Math.max(0, Math.floor((y - y0) / ((y1 - y0) / ny))));
  return g[j][i] || 0;
}
