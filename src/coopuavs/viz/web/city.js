// city.js — instanced city rendering (SRS HMI-MAP-007): one InstancedMesh
// per building kind (hundreds of buildings ≈ a dozen draw calls), procedural
// facade canvas textures (window frames + sky-reflection glazing, balconies,
// curtain-wall mullions, storefronts, baked edge AO) with night-lit windows,
// gravel roofs with parapets and vents, kind-distinct roof marks (hospital
// red cross, school disc), instanced rooftop clutter (HVAC, water tanks),
// soft baked contact shadows, trunk+canopy park/riverbank trees, animated
// river water, zone-tinted roof overlays so the red civilian-presence zoning
// stays readable where buildings stand on it, and zone border outlines.
// Everything is generated once per scene build — no per-frame CPU cost
// beyond one texture-offset update for the water.
import * as THREE from 'three';
import { ZONE_COLORS, disposeGroup } from './util.js';

const KIND_STYLE = {
  residential_high: { wall: '#9a9da6', winRows: 12, winCols: 6, balconies: true, shop: true },
  residential_low: { wall: '#a88267', winRows: 3, winCols: 4, door: true },
  school: { wall: '#cfa75c', winRows: 3, winCols: 8, accent: '#e8c75a', roofMark: 'disc', door: true },
  hospital: { wall: '#d4d9df', winRows: 6, winCols: 8, accent: '#d23c3c', roofMark: 'cross' },
  commercial: { wall: '#46637e', winRows: 10, winCols: 8, glass: true, shop: true },
  industrial: { wall: '#76808a', winRows: 1, winCols: 6, corrugated: true, door: true },
};
const FALLBACK_KIND = 'commercial';

// Deterministic pseudo-random in [0,1) (no RNG: replay-stable textures).
const fr = (i, s = 0) =>
  (((Math.imul(i + 1, 2654435761) ^ Math.imul(s + 101, 40503)) >>> 9) % 65536) / 65536;

// '#rrggbb' lightened (f>1, toward white) or darkened (f<1, toward black).
function shade(hex, f) {
  const n = parseInt(hex.slice(1), 16);
  const ch = (v) => Math.max(0, Math.min(255, Math.round(
    f >= 1 ? v + (255 - v) * (f - 1) : v * f)));
  return `rgb(${ch((n >> 16) & 255)},${ch((n >> 8) & 255)},${ch(n & 255)})`;
}

function speckle(ctx, w, h, n, salt, alpha = 0.06) {
  for (let i = 0; i < n; i++) {
    ctx.globalAlpha = alpha * (0.4 + 0.6 * fr(i, salt + 2));
    ctx.fillStyle = fr(i, salt) < 0.5 ? '#000' : '#fff';
    ctx.fillRect(fr(i, salt + 3) * w, fr(i, salt + 4) * h,
      1 + fr(i, salt + 5) * 2, 1 + fr(i, salt + 6) * 2);
  }
  ctx.globalAlpha = 1;
}

const LIT_PALETTE = ['#ffd98a', '#ffe9bd', '#ffc46a', '#cfe2ff'];

function facadeTexture(style, salt) {
  const W = 256, H = 512;
  const cnv = document.createElement('canvas');
  cnv.width = W; cnv.height = H;
  const ctx = cnv.getContext('2d');
  const lit = document.createElement('canvas');
  lit.width = W; lit.height = H;
  const lctx = lit.getContext('2d');
  lctx.fillStyle = '#000';
  lctx.fillRect(0, 0, W, H);

  // wall: vertical gradient (sunlit top, grimy base) + weathering speckle
  const grad = ctx.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0, shade(style.wall, 1.08));
  grad.addColorStop(0.75, style.wall);
  grad.addColorStop(1, shade(style.wall, 0.78));
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, W, H);
  if (style.corrugated) {                    // industrial cladding ribs
    for (let x = 0; x < W; x += 8) {
      ctx.fillStyle = (x / 8) % 2 ? shade(style.wall, 0.92) : shade(style.wall, 1.05);
      ctx.fillRect(x, 10, 8, H - 10);
    }
  }
  speckle(ctx, W, H, 600, salt);

  // window field between the parapet and the ground-floor band
  const topY = 14;
  const botH = style.shop ? 62 : style.door ? 50 : 20;
  const fieldH = H - topY - botH;
  const rows = style.winRows, cols = style.winCols;
  const cw = W / cols, chh = fieldH / rows;

  if (style.glass) {
    // curtain wall: full glazing gradient + mullion/spandrel grid
    const gg = ctx.createLinearGradient(0, topY, 0, topY + fieldH);
    gg.addColorStop(0, '#7e9ab2');           // sky reflection up high
    gg.addColorStop(0.4, '#4a677f');
    gg.addColorStop(1, '#22323f');
    ctx.fillStyle = gg;
    ctx.fillRect(2, topY, W - 4, fieldH);
    ctx.fillStyle = shade(style.wall, 0.62);
    for (let r = 0; r <= rows; r++)          // spandrel bands per floor
      ctx.fillRect(2, topY + r * chh - 3, W - 4, 6);
    for (let c = 0; c <= cols; c++)          // mullions
      ctx.fillRect(c * cw - 1, topY, 2, fieldH);
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        if (fr(r * cols + c, salt + 7) < 0.3) {
          lctx.fillStyle = LIT_PALETTE[Math.floor(fr(r * cols + c, salt + 8) * 4)];
          lctx.fillRect(c * cw + 2, topY + r * chh + 4, cw - 4, chh - 8);
        }
      }
    }
  } else {
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const x = c * cw + cw * 0.2, y = topY + r * chh + chh * 0.18;
        const w = cw * 0.6, h = chh * 0.55;
        ctx.fillStyle = shade(style.wall, 0.72);            // frame
        ctx.fillRect(x - 2, y - 2, w + 4, h + 4);
        const wg = ctx.createLinearGradient(0, y, 0, y + h); // glazing
        wg.addColorStop(0, '#7e96ad');                       // sky reflection
        wg.addColorStop(0.25, '#36495e');
        wg.addColorStop(1, '#1d2835');
        ctx.fillStyle = wg;
        ctx.fillRect(x, y, w, h);
        ctx.fillStyle = 'rgba(0,0,0,0.35)';                  // sill shadow
        ctx.fillRect(x - 2, y + h + 2, w + 4, 2);
        if (style.balconies) {
          ctx.fillStyle = shade(style.wall, 1.18);           // balcony slab
          ctx.fillRect(x - 6, y + h + 4, w + 12, 5);
          ctx.fillStyle = 'rgba(0,0,0,0.25)';
          ctx.fillRect(x - 6, y + h + 9, w + 12, 3);
        }
        if (fr(r * cols + c, salt + 7) < 0.32) {
          lctx.fillStyle = LIT_PALETTE[Math.floor(fr(r * cols + c, salt + 8) * 4)];
          lctx.fillRect(x, y, w, h);
        }
      }
    }
  }

  // ground floor: storefront glazing / entrance door
  if (style.shop) {
    ctx.fillStyle = '#10151c';
    ctx.fillRect(0, H - botH, W, botH);
    for (let c = 0; c <= 6; c++) {           // structural piers
      ctx.fillStyle = shade(style.wall, 0.85);
      ctx.fillRect(c * (W / 6) - 3, H - botH, 6, botH);
    }
    if (style.accent || style.glass) {       // awning / fascia strip
      ctx.fillStyle = style.accent || '#3d5870';
      ctx.fillRect(0, H - botH, W, 8);
    }
    for (let c = 0; c < 6; c++) {            // shopfronts glow at night
      if (fr(c, salt + 9) < 0.7) {
        lctx.fillStyle = '#ffe2a8';
        lctx.fillRect(c * (W / 6) + 6, H - botH + 12, W / 6 - 12, botH - 16);
      }
    }
  } else if (style.door) {
    ctx.fillStyle = '#1a212b';
    ctx.fillRect(W / 2 - 16, H - botH + 6, 32, botH - 6);
    ctx.fillStyle = shade(style.wall, 0.72);
    ctx.fillRect(W / 2 - 19, H - botH + 2, 38, 4);
  }
  if (style.accent) {                        // accent band above ground floor
    ctx.fillStyle = style.accent;
    ctx.fillRect(0, H - botH - 12, W, 10);
  }

  // baked AO: dark parapet cap, base grime, side-edge falloff
  ctx.fillStyle = shade(style.wall, 0.62);
  ctx.fillRect(0, 0, W, 7);
  const base = ctx.createLinearGradient(0, H - 26, 0, H);
  base.addColorStop(0, 'rgba(0,0,0,0)');
  base.addColorStop(1, 'rgba(0,0,0,0.32)');
  ctx.fillStyle = base;
  ctx.fillRect(0, H - 26, W, 26);
  for (const [x0, x1] of [[0, 7], [W - 7, W]]) {
    const eg = ctx.createLinearGradient(x0, 0, x1, 0);
    eg.addColorStop(x0 ? 1 : 0, 'rgba(0,0,0,0.28)');
    eg.addColorStop(x0 ? 0 : 1, 'rgba(0,0,0,0)');
    ctx.fillStyle = eg;
    ctx.fillRect(x0, 0, 7, H);
  }

  const tex = new THREE.CanvasTexture(cnv);
  const litTex = new THREE.CanvasTexture(lit);
  tex.colorSpace = THREE.SRGBColorSpace;
  litTex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = litTex.anisotropy = 4;
  return { tex, litTex };
}

function roofTexture(style, salt) {
  const S = 256;
  const cnv = document.createElement('canvas');
  cnv.width = cnv.height = S;
  const ctx = cnv.getContext('2d');
  ctx.fillStyle = '#474d56';
  ctx.fillRect(0, 0, S, S);
  speckle(ctx, S, S, 900, salt + 20, 0.1);   // gravel
  for (let i = 0; i < 3; i++) {              // membrane seams
    ctx.fillStyle = 'rgba(0,0,0,0.18)';
    ctx.fillRect(30 + fr(i, salt + 30) * (S - 60), 16, 1, S - 32);
  }
  ctx.strokeStyle = '#5d646e'; ctx.lineWidth = 10;   // parapet ledge
  ctx.strokeRect(7, 7, S - 14, S - 14);
  ctx.strokeStyle = 'rgba(0,0,0,0.4)'; ctx.lineWidth = 2;
  ctx.strokeRect(14, 14, S - 28, S - 28);
  for (let i = 0; i < 5; i++) {              // vents / skylight boxes
    const x = 30 + fr(i, salt + 40) * (S - 84), y = 30 + fr(i, salt + 41) * (S - 84);
    if (style.roofMark && Math.hypot(x + 12 - S / 2, y + 12 - S / 2) < 110) continue;
    const w = 14 + fr(i, salt + 42) * 14;
    ctx.fillStyle = 'rgba(0,0,0,0.35)';
    ctx.fillRect(x + 3, y + 3, w, w);        // drop shadow
    ctx.fillStyle = '#5a6069';
    ctx.fillRect(x, y, w, w);
    ctx.fillStyle = '#6a727c';
    ctx.fillRect(x, y, w, 3);
  }
  if (style.roofMark === 'cross') {          // hospital
    ctx.fillStyle = '#e9edf2';
    ctx.beginPath(); ctx.arc(128, 128, 88, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#d23c3c';
    ctx.fillRect(104, 56, 48, 144);
    ctx.fillRect(56, 104, 144, 48);
  } else if (style.roofMark === 'disc') {    // school
    ctx.fillStyle = '#e8c75a';
    ctx.beginPath(); ctx.arc(128, 128, 68, 0, Math.PI * 2); ctx.fill();
  }
  const tex = new THREE.CanvasTexture(cnv);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

// Soft rectangular blob for baked contact shadows.
function shadowTexture() {
  const S = 128;
  const cnv = document.createElement('canvas');
  cnv.width = cnv.height = S;
  const ctx = cnv.getContext('2d');
  ctx.fillStyle = '#000';
  for (let k = 0; k < 14; k++) {             // stacked insets ≈ soft edge
    ctx.globalAlpha = 0.1;
    const inset = 2 + k * 3.2;
    ctx.fillRect(inset, inset, S - 2 * inset, S - 2 * inset);
  }
  ctx.globalAlpha = 1;
  return new THREE.CanvasTexture(cnv);
}

function grassTexture() {
  const S = 64;
  const cnv = document.createElement('canvas');
  cnv.width = cnv.height = S;
  const ctx = cnv.getContext('2d');
  ctx.fillStyle = '#2e5d38';
  ctx.fillRect(0, 0, S, S);
  for (let i = 0; i < 240; i++) {
    ctx.fillStyle = ['#27502f', '#356741', '#2a5836', '#3d7048'][Math.floor(fr(i, 50) * 4)];
    ctx.fillRect(fr(i, 51) * S, fr(i, 52) * S, 2, 2);
  }
  const tex = new THREE.CanvasTexture(cnv);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(8, 8);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

function waterTexture() {
  const S = 128;
  const cnv = document.createElement('canvas');
  cnv.width = cnv.height = S;
  const ctx = cnv.getContext('2d');
  ctx.fillStyle = '#21466b';
  ctx.fillRect(0, 0, S, S);
  for (let i = 0; i < 160; i++) {            // streaky ripples
    ctx.globalAlpha = 0.05 + 0.07 * fr(i, 60);
    ctx.fillStyle = fr(i, 61) < 0.7 ? '#6f9cc4' : '#16334e';
    ctx.fillRect(fr(i, 62) * S, fr(i, 63) * S, 8 + fr(i, 64) * 26, 1 + fr(i, 65) * 2);
  }
  ctx.globalAlpha = 1;
  const tex = new THREE.CanvasTexture(cnv);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
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
    this.shadowMat = null;                 // contact shadows (daylight drive)
    this.waterTex = null;                  // animated in tick()
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

    let salt = 0;
    for (const [kind, list] of byKind) {
      salt += 13;
      const style = KIND_STYLE[kind] || KIND_STYLE[FALLBACK_KIND];
      const { tex, litTex } = facadeTexture(style, salt);
      const side = new THREE.MeshLambertMaterial({
        map: tex, emissiveMap: litTex, emissive: 0xffffff, emissiveIntensity: 0.0,
      });
      const roof = new THREE.MeshLambertMaterial({ map: roofTexture(style, salt) });
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
        // deterministic per-instance shade + slight warm/cool hue variance
        const v = 0.88 + 0.2 * fr(i, salt + 1);
        tint.setRGB(v * (0.97 + 0.06 * fr(i, salt + 2)), v,
          v * (0.97 + 0.06 * fr(i, salt + 3)));
        mesh.setColorAt(i, tint);
      });
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
      this.root.add(mesh);
    }

    // Parks and water: textured ground patches just above the terrain.
    const grass = flats.some((b) => b.kind === 'park')
      ? new THREE.MeshLambertMaterial({ map: grassTexture() }) : null;
    let water = null;
    for (const b of flats) {
      const [x0, y0, x1, y1] = b.rect;
      let mat = grass;
      if (b.kind === 'water') {
        if (!water) {
          this.waterTex = waterTexture();
          water = new THREE.MeshPhongMaterial({
            map: this.waterTex, specular: 0x9db8d4, shininess: 70,
          });
        }
        this.waterTex.repeat.set(Math.max(2, (x1 - x0) / 280), Math.max(1, (y1 - y0) / 280));
        mat = water;
      }
      const m = new THREE.Mesh(new THREE.PlaneGeometry(x1 - x0, y1 - y0), mat);
      m.rotation.x = -Math.PI / 2;
      m.position.set((x0 + x1) / 2, b.kind === 'water' ? -0.5 : -0.4, -((y0 + y1) / 2));
      this.root.add(m);
    }

    this._vegetation(flats);
    this._contactShadows(buildings);
    this._roofClutter(buildings);
    this._roofTints(sc, buildings);
    this._zoneBorders(sc);
  }

  // Trees with trunks and canopies across all parks (and along riverbanks),
  // batched into two InstancedMeshes total.
  _vegetation(flats) {
    const spots = [];
    for (const b of flats) {
      const [x0, y0, x1, y1] = b.rect;
      const w = x1 - x0, d = y1 - y0;
      if (b.kind === 'park') {
        const n = Math.max(4, Math.min(40, Math.floor((w * d) / 7000)));
        for (let i = 0; i < n; i++) {
          spots.push([x0 + 10 + fr(i, 70 + spots.length) * (w - 20),
            y0 + 10 + fr(i, 71 + spots.length) * (d - 20)]);
        }
      } else if (b.kind === 'water' && w > 1500) {
        // tree rows on the unbuilt riverbanks
        for (let x = x0 + 90; x < x1 - 90; x += 170) {
          const i = Math.floor(x);
          spots.push([x + (fr(i, 72) - 0.5) * 60, y0 - 30 - fr(i, 73) * 40]);
          spots.push([x + (fr(i, 74) - 0.5) * 60, y1 + 30 + fr(i, 75) * 40]);
        }
      }
      if (spots.length > 560) break;
    }
    if (!spots.length) return;
    const n = Math.min(spots.length, 560);

    const trunkGeo = new THREE.CylinderGeometry(0.5, 0.7, 1, 5);
    trunkGeo.translate(0, 0.5, 0);
    const canopyGeo = new THREE.IcosahedronGeometry(1, 1);
    const trunks = new THREE.InstancedMesh(
      trunkGeo, new THREE.MeshLambertMaterial({ color: 0x4a3a28 }), n);
    const canopies = new THREE.InstancedMesh(
      canopyGeo, new THREE.MeshLambertMaterial({ color: 0xffffff }), n);
    const m4 = new THREE.Matrix4();
    const col = new THREE.Color();
    for (let i = 0; i < n; i++) {
      const [x, y] = spots[i];
      const th = 2.5 + 2.5 * fr(i, 76);      // trunk height
      const cr = 2.4 + 2.4 * fr(i, 77);      // canopy radius
      m4.makeScale(1, th, 1);
      m4.setPosition(x, 0, -y);
      trunks.setMatrixAt(i, m4);
      m4.makeScale(cr, cr * 0.85, cr);
      m4.setPosition(x, th + cr * 0.55, -y);
      canopies.setMatrixAt(i, m4);
      canopies.setColorAt(i, col.setHSL(0.28 + 0.06 * fr(i, 78), 0.45,
        0.22 + 0.13 * fr(i, 79)));
    }
    trunks.instanceMatrix.needsUpdate = true;
    canopies.instanceMatrix.needsUpdate = true;
    if (canopies.instanceColor) canopies.instanceColor.needsUpdate = true;
    this.root.add(trunks, canopies);
  }

  // Baked soft contact shadows under every building: grounds the boxes
  // without a real shadow map (a 12 km scene is far beyond useful shadow-map
  // resolution, and this costs one draw call and zero per-frame work).
  _contactShadows(buildings) {
    const list = buildings.filter((b) => b.height > 0);
    if (!list.length) return;
    const unit = new THREE.PlaneGeometry(1, 1);
    unit.rotateX(-Math.PI / 2);
    this.shadowMat = new THREE.MeshBasicMaterial({
      map: shadowTexture(), transparent: true, opacity: 0.3, depthWrite: false,
    });
    const mesh = new THREE.InstancedMesh(unit, this.shadowMat, list.length);
    const m4 = new THREE.Matrix4();
    list.forEach((b, i) => {
      const [x0, y0, x1, y1] = b.rect;
      const w = x1 - x0, d = y1 - y0;
      m4.makeScale(w * 1.22, 1, d * 1.22);
      // sun sits east/south-ish: skew the blob west/north
      m4.setPosition((x0 + x1) / 2 - w * 0.05, 0.35, -((y0 + y1) / 2 + d * 0.05));
      mesh.setMatrixAt(i, m4);
    });
    mesh.instanceMatrix.needsUpdate = true;
    this.root.add(mesh);
  }

  // Rooftop HVAC boxes + water tanks on larger buildings (two draw calls).
  _roofClutter(buildings) {
    const units = [], tanks = [];
    buildings.forEach((b, bi) => {
      if (!(b.height >= 14)) return;
      const [x0, y0, x1, y1] = b.rect;
      const w = x1 - x0, d = y1 - y0;
      if (w < 24 || d < 24) return;
      const n = 1 + Math.floor(fr(bi, 80) * 2.9);
      for (let k = 0; k < n && units.length < 900; k++) {
        const s = bi * 7 + k;
        units.push({
          x: x0 + w * (0.18 + 0.64 * fr(s, 81)), y: y0 + d * (0.18 + 0.64 * fr(s, 82)),
          h: b.height, sx: 3 + 3.5 * fr(s, 83), sy: 2 + 2.2 * fr(s, 84),
          sz: 3 + 3.5 * fr(s, 85),
        });
      }
      if ((b.kind === 'residential_high' || b.kind === 'commercial')
          && b.height > 30 && fr(bi, 86) < 0.5 && tanks.length < 300) {
        tanks.push({
          x: x0 + w * (0.25 + 0.5 * fr(bi, 87)), y: y0 + d * (0.25 + 0.5 * fr(bi, 88)),
          h: b.height, r: 1.8 + 1.2 * fr(bi, 89),
        });
      }
    });
    const m4 = new THREE.Matrix4();
    if (units.length) {
      const geo = new THREE.BoxGeometry(1, 1, 1);
      geo.translate(0, 0.5, 0);
      const mesh = new THREE.InstancedMesh(
        geo, new THREE.MeshLambertMaterial({ color: 0x5a616a }), units.length);
      units.forEach((u, i) => {
        m4.makeScale(u.sx, u.sy, u.sz);
        m4.setPosition(u.x, u.h, -u.y);
        mesh.setMatrixAt(i, m4);
      });
      mesh.instanceMatrix.needsUpdate = true;
      this.root.add(mesh);
    }
    if (tanks.length) {
      const geo = new THREE.CylinderGeometry(1, 1, 1, 10);
      geo.translate(0, 0.5, 0);
      const mesh = new THREE.InstancedMesh(
        geo, new THREE.MeshLambertMaterial({ color: 0x79828c }), tanks.length);
      tanks.forEach((t, i) => {
        m4.makeScale(t.r, 3.4, t.r);
        m4.setPosition(t.x, t.h, -t.y);
        mesh.setMatrixAt(i, m4);
      });
      mesh.instanceMatrix.needsUpdate = true;
      this.root.add(mesh);
    }
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

  // Night windows glow as daylight falls; contact shadows fade with the sun
  // (driven from SceneView.applyEnv).
  applyDaylight(daylight) {
    const d = Math.min(Math.max(daylight ?? 0.35, 0), 1);
    const glow = 0.85 * (1 - d);
    for (const m of this.facadeMats) m.emissiveIntensity = glow;
    if (this.shadowMat) this.shadowMat.opacity = 0.08 + 0.28 * d;
  }

  // Per-frame: drift the river ripple texture (one offset update, no draws).
  tick(nowMs) {
    if (this.waterTex) {
      this.waterTex.offset.set((nowMs * 1.2e-5) % 1, (nowMs * 5e-6) % 1);
    }
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
