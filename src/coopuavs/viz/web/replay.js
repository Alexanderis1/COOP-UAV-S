// replay.js — ?replay=1 mode: load GET /recording.json
// ({scene, frames, truth, summary}, ICD §6) and play it back locally with
// timeline scrub + speed. Emits the same {type, data} messages live mode
// would, so the rest of the UI is identical (SRS HMI-003).

const RECORD_HZ = 5;   // recordings are written at 5 Hz sim time (ICD §2.2)

export class Replay {
  constructor({ onOps, onEval, onTimeline }) {
    this.onOps = onOps;
    this.onEval = onEval;
    this.onTimeline = onTimeline;   // (idx, length, playing) -> UI scrub state
    this.frames = [];
    this.truth = null;
    this.summary = null;
    this.idx = 0;
    this.playing = true;
    this.speed = 1;
    this._summaryShown = false;
    this._raf = null;
  }

  async load(url = 'recording.json') {
    const r = await fetch(url);
    if (!r.ok) throw new Error(`GET ${url} -> ${r.status}`);
    const rec = await r.json();
    this.frames = rec.frames || [];
    this.truth = rec.truth || null;
    this.summary = rec.summary || null;
    this.hasEval = Array.isArray(this.truth) && this.truth.length > 0;
    this.onOps({ type: 'scene', data: rec.scene || {} });
    if (rec.scene?.run)
      this.onOps({ type: 'run_started', data: rec.scene.run });
    this.idx = 0;
    this._emit(0);
    this._loop();
  }

  _truthFor(i) {
    if (!this.hasEval) return null;
    if (this.truth[i] !== undefined) return this.truth[i];
    // fallback: align by time
    const t = this.frames[i]?.t ?? 0;
    let best = null;
    for (const tr of this.truth) { if ((tr.t ?? 0) <= t) best = tr; else break; }
    return best;
  }

  _emit(i) {
    const f = this.frames[i];
    if (!f) return;
    this.onOps({ type: 'frame', data: f });
    const tr = this._truthFor(i);
    if (tr) this.onEval({ type: 'truth', data: tr });
    if (i >= this.frames.length - 1 && this.summary && !this._summaryShown) {
      this._summaryShown = true;
      this.onOps({ type: 'summary', data: this.summary });
    }
  }

  _loop() {
    let acc = 0, last = performance.now();
    const tick = (now) => {
      this._raf = requestAnimationFrame(tick);
      acc += ((now - last) / 1000) * RECORD_HZ * this.speed;
      last = now;
      let stepped = false;
      while (this.playing && acc >= 1 && this.idx < this.frames.length - 1) {
        this.idx++; acc -= 1; stepped = true;
      }
      acc = Math.min(acc, 2);
      if (stepped) this._emit(this.idx);
      this.onTimeline?.(this.idx, this.frames.length, this.playing);
    };
    this._raf = requestAnimationFrame(tick);
  }

  seek(i) {
    this.idx = Math.max(0, Math.min(i, this.frames.length - 1));
    this._emit(this.idx);
  }
  setPlaying(b) { this.playing = b; }
  setSpeed(v) { this.speed = Math.max(0.1, +v || 1); }
}
