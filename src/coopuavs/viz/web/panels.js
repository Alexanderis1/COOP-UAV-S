// panels.js — DOM panels: run control, new-execution form (HMI-SCN-002),
// authorisation queue (HMI-AUT-001/002), entity inspector (HMI-MAP-004),
// event & decision log, eval metrics (HMI-EVAL-005), layer toggles, modal.
import {
  CLASSES, CLS_CSS, CLS_SHORT, ZONE_CSS, argmaxClass, clamp, dist3, esc,
  fmtNum, fmtPct,
} from './util.js';

const $ = (id) => document.getElementById(id);

const LAYERS = [
  { key: 'coverage', label: 'sensor coverage' },
  { key: 'rings', label: 'turret range rings' },
  { key: 'impacts', label: 'predicted impacts' },
  { key: 'ghosts', label: 'ghost threats (eval)' },
  { key: 'trails', label: 'vector trails' },
  { key: 'labels', label: 'labels' },
  { key: 'grid', label: 'risk-zone raster' },
];

export class Panels {
  /**
   * deps: { state, send(type,data), focus(sel), setLayer(key,on), mode }
   * state: { scene, frame, truth, evalOn, selection, auth:Map, runInfo }
   */
  constructor(deps) {
    Object.assign(this, deps);
    this.replayApi = null;
    this.logFilter = 'all';
    this.logEntries = 0;
    this._spdDrag = false;
    this._lastMetricsRender = 0;
    this._toastTimer = null;
    this._buildExecForm();
    this._buildLayers();
    this._wire();
    $('mode-pill').textContent = this.mode.toUpperCase();
    if (this.mode === 'mock') $('ops-pill').style.display = 'none';
  }

  // ================================================================ wiring
  _wire() {
    // collapsible sections
    for (const sec of document.querySelectorAll('.dock section'))
      sec.querySelector('h2').addEventListener('click', () => sec.classList.toggle('collapsed'));
    $('sec-metrics').classList.add('collapsed');

    // --- run control
    const spd = $('spd');
    spd.addEventListener('pointerdown', () => { this._spdDrag = true; });
    spd.addEventListener('pointerup', () => { this._spdDrag = false; });
    spd.addEventListener('input', () => { $('spd-val').textContent = this._sliderSpeed().toFixed(1) + '×'; });
    spd.addEventListener('change', () => this.send('set_speed', { speed: this._sliderSpeed() }));
    $('btn-pause').addEventListener('click', () => {
      const st = this.state.frame?.run?.status;
      this.send(st === 'paused' ? 'resume' : 'pause', {});
    });
    $('btn-stop').addEventListener('click', () => this.send('stop_run', {}));
    $('posture-sel').addEventListener('change', (e) =>
      this.send('set_posture', { posture: e.target.value }));

    // --- replay timeline (local control, HMI-MAP-003)
    if (this.mode === 'replay') {
      $('replay-ctl').style.display = 'block';
      for (const id of ['spd', 'btn-pause', 'btn-stop', 'posture-sel', 'btn-launch'])
        $(id).disabled = true;
      $('rp-play').addEventListener('click', () => {
        if (!this.replayApi) return;
        const p = !this.replayApi.playing;
        this.replayApi.setPlaying(p);
        $('rp-play').textContent = p ? '⏸' : '▶';
      });
      $('rp-seek').addEventListener('input', (e) => {
        this.replayApi?.setPlaying(false);
        $('rp-play').textContent = '▶';
        this.replayApi?.seek(+e.target.value);
      });
      $('rp-speed').addEventListener('change', (e) => this.replayApi?.setSpeed(+e.target.value));
    }

    // --- launch
    $('btn-launch').addEventListener('click', () => this._launch());

    // --- auth queue (delegated)
    $('auth-list').addEventListener('click', (e) => {
      const card = e.target.closest('.auth-card');
      if (!card) return;
      const id = +card.dataset.id;
      if (e.target.matches('button.approve, button.deny')) {
        this.send('authorize', { id, approve: e.target.classList.contains('approve') });
        e.stopPropagation();
        return;
      }
      const rec = this.state.auth.get(id);
      if (rec) this.focus({ track_id: rec.req.track_id, uav_id: rec.req.shooter });
    });

    // --- log filters
    for (const f of ['all', 'events', 'decisions'])
      $(`log-filt-${f}`).addEventListener('click', () => {
        this.logFilter = f;
        for (const g of ['all', 'events', 'decisions'])
          $(`log-filt-${g}`).classList.toggle('active', g === f);
        this._applyLogFilter();
      });
    $('log-clear').addEventListener('click', () => { $('log-list').innerHTML = ''; });
    $('log-list').addEventListener('click', (e) => {
      const d = e.target.closest('div.focusable');
      if (!d) return;
      this.focus({
        track_id: d.dataset.track ? +d.dataset.track : null,
        uav_id: d.dataset.uav || null,
      });
    });

    // --- inspector RTB (delegated)
    $('inspect-body').addEventListener('click', (e) => {
      if (e.target.id === 'btn-rtb' && this.state.selection?.kind === 'uav')
        this.send('uav_command', { uav_id: this.state.selection.id, command: 'rtb' });
    });

    // --- eval badge toggles ghosts globally (HMI-EVAL-004)
    $('eval-badge').addEventListener('click', () => {
      const cb = document.querySelector('#layers-list input[data-key="ghosts"]');
      if (cb && !cb.disabled) { cb.checked = !cb.checked; cb.dispatchEvent(new Event('change')); }
    });

    $('summary-close').addEventListener('click', () => $('summary-modal').classList.remove('open'));
  }

  _sliderSpeed() {  // slider 0..100 -> 0.1..10 (log scale)
    return +Math.pow(10, $('spd').value / 50 - 1).toFixed(1);
  }
  _speedToSlider(s) { return Math.round((Math.log10(clamp(s, 0.1, 10)) + 1) * 50); }

  // ============================================================ exec form
  _buildExecForm() {
    const rows = CLASSES.map((c) => `
      <tr>
        <td><span class="cls-tag" style="background:${CLS_CSS[c]}"></span>${CLS_SHORT[c]}</td>
        <td><input type="number" min="0" max="60" value="0" id="x-cnt-${c}"></td>
        <td><select id="x-tgt-${c}" class="x-target"><option value="auto">auto</option></select></td>
        <td><input type="number" placeholder="—" id="x-axis-${c}" title="approach axis deg"></td>
        <td><input type="number" min="0" value="5" id="x-t0-${c}" title="first_time s"></td>
        <td><input type="number" min="0" value="8" id="x-gap-${c}" title="spacing s"></td>
      </tr>`).join('');
    $('exec-classes').innerHTML = `
      <table>
        <tr><th>class</th><th>n</th><th>target</th><th>axis°</th><th>t₀</th><th>gap</th></tr>
        ${rows}
      </table>`;
    const w = [
      ['w-wind', 'wind m/s', 0, 20, 0.5, 5],
      ['w-winddir', 'wind dir °', 0, 360, 5, 230],
      ['w-fog', 'fog', 0, 1, 0.05, 0.1],
      ['w-precip', 'precip', 0, 1, 0.05, 0],
      ['w-day', 'daylight', 0, 1, 0.05, 0.3],
    ];
    $('exec-weather').innerHTML = w.map(([id, lab, lo, hi, st, dv]) => `
      <div class="wrow"><label>${lab}</label>
        <input type="range" id="${id}" min="${lo}" max="${hi}" step="${st}" value="${dv}">
        <output id="${id}-o">${dv}</output></div>`).join('');
    for (const [id] of w)
      $(id).addEventListener('input', () => { $(`${id}-o`).textContent = $(id).value; });
  }

  _launch() {
    const threats = {};
    for (const c of CLASSES) {
      const n = +$(`x-cnt-${c}`).value || 0;
      if (n <= 0) continue;
      const axis = $(`x-axis-${c}`).value;
      threats[c] = {
        count: n,
        target: $(`x-tgt-${c}`).value,
        axis_deg: axis === '' ? null : +axis,
        first_time: +$(`x-t0-${c}`).value || 0,
        spacing: +$(`x-gap-${c}`).value || 0,
      };
    }
    if (!Object.keys(threats).length) {
      this.showExecError('at least one threat class needs count > 0');
      return;
    }
    const seedRaw = $('x-seed').value.trim();
    const seed = seedRaw === '' || Number.isNaN(parseInt(seedRaw, 10))
      ? null : parseInt(seedRaw, 10);
    this.showExecError(null);
    this.send('start_run', {
      threats,
      weather: {
        wind_speed: +$('w-wind').value, wind_dir_deg: +$('w-winddir').value,
        fog: +$('w-fog').value, precip: +$('w-precip').value, daylight: +$('w-day').value,
      },
      duration: +$('x-duration').value || null,
      speed: clamp(+$('x-speed').value || 1, 0.1, 10),
      seed,
      posture: $('x-posture').value,
    });
  }

  showExecError(msg) {
    const el = $('exec-err');
    el.style.display = msg ? 'block' : 'none';
    el.textContent = msg || '';
  }

  // =============================================================== layers
  _buildLayers() {
    $('layers-list').innerHTML = LAYERS.map((l) => `
      <div class="row"><label>${l.label}</label>
        <input type="checkbox" data-key="${l.key}" checked></div>`).join('');
    $('layers-list').addEventListener('change', (e) => {
      const cb = e.target;
      if (cb.dataset.key) {
        this.setLayer(cb.dataset.key, cb.checked);
        if (cb.dataset.key === 'ghosts')
          $('eval-badge').classList.toggle('ghosts-off', !cb.checked);
      }
    });
  }

  setEval(on) {
    $('eval-badge').style.display = on ? 'inline-block' : 'none';
    $('metrics-offline').style.display = on ? 'none' : 'block';
    if (!on) $('metrics-body').innerHTML = '';
    const cb = document.querySelector('#layers-list input[data-key="ghosts"]');
    if (cb) cb.disabled = !on;
  }

  setOpsStatus(on) {
    const p = $('ops-pill');
    p.className = 'pill ' + (on ? 'on' : 'off');
    p.textContent = on ? 'OPS ✓' : 'OPS ✗ reconnecting';
  }

  setReplay(api) { this.replayApi = api; }
  updateTimeline(idx, len) {
    const sk = $('rp-seek');
    if (+sk.max !== len - 1) sk.max = Math.max(0, len - 1);
    if (document.activeElement !== sk) sk.value = idx;
    $('rp-pos').textContent = `${idx} / ${Math.max(0, len - 1)}`;
  }

  // ============================================================ run state
  onRunStarted(d) {
    $('run-seed').textContent = d?.seed ?? '—';
    $('exec-seed-echo').textContent = d?.seed ?? '—';
    this.showExecError(null);
    this.renderAuth();
    this.addDecision({ t: 0, actor: 'c2', kind: 'run', text: `run "${d?.name ?? '?'}" started — seed ${d?.seed}` });
  }

  onError(d) {
    const msg = d?.message || 'backend error';
    this.showExecError(msg);
    this.toast(msg);
  }

  onFrame(f) {
    const run = f.run || {};
    const st = run.status || 'idle';
    $('top-time').textContent = (f.t ?? 0).toFixed(1) + ' s';
    $('top-status').textContent = st;
    $('run-time').textContent = (f.t ?? 0).toFixed(1) + ' s';
    const rs = $('run-status');
    rs.textContent = st; rs.className = 'val ' + st;
    $('btn-pause').textContent = st === 'paused' ? 'RESUME' : 'PAUSE';
    if (!this._spdDrag && run.speed != null) {
      $('spd').value = this._speedToSlider(run.speed);
      $('spd-val').textContent = (+run.speed).toFixed(1) + '×';
    }
    const ps = $('posture-sel');
    if (run.posture && document.activeElement !== ps) ps.value = run.posture;

    for (const ev of f.events || []) this.addEvent(ev);
    for (const dc of f.decisions || []) this.addDecision(dc);
    this._tickAuthCountdowns(f.t ?? 0);
    this.renderInspector();
  }

  // ================================================== auth queue (HMI-AUT)
  onAuthRequest(d) {
    this.state.auth.set(d.id, { req: d, resolved: null });
    this.renderAuth();
  }
  onAuthResolved(d) {
    const rec = this.state.auth.get(d.id);
    if (rec) rec.resolved = d;
    this.renderAuth();
  }

  renderAuth() {
    const list = [...this.state.auth.values()].sort((a, b) => b.req.id - a.req.id);
    const pending = list.filter((r) => !r.resolved);
    const resolved = list.filter((r) => r.resolved).slice(0, 12);
    $('auth-empty').style.display = pending.length ? 'none' : 'block';
    const html = [];
    for (const r of pending) {
      const q = r.req, roe = q.roe || {};
      html.push(`
        <div class="auth-card ${roe.decision === 'authorized' ? 'authorized' : roe.decision === 'denied' ? 'denied-roe' : ''}" data-id="${q.id}">
          <div class="row"><b>#${q.id} ${esc(q.shooter)} → track #${q.track_id}</b>
            <span class="cd" data-cd="${q.id}">—</span></div>
          <div class="row"><span>${esc(q.effector)} · Pk ${fmtPct(q.p_kill)}</span>
            <span>ROE: <b>${esc(roe.decision ?? '?')}</b></span></div>
          <div class="row"><span>collateral ${fmtNum(roe.expected_collateral, 2)}</span>
            <span>${esc(roe.reason ?? '')}</span></div>
          <div class="rationale">${esc(q.rationale ?? '')}</div>
          <div class="timebar"><div data-tb="${q.id}"></div></div>
          <div class="auth-actions">
            <button class="approve">APPROVE</button>
            <button class="deny">DENY</button>
          </div>
        </div>`);
    }
    for (const r of resolved) {
      const q = r.req, ok = r.resolved.approved;
      html.push(`
        <div class="auth-card resolved ${ok ? 'approved' : 'rejected'}" data-id="${q.id}">
          #${q.id} ${esc(q.shooter)} → track #${q.track_id} —
          <b>${ok ? 'APPROVED' : 'REFUSED'}</b> by ${esc(r.resolved.by)}
        </div>`);
    }
    $('auth-list').innerHTML = html.join('');
  }

  _tickAuthCountdowns(simT) {
    for (const r of this.state.auth.values()) {
      if (r.resolved) continue;
      const el = document.querySelector(`[data-cd="${r.req.id}"]`);
      const tb = document.querySelector(`[data-tb="${r.req.id}"]`);
      if (!el) continue;
      const total = Math.max(0.1, r.req.expires_t - r.req.t);
      const remain = r.req.expires_t - simT;
      el.textContent = remain > 0 ? `expires ${remain.toFixed(1)} s` : 'EXPIRED';
      el.classList.toggle('late', remain < 4);
      if (tb) tb.style.width = clamp((remain / total) * 100, 0, 100) + '%';
    }
  }

  // ================================================ inspector (HMI-MAP-004)
  renderInspector() {
    const sel = this.state.selection;
    const body = $('inspect-body');
    $('inspect-empty').style.display = sel ? 'none' : 'block';
    if (!sel) { body.innerHTML = ''; return; }
    const f = this.state.frame || {};
    let html = '';

    if (sel.kind === 'uav') {
      const u = (f.uavs || []).find((x) => x.id === sel.id);
      if (!u) { body.innerHTML = `<div class="row"><label>${esc(sel.id)}</label><span>no data</span></div>`; return; }
      const spd = Math.hypot(...(u.vel || [0, 0, 0]));
      html = `
        <div class="subhead">INTERCEPTOR ${esc(u.id)}</div>
        <div class="row"><label>mode</label><span class="val">${esc(u.mode)}</span></div>
        <div class="row"><label>battery</label><span class="val">${fmtPct(u.battery)}</span></div>
        <div class="row"><label>ammo</label><span class="val">${u.ammo ?? '—'}</span></div>
        <div class="row"><label>task</label><span class="val">${u.task_id != null ? 'track #' + u.task_id : '—'}</span></div>
        <div class="row"><label>link</label><span class="val">${fmtPct(u.link)}</span></div>
        <div class="row"><label>speed</label><span class="val">${spd.toFixed(0)} m/s</span></div>
        <div class="row"><label>alt</label><span class="val">${(u.pos?.[2] ?? 0).toFixed(0)} m</span></div>
        <button id="btn-rtb" class="deny" style="width:100%;margin-top:6px">⏎ RETURN TO BASE</button>`;
    } else if (sel.kind === 'track') {
      const t = (f.tracks || []).find((x) => x.id === sel.id);
      if (!t) { body.innerHTML = `<div class="row"><label>track #${sel.id}</label><span>dropped</span></div>`; return; }
      const cls = argmaxClass(t.belief);
      const bars = CLASSES.map((c) => `
        <div class="bar-row"><label>${CLS_SHORT[c]}</label>
          <div class="bar"><span style="width:${((t.belief?.[c] || 0) * 100).toFixed(0)}%;background:${CLS_CSS[c]}"></span></div>
          <span>${fmtPct(t.belief?.[c] || 0)}</span></div>`).join('');
      html = `
        <div class="subhead">TRACK #${t.id} · ${esc(cls)}</div>
        ${bars}
        <div class="row"><label>p_decoy</label><span class="val">${fmtPct(t.p_decoy)}</span></div>
        <div class="row"><label>score</label><span class="val">${fmtNum(t.score, 2)}</span></div>
        <div class="row"><label>tti</label><span class="val">${t.tti != null ? t.tti.toFixed(1) + ' s' : '—'}</span></div>
        <div class="row"><label>impact</label><span class="val">${Array.isArray(t.impact)
    ? `${t.impact[0].toFixed(0)}, ${t.impact[1].toFixed(0)}` : '—'}</span></div>
        <div class="row"><label>speed</label><span class="val">${Math.hypot(...(t.vel || [0])).toFixed(0)} m/s</span></div>`;
      // truth-vs-track deltas, eval only (HMI-EVAL-003)
      if (this.state.evalOn && this.state.truth) {
        const e = (this.state.truth.enemies || []).find((x) => x.track_id === t.id);
        if (e) {
          const err = dist3(e.pos || [0, 0, 0], t.pos || [0, 0, 0]);
          const clsOk = e.cls === cls;
          html += `
            <div class="delta-box">
              <div class="subhead">TRUTH Δ (EVAL)</div>
              <div class="row"><label>pos error</label><span class="val">${err.toFixed(1)} m</span></div>
              <div class="row"><label>true class</label>
                <span class="val" style="color:${clsOk ? 'var(--ok)' : 'var(--bad)'}">${esc(e.cls)} ${clsOk ? '✓' : '≠ belief'}</span></div>
              <div class="row"><label>decoy truth</label>
                <span class="val">${e.warhead === false ? 'DECOY' : 'armed'} · est ${fmtPct(t.p_decoy)}</span></div>
              <div class="row"><label>acquired at</label><span class="val">${e.acquired_t != null ? e.acquired_t.toFixed(1) + ' s' : '—'}</span></div>
            </div>`;
        }
      }
    } else if (sel.kind === 'turret') {
      const tu = (f.turrets || []).find((x) => x.id === sel.id);
      if (!tu) { body.innerHTML = `<div class="row"><label>${esc(sel.id)}</label><span>no data</span></div>`; return; }
      html = `
        <div class="subhead">TURRET ${esc(tu.id)}</div>
        <div class="row"><label>state</label><span class="val">${esc(tu.state)}</span></div>
        <div class="row"><label>az / el</label><span class="val">${fmtNum(tu.az, 0)}° / ${fmtNum(tu.el, 0)}°</span></div>
        <div class="row"><label>ammo</label><span class="val">${tu.ammo ?? '—'}</span></div>
        <div class="row"><label>target</label><span class="val">${tu.target != null ? 'track #' + tu.target : '—'}</span></div>`;
    } else if (sel.kind === 'ghost') {
      const e = (this.state.truth?.enemies || []).find((x) => x.id === sel.id);
      if (!e) { body.innerHTML = '<div class="row"><label>ghost</label><span>gone</span></div>'; return; }
      html = `
        <div class="subhead" style="color:#9aa4b0">GHOST ${esc(e.id)} (truth, unacquired)</div>
        <div class="row"><label>class</label><span class="val">${esc(e.cls)}</span></div>
        <div class="row"><label>warhead</label><span class="val">${e.warhead ? 'armed' : 'DECOY'}</span></div>
        <div class="row"><label>target</label><span class="val">${esc(e.target ?? '—')}</span></div>
        <div class="row"><label>alt</label><span class="val">${(e.pos?.[2] ?? 0).toFixed(0)} m</span></div>
        <div class="row"><label>speed</label><span class="val">${Math.hypot(...(e.vel || [0])).toFixed(0)} m/s</span></div>`;
    }
    body.innerHTML = html;
  }

  // ====================================================== log (HMI-MAP-004)
  addEvent(ev) {
    const extra = Object.entries(ev)
      .filter(([k]) => !['t', 'kind'].includes(k))
      .map(([k, v]) => `${k}=${v}`).join(' ');
    this._addLog({
      cat: 'events', cls: ev.kind,
      text: `${String(ev.kind || '?').toUpperCase()} ${extra}`,
      t: ev.t, track_id: ev.track_id ?? ev.track ?? null, uav_id: ev.uav_id ?? null,
    });
  }
  addDecision(dc) {
    this._addLog({
      cat: 'decisions', cls: `decision ${dc.actor || ''}`,
      text: `${dc.actor ?? 'c2'} · ${dc.text ?? dc.kind ?? ''}`,
      t: dc.t, track_id: dc.track_id ?? null, uav_id: dc.uav_id ?? null,
    });
  }
  _addLog({ cat, cls, text, t, track_id, uav_id }) {
    const d = document.createElement('div');
    d.className = `${cls || ''} cat-${cat}`;
    d.dataset.cat = cat;
    if (track_id != null) d.dataset.track = track_id;
    if (uav_id != null) d.dataset.uav = uav_id;
    if (track_id != null || uav_id != null) d.classList.add('focusable');
    d.innerHTML = `<span class="t">[${(t ?? 0).toFixed(1).padStart(7)}]</span> ${esc(text)}`;
    if (this.logFilter !== 'all' && this.logFilter !== cat) d.style.display = 'none';
    const list = $('log-list');
    list.prepend(d);
    while (list.children.length > 400) list.lastChild.remove();
  }
  _applyLogFilter() {
    for (const d of $('log-list').children)
      d.style.display = (this.logFilter === 'all' || d.dataset.cat === this.logFilter) ? '' : 'none';
  }

  // ============================================== metrics (HMI-EVAL-005)
  onTruthMetrics(m) {
    if (!m) return;
    const now = performance.now();
    if (now - this._lastMetricsRender < 500) return;
    this._lastMetricsRender = now;
    $('metrics-body').innerHTML = this._metricsHtml(m);
  }

  _metricsHtml(m) {
    const det = m.detection || {}, eco = m.economics || {}, col = m.collateral || {}, au = m.auth || {};
    const attr = m.attrition || {};
    const attrRows = Object.entries(attr).map(([c, a]) => `
      <tr><td style="color:${CLS_CSS[c] || '#fff'}">${CLS_SHORT[c] || c}</td>
      <td>${a.spawned ?? 0}</td><td>${a.killed ?? 0}</td><td>${a.leaked ?? 0}</td></tr>`).join('');
    const zoneRow = (o) => ['SAFE', 'DANGEROUS', 'CRITICAL']
      .map((z) => `<span style="color:${ZONE_CSS[z]}">${z[0]}:${o?.[z] ?? 0}</span>`).join(' ');
    return `
      <div class="subhead">DETECTION</div>
      <div class="row"><label>acquired</label><span class="val">${det.acquired ?? 0} / ${det.total ?? 0}</span></div>
      <div class="row"><label>mean latency</label><span class="val">${det.mean_latency != null ? det.mean_latency + ' s' : '—'}</span></div>
      <div class="subhead">ATTRITION</div>
      <table><tr><th>class</th><th>spawn</th><th>kill</th><th>leak</th></tr>${attrRows || ''}</table>
      <div class="subhead">ECONOMICS</div>
      <div class="row"><label>shots / kills</label><span class="val">${eco.shots ?? 0} / ${eco.kills ?? 0}</span></div>
      <div class="row"><label>ammo per kill</label><span class="val">${eco.ammo_per_kill ?? '—'}</span></div>
      <div class="row"><label>decoy shots</label><span class="val">${eco.decoy_shots ?? 0}</span></div>
      <div class="subhead">COLLATERAL</div>
      <div class="row"><label>wrecks</label><span class="val">${zoneRow(col.wrecks_by_zone)}</span></div>
      <div class="row"><label>strays</label><span class="val">${zoneRow(col.strays_by_zone)}</span></div>
      <div class="row"><label>debris cost</label><span class="val">${col.debris_cost ?? 0}</span></div>
      <div class="subhead">AUTHORISATION</div>
      <div class="row"><label>req / ok / deny / exp</label>
        <span class="val">${au.requests ?? 0} / ${au.approved ?? 0} / ${au.denied ?? 0} / ${au.expired ?? 0}</span></div>
      <div class="row"><label>mean latency</label><span class="val">${au.mean_latency != null ? au.mean_latency + ' s' : '—'}</span></div>`;
  }

  // ========================================================= summary modal
  onSummary(s) {
    if (!s) return;
    const wz = s.wrecks_by_zone || {};
    let html = `
      <table>
        <tr><td>run</td><td>${esc(s.name ?? '—')}</td></tr>
        <tr><td>seed</td><td>${s.seed ?? '—'}</td></tr>
        <tr><td>kills</td><td>${s.kills ?? '—'} / ${s.enemies_total ?? '—'}</td></tr>
        <tr><td>armed leakers</td><td>${s.armed_leakers ?? '—'}</td></tr>
        <tr><td>wrecks by zone</td><td>${Object.entries(wz).map(([z, n]) => `${z}:${n}`).join('  ') || '—'}</td></tr>
        <tr><td>end time</td><td>${s.t_end != null ? s.t_end + ' s' : '—'}</td></tr>
      </table>`;
    if (s.metrics) html += `<hr>${this._metricsHtml(s.metrics)}`;
    $('summary-body').innerHTML = html;
    $('summary-modal').classList.add('open');
  }

  toast(msg) {
    const t = $('toast');
    t.textContent = msg;
    t.style.display = 'block';
    clearTimeout(this._toastTimer);
    this._toastTimer = setTimeout(() => { t.style.display = 'none'; }, 3500);
  }

  // populate target dropdowns from scene assets (HMI-SCN-002)
  onScene(sc) {
    const names = (sc?.assets || []).map((a) => a.name);
    for (const sel of document.querySelectorAll('.x-target')) {
      const cur = sel.value;
      sel.innerHTML = '<option value="auto">auto</option>' +
        names.map((n) => `<option value="${esc(n)}">${esc(n)}</option>`).join('');
      if ([...sel.options].some((o) => o.value === cur)) sel.value = cur;
    }
    if (sc?.run) {
      $('run-seed').textContent = sc.run.seed ?? '—';
    }
  }
}
