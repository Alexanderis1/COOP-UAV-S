// app.js — Element-3 Command Interface entry point.
// Data sources (ICD §1): live WS /ops + /eval (default), ?replay=1 loads
// /recording.json, ?mock=1 runs the built-in deterministic synthetic feed.
// If /eval is not connected the UI runs in production mode: no ghosts, no
// truth deltas, no EVALUATION badge (SRS HMI-EVAL-004).
import { SceneView } from './scene.js';
import { Entities } from './entities.js';
import { Panels } from './panels.js';
import { Channel } from './net.js';
import { Replay } from './replay.js';
import { MockServer } from './mock.js';

const params = new URLSearchParams(location.search);
const mode = params.has('mock') ? 'mock' : params.has('replay') ? 'replay' : 'live';

const state = {
  scene: null, frame: null, truth: null,
  evalOn: false, selection: null,
  auth: new Map(),          // id -> { req, resolved }
  runInfo: null,
};

// ---------------------------------------------------------------- 3D + UI
const view = new SceneView(document.getElementById('view'), (pick) => select(pick));
const ents = new Entities(view, {
  onAcquired: (e, t) => {
    panels.addEvent({
      t, kind: 'acquired', ghost: e.id, cls: e.cls, track_id: e.track_id,
    });
  },
});
if (mode === 'replay') ents.extrapolate = false;

function send(type, data = {}) {
  if (mode === 'mock') mock.send({ type, data });
  else if (mode === 'live') opsCh?.send({ type, data });
  else panels.toast('replay mode — control commands are unavailable');
}

function setLayer(key, on) {
  if (key === 'coverage' || key === 'rings' || key === 'grid') view.setLayer(key, on);
  else ents.setLayer(key, on);
}

function select(sel) {
  state.selection = sel;
  ents.select(sel);
  panels.renderInspector();
}

// focus an entity (or shooter+track pair) in 3D, from auth cards / log lines
function focus(ref) {
  if (!ref) return;
  const pts = [];
  let trackSelected = false;
  if (ref.track_id != null) {
    const p = ents.entityPos('track', ref.track_id);
    if (p) {
      pts.push(p);
      select({ kind: 'track', id: ref.track_id });
      trackSelected = true;
    }
  }
  if (ref.uav_id != null) {
    const p = ents.entityPos('uav', ref.uav_id) || ents.entityPos('turret', ref.uav_id);
    if (p) {
      pts.push(p);
      if (!trackSelected) {
        const kind = ents.uavs.has(ref.uav_id) ? 'uav' : 'turret';
        select({ kind, id: ref.uav_id });
      }
    }
  }
  if (!pts.length) return;
  const mid = pts[0].clone();
  for (let i = 1; i < pts.length; i++) mid.add(pts[i]);
  mid.divideScalar(pts.length);
  view.focus(mid);
}

const panels = new Panels({ state, send, focus, setLayer, mode });

// --------------------------------------------------------- message routing
function handleOps(msg) {
  const { type, data } = msg;
  switch (type) {
    case 'scene':
      state.scene = data;
      view.buildStatic(data);
      ents.reset();
      state.selection = null;
      panels.onScene(data);
      break;
    case 'frame':
      state.frame = data;
      ents.applyFrame(data);
      view.applyEnv(data.env);
      panels.onFrame(data);
      break;
    case 'auth_request':
      panels.onAuthRequest(data);
      break;
    case 'auth_resolved':
      panels.onAuthResolved(data);
      break;
    case 'run_started':
      state.runInfo = data;
      state.auth.clear();
      ents.reset();
      state.selection = null;
      panels.renderAuth();
      panels.onRunStarted(data);
      break;
    case 'summary':
      panels.onSummary(data);
      break;
    case 'error':
      panels.onError(data);
      break;
    default:
      break;
  }
}

function handleEval(msg) {
  if (msg.type !== 'truth') return;
  state.truth = msg.data;
  if (!state.evalOn) return;     // production mode: eval path unreachable
  ents.applyTruth(msg.data);
  panels.onTruthMetrics(msg.data.metrics);
}

function setEval(on) {
  state.evalOn = on;
  panels.setEval(on);
  if (!on) {
    state.truth = null;
    ents.clearEval();
    if (state.selection?.kind === 'ghost') select(null);
  }
}

// -------------------------------------------------------------- transports
let opsCh = null, evalCh = null, replay = null, mock = null;

if (mode === 'mock') {
  mock = new MockServer({ onOps: handleOps, onEval: handleEval });
  setEval(true);
  mock.start();
} else if (mode === 'replay') {
  document.getElementById('ops-pill').style.display = 'none';
  replay = new Replay({
    onOps: handleOps,
    onEval: handleEval,
    onTimeline: (idx, len, playing) => panels.updateTimeline(idx, len, playing),
  });
  panels.setReplay(replay);
  replay.load()
    .then(() => setEval(replay.hasEval))
    .catch((e) => panels.toast('recording load failed: ' + e.message));
} else {
  const host = location.hostname || 'localhost';
  fetch('/runtime-config.json')
    .then((r) => r.json())
    .catch(() => ({}))                 // older backend: fall back to 8001
    .then((cfg) => {
      const wsPort = cfg.ws_port || 8001;
      opsCh = new Channel(`ws://${host}:${wsPort}/ops`, {
        onMessage: handleOps,
        onStatus: (on) => panels.setOpsStatus(on),
      });
      evalCh = new Channel(`ws://${host}:${wsPort}/eval`, {
        onMessage: handleEval,
        onStatus: (on) => setEval(on),     // refuse/drop => production mode
      });
    });
}

// ------------------------------------------------------------- render loop
view.start((now) => ents.tick(now));
