# COOP-UAV-S — Runtime Interface Contract (ICD-RUNTIME) — v0.4

> Implements SRS §6 for the current stage. This is the **binding wire
> contract** between Element 2 (simulator backend) and Element 3 (web
> interface + ORC). Both sides are built against this document; change it
> only deliberately and in one commit with both sides. v0.4 adds the
> optional sitl `uavs[]` fields (`att`/`nav_q`/`health`, §2.2) —
> strictly additive over v0.3.

## 1. Topology

One long-running backend process:

```
coopuavs serve [--port 8000] [--ws-port 8001] [--preset scenarios/residential_raid.yaml]
```

- **HTTP :8000** — static frontend from `src/coopuavs/viz/web/`.
- **WS :8001 path `/ops`** — operational channel (production-identical
  surface): frames, auth requests, run state, control commands.
- **WS :8001 path `/eval`** — evaluation-only channel (SRS ICD-002):
  ground truth incl. unacquired threats, live metrics. The frontend must
  degrade gracefully (no ghosts, no truth deltas, no eval badge) if this
  endpoint refuses connection.

All messages are JSON: `{"type": <string>, "data": <object>}`.
Positions are `[x, y, z]` metres in the map (ENU) frame. Time `t` is
simulation seconds. All threat-class strings use the `ThreatClass` values:
`owa_strategic | owa_jet | fpv | loitering | decoy`.

## 2. `/ops` server → client

### 2.1 `scene` — once per run start (and to late joiners)

```json
{"type":"scene","data":{
  "bounds":[xmin,ymin,xmax,ymax], "cell_size":m, "grid":[[0|1|2,...]],
  "assets":[{"name":s,"pos":[x,y,z],"value":v}],
  "buildings":[{"rect":[x0,y0,x1,y1],"height":h,
                "kind":"residential_high|residential_low|school|hospital|commercial|industrial|park|water",
                "material":"concrete|brick|glass_steel|light_metal|wood|none",
                "name":s}],
  "sensors":[{"name":s,"type":"radar|rf|eo_ir|acoustic","pos":[x,y,z],"range":m}],
  "turrets":[{"id":s,"pos":[x,y,z],"range":m}],
  "homes":[{"uav_id":s,"pos":[x,y,z]}],
  "stations":[{"id":s,"pos":[x,y,z],"rooftop":bool}],
  "run":{"name":s,"seed":int,"duration":s,"eval":bool}}}
```

`kind`, `material` and `name` are v0.3 additions (SRS SIM-ENV-004,
ICD-003); clients must tolerate their absence in older recordings.
`grid` semantics (SRS SIM-ENV-005): `0` SAFE = green, civilian-free;
`1` DANGEROUS = yellow, civilians possibly present; `2` CRITICAL = red,
civilians certainly present. `homes` is kept for backward compatibility;
`stations` is authoritative when present.

### 2.2 `frame` — at record rate (5 Hz sim time) while running

```json
{"type":"frame","data":{
  "t":s,
  "run":{"status":"idle|running|paused|done","speed":f,"posture":"human_confirm|pre_authorized|weapons_hold"},
  "tracks":[{"id":i,"pos":[..],"vel":[..],"p_decoy":f,
             "belief":{"<class>":p,...},"score":f|null,
             "impact":[x,y,0]|null,"tti":s|null}],
  "uavs":[{"id":s,"pos":[..],"vel":[..],"mode":s,"ammo":i,
           "battery":f,"task_id":i|null,"link":f,
           "kind":"interceptor|sentinel","effector":"net|projectile"|null,
           "att":[w,x,y,z]?,"nav_q":f?,"health":{..}?}],
  "turrets":[{"id":s,"az":deg,"el":deg,"ammo":i,
              "state":"idle|slewing|tracking|firing|empty","target":i|null}],
  "wrecks":[{"pos":[..],"zone":"SAFE|DANGEROUS|CRITICAL","mechanism":"net|projectile"}],
  "strays":[{"pos":[x,y,0],"zone":s}],
  "debris":[{"id":s,"pos":[..],"vel":[..],"impact":[x,y,0],
             "zone":"SAFE|DANGEROUS|CRITICAL","t_impact":s}],
  "stations":[{"id":s,"occupied":i}],
  "env":{"wind":[x,y,z],"fog":f,"precip":f,"daylight":f},
  "events":[{"t":s,"kind":s,...}],
  "decisions":[{"t":s,"actor":"c2|orc|operator","kind":s,"text":s,
                "track_id":i|null,"uav_id":s|null}]}}
```

`link` is 0..1 datalink quality. `events`/`decisions` carry only entries
new since the previous frame. v0.3 (ICD-003): `uavs[].kind`
(sentinels are unarmed, `ammo: 0`, mode may be `PATROL`), the live
`debris` array (SIM-DEB-002 — debris disappears from the array when
neutralized or landed; a landed one becomes a wreck), and `stations`
occupancy counts. Attribution fields on engagement events
(SIM-GT-004): `kill`/`miss` events always carry
`{"uav_id":s,"enemy_id":s|"debris_id":s,"effector":"net|projectile|turret_gun",
"pk":f,"target_kind":"track|debris"}`; new event kinds
`debris_spawn`, `debris_impact`, `debris_neutralized` (with
`saved_zone`), `fire_blocked_los`.

v0.4 (P4-7, additive): in `fidelity.fleet=sitl` runs `uavs[]` entries
MAY carry `att` ([w,x,y,z] EKF attitude estimate), `nav_q`
(`sigma_pos_h`, m — the navigation solution's own horizontal 1-sigma)
and `health` (UavHealth summary, populated from P5 on). The keys are
present exactly when the platform reports them; pointmass recordings
contain none of them and remain byte-compatible with v0.3 parsers.
Note `pos`/`vel` in sitl runs are the platform's EKF **estimates** (the
operational picture — nav error is visible by design); ground truth
stays on `/eval`.

### 2.3 Authorisation flow (SRS HMI-AUT, ORC-002)

```json
{"type":"auth_request","data":{
  "id":i,"t":s,"shooter":s,"track_id":i,"effector":"net|projectile",
  "p_kill":f,"roe":{"decision":"authorized|hold|denied","reason":s,
  "expected_collateral":f},"rationale":s,"expires_t":s}}

{"type":"auth_resolved","data":{"id":i,"approved":bool,
  "by":"operator|orc|timeout|posture"}}
```

Every pending request must be answered by `auth_resolved` exactly once
(approval, denial, expiry, or posture auto-clear).

### 2.4 Run lifecycle

```json
{"type":"run_started","data":{"name":s,"seed":i,"eval":bool}}
{"type":"summary","data":{...world summary + "metrics":{see §4}}}
{"type":"error","data":{"message":s}}
```

## 3. `/ops` client → server (control)

```json
{"type":"start_run","data":{
  "threats":{"<class>":{"count":i,"target":"<asset name>|auto",
             "axis_deg":f|null,"first_time":s,"spacing":s}},
  "weather":{"wind_speed":f,"wind_dir_deg":f,"fog":0..1,
             "precip":0..1,"daylight":0..1},
  "duration":s|null,"speed":f,"seed":i|null,
  "posture":"human_confirm|pre_authorized|weapons_hold"}}

{"type":"stop_run","data":{}}
{"type":"pause","data":{}}
{"type":"resume","data":{}}
{"type":"set_speed","data":{"speed":f}}          // 0.1 .. 10
{"type":"set_posture","data":{"posture":s}}
{"type":"authorize","data":{"id":i,"approve":bool}}
{"type":"uav_command","data":{"uav_id":s,"command":"rtb"}}
```

`start_run` while a run is active ⇒ `error`. Missing/`null` seed ⇒ backend
picks one and reports it in `run_started` (HMI-SCN-002 reproducibility).
Unknown asset names ⇒ `error` with a structured message (HMI-SCN-003).

## 4. `/eval` server → client

```json
{"type":"truth","data":{
  "t":s,
  "enemies":[{"id":s,"cls":s,"pos":[..],"vel":[..],"alive":b,"killed":b,
              "warhead":b,"target":s,
              "acquired":b,"acquired_t":s|null,"track_id":i|null}],
  "metrics":{
    "detection":{"acquired":i,"total":i,"latencies":[{"id":s,"cls":s,
                  "latency":s|null}],"mean_latency":s|null},
    "attrition":{"<class>":{"spawned":i,"killed":i,"leaked":i}},
    "economics":{"shots":i,"kills":i,"ammo_per_kill":f|null,
                 "decoy_shots":i},
    "collateral":{"wrecks_by_zone":{...},"strays_by_zone":{...},
                  "debris_cost":f,"debris_intercepts":i,
                  "debris_saved_cost":f},
    "auth":{"requests":i,"approved":i,"denied":i,"expired":i,
            "mean_latency":s|null},
    "engagements":{
      "by_shooter":{"<id>":{"weapon":s,"shots":i,"hits":i,"kills":i,
                            "debris_kills":i,"mean_pk":f|null}},
      "by_weapon":{"net|projectile|turret_gun":{"shots":i,"hits":i,
                            "kills":i,"debris_kills":i,"mean_pk":f|null}}}}}}
```

Sent at the same cadence as `frame`. Ghost rule (SRS HMI-EVAL-001/002):
an enemy with `acquired == false` renders as a **grey wireframe ghost**;
on `acquired == true` it transitions to the normal tracked representation.
The ORC never consumes this channel (ORC-006).

## 5. Backend internal seams (for the server/ORC layer)

Provided by the sim layer; the serve layer builds on exactly these:

- `coopuavs.sim.scenario.build_parametric(request: dict, preset_cfg: dict, seed: int) -> Scenario`
  — `request` is the `start_run.data` payload; preset supplies map, zones,
  sensors, fleet, turrets, ROE.
- `coopuavs.sim.runctl.RunController(scenario)` —
  `.tick(wall_dt) -> list[frame_payload]` (steps sim per speed/pause
  state), `.pause() / .resume() / .set_speed(x) / .stop()`,
  `.status -> "idle|running|paused|done"`, `.frame() -> dict` (§2.2 data),
  `.truth() -> dict` (§4 data), `.scene() -> dict` (§2.1 data),
  `.summary() -> dict`.
- `coopuavs.sim.evaluation.EvalTracker` — sim-side node owning
  acquisition matching (truth↔track gating) and the §4 metrics.
- Authorisation seam: the C2 publishes `engagement/fire_request` with its
  ROE evaluation attached; the **Orchestrator** node decides per posture:
  auto-clear, deny, or surface to `/ops` as `auth_request` and publish the
  `engagement/clearance` only after `authorize` (or expiry).

## 6. Frontend obligations summary

- 3D map: terrain/zone raster, buildings, assets, sensor coverage
  (toggle), turret arcs, homes; entities per §2.2 with selection +
  detail panels (HMI-MAP-004); event + decision log; layer toggles.
- Ghost overlay per §4 + "EVALUATION" badge while `/eval` is connected.
- Scenario launch form per §3 `start_run` (counts per class, objectives,
  weather, speed, seed, posture); run controls (pause/resume/speed/stop).
- Authorisation queue with approve/deny and live expiry countdown;
  posture selector; per-UAV RTB command.
- Metrics panel fed by `/eval` metrics and the final `summary`.
- Replay mode: `?replay=1` loads `/recording.json` (same frame schema:
  `{"scene":…, "frames":[…], "truth":[…]|null, "summary":…}`).
- Mock mode: `?mock=1` renders a built-in synthetic feed (development).
