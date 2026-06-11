# RNG re-baseline report (PLAN_PROBLEM1 P0-6/P0-7)

*2026-06-11 — `feature/urban-environment`. 10-seed Monte-Carlo of
`scenarios/residential_raid.yaml` (seeds 0–9) before and after migrating every
randomness consumer off the shared call-order-coupled `world.rng` onto
name-keyed `RngRegistry` streams (DESIGN_REVIEW 5.1). Raw rows:
`rng_rebaseline_before.json` / `rng_rebaseline_after.json`
(`scripts/mc_report.py`).*

The migration re-sources every draw, so individual battles legitimately
re-roll; the question is whether the *distribution* of outcomes moved and
whether the verified floors hold. They do:

| metric | before | after | floor |
|---|---|---|---|
| 5-seed kills (seeds 0–4) | 24 | 33 | >= 10 — **pass both** |
| 5-seed shots/kill | 6.708 | 5.606 | <= 9.0 — **pass both** |
| 10-seed kills | 56 | 64 | — |
| 10-seed shots | 295 | 343 | — |
| 10-seed leakers | 34 | 26 | — |
| 10-seed CRITICAL wrecks | 0 | 0 | == 0 invariant — **holds both** |

Hit-rate floors re-affirmed without user sign-off (both sides comfortably
inside). Characterization pins (`tests/fixtures/golden/`) re-recorded ONCE at
this point, per the P0-7 sanction; they now pin the post-migration streams.

Stream names now in force: `weather`, `comms`, `sensor/<name>`,
`adjudicator`, `debris`, `threat/<id>` (plus build-time parametric placement,
which already had its own generator). The shared `world.rng` is proven virgin
through a full battle (`tests/test_rng_streams.py`), and the order-independence
capstone shows an extra 10k-draw consumer changes no outcome.
