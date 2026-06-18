# COOP-UAV-S — Technical Research Survey

**Scope.** State-of-the-art survey for a simulated cooperative counter-UAS (C-UAS) system:
friendly interceptor UAVs + a ground base station (radar, RF, acoustic, EO/IR) that detect,
track, classify, and *kinetically* intercept (net guns / projectiles) hostile one-way-attack
(OWA) drone raids — Shahed-136/Geran-2 types at 50–65 m/s, Gerbera-style decoys, FPV
kamikazes, Lancet-class loitering munitions — over a residential area modeled on the
Ukrainian theatre.

**Date of research:** 2026-06-10. All citations below were located and checked via web
search at that date. Where a reference is a "standard textbook/classic" that could not be
re-verified with a stable URL, it is explicitly marked **[standard reference, URL not
verified]**. Nothing below is fabricated; anything uncertain is flagged.

---

## Executive summary

1. **Cooperative pursuit of a faster evader is a solved problem *geometrically* and an
   active problem *practically*.** The core tool is the **Apollonius circle** (the locus of
   points reachable by the evader before a given pursuer, under a constant speed ratio).
   Multiple slower pursuers capture a faster evader iff the intersection of the evader's
   "safe-reachable set" (intersection of Apollonius disks / Cartesian ovals) can be driven
   to zero area — i.e., the evader is *encircled*. Recent work (2022–2025) gives explicit
   area-minimizing cooperative controls and MARL policies seeded with Apollonius
   partitions. For COOP-UAV-S the recommended baseline is: Apollonius-based containment +
   target-motion prediction + proportional navigation endgame, with MARL (MAPPO) as an
   optional research layer, *not* the foundation.

2. **Weapon-target assignment (WTA) is NP-complete but small instances are easy.** For the
   raid sizes you will simulate (5–50 threats, 5–30 interceptors), exact assignment via
   the Hungarian algorithm (`scipy.optimize.linear_sum_assignment`) on a utility matrix
   (Pk × threat value × collateral-risk penalty) solved *every planning tick* is the right
   baseline; CBBA (Choi–Brunet–How 2009) is the right decentralized upgrade and has
   open-source Python implementations. Dynamic WTA with shoot-look-shoot is well studied
   and maps directly onto your engagement loop.

3. **Multi-target tracking should be built on an existing library, not hand-rolled.**
   dstl's **Stone Soup** (UK Defence Science and Technology Laboratory) implements
   KF/EKF/UKF/IMM, GNN/JPDA/MHT-style association, and GM-PHD, plus sensor and platform
   simulation — it is almost a drop-in for your fusion node. A pragmatic architecture is:
   per-sensor detection models → centralized GM-PHD or IMM+GNN tracker → track-level
   classification. JPDA/MHT/LMB are described so you can justify the choice.

4. **Decoy discrimination (Gerbera-vs-Shahed) is the single most operationally relevant
   classification problem and is under-published.** Verified facts: Gerbera is ~20%
   smaller, foam/plywood construction, carries radar reflectors (Luneburg-type lenses) to
   *mimic* the Shahed RCS, warhead ≤5 kg vs 50–90 kg; roughly half of "Shahed" launches in
   late-2024 were decoys. Discriminators that survive deliberate RCS spoofing: micro-Doppler
   (propeller modulation), acoustic signature, IR/thermal emission, mass-driven kinematics
   (climb/turn performance), and terminal behavior. Your simulation's probabilistic
   classifier should fuse these with a Bayesian evidence model and explicitly cost
   misclassification in the WTA utility.

5. **Guidance: proportional navigation (PN) variants dominate**, including for multirotor
   pursuit (AIAA 2021 evaluation) and for real interceptors. Real systems anchor your
   parameter ranges: Anduril Anvil ~320 km/h kinetic ramming; Fortem DroneHunter F700 net
   capture; Raytheon Coyote Block 2 jet interceptor 555–595 km/h, 10–15 km range,
   fragmentation warhead; Ukrainian Wild Hornets **Sting** ~280–315 km/h with claimed
   80–95% per-sortie hit rates and >3,900 Shahed kills by early 2026. Net-gun effective
   range is *tens of meters* (≈10–35 m hand-held/turret class; drone-borne pods engage
   after closing to similar distances), which makes the terminal geometry problem central.

6. **Collateral-damage-aware engagement has a ready-made scaffold: the JARUS SORA ground
   risk model** plus the academic ground-risk-map literature (Primatesta et al. 2020;
   ballistic/uncontrolled-descent footprint Monte Carlo). Casualty-area models (person
   radius + debris cross-section, sheltering factors, population density layers) convert a
   sampled crash footprint into expected casualties — exactly the quantity your shot
   authorization should minimize. This is the project's most novel integration point:
   WTA utility = P(kill) × threat value − E[ground casualties | intercept point].

7. **Simulation:** your custom time-stepped Python sim with ROS 2-shaped pub/sub is the
   correct hackathon choice; the literature-backed migration path is
   **ROS 2 + PX4 SITL + Gazebo via Aerostack2** (multi-UAV, plugin behaviors, mission
   level), with **gym-pybullet-drones** for any RL training loops and **Pegasus/Isaac
   Sim** only if photorealism is later needed. Keep your message dataclasses
   field-compatible with real ROS 2 msg types now to make migration mechanical.

8. **Ukraine operational data is abundant enough to build a realistic threat generator:**
   cruise ~50–85 m/s, cruise altitudes that have migrated from <300 m (2022–23) to
   1.5–5 km (2025) with terminal dives from ~1–2.5 km at up to ~90 m/s, raids of 100–700+
   vehicles per night, 40–60% decoys in many salvos, deliberate routing changes and
   loitering to confuse defenses; interceptor-drone effectiveness reported >70% fleet-wide
   and 80–95% for the best crews. Layered defense doctrine (acoustic cueing → mobile fire
   groups → interceptor drones → SAMs reserved for cruise/ballistic threats) is exactly
   the architecture COOP-UAV-S simulates.

---

## 1. Cooperative multi-UAV pursuit-evasion and target interception

### State of the art

**Differential-game foundations.** The field starts with Rufus Isaacs' *Differential
Games* (1965), which formulates pursuit-evasion as a zero-sum game over kinematic
"simple motion" players and introduces the geometric machinery (barrier surfaces,
dominance regions) still used today. The **Apollonius circle** is the locus of points `x`
with `|x − E| / |x − P| = γ` for evader E, pursuer P, speed ratio `γ = v_E / v_P`. For a
*slower* pursuer (γ > 1) the circle bounds the region the evader can reach safely; for a
*faster* pursuer it bounds the evader's escape set. Key consequences used by every modern
paper:

- A single slower pursuer can never capture a smart faster evader in open space.
- N slower pursuers can capture a faster evader **iff the evader starts inside (and can be
  kept inside) the intersection of their dominance regions** — geometrically, the evader's
  *safe-reachable set* (intersection of the per-pursuer Apollonius disks, or Cartesian
  ovals when capture radius > 0) must be bounded and shrinkable to zero area.
- The minimum number of pursuers and the required angular spacing around the evader can be
  derived in closed form for the constant-speed case (encirclement: pursuers must subtend
  the evader so that no Apollonius gap exists; with speed ratio γ > 1 you need roughly
  N ≥ π / arcsin(1/γ) pursuers to close the ring — derivations of this type appear in the
  Wang 2022 and Jin & Qu papers below; verify the exact bound against the paper you adopt).

**Modern geometric/optimal-control results (2017–2025).** The AFRL group (Eloy Garcia,
David Casbeer, Alexander Von Moll, Meir Pachter) produced the canonical recent series:
two-pursuers-one-evader geometric solutions, M-pursuer single-evader cooperative pursuit,
multiple-pursuer-multiple-evader games (IEEE TAC 2020), and border-defense variants. A
2025 arXiv paper ("Area-Optimal Control Strategies for Heterogeneous Multi-Agent
Pursuit") formalizes exactly the strategy COOP-UAV-S needs: define the evader's
safe-reachable set as the intersection of Apollonius circles and have pursuers
cooperatively minimize its **area** as a zero-sum objective — a clean, implementable
scalar objective for your planner.

**Faster-evader encirclement.** Wang (2022, *Complexity*) gives a cooperative hunting
strategy against a *superior* (faster) evader, using Apollonius circles to derive the
initial-position conditions and the minimum pursuer count, with formation maintenance
during the chase. A *Connection Science* (2023) paper extends Apollonius-circle pursuit
of a faster evader to obstacle environments. Jin & Qu give an earlier treatment of
multi-pursuer vs one fast evader. The common structure of all of them:

1. **Containment phase** — distribute pursuers on a ring/arc around the predicted evader
   path so the union of their dominance regions closes (no escape gap).
2. **Contraction phase** — shrink the ring while maintaining gap closure (area-minimizing
   or angle-preserving controls).
3. **Capture phase** — once the evader's safe set is inside one pursuer's capture
   envelope, switch to a terminal guidance law (PN).

**Caveat that matters for COOP-UAV-S:** a Shahed-type OWA drone is *not* an optimal
evader — it is essentially non-cooperative ballistic-ish traffic flying a preplanned,
possibly waypoint-randomized route with limited maneuvering (≤ ~1 g lateral typically;
terminal dive). Against such a target the pursuit problem degenerates from a differential
game to **intercept-point prediction + assignment + rendezvous**, which is *much* easier
and means your "cornering" behavior is mostly valuable against the FPV/Lancet threats and
as robustness against route randomization. Design the planner so the "game-theoretic
worst-case evader" mode and the "predicted-trajectory rendezvous" mode are both present,
and select per threat class.

**MARL approaches.** MADDPG (Lowe et al. 2017) and MAPPO (Yu et al. 2022) are the two
workhorse algorithms applied to pursuit-evasion. Representative recent results:
EO-MADDPG (Aerospace 2026) and Apollonius-partition-seeded MARL (Neurocomputing 2025)
both *inject the geometric prior into the learning problem* — strong evidence that pure
end-to-end RL is sample-inefficient here and geometry-guided rewards/partitions are the
right hybrid. Tsinghua's dual-curriculum framework (arXiv 2312.12255) and online-planning
DRL in unknown environments (arXiv 2409.15866) show transfer to real Crazyflie quadrotors.
Consensus/formation-control results (cyclic pursuit, standoff circling of a moving target)
provide the containment-ring controller without learning.

### Key references

| Reference | Notes |
|---|---|
| R. Isaacs, *Differential Games: A Mathematical Theory with Applications to Warfare and Pursuit, Control and Optimization*, Wiley, 1965 (Dover reprint 1999). [Google Books](https://books.google.com/books/about/Differential_Games.html?id=gtlQAAAAMAAJ) | Foundational text; read Ch. 1–6 + the "two pursuers" examples. |
| E. Garcia, D. W. Casbeer, A. Von Moll, M. Pachter, "Multiple Pursuer Multiple Evader Differential Games," *IEEE Trans. Automatic Control*, 2020. [Semantic Scholar](https://www.semanticscholar.org/paper/2b8f9df3d700d5c3b762ce1343a4220d9423ba6b) | Canonical M-vs-N decomposition: solve pairwise games + assignment. |
| A. Von Moll, D. Casbeer, E. Garcia, D. Milutinović, M. Pachter, "Cooperative Pursuit by Multiple Pursuers of a Single Evader" (faster pursuers, point capture). [ResearchGate](https://www.researchgate.net/publication/339420245_Cooperative_Pursuit_by_Multiple_Pursuers_of_a_Single_Evader) ; companion "Multiple-Pursuer, Single-Evader Border Defense Differential Game," *J. Aerospace Information Systems*. [AIAA](https://arc.aiaa.org/doi/10.2514/1.I010740) | The AFRL series; geometric optimal strategies, Apollonius machinery. |
| X. Wang et al., "Cooperative Hunting Strategy with a Superior Evader Based on Differential Game," *Complexity*, 2022. [Wiley](https://onlinelibrary.wiley.com/doi/10.1155/2022/2239182) | Minimum pursuer count + initial-distribution conditions for capturing a *faster* evader; formation-keeping hunt. Directly applicable. |
| "Collaborative pursuit-evasion game of multi-UAVs based on Apollonius circle in the environment with obstacle," *Connection Science*, 2023. [Taylor & Francis](https://www.tandfonline.com/doi/full/10.1080/09540091.2023.2168253) | Faster evader + obstacles; Apollonius + geometric algorithm hybrid. |
| "Area-Optimal Control Strategies for Heterogeneous Multi-Agent Pursuit," arXiv:2511.15036, 2025. [arXiv](https://arxiv.org/abs/2511.15036) | Safe-reachable set = ∩ Apollonius circles; pursuers minimize its area. **Recommended core algorithm.** |
| "Apollonius partitions based pursuit-evasion strategies via multi-agent reinforcement learning," *Neurocomputing*, 2025. [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0925231225003157) | Geometry-seeded MARL; slower-evader dominant regions. |
| Jin & Qu, "Pursuit-evasion games with multi-pursuer vs. one fast evader." [Semantic Scholar](https://www.semanticscholar.org/paper/dc930f40870694d00ee018b0f938889a30d24fb0) | Early multi-vs-fast-evader treatment. |
| "A Dual Curriculum Learning Framework for Multi-UAV Pursuit-Evasion in Diverse Environments," arXiv:2312.12255. [arXiv](https://arxiv.org/pdf/2312.12255) | MAPPO + curriculum; encirclement rewards; sim-to-real on quadrotors. |
| "Multi-UAV Pursuit-Evasion with Online Planning in Unknown Environments by Deep Reinforcement Learning," arXiv:2409.15866. [arXiv](https://arxiv.org/html/2409.15866v1) | Online-planning DRL variant. |
| "EO-MADDPG: An Improved Reinforcement Learning Approach for Multi-UAV Pursuit–Evasion Games," *Aerospace* 13(3):296, 2026. [MDPI](https://doi.org/10.3390/aerospace13030296) | MADDPG + evolutionary tweaks; consensus + Apollonius guidance baked in. |
| R. Lowe et al., "Multi-Agent Actor-Critic for Mixed Cooperative-Competitive Environments" (MADDPG), NeurIPS 2017, arXiv:1706.02275; C. Yu et al., "The Surprising Effectiveness of PPO in Cooperative Multi-Agent Games" (MAPPO), NeurIPS 2022, arXiv:2103.01955. **[standard references; arXiv IDs from memory — verify before citing in a paper]** | The two MARL baselines. |

### Recommended approach for COOP-UAV-S

1. **Implement the Apollonius toolkit first** (pure geometry, ~200 lines with `shapely`):
   per-pursuer Apollonius circle, evader safe-reachable set as polygon intersection, gap
   detection on the containment ring, area and area-gradient of the safe set.
2. **Three-mode pursuit planner per engagement group:**
   - *Rendezvous mode* (default vs Shahed/Gerbera): predict target trajectory (IMM output),
     compute PN-feasible intercept points for each interceptor, fly to lead points.
   - *Containment mode* (vs maneuvering FPV/Lancet or when target deviates): assign
     interceptors to bearing slots around the predicted position; controller = move to
     close the largest Apollonius gap, then descend the area gradient of the safe set.
   - *Terminal mode*: hand off to the guidance law (Section 5) when range < envelope.
3. **Mode switching** on track-class + maneuver-detection (innovation magnitude from the
   IMM filter is a free maneuver detector).
4. Treat MARL (MAPPO via the dual-curriculum recipe) as a *stretch goal* benchmarked
   against the geometric planner — the literature says geometry-seeded learning wins, so
   you lose nothing by shipping geometry first.
5. Exploit the asymmetry: your interceptors at ~70–80 m/s vs Shahed at 50–65 m/s are
   actually *faster*, so classic capture applies; the slower-pursuer machinery is needed
   when interceptors are quadrotor-class (~30–50 m/s) or vs diving targets (~90 m/s) —
   model both regimes.

### Study list

- Isaacs Ch. 1–2 (game of kind vs game of degree), then any one AFRL paper end-to-end.
- Apollonius circle derivation + safe-reachable set (Area-Optimal arXiv:2511.15036).
- Capture conditions for N slower pursuers (Wang 2022).
- Cyclic pursuit / standoff circling (consensus formation control basics).
- MADDPG/MAPPO papers + PettingZoo's `simple_tag` (the canonical pursuit MARL toy).

---

## 2. Weapon-target assignment (WTA) and threat evaluation (TEWA)

### State of the art

**Problem statement.** Static WTA: given weapons `w` with per-target kill probabilities
`p_{w,t}` and target values `V_t`, choose an assignment minimizing surviving expected
value `Σ_t V_t Π_w (1 − p_{w,t})^{x_{w,t}}`. NP-complete in general (Lloyd &
Witsenhausen, 1986 **[standard reference, URL not verified]**), but: (a) the
*one-weapon-per-target* relaxation is a linear assignment problem solved exactly in
O(n³) by the Hungarian algorithm; (b) instances with tens of weapons/targets fall to
MILP, greedy-with-exchange, or auction methods in milliseconds.

**Dynamic WTA (DWTA)** adds time: engagement windows, weapon flyout times, shoot-look-shoot
(fire, observe outcome, re-engage), and arriving targets. The standard decomposition is
multi-stage: at each decision epoch solve a static WTA over currently feasible
weapon-target pairs, with lookahead via expected leakage. Surveys: Cai et al. (2006,
*J. Systems Engineering and Electronics*) and the 2024 comprehensive survey in
*Engineering Applications of AI* (models, algorithms, applications — the best single
modern entry point). Kline, Ahner & Hill's WTA survey (*Computers & Operations Research*,
2019) covers exact and heuristic methods.

**TEWA (Threat Evaluation and Weapon Assignment)** wraps WTA with a threat-ranking front
end: per-track threat value from (time-to-impact on defended asset, CPA, speed, class,
warhead estimate). Classic implementations use weighted-sum or fuzzy scoring; the
literature (e.g., the two-stage dynamic TEWA algorithm, arXiv:0906.5038; optimal dynamic
threat evaluation with shoot-look-shoot, *Knowledge-Based Systems* 2010) confirms the
two-stage architecture: **threat evaluation → resource scheduling**, re-run as a closed
loop. For your decoy problem, threat value must be an *expectation over class
posterior*: `V_t = Σ_c P(class=c | track) · V_c` — this is the formally correct way to
make decoys "cheap to ignore but expensive to be wrong about."

**Decentralized/market-based allocation.** The Consensus-Based Bundle Algorithm
(**CBBA**, Choi, Brunet & How, *IEEE Trans. Robotics* 2009) is the standard decentralized
auction for multi-agent multi-task allocation: each agent greedily builds a task bundle by
bidding, then a local-communication consensus phase resolves conflicts; guarantees
conflict-free assignment with ≥50% of optimal for submodular score functions, robust to
inconsistent situational awareness and changing network topology. Two maintained Python
implementations exist (zehuilu/CBBA-Python; keep9oing/consensus-based-bundle-algorithm).
CBBA naturally encodes time-discounted rewards (intercept earlier = better) and is the
correct choice if/when you decentralize the base-station planner onto the interceptors.

### Key references

| Reference | Notes |
|---|---|
| H.-L. Choi, L. Brunet, J. P. How, "Consensus-Based Decentralized Auctions for Robust Task Allocation," *IEEE Trans. Robotics* 25(4), 2009. [MIT DSpace](https://dspace.mit.edu/handle/1721.1/52330) ; [ACL project page](https://acl.mit.edu/projects/consensus-based-bundle-algorithm) | CBBA. Read fully; it is short and implementable. |
| Python CBBA implementations: [zehuilu/CBBA-Python](https://github.com/zehuilu/CBBA-Python), [keep9oing/consensus-based-bundle-algorithm](https://github.com/keep9oing/consensus-based-bundle-algorithm) | Reuse or port into your sim. |
| "A comprehensive survey of weapon target assignment problem: Model, algorithm, and application," *Engineering Applications of Artificial Intelligence*, 2024. [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0952197624013708) | Best modern survey; use its taxonomy for your related-work section. |
| Cai, Liu, Chen et al., "Survey of the research on dynamic weapon-target assignment problem," *J. Systems Engineering and Electronics*, 2006. [ResearchGate](https://www.researchgate.net/publication/223219633_Survey_of_the_research_on_dynamic_weapon-target_assignment_problem) | DWTA formulations and staging. |
| Kline, Ahner, Hill, "The Weapon-Target Assignment Problem," *Computers & Operations Research*, 2019. [Academia.edu copy](https://www.academia.edu/2959352/The_Weapon_Target_Assignment_Problem) | Exact + heuristic algorithm review. |
| "An optimal dynamic threat evaluation and weapon scheduling technique," *Knowledge-Based Systems*, 2010. [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0950705109001543) ; "A Novel Two-Stage Dynamic Decision Support based Optimal Threat Evaluation and Defensive Resource Scheduling Algorithm for Multi Air-borne Threats," arXiv:0906.5038. [arXiv](https://arxiv.org/pdf/0906.5038) | TEWA closed loop, shoot-look-shoot, stable-marriage variant. |
| "A Survey on Weapon Target Allocation Models and Applications," IntechOpen, 2021. [IntechOpen](https://www.intechopen.com/chapters/75331) | Free-access overview. |
| H. W. Kuhn, "The Hungarian Method for the Assignment Problem," *Naval Research Logistics Quarterly* 2:83–97, 1955. **[standard reference, URL not verified]** — in practice use `scipy.optimize.linear_sum_assignment` (modified Jonker–Volgenant). | Exact linear assignment in SciPy. |

### Recommended approach for COOP-UAV-S

1. **Threat evaluation node:** per track compute time-to-impact against the nearest
   defended/critical zone, predicted impact point, and class posterior; threat value
   `V_t = E_class[V_c] / (1 + t_impact/τ)`.
2. **Assignment node (centralized baseline):** build utility matrix
   `U[i,t] = Pk(i,t) · V_t − λ · E[collateral(i,t)] − μ · cost(interceptor_i)`, where
   `Pk(i,t)` comes from the engagement-envelope model (Section 5) evaluated at the
   predicted intercept point and `E[collateral]` from the debris model (Section 6).
   Solve with `linear_sum_assignment`; allow k-to-1 assignment for high-value targets by
   matrix replication (duplicate target columns with diminishing returns
   `1−(1−Pk)^k`).
3. **Re-solve every planning tick (e.g., 1 Hz) with hysteresis** (switching penalty added
   to off-diagonal of current assignment) to avoid churn — this is the standard DWTA trick.
4. **Shoot-look-shoot:** after each engagement, kill assessment updates the track; failed
   intercepts re-enter the pool with updated geometry. Reserve logic: keep `r` interceptors
   unassigned while expected leakers > 0.
5. **CBBA as phase 2**: same utility function, bids carry intercept-time discount;
   demonstrates graceful degradation when the base station link drops — a good demo
   scenario and differentiator.

### Study list

- Hungarian algorithm mechanics + `linear_sum_assignment` docs.
- CBBA paper (bundle construction, conflict-resolution table — the action table is the
  only tricky part).
- DWTA survey sections on shoot-look-shoot and engagement windows.
- Expected-value-with-class-uncertainty formulation (derive yourself; it's three lines but
  it is the project's TEWA novelty hook).

---

## 3. Multi-sensor multi-target tracking and fusion

### State of the art

**Single-target filtering.** Kalman filter for linear-Gaussian; EKF/UKF for nonlinear
(range-bearing radar measurements). For targets that switch dynamics (cruise → weave →
dive), the **Interacting Multiple Model (IMM)** estimator (Blom & Bar-Shalom, *IEEE TAC*
1988 **[standard reference, URL not verified]**) runs a small bank of motion models
(constant velocity, coordinated turn, constant acceleration/dive) with Markov switching
and is the de facto standard for air targets. The IMM's model-probability output doubles
as a **behavior feature for classification** (decoys may hold steadier CV; attack profiles
show dive-model activation).

**Data association (vector-type trackers).**
- **GNN** (global nearest neighbor): gate + Hungarian on Mahalanobis cost. Simple, fine
  for medium clutter; your baseline.
- **JPDA** (joint probabilistic data association, Bar-Shalom & Fortmann, *Tracking and
  Data Association*, Academic Press 1988 **[standard reference]**): soft association,
  good in clutter, suffers track coalescence with closely spaced raids.
- **MHT** (multiple hypothesis tracking, D. Reid, "An Algorithm for Tracking Multiple
  Targets," *IEEE Trans. Automatic Control* 24(6):843–854, 1979): deferred decisions over
  hypothesis trees; the gold standard, heavy to implement (Stone Soup has components).

**Random-finite-set (RFS) trackers.** Mahler's PHD filter (2003) propagates target
*intensity* instead of labeled tracks; the **GM-PHD** closed form (B.-N. Vo & W.-K. Ma,
"The Gaussian Mixture Probability Hypothesis Density Filter," *IEEE Trans. Signal
Processing* 54(11):4091–4104, 2006) handles unknown, time-varying target count and
birth/death natively — exactly the "raid appears at the radar horizon" problem. Labeled
extensions (GLMB — Vo & Vo 2013/2014; **LMB** — Reuter, Vo, Vo, Dietmayer, *IEEE TSP*
2014 **[standard references, URLs not verified]**) restore track identity. Practical
recent examples: a labeled GM-PHD for explicit multi-target tracking (*Sensors* 21(11):
3932, 2021) and multi-feature-matching GM-PHD for radar MTT (*Sensors* 22(14):5339, 2022).

**Library:** dstl **Stone Soup** (github.com/dstl/Stone-Soup, MIT license) provides all
of the above as composable components (predictors, updaters, hypothesisers, data
associators, deleters, initiators), plus platform/sensor simulators and metrics (OSPA,
SIAP). It is maintained by the UK Defence Science and Technology Laboratory and academia;
first beta 2019. `filterpy` (rlabbe) is the lighter alternative for KF/EKF/UKF/IMM if you
want minimal dependencies.

**Sensor phenomenology for small drones (what your detection models should encode).**
- **Radar:** Shahed-class RCS is small (sub-0.1 m² class head-on per open-source
  analysis — see drone-warfare.com's Shahed-136 research page; treat exact numbers as
  estimates) and low altitude puts targets in ground clutter; detection range is
  elevation-and-terrain limited. Micro-Doppler from the propeller is a classification
  bonus (Section 4). Decoys carry radar reflectors to *inflate* RCS — model RCS as
  class-conditional but *overlapping* distributions.
- **RF:** Shahed/Geran variants are largely RF-silent in cruise (inertial+GNSS; some have
  SIM/telemetry links), so RF sensors mainly catch FPV/Lancet video links and controlled
  decoys. Passive RF gives bearing (and TDOA position with multiple receivers).
- **Acoustic:** the Ukrainian **Sky Fortress** network (~14,000 cheap sensor nodes,
  $400–1000 each) reportedly tracks ~20% of all targets entering Ukrainian airspace from
  sound alone and cues mobile fire groups; **Zvook** uses acoustic mirrors on radio
  towers, claiming ~5 km detection on Shahed-class and ~7 km on cruise missiles, and
  degraded only ~3% when Russia altered Shahed acoustic signatures. Acoustic = short
  range, passive, cheap, classification-rich (engine note), poor in wind/rain.
- **EO/IR:** visual confirmation and terminal tracking; range limited, weather/night
  dependent (IR helps — Shahed's MD-550 engine is a thermal source). Essential for
  decoy discrimination by silhouette/size.
- **Fusion:** the C-UAS literature (e.g., *Sensors* 2022 "Review and Simulation of
  Counter-UAS Sensors for Unmanned Traffic Management"; industry surveys) is unanimous
  that no single modality suffices; fielded systems fuse RF + radar + acoustic + EO/IR
  with AI-driven correlation.

**Fusion architecture options:** (a) centralized measurement-level fusion — all
detections into one tracker; statistically optimal, what you should do in sim; (b)
track-to-track fusion — per-sensor trackers + covariance intersection at the base
station; more realistic for distributed sensors, needed if you model comm dropouts.

### Key references

| Reference | Notes |
|---|---|
| D. Reid, "An Algorithm for Tracking Multiple Targets," *IEEE Trans. Automatic Control* 24(6):843–854, 1979. **[bibliographic details verified via secondary sources]** | MHT origin. |
| B.-N. Vo, W.-K. Ma, "The Gaussian Mixture Probability Hypothesis Density Filter," *IEEE Trans. Signal Processing* 54(11):4091–4104, 2006. **[bibliographic details verified via secondary sources]** | GM-PHD; the closed-form recursion you would implement. |
| Y. Bar-Shalom, T. Fortmann, *Tracking and Data Association*, Academic Press, 1988; H. Blom, Y. Bar-Shalom, "The IMM algorithm for systems with Markovian switching coefficients," *IEEE TAC* 33(8), 1988. **[standard references, URLs not verified]** | JPDA + IMM canon. |
| S. Reuter, B.-T. Vo, B.-N. Vo, K. Dietmayer, "The Labeled Multi-Bernoulli Filter," *IEEE Trans. Signal Processing* 62(12), 2014. **[standard reference, URL not verified]** | LMB; labeled RFS upgrade path. |
| dstl Stone Soup framework. [GitHub](https://github.com/dstl/Stone-Soup) ; framework paper: "An open source framework for tracking and state estimation ('Stone Soup')," 2017. [ResearchGate](https://www.researchgate.net/publication/316653853_An_open_source_framework_for_tracking_and_state_estimation_Stone_Soup') ; multi-target tutorial: [06_DataAssociation-MultiTargetTutorial](https://github.com/dstl/Stone-Soup/blob/main/docs/tutorials/06_DataAssociation-MultiTargetTutorial.py) | **Primary reuse candidate.** Also see the 3D platform MTT example. |
| "A Labeled GM-PHD Filter for Explicitly Tracking Multiple Targets," *Sensors* 21(11):3932, 2021. [MDPI](https://www.mdpi.com/1424-8220/21/11/3932) | Practical labeled GM-PHD recipe. |
| "Multi-Feature Matching GM-PHD Filter for Radar Multi-Target Tracking," *Sensors* 22(14):5339, 2022. [MDPI](https://www.mdpi.com/1424-8220/22/14/5339) | GM-PHD with feature-aided association. |
| "Review and Simulation of Counter-UAS Sensors for Unmanned Traffic Management," *Sensors*, 2022. [PMC](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8747651/) | Sensor performance models you can copy into your detection nodes. |
| US Army, "Listening to the Sky: Acoustic Drone Detection Systems – Ukraine & Emerging Technologies," 2026. [army.mil](https://www.army.mil/article/292099/listening_to_the_sky_acoustic_drone_detection_systems_ukraine_emerging_technologies) ; Sky Fortress overview [United24](https://united24media.com/war-in-ukraine/sky-fortress-ukraines-acoustic-detection-system-that-tracks-drones-cheap-and-fast-9451) ; [Zvook](https://www.zvook.tech/en) | Acoustic network parameters for your acoustic sensor model. |
| Shahed-136 RCS/countermeasures research page. [drone-warfare.com](https://drone-warfare.com/research/shahed-136/) | RCS and cost estimates (open-source, treat as approximate). |

### Recommended approach for COOP-UAV-S

1. **Detection layer per sensor** (your sim nodes): probability-of-detection curves
   `Pd(range, altitude, RCS/loudness, weather)` + measurement noise models: radar =
   range/azimuth/elevation (+ radial velocity + micro-Doppler feature), acoustic =
   bearing(+rough range) with ~3–5 km max range, RF = bearing/TDOA only for emitting
   classes, EO/IR = angles + class evidence inside ~2–4 km. Add false alarms (Poisson
   clutter) — without clutter your tracker proves nothing.
2. **Tracker:** start with `filterpy`-based IMM (CV + coordinated-turn + dive models) +
   gating + Hungarian GNN, all centralized. If time permits, swap in Stone Soup and run
   GM-PHD vs GNN on the same scenario, reporting OSPA — an easy, publishable-looking
   ablation.
3. **Track fusion:** measurement-level fusion at base station; simulate per-sensor
   latency and dropouts via your message bus.
4. **Kill assessment** is just tracking: a successful intercept = track transitions to
   ballistic-debris dynamics (feeds Section 6).
5. Keep tracker output as ROS-2-shaped `TrackArray` messages (id, state, covariance,
   class posterior, model probabilities) — that message *is* the interface to TEWA.

### Study list

- Roger Labbe, *Kalman and Bayesian Filters in Python* (free Jupyter book accompanying
  `filterpy`) — fastest practical on-ramp. **[well-known resource; github.com/rlabbe]**
- IMM chapter of Bar-Shalom (or Labbe's IMM notebook).
- Stone Soup tutorials 1–10 (KF → multi-target association → PHD).
- GM-PHD recursion (Vo & Ma 2006, Section III) — implementable in a day.
- OSPA metric definition (for evaluation plots).

---

## 4. Drone classification and decoy discrimination

### State of the art

**Micro-Doppler radar.** Rotating propellers/rotors impose periodic modulation
(blade flashes, HERM lines — HElicopter Rotor Modulation) on the radar return, visible in
STFT spectrograms. Verified results: drones vs birds separate cleanly at K-band and
W-band (Rahman & Robertson, *Scientific Reports* 2018); drone *type* recognition
(fixed-wing puller-prop vs multirotor vs VTOL hybrid) from blade-modulation structure
(*Drones* 7(4):280, 2023); dual-band (K+X) spectrogram PCA classification (IEEE Radar
2018); multifunctional-radar micro-Doppler classification (*Drones* 6(5):124, 2022);
HERM-line behavior under maneuvering (*Sensors* 2020, PMC7590031). For COOP-UAV-S:
a Shahed's single piston engine + pusher prop gives a strong, characteristic prop-modulation
line; a Gerbera with a smaller engine/electric motor differs in modulation frequency and
strength *even when its RCS is spoofed* — this is the physically-grounded discriminator
to model.

**RF fingerprinting.** Deep learning on raw IQ/spectrograms identifies drone presence,
model, and even individual airframe (transmitter imperfections). Datasets: **DroneRF**
(2.4 GHz band; Bebop/AR/Phantom + background) and newer UAVSig; methods: CNNs on
PSD/STFT/wavelet features (*Sensors* 23(9):4202, 2023 end-to-end; complex-valued CNN,
*Drones* 6(12):374, 2022); domain shift is the open problem (CrossRF, arXiv:2505.18200).
Limitation for your scenario: GNSS/inertial Shaheds are mostly RF-quiet; RF
classification matters for FPV (analog video ~5.8 GHz), Lancet (downlink), and any
decoys that emit.

**Acoustic classification.** Engine/prop acoustic signatures separate piston-engine OWA
drones from electric multirotors trivially; Ukraine's Zvook reports robust ML
classification with only ~3% accuracy degradation after Russian signature-alteration
attempts (see Section 3 references). Acoustic is also the cheapest sensor to spoof in
principle (decoys could carry speakers) — worth a red-team scenario in sim.

**The Gerbera-vs-Shahed problem (verified open-source facts).**
- Gerbera: foam/plywood construction, wingspan ~2 m vs ~2.5 m Geran-2, cost ~$10k vs
  ~$20–50k+ (estimates vary), production ≥50/day; many carry **radar reflectors
  (Luneburg lenses) explicitly to mimic Shahed RCS**; variants range from pure decoy to
  recon to small-warhead (≤5 kg) strike vs Shahed's 50–90 kg.
  Sources: [Wikipedia: Gerbera](https://en.wikipedia.org/wiki/Gerbera_(drone)),
  [ISIS report on Russian decoy drones](https://isis-online.org/isis-reports/russian-decoy-drones-that-depend-on-western-parts-pose-a-great-challenge/),
  [CEPA "The Phony War"](https://cepa.org/article/the-phony-war-ukraine-and-russias-decoy-drones/).
- Scale of the problem: Ukrainian DIU reported ~half of launched "Shaheds" were decoys
  (Nov 2024); open-source monthly analyses put decoy share at 40–60% in some months
  ([ISIS monthly Shahed analyses](https://isis-online.org/isis-reports/monthly-analysis-of-russian-shahed-136-deployment-against-ukraine)).
- Ukrainian counter-move: **AI-assisted radar filtering** that classifies drone type and
  separates lethal threats from decoys faster
  ([Kyiv Post](https://www.kyivpost.com/post/55848)).
- Published *algorithms* for decoy discrimination are scarce (operationally sensitive) —
  this is a genuine literature gap your project can legitimately claim to explore in
  simulation.

**Behavioral/kinematic discrimination (the COOP-UAV-S angle).** Physics that differs
between a 200+ kg Shahed and a ~20 kg foam Gerbera even with identical RCS:
wing loading → gust response (light decoys bounce more in turbulence), climb rate,
turn radius at speed, cruise speed sustainment, terminal-dive capability, IR signature
(engine size), acoustic power, and *mission behavior* (decoys often fly racetracks or
loiter to maximize defender attention; strike profiles converge on targets). A Bayesian
recursive classifier over track features (speed variance, vertical activity,
micro-Doppler line frequency, acoustic class, RCS fluctuation statistics) is the right
model — and the IMM model probabilities from Section 3 are free features.

**Adversarial robustness.** The decoy *is* the adversarial attack: the enemy controls
RCS (reflectors), paint (black, anti-EO), routes, and possibly acoustic signature.
Design the classifier study around: (i) overlapping class-conditional feature
distributions, (ii) value-of-information — when is it worth tasking an interceptor or
EO/IR sensor to *visually confirm* before assignment (a sensor-management problem), and
(iii) decision-theoretic thresholds tied to interceptor inventory (with many cheap
interceptors you can afford to shoot decoys; with few, confirmation matters).

### Key references

| Reference | Notes |
|---|---|
| Rahman & Robertson, "Radar micro-Doppler signatures of drones and birds at K-band and W-band," *Scientific Reports* 8, 2018. [Nature](https://www.nature.com/articles/s41598-018-35880-9) | Drone-vs-bird micro-Doppler; spectrogram examples worth replicating synthetically. |
| "Classification of drones based on micro-Doppler signatures with dual-band radar sensors," IEEE, 2018. [IEEE Xplore](https://ieeexplore.ieee.org/document/8293214/) | STFT + PCA + feature fusion pipeline. |
| "Exploring Radar Micro-Doppler Signatures for Recognition of Drone Types," *Drones* 7(4):280, 2023. [MDPI](https://www.mdpi.com/2504-446X/7/4/280) | Fixed-wing vs multirotor vs VTOL from blade type — closest analog to Shahed-vs-quad discrimination. |
| "Drones Classification by the Use of a Multifunctional Radar and Micro-Doppler Analysis," *Drones* 6(5):124, 2022. [MDPI](https://www.mdpi.com/2504-446X/6/5/124) | Operational-radar perspective. |
| "An Investigation of Rotary Drone HERM Line Spectrum under Manoeuvering Conditions," *Sensors*, 2020. [PMC](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7590031/) | HERM lines under maneuver — robustness caveats. |
| "RF-Enabled Deep-Learning-Assisted Drone Detection and Identification: An End-to-End Approach," *Sensors* 23(9):4202, 2023. [MDPI](https://www.mdpi.com/1424-8220/23/9/4202) | End-to-end RF pipeline incl. DroneRF-style data. |
| "Deep Complex-Valued CNN for Drone Recognition Based on RF Fingerprinting," *Drones* 6(12):374, 2022. [MDPI](https://www.mdpi.com/2504-446X/6/12/374) ; "CrossRF: A Domain-Invariant Deep Learning Approach for RF Fingerprinting," arXiv:2505.18200. [arXiv](https://arxiv.org/pdf/2505.18200) | RF fingerprinting SOTA + the domain-shift problem. |
| Gerbera/decoy open-source intelligence: [Wikipedia](https://en.wikipedia.org/wiki/Gerbera_(drone)), [ISIS decoy report](https://isis-online.org/isis-reports/russian-decoy-drones-that-depend-on-western-parts-pose-a-great-challenge/), [CEPA](https://cepa.org/article/the-phony-war-ukraine-and-russias-decoy-drones/), [Kyiv Post AI filtering](https://www.kyivpost.com/post/55848), [Army Recognition analysis](https://www.armyrecognition.com/news/army-news/2025/exclusive-analysis-russias-low-cost-gerbera-kamikaze-drones-break-ukrainian-defenses-threaten-nato-frontier) | Ground truth for your decoy threat model parameters. |

### Recommended approach for COOP-UAV-S

1. **Don't simulate raw waveforms.** Model each sensor as emitting *class-evidence
   likelihood vectors* with class-conditional feature distributions you parameterize from
   the references (e.g., micro-Doppler line frequency: Shahed prop ~50–100 Hz blade-pass
   class vs decoy motor — pick plausible separated-but-overlapping Gaussians; RCS: heavily
   overlapping by design; acoustic class: good separation inside 3–5 km; EO/IR size:
   decisive inside ~2 km).
2. **Recursive Bayesian class posterior per track:**
   `P(c | z_1:k) ∝ P(c | z_1:k−1) · Π_sensors L(z_k | c)`, with a confusion-matrix
   knob per sensor so you can sweep "how good must classification be before
   collateral-aware WTA beats shoot-everything."
3. **Add kinematic-behavior features** (wing-loading-driven gust response as
   class-dependent process noise; decoy loiter patterns in the threat generator) so
   behavior-based discrimination has signal to find.
4. **Headline experiment:** defended-asset damage + interceptors expended vs decoy ratio
   (0–60%) under three policies — ignore classification / threshold classification /
   expected-value TEWA with confirmation tasking. This directly addresses the
   Gerbera-vs-Shahed problem with a result no public paper currently shows.

### Study list

- Micro-Doppler basics: Rahman & Robertson 2018 + V. C. Chen's *The Micro-Doppler Effect
  in Radar* (Artech House) **[standard reference, URL not verified]**.
- Bayesian recursive classification + confusion matrices (any estimation text).
- Sensor management / value of information (Stone Soup has a sensor-management tutorial).
- OSINT pages above for realistic decoy parameters.

---

## 5. Guidance laws for drone-on-drone intercept

### State of the art

**Proportional navigation (PN) family.** Commanded acceleration `a = N · λ̇ · V_c`
(N ≈ 3–5, λ̇ = line-of-sight rate, V_c = closing velocity). Variants: **Pure PN**
(acceleration ⊥ LOS), **True PN** (⊥ pursuer velocity), **Augmented PN** (adds
`N/2 · a_T` target-maneuver compensation — capture zone provably expands vs maneuvering
targets, see JGCD capturability analysis below). The canonical engineering text is
Zarchan, *Tactical and Strategic Missile Guidance* (AIAA, 6th ed. 2012; 7th ed. 2019).
PN is optimal (minimum-effort) against non-maneuvering targets — which a cruising Shahed
approximately is — and is what you should implement first.

**PN on multirotors/interceptor UAVs (verified studies).** "Evaluation of Proportional
Navigation for Multirotor Pursuit" (AIAA SciTech 2021) shows PN works well for multirotor
pursuit with metrics of time-to-intercept, miss distance, and command variance, with
modifications needed because multirotors control acceleration through attitude (lag) and
have tight accel limits. A 2021 *Intelligent Service Robotics* paper evaluates several
3D PN-based laws (AIPNG, modified AIPNG, ATPNG) in real time on quadrotors. CMU MSR
thesis "Quadrotor Guidance for Targeting Aerial Objects" (Bhattacharya, 2020) covers the
vision-in-the-loop version. The CTU Prague MRS lab work is the closest academic analog
to your whole stack: "Autonomous capture of agile flying objects using UAVs: the MBZIRC
2020 challenge" (*Robotics and Autonomous Systems*, 2022) — onboard LiDAR detection,
target state estimation, interception trajectory planning into a carried net; and
"Towards Safe Mid-Air Drone Interception: Strategies for Tracking & Capture"
(arXiv:2405.13542) compares tracking/capture strategies explicitly for safety. Their
commercial spin-off **Eagle.One** (with Fly4Future) is a fully autonomous net-capture
drone hunter.

**Terminal-constraint/optimal guidance.** Impact-angle- and impact-point-constrained
guidance (trajectory shaping; biased PN; optimal guidance laws with terminal constraints,
all covered in Zarchan and the JGCD literature) is *directly relevant to your
collateral-damage innovation*: constraining the intercept geometry constrains where the
wreck falls. A practical sim-level approach: choose the *intercept point* (not just any
PN collision course) by optimizing predicted debris footprint over the zone map, then fly
PN to that point — i.e., move the optimal-control burden into the planner and keep the
guidance law dumb.

**Real interceptor systems (verified open-source parameters).**

| System | Type / kill mechanism | Speed | Range / endurance | Notes & sources |
|---|---|---|---|---|
| **Anduril Anvil / Anvil-M** | Quadrotor kinetic rammer; Anvil-M adds munition | up to ~200 mph (~320 km/h) reported | Group 1–2 UAS targets; cued by Lattice C2 | Human-authorized defeat; mobile kit demoed at Falcon Peak 2025. [Anduril](https://www.anduril.com/anvil), [Defense Post](https://thedefensepost.com/2025/10/20/anduril-demos-cuas-falcon-peak/) |
| **Fortem DroneHunter F700** | Multirotor, radar-guided **net gun** (multiple net pods); tows or parachutes captured drone | rotorcraft-class (~tens of m/s) | repeat engagements per sortie | Non-destructive capture incl. fixed-wing targets; NetGun pods for Group 1–3. [Fortem](https://fortemtech.com/products/dronehunter-f700/) |
| **Raytheon Coyote Block 2 / 2+** | Tube/rocket-launched jet interceptor, tungsten **fragmentation warhead** | 555–595 km/h (345–370 mph) | 10–15 km engagement; ~4 min loiter, re-attack capable | Paired with KuRFS radar in US Army LIDS/M-LIDS; combat kills achieved; ~6,700 units planned FY25–29. [Wikipedia](https://en.wikipedia.org/wiki/Raytheon_Coyote), [RTX](https://www.rtx.com/raytheon/what-we-do/integrated-air-and-missile-defense/coyote) |
| **Wild Hornets Sting (UA)** | FPV-style high-speed quad interceptor, impact/charge kill, human-piloted with assisted terminal | ~280 km/h max (some reports up to ~315 km/h in dive) | vs Shahed/Geran, Lancet, recon UAVs | Claimed 80–95% per-sortie hit rate depending on crew; >3,900 Shahed-class kills by Feb 2026; ~6 days pilot training. [Wild Hornets](https://wildhornets.com/en/sting-interceptor), [Wikipedia](https://en.wikipedia.org/wiki/Sting_(drone)), [United24](https://united24media.com/latest-news/ukrainian-interceptor-drones-tear-through-russian-shahed-swarms-with-95-kill-rate-18022), [Defense Express](https://en.defence-ua.com/news/ukrainian_wild_hornets_workshop_reveals_how_many_russian_drones_downed_by_sting_interceptors_in_five_months-16013.html) |
| **ParaZero DefendAir net pods** | Net effector in 3 configs | — | drone-mounted: engage after ~2 km closing; turret: ≤100 m; hand-held: ≤35 m; nets 9–100 m² | The best public numbers for **net engagement envelopes**. [Interesting Engineering](https://interestingengineering.com/military/net-based-counter-drone-system) |

**Published Pk data: essentially none** for Western systems (procurement numbers and
"combat kills" only). Ukrainian claims (70% fleet-wide; 80–95% Sting crews; "one launch
one kill" best days) are self-reported and should be modeled as optimistic upper bounds;
CSIS analyses note overall Shahed get-through rates rose during 2025 despite high
per-engagement claims (saturation, altitude changes). For the sim, make Pk an explicit
envelope function, not a constant.

**Net-gun engagement modeling.** Public engineering data: "Design and Testing of a
Net-Launch Device for Drone Capture" (2022, via ResearchGate) and the DefendAir numbers
above. Practical envelope model: net effective only within ~10–30 m, inside a forward
cone (~±15–30°), with closing speed below a threshold (net integrity / aiming), and Pk
falling with target speed and crossing angle. MBZIRC 2020 results (RAS 2022 paper) show
even catching a *cooperatively flown* ball-towing drone was hard — encode that humility:
single-shot net Pk vs a 60 m/s Shahed should be modest (e.g., 0.3–0.6 in-envelope), which
is precisely what makes cooperative multi-interceptor tactics and shoot-look-shoot
valuable in your study.

### Key references

| Reference | Notes |
|---|---|
| P. Zarchan, *Tactical and Strategic Missile Guidance*, AIAA Progress in Astronautics and Aeronautics, 6th ed. 2012 / 7th ed. 2019. [AIAA](https://arc.aiaa.org/doi/10.2514/4.868948) | THE guidance text. Ch. 2 (PN basics), Ch. 8 (APN), terminal-constraint chapters. |
| "Evaluation of Proportional Navigation for Multirotor Pursuit," AIAA SciTech 2021. [AIAA](https://arc.aiaa.org/doi/10.2514/6.2021-1813) | PN adapted to multirotor dynamics — your interceptor model. |
| "Real-time interception performance evaluation of certain proportional navigation based guidance laws in aerial ground engagement," *Intelligent Service Robotics*, 2021. [Springer](https://link.springer.com/article/10.1007/s11370-021-00404-4) | 3D PN-variant comparison on quadrotor. |
| "Capturability of Augmented Pure Proportional Navigation Guidance Against Time-Varying Target Maneuvers," *J. Guidance, Control, and Dynamics*. [AIAA](https://arc.aiaa.org/doi/abs/10.2514/1.G000561) | Why APN for the diving/weaving cases. |
| Vrba et al. (CTU MRS), "Autonomous capture of agile flying objects using UAVs: The MBZIRC 2020 challenge," *Robotics and Autonomous Systems*, 2022. [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0921889021002396) ; "Towards Safe Mid-Air Drone Interception: Strategies for Tracking & Capture," arXiv:2405.13542. [arXiv](https://arxiv.org/pdf/2405.13542) ; [Eagle.One](https://mrs.fel.cvut.cz/projects/eagle-one) | Closest end-to-end academic analog (detect→track→plan→net). |
| "Design and Testing of a Net-Launch Device for Drone Capture," 2022. [ResearchGate](https://www.researchgate.net/publication/357563196_Design_and_Testing_of_a_Net-Launch_Device_for_Drone_Capture) | Net launcher engineering data. |
| System pages cited in the table above (Anduril, Fortem, RTX/Wikipedia Coyote, Wild Hornets, ParaZero coverage). | Parameter anchors for the sim. |
| "Counter-Unmanned Aircraft System(s) (C-UAS): State of the Art, Challenges and Future Trends," arXiv:2008.12461. [arXiv](https://arxiv.org/pdf/2008.12461) | Broad C-UAS survey incl. effectors taxonomy. |

### Recommended approach for COOP-UAV-S

1. **Guidance node per interceptor:** PN with N=3–4 on the fused track; switch to APN
   when IMM dive/turn-model probability exceeds threshold; acceleration saturation +
   first-order attitude lag to stay honest about multirotor dynamics.
2. **Intercept-point selection in the planner, not the guidance law:** sample candidate
   PN-feasible intercept points along the predicted target path, score each by
   `Pk(geometry) − λ·E[collateral | debris from this point]`, pick argmax. This is where
   your two innovations (cooperation + collateral-awareness) meet.
3. **Pk envelope model:** `Pk = f(range_at_release, aspect angle, closing speed, target
   class, effector type)` — net (short range, low closing speed, high vs slow targets) vs
   frag/ram (longer reach, works vs fast targets, worse debris). Two effector types make
   the WTA and collateral tradeoffs non-trivial — keep both.
4. **Shoot-look-shoot** ties guidance to Section 2; failed net pass → re-attack loop
   (DroneHunter and Coyote both advertise re-engagement).
5. Calibrate scenario numbers from the table: interceptor 70–90 m/s (Sting-class),
   Shahed 50–65 m/s cruise / up to ~90 m/s dive, engagement initiation 5–15 km from
   defended zone.

### Study list

- Zarchan Ch. 2 + simulate 2D PN in 50 lines before anything else.
- AIAA 2021 multirotor-PN paper (what breaks when the "missile" is a quad).
- MBZIRC 2020 RAS paper end-to-end (architecture mirror of your project).
- APN capturability (skim; take the design rule N≥3, add a_T/2 term).

---

## 6. Collateral damage and ground risk modeling

### State of the art

**Regulatory scaffold — JARUS SORA.** The Specific Operations Risk Assessment (JARUS,
adopted by EASA; SORA 2.0 → 2.5) assigns a **Ground Risk Class (GRC)** from UA dimension
+ kinetic energy + operational scenario (controlled area / sparsely populated / populated
/ assemblies of people), then applies mitigations (sheltering, ERP, parachute) to reach a
final GRC and SAIL level. Three harm categories: ground third parties, air third parties,
critical infrastructure. SORA's GRC table is a *defensible, citable basis for your
safe/dangerous/critical zone taxonomy* — map your city zones to population-density bands
and sheltering factors exactly as SORA does, then your "shot authorization" becomes a
quantitative extension of an accepted methodology (with the twist that *you* choose where
the vehicle falls).

**Quantitative ground-risk models (academic).** The standard casualty chain:

`P(casualty) = P(crash) × P(impact in cell | crash) × P(person in casualty area | cell) × P(fatality | hit)`

- **Impact distribution:** ballistic/uncontrolled-descent Monte Carlo from the failure
  point — 6-DOF or point-mass-with-drag descent under wind and parameter uncertainty
  produces a 2-D impact PDF ("Accurate Ground Impact Footprints and Probabilistic Maps
  for Risk Analysis of UAV Missions," 2019; ground-risk estimation with
  multi-uncertainties literature). For a *shot-down* drone you additionally inherit the
  pre-impact velocity vector — a Shahed killed in a 60 m/s cruise carries ~2.5 km of
  ballistic forward throw from 2 km altitude (your sim should compute this, it is the
  whole point of intercept-point optimization). Note: one MDPI Electronics paper in this
  space (12(4):829) was **retracted** — avoid citing it.
- **Casualty area:** for vertical impacts, a disk of radius (person radius + debris
  radius); for shallow impacts, a swept rectangle (glide/slide). Sheltering factor
  reduces P(fatality) indoors. These models trace back to range-safety practice
  (RCC 321 standard **[standard reference, URL not verified]**) and are restated in the
  papers below.
- **Risk maps:** Primatesta, Rizzo & la Cour-Harbo, "Ground Risk Map for Unmanned
  Aircraft in Urban Environments" (*J. Intelligent & Robotic Systems*, 2020) — layered
  raster: population density × sheltering × obstacles × no-fly, combined into expected
  fatalities per flight-hour per cell; the de facto reference implementation of exactly
  the map COOP-UAV-S needs. Zhang et al. (*J. Advanced Transportation*, 2018) give an
  NAS-level risk estimation framework; a 2025 *Risk Analysis* paper does risk-based UAV
  path planning over complex air-ground environments.
- **Risk-aware path planning:** plan interceptor routes (and intercept points) over the
  risk raster with cost = ∫ risk d t — standard A*/RRT*-with-risk-cost formulations in the
  above literature transfer directly.

**The C-UAS twist (your novelty).** The literature above is all about *accidental* UAS
crashes. Engagement-driven debris placement — choosing WHERE to make an intentional kill
to minimize ground casualties — is essentially unpublished in open academic literature
(it exists in classified fire-control doctrine; Israeli/Ukrainian practice of debris
casualties from intercepts is widely reported in news). COOP-UAV-S's probabilistic
"debris-footprint-aware shot authorization" is therefore a defensible novelty claim:
you are composing (a) SORA-style zone maps, (b) ballistic debris Monte Carlo with
intercept initial conditions, and (c) WTA utility — each individually standard, jointly
new in the open literature. Also model the *negative* case: warhead detonation on a
critical zone if you DON'T intercept — the authorization problem is a comparison of two
risk integrals, not a veto.

### Key references

| Reference | Notes |
|---|---|
| JARUS, "Guidelines on Specific Operations Risk Assessment (SORA)," JAR-DEL-WG6-D.04. [Executive summary PDF](http://jarus-rpas.org/wp-content/uploads/2023/07/jar_doc_06_jjarus_sora_executive_summary.pdf) ; [EASA SORA page](https://www.easa.europa.eu/en/domains/drones-air-mobility/operating-drone/specific-category-civil-drones/specific-operations-risk-assessment-sora) | GRC tables, mitigation factors, harm categories. |
| S. Primatesta, A. Rizzo, A. la Cour-Harbo, "Ground Risk Map for Unmanned Aircraft in Urban Environments," *J. Intelligent & Robotic Systems*, 2020. [Springer](https://link.springer.com/article/10.1007/s10846-019-01015-z) | **Reference implementation for your zone/risk raster.** |
| "Accurate Ground Impact Footprints and Probabilistic Maps for Risk Analysis of UAV Missions," IEEE Aerospace 2019. [ResearchGate](https://www.researchgate.net/publication/331502323_Accurate_Ground_Impact_Footprints_and_Probabilistic_Maps_for_Risk_Analysis_of_UAV_Missions) | Monte Carlo impact footprints (6-DOF + wind). |
| X. Zhang et al., "Safety Assessment and Risk Estimation for Unmanned Aerial Vehicles Operating in National Airspace System," *J. Advanced Transportation*, 2018. [Hindawi](https://www.hindawi.com/journals/jat/2018/4731585/) | Casualty-area + fatality-probability formulation. |
| Liu et al., "Ground Risk Assessment of UAV Operations Based on Horizontal Distance Estimation under Uncertain Conditions," *Math. Problems in Engineering*, 2021. [Wiley](https://onlinelibrary.wiley.com/doi/10.1155/2021/3384870) | Vertical vs horizontal impact casualty areas. |
| Zhou et al., "A risk-based unmanned aerial vehicle path planning scheme for complex air–ground environments," *Risk Analysis*, 2025. [Wiley](https://onlinelibrary.wiley.com/doi/10.1111/risa.17685) | Risk-cost path planning pattern. |
| A. la Cour-Harbo, "Quantifying risk of ground impact fatalities for small unmanned aircraft," *J. Intelligent & Robotic Systems*, 2019 — ballistic descent model series. **[known body of work by a co-author of the verified Primatesta paper; exact citation not re-verified — check before formal citation]** | Ballistic descent math if you want the closed forms. |
| RCC Standard 321 (Range Safety Criteria for Unmanned Aerial Vehicles) **[standard reference, URL not verified]** | Origin of casualty-area/Ec methodology. |

### Recommended approach for COOP-UAV-S

1. **City risk raster** (e.g., 25–50 m cells): layers = population density (day/night),
   sheltering factor, zone class {safe, dangerous, critical-infrastructure}; store as
   numpy arrays, polygons via `shapely`, optional `geopandas` for real city geometry.
2. **Debris model per kill type:** intercept gives initial state (position, velocity,
   altitude, target mass, breakup flag). Sample N=200–1000 fragments/trajectories:
   point-mass + drag (class-dependent ballistic coefficient distribution) + wind +
   breakup dispersion → impact PDF → convolve with casualty area → expected casualties.
   Precompute lookup tables over (altitude, speed, heading) bins for real-time WTA.
3. **Authorization rule:** engage at candidate intercept point `x` iff
   `E[casualties | intercept at x] < E[casualties | leakage] − margin`, choose `x`
   minimizing the left side subject to Pk floor. Surface both numbers on the dashboard —
   that comparison is your demo's money shot.
4. **Account for the warhead:** detonation vs inert decoy changes the debris energy —
   classification posterior (Section 4) enters the debris model too, coupling all four
   subsystems.
5. Validate qualitatively against reported Ukrainian experience (falling intercept debris
   causing casualties is a documented phenomenon motivating exactly this optimization).

### Study list

- SORA executive summary (1 hr) + GRC annex.
- Primatesta 2020 paper (the raster recipe).
- Ballistic descent with drag (derive: ~10 lines of ODE; verify against footprint paper).
- Expected-casualty arithmetic (casualty area × density × (1−shelter)).

---

## 7. Simulation frameworks and the migration path

### State of the art

| Framework | What it is | Verdict for COOP-UAV-S |
|---|---|---|
| **Custom Python time-stepped sim (current)** | Your pub/sub bus + dataclass msgs + node lifecycle | Correct for now: full control of time, determinism, 100s of agents, trivial CI. |
| **Aerostack2** | ROS 2 framework for multi-robot aerial systems: platform-abstracted (PX4, DJI, Crazyflie, Gazebo, Isaac), behaviors, mission specification. arXiv:2303.18237. [arXiv](https://arxiv.org/pdf/2303.18237), [GitHub aerostack2](https://github.com/aerostack2) | **Primary migration target.** Plugin architecture maps to your nodes; multi-UAV native. |
| **PX4 SITL + Gazebo (+ ROS 2 via XRCE-DDS)** | Autopilot-in-the-loop; multi-vehicle supported. [PX4 multi-vehicle docs](https://docs.px4.io/main/en/ros2/multi_vehicle) | The realism layer under Aerostack2; multi-vehicle scaling is the practical bottleneck (each instance is heavy). |
| **gym-pybullet-drones** (utiasDSL) | PyBullet Gymnasium envs for single/multi-agent quadrotor RL. Panerati et al., "Learning to Fly — a Gym Environment with PyBullet Physics for RL of Multi-agent Quadcopter Control," IROS 2021, doi:10.1109/IROS51168.2021.9635857. [GitHub](https://github.com/utiasDSL/gym-pybullet-drones), [paper](https://dl.acm.org/doi/10.1109/IROS51168.2021.9635857) | Use for the MARL stretch goal; don't base the main sim on it (physics fidelity you don't need, scenario tooling you'd have to build anyway). |
| **Crazyswarm2** | ROS 2 stack for swarms of Crazyflie nano-quads (successor of Preiss et al. ICRA 2017 Crazyswarm). [GitHub IMRCLab/crazyswarm2 — well-known repo, URL from memory] | Only relevant if you ever do a lab hardware demo with nano-quads. |
| **Pegasus Simulator** | Isaac Sim-based multi-vehicle PX4 sim framework; integrates with Aerostack2. [aerostack2/project_pegasus_isaac_sim](https://github.com/aerostack2/project_pegasus_isaac_sim) | Photorealism/sensor realism later; GPU-heavy. |
| **AirSim → Colosseum** | Microsoft AirSim was archived (2022); community fork Colosseum continues (CodexLabsLLC/Colosseum) **[well-known status; repo URL from memory]** | Avoid building on it; ecosystem momentum moved on. |
| **MRS UAV System (CTU)** | ROS-based full UAV control/estimation stack used in the interception papers of Section 5. [GitHub](https://github.com/ctu-mrs/mrs_uav_system) | Reference architecture; their interception pipeline is the migration blueprint. |
| **aerial-autonomy-stack** | Faster-than-real-time, autopilot-agnostic ROS 2 sim/deploy framework, arXiv:2602.07264. [arXiv](https://arxiv.org/html/2602.07264v1) | New (2026); watch it — "faster than real time" matters for Monte Carlo campaigns. |
| **Stone Soup** (again) | Tracker + *sensor/platform simulator* + metrics. [GitHub](https://github.com/dstl/Stone-Soup) | Reuse its sensor models & OSPA metrics inside your custom sim. |

**Migration-proofing rules (the actionable part):**

1. **Mirror ROS 2 semantics exactly now:** topic names (`/uav_1/odom`,
   `/tracks`, `/engagements/cmd`), QoS-like delivery options (reliable vs best-effort,
   latched), dataclass fields matching `geometry_msgs/PoseStamped`,
   `nav_msgs/Odometry`, and a custom `TrackArray`/`EngagementCommand` whose `.msg`
   definitions you write *now* even though only dataclasses exist. Then migration =
   swapping the bus for `rclpy` publishers, not refactoring.
2. **Keep sim time explicit** (`/clock`-style stamped messages) so the move to Gazebo's
   sim time is mechanical; never call wall-clock in nodes.
3. **Separate plant from autonomy:** vehicle dynamics + sensors live behind the same
   message interface that PX4 SITL+Gazebo would provide; planner/tracker/WTA nodes never
   know they're in a toy sim.
4. **Scenario/config as data** (YAML raid scripts, zone maps) reusable across backends.
5. For RL: wrap your sim in a PettingZoo `ParallelEnv` interface — then MAPPO via any
   library runs against either your sim or gym-pybullet-drones.

### Key references

- Fernandez-Cortizas et al., "Aerostack2: A Software Framework for Developing Multi-robot
  Aerial Systems," arXiv:2303.18237. [arXiv](https://arxiv.org/pdf/2303.18237)
- Panerati et al., IROS 2021 (full citation above). [ACM/IEEE](https://dl.acm.org/doi/10.1109/IROS51168.2021.9635857)
- PX4 multi-vehicle ROS 2 simulation docs. [PX4](https://docs.px4.io/main/en/ros2/multi_vehicle)
- "ROS-Based Multi-Domain Swarm Framework for Fast Prototyping," *Aerospace* 12(8):702.
  [MDPI](https://www.mdpi.com/2226-4310/12/8/702)
- "A Modular and Scalable System Architecture for Heterogeneous UAV Swarms Using ROS 2 and
  PX4-Autopilot," arXiv:2510.27327. [arXiv](https://arxiv.org/html/2510.27327v1)
- MRS UAV system. [GitHub](https://github.com/ctu-mrs/mrs_uav_system)

### Study list

- Aerostack2 paper §architecture (map their behavior/plugin split onto your nodes).
- ROS 2 concepts: executors, QoS, lifecycle nodes (docs.ros.org) — enough to fake them.
- PX4 offboard control via ROS 2 (one tutorial), to size the migration honestly.
- PettingZoo ParallelEnv API.

---

## 8. Real-world operational data from Ukraine

*(All figures are open-source reporting/think-tank analysis; treat as approximate and
date-stamped. Primary aggregators: CSIS, Institute for Science and International
Security (ISIS) monthly Shahed analyses, CEPA, specialist press.)*

### Threat side — Shahed/Geran raid characteristics

- **Performance:** Shahed-136/Geran-2 cruise ≈ 180–200 km/h (50–55 m/s); upgraded
  Geran variants reported faster; dive speeds ~90 m/s reported. Warhead 50 kg standard,
  90 kg on newer variants. Jet-powered Geran-3 (significantly faster, reported ~500+
  km/h class) exists in smaller numbers. ([Wikipedia: HESA Shahed 136](https://en.wikipedia.org/wiki/HESA_Shahed_136), [Kyiv Independent on Russian modifications](https://kyivindependent.com/how-russia-modified-irans-shahed-136-drones-and-what-it-means-for-ukraine/))
- **Altitude evolution:** 2022–24 low-level ingress (<300 m, terrain masking); from
  Feb–Mar 2025 average cruise ~1,500 m overland, 2,000–2,500 m from maritime axes; by
  late 2025, 2–5 km cruise to defeat guns/mobile groups, then descend to ~1 km,
  stabilize, and dive on target (accuracy-driven profile). Simultaneously, *some* raids
  still use very low altitude to hide from radar — bimodal altitude distribution.
  ([United24 altitude analysis](https://united24media.com/war-in-ukraine/why-russias-drone-swarms-are-getting-deadlier-by-flying-higher-9305), [Odessa Journal / Kovalenko](https://odessa-journal.com/alexander-kovalenko-russia-has-changed-its-tactics-of-kamikaze-drone-strikes-on-ukraine))
- **Raid scale:** from <50/night early-war to ~140/day average (Feb 2025), ~211/day
  sustained over a 90-day window (~19,000 drones), with peak salvos 700+/night by late
  2025. ([CSIS "Drone Saturation"](https://www.csis.org/analysis/drone-saturation-russias-shahed-campaign), [ISIS monthly analysis](https://isis-online.org/isis-reports/monthly-analysis-of-russian-shahed-136-deployment-against-ukraine))
- **Tactics:** saturation salvos timed to exhaust interceptor stocks and clutter radar;
  routing along rivers/valleys; mid-route loitering and dog-legs; mixing decoys
  (Gerbera/Parodiya) at 40–60% of salvos in many months (~50% per DIU Nov 2024; ~75% of
  Alabuga new production reported as decoys at one point; April 2026: ~66% genuine
  strike Shaheds with the rest Gerbera/Italmas/Garpiya/decoys). Targets: energy grid,
  defense industry, cities. ([ISIS 2025 review](https://isis-online.org/isis-reports/a-comprehensive-analytical-review-of-russian-shahed-type-uavs-deployment-against-ukraine-in-2025), [CEPA decoys](https://cepa.org/article/the-phony-war-ukraine-and-russias-decoy-drones/))
- **Economics:** Shahed unit cost estimates ~$20–70k (falling with localization); Gerbera
  ~$10k; defender SAMs $100k–$4M+ per shot → the cost-exchange problem that motivates
  interceptor drones. ([CSIS](https://www.csis.org/analysis/drone-saturation-russias-shahed-campaign), [re:russia missile-financial balance](https://re-russia.net/en/analytics/0323/))

### Defense side — layered doctrine and interceptor programs

- **Layered architecture (current Ukrainian practice):** passive acoustic networks
  (Sky Fortress ~14k nodes; Zvook) + radar + visual observers → fused picture to tablets
  → **mobile fire groups** (gun trucks, MANPADS) → **interceptor drones** (Sting et al.)
  → gun systems (Gepard) and EW → SAMs reserved for cruise/ballistic missiles. In Jan
  2026 Zelenskyy announced an Air Force reorganization explicitly centered on mobile fire
  groups + interceptor drones (Deputy Commander Yelizarov). ([New Geopolitics "Small Air Defense" revolution](https://www.newgeopolitics.org/2026/03/11/ukraines-small-air-defense-revolution-and-why-america-should-be-paying-attention/), [Complexity and Layering](https://www.newgeopolitics.org/2025/11/15/complexity-and-layering-how-ukraines-air-defence-must-operate/), [CEPA air defenses](https://cepa.org/article/ukraines-air-defenses-world-class-and-improving/))
- **Interceptor drone results:** Ukrainian command cites >70% effectiveness for
  interceptor drones vs Shaheds fleet-wide; Wild Hornets Sting: 280+ km/h, >3,900
  Shahed/Geran kills by Feb 2026, top anti-Shahed interceptor 7 months running, crew hit
  rates 80–95% claimed; cost low-thousands of dollars per interceptor. Russia counter-
  adapted with higher altitudes, night raids, and Geran-3 speed. Other programs: Besomar,
  ODIN, General Chereshnya, etc. (names in specialist press; capabilities mostly
  unpublished). ([Wikipedia: Sting](https://en.wikipedia.org/wiki/Sting_(drone)), [Defense Express](https://en.defence-ua.com/news/ukrainian_wild_hornets_workshop_reveals_how_many_russian_drones_downed_by_sting_interceptors_in_five_months-16013.html), [United24](https://united24media.com/latest-news/ukrainian-interceptor-drones-tear-through-russian-shahed-swarms-with-95-kill-rate-18022))
- **Key caveat for modeling:** per-engagement success claims coexist with *rising
  overall leakage* during 2025 (CSIS) — i.e., the binding constraints are detection
  coverage, interceptor availability/positioning, and C2 latency, not terminal Pk. That
  is an argument FOR your project's focus on allocation + cooperation rather than
  effector modeling alone. ([CSIS "The New Salvo War"](https://www.csis.org/analysis/the-new-salvo-war), [CSIS October campaign](https://www.csis.org/analysis/russias-intense-air-campaign-october))

### Recommended approach for COOP-UAV-S (threat generator spec)

- Raid generator: N ∈ [10, 100] vehicles in waves; per-vehicle class sampled with decoy
  fraction 0.4–0.6; ingress altitude bimodal {100–300 m | 1.5–3 km}; speed 50–65 m/s
  (occasional 80–140 m/s "Geran-3" outliers); waypoint routes with dog-legs and
  river-following; terminal: descend to ~1 km then dive at ~90 m/s within 2–5 km of
  target; small random heading jitter; optional mid-route loiter.
- Defense laydown: acoustic net (cheap, dense, bearing-only, 5 km), 1–2 radars
  (terrain-masked low-altitude gaps), EO/IR around critical zones, 2–4 interceptor
  launch sites with 4–12 interceptors each, reload/turnaround times.
- Metrics: leakage %, expected ground casualties (intercept debris vs warhead impacts),
  interceptors expended per kill, cost-exchange ratio, decoy-engagement rate.

### Study list

- CSIS "Drone Saturation: Russia's Shahed Campaign" (best single doctrine read).
- ISIS monthly Shahed analyses (numbers for your generator).
- New Geopolitics "Small Air Defense Revolution" (defender architecture).
- Army.mil acoustic-detection report (sensor layer ground truth).

---

## What to study, ordered

A 6-week-shaped learning path for one engineer (parallelizable across teammates by topic):

1. **Week 1 — Estimation core.** Labbe's *Kalman and Bayesian Filters in Python*
   (KF → EKF → IMM chapters); implement IMM on a synthetic Shahed trajectory. Read Stone
   Soup tutorials 1–6.
2. **Week 1–2 — Geometry of pursuit.** Apollonius circle derivation; Wang 2022 +
   Area-Optimal (arXiv:2511.15036); implement safe-reachable-set computation with shapely;
   2D PN simulator from Zarchan Ch. 2.
3. **Week 2 — Assignment.** Hungarian via SciPy on a toy WTA; read CBBA paper; port/adapt
   a Python CBBA implementation; add shoot-look-shoot loop.
4. **Week 3 — Multi-target tracking.** GNN tracker with clutter + birth/death; OSPA
   metric; (optional) GM-PHD from Vo & Ma 2006 or via Stone Soup.
5. **Week 3–4 — Classification & decoys.** Bayesian recursive class posterior; read
   Rahman & Robertson 2018 + Gerbera OSINT; build class-conditional feature models;
   expected-value TEWA integration.
6. **Week 4 — Ground risk.** SORA executive summary; Primatesta 2020; ballistic descent
   ODE + Monte Carlo footprint; risk raster + expected-casualty integral; couple into WTA
   utility.
7. **Week 5 — Guidance & engagement.** PN/APN on multirotor dynamics (AIAA 2021 paper);
   Pk envelope models (net vs frag) anchored to Anvil/DroneHunter/Coyote/Sting numbers;
   intercept-point optimization over the risk raster.
8. **Week 5–6 — Integration & experiments.** Threat generator from Ukraine data; headline
   experiments (decoy-ratio sweep; collateral-aware vs naive WTA; cooperative containment
   vs independent pursuit); dashboard polish.
9. **Stretch.** PettingZoo wrapper + MAPPO benchmark vs geometric planner; Aerostack2 /
   PX4 SITL pilot migration of one interceptor node.

---

## Recommended libraries and tools

| Package | Role in COOP-UAV-S | Notes |
|---|---|---|
| `numpy`, `scipy` | Everything; `scipy.optimize.linear_sum_assignment` (Hungarian/JV) for WTA; `scipy.integrate.solve_ivp` for debris ballistics; `scipy.spatial` (KDTree) for gating/proximity; `scipy.stats` for Pk/casualty sampling | Core. |
| `filterpy` | KF/EKF/UKF/**IMM** estimators; lightweight, readable | rlabbe's library; pairs with the free *Kalman and Bayesian Filters in Python* book. |
| `stonesoup` | Full MTT framework: associators (GNN/JPDA), GM-PHD, sensor & platform simulators, OSPA/SIAP metrics | [dstl/Stone-Soup](https://github.com/dstl/Stone-Soup), MIT license. Use at least for metrics + as ablation reference even if you keep your own tracker. |
| `shapely` | Apollonius/safe-set polygon intersections; zone polygons; debris footprint vs zone overlap | Workhorse for both the pursuit geometry and the risk model. |
| `networkx` | Comms topology for CBBA consensus; engagement graphs; (also has assignment/matching algorithms) | Light use. |
| `geopandas` + `rasterio` (optional) | Real city zone maps / population rasters if you import OSM or GHSL population data | Only if you want a real Ukrainian-city-shaped map; otherwise numpy rasters suffice. |
| `pettingzoo` + `gymnasium` | MARL environment interface for the pursuit stretch goal | Wrap your sim as a `ParallelEnv`. |
| `stable-baselines3` (PPO) or a MAPPO implementation | RL training | MAPPO reference implementations exist from the Yu et al. paper's repo; SB3 covers single-policy PPO with parameter sharing — usually enough. |
| `gym-pybullet-drones` | Higher-fidelity quad RL envs for cross-checking learned policies | [utiasDSL](https://github.com/utiasDSL/gym-pybullet-drones). |
| `numba` (optional) | JIT the Monte Carlo debris sampler and detection loops if Python becomes the bottleneck | Profile first. |
| `websockets` / `fastapi` + Three.js | Live 3D dashboard transport (already in your stack) | Send compact binary or msgpack track/state frames at 10–20 Hz; decimate server-side. |
| `pydantic` or stdlib `dataclasses` | ROS-2-shaped message definitions with validation | Keep field names aligned to ROS 2 msg conventions for migration. |
| `pytest` + deterministic seeds | Scenario regression tests; Monte Carlo CI budgets | Determinism is the payoff of the custom sim — protect it. |
| (migration) ROS 2 + Aerostack2 + PX4 SITL + Gazebo | Phase-2 realism | See Section 7. |

---

## Verification notes

- All URLs above were returned by web searches performed on 2026-06-10; bibliographic
  details for classic papers (Reid 1979; Vo & Ma 2006; Isaacs 1965; Zarchan; Choi-Brunet-
  How 2009; Panerati 2021) were confirmed against search results.
- Items marked **[standard reference, URL not verified]** are canonical works cited from
  domain knowledge (Blom & Bar-Shalom IMM 1988; Bar-Shalom & Fortmann 1988; Reuter et al.
  LMB 2014; Mahler 2003; Kuhn 1955; Lloyd & Witsenhausen 1986; V. C. Chen micro-Doppler
  book; RCC 321; la Cour-Harbo ballistic descent series; Lowe 2017 / Yu 2022 arXiv IDs;
  Crazyswarm2 and Colosseum repo URLs). Verify these before quoting them in any formal
  publication; none are load-bearing for implementation decisions.
- Operational Ukraine figures (raid sizes, decoy ratios, interceptor success rates) are
  open-source claims by interested parties and analysts; ranges rather than point values
  were reported deliberately, and they should be treated as scenario parameters, not
  ground truth.
- One relevant MDPI paper (Electronics 12(4):829, ground risk estimation) is flagged
  **RETRACTED** on the publisher page — do not cite it.

---

## P1 physics core - equation sources (added 2026-06-11)

Per-equation traceability for `src/coopuavs/physics/` (plan rule: one
citation per implemented equation; module docstrings carry the same
references next to the code). Items marked **[standard reference, URL not
verified]** follow the convention of the Verification notes above; every
equation is additionally pinned by an analytic unit test, so no citation
below is load-bearing for correctness.

| Model / equation | Implementation | Source |
|---|---|---|
| Quaternion kinematics `q_dot = 1/2 q (x) (0, w)` | `rigid_body.quat_derivative` | J. Sola, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508 (2017), eq. (199). |
| Euler rotational dynamics `w_dot = J^-1 (tau - w x Jw)` | `rigid_body.derivatives` | Beard & McLain, *Small Unmanned Aircraft* (Princeton UP, 2012), eq. 3.15-3.17. [standard reference] |
| Classic RK4, per-step quaternion renorm | `rigid_body.rk4_step` | Press et al., *Numerical Recipes*, 3rd ed., sec. 17.1. [standard reference] |
| ISA troposphere T/p/rho/a | `atmosphere.py` | U.S. Standard Atmosphere 1976 (NOAA-S/T 76-1562); ICAO Doc 7488. [standard reference] |
| Dryden forming filters + low-altitude sigma/L table | `dryden.py` | MIL-F-8785C, *Flying Qualities of Piloted Airplanes* (1980), sec. 3.7.2; transfer-function forms as in Beard & McLain sec. 4.4. |
| Rotor thrust `kf w^2`, yaw reaction `km w^2`, allocation | `multirotor.wrench` | Mahony, Kumar & Corke, *Multirotor aerial vehicles*, IEEE Robotics & Automation Magazine 19(3), 2012. [standard reference] |
| Ground effect `T_IGE/T_OGE = 1/(1-(R/4z)^2)` | `multirotor._ground_effect` | Cheeseman & Bennett, *The effect of the ground on a helicopter rotor in forward flight*, ARC R&M 3021 (1955). [standard reference] |
| Linear rotor drag `f = -D v_air_body` | `multirotor.wrench` | Faessler, Franchi & Scaramuzza, *Differential flatness of quadrotor dynamics subject to rotor drag...*, IEEE RA-L 3(2), 2018. |
| Brushless motor/ESC: `i=(dV-Ke w)/Rw`, `J_r w_dot = Kt i - k_q w^2`, `Ke=Kt=60/(2 pi KV)` | `motor.py` | Standard DC-machine model; multirotor application per Mahony et al. 2012. [standard reference] |
| Thevenin 1-RC battery ECM + OCV(SOC) | `battery.py` | Chen & Rincon-Mora, *Accurate electrical battery model capable of predicting runtime and I-V performance*, IEEE Trans. Energy Conversion 21(2), 2006. |
| Implicit DC-bus fixed point `v_bus = (OCV - V1 + (R0/R_w) Ke sum(theta_r w_r)) / (1 + R0 sum(theta_r^2)/R_w)`, `i_bus = (sum(theta_r^2) v_bus - Ke sum(theta_r w_r)) / R_w` | `powertrain.py` | No external source: closed-form simultaneous (Kirchhoff) solution of the two component models above, at pre-step omega/SOC/V1. Stability rationale: the quasi-static armature plus the ECM R0 feedthrough give any explicit one-step-lag composition a loop gain `g = R0 sum(theta_r^2)/R_w` (= 3.6 theta^2 for interceptor_quad, 2.0 theta^2 for fpv_quad) — a fixed-point iteration that diverges for g > 1 (above ~hover throttle) at ANY dt, so the loop must be solved implicitly. Bus current is then clamped to the YAML `i_bus_max_a` (ESC/BMS limit, ~1.5x steady full-throttle draw) and bus voltage to [3.0, 4.2] V/cell. |
| Fixed-wing aero: blended lift 4.9-4.10, induced drag 4.11, lateral set 4.14, prop 4.15, stability->body 4.19 | `fixedwing.py` (FRD verbatim, `M=diag(1,-1,-1)` flip to FLU) | Beard & McLain 2012, ch. 4. [standard reference] |
| Slab-method segment vs AABB | `collision.py` | Ericson, *Real-Time Collision Detection* (2005), sec. 5.3.3 (Kay-Kajiya). [standard reference] |
| Oracle simulator | `scripts/oracle/export_rotorpy.py` | Folk, Paulos & Kumar, *RotorPy: a Python-based multirotor simulator...*, arXiv:2306.04485 (2023); rotorpy 2.1.2. |

Airframe parameter files (`physics/params/*.yaml`) are
invented-but-self-consistent (no public data for these classes) and are
pinned by trim/terminal/envelope tests - the YAML headers say so explicitly.

Known model-validity limitations (gate review 2, 2026-06-11; both kept
as-is by decision, with the same warnings carried in the code):

- **B&M eq. 4.15 windmill drag** (`fixedwing.py`): at throttle cut the
  verbatim prop model produces NEGATIVE thrust
  `~ -1/2 rho S_prop C_prop Va^2` — about -643 N for shahed_fw at 50 m/s
  cruise, ~2.5x the total aero drag (~260 N). Faithful to the book away
  from its design point; P6 threat behaviors must not model throttle-cut
  glides / engine-out trajectories without revisiting (clamp or a
  momentum-theory windmill model).
- **interceptor_quad constant kf/km at dash** (`params/interceptor_quad.yaml`):
  the committed KV/R_w/12S imply a full-throttle ceiling of ~1400 rad/s
  (13.4 krpm) on the 0.178 m prop — tip Mach ~0.73 on a stiff 44.4 V bus
  (~M 0.59-0.69 with pack sag), ~1.7x class-typical 14" prop rpm ratings;
  at the 80 m/s dash the helical advancing-tip Mach reaches ~0.8, where
  compressibility invalidates the constant-kf/km quadratic model. Hover
  (738 rad/s, tip M ~0.38) is fine. No pin is affected (the RotorPy oracle
  shares the constant-coefficient model class); revisit (kf(Mach) rolloff
  or larger/slower props) if dash-regime fidelity becomes load-bearing.

## P2 hardware device models - equation sources (added 2026-06-11)

Per-equation traceability for `src/coopuavs/hw/` (same rule as P1: one
citation per implemented equation, module docstrings carry the same
references; every equation is additionally pinned by an analytic unit
test, so no citation below is load-bearing for correctness).

| Model / equation | Implementation | Source |
|---|---|---|
| IMU error budget: white noise density N, bias-instability proxy (first-order Gauss-Markov), bias random walk K, turn-on bias | `hw/imu.py` | El-Sheimy, Hou & Niu, *Analysis and modeling of inertial sensors using Allan variance*, IEEE Trans. Instrumentation & Measurement 57(1), 2008; IEEE Std 952-1997 Annex B/C; parameter convention per the Kalibr IMU noise model (noise density / random walk per sqrt(Hz)/sqrt(s)). [standard references] |
| Discrete white noise sigma_d = N/sqrt(dt); exact-ZOH GM `x[k] = phi x[k-1] + sigma sqrt(1-phi^2) eps` with stationary cold start; RW `b[k] = b[k-1] + K sqrt(dt) eps` | `hw/stoch.py` | Brown & Hwang, *Introduction to Random Signals and Applied Kalman Filtering*, 4th ed., ch. 3 (Gauss-Markov ZOH discretization). [standard reference] |
| Analytic Allan variances: `AVAR_N = N^2/tau`, `AVAR_K = K^2 tau/3`, `AVAR_GM = sigma^2 (T/tau)[2 - (T/tau)(3 - 4e^(-tau/T) + e^(-2tau/T))]` | `hw/stoch.py avar_*`; estimator `tests/allan_util.py` (fully-overlapping ADEV) | IEEE Std 952-1997 Annex C. NOTE: the GM curve here is re-derived from the autocorrelation `R(u) = sigma^2 e^(-|u|/T)`; IEEE writes the same curve parameterized by the driving noise q with `sigma^2 = q^2 T / 2` (a factor-2 trap when transcribing; the Monte-Carlo Allan suite pins ours). |
| Specific force `f_b = q^-1 (a_world - g_world)` (accelerometer reads +g up at rest, 0 in free fall) | `hw/imu.py sample` | Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. (2013), ch. 2 strapdown conventions. [standard reference] |
| GNSS error decomposition: white tracking noise over slowly correlated (GM) iono/tropo/multipath residual, h/v split; Doppler-white velocity | `hw/gps.py` | Groves 2013, ch. 9. [standard reference] |
| Baro chain: `p = p_ISA(alt) + GM drift + white`, exact inverse `h = (T0/L)(1 - (p/p0)^(R L/g0))` | `hw/baro.py` | U.S. Standard Atmosphere 1976 (same source as `physics/atmosphere.py`); slowly-varying baro bias convention per PX4 EKF2 (which estimates exactly such an offset). [standard reference] |
| Theater geomagnetic field from magnitude/declination/inclination: `B_ENU = |B| [cos I sin D, cos I cos D, -sin I]` | `hw/mag.py theater_field_enu` | Standard geomagnetic element definitions (e.g. NOAA NCEI / WMM documentation: D east of true north, I dip below horizontal). [standard reference] |
| Mag error budget: per-power-up hard iron + GM bias + white | `hw/mag.py` | PX4 EKF2 magnetometer bias convention; hard/soft iron taxonomy standard (soft iron neglected - documented deviation). |
| Rate-limited first-order gimbal servo `delta = clip(err min(dt/tau, 1), +-slew dt)` | `hw/seeker_gimbal.py` | Standard rate-limited actuator form, cf. Beard & McLain 2012 ch. 6 actuator models; `min(dt/tau, 1)` deadbeat discretization is ours (pinned: never overshoots). [standard reference] |
| ESC telemetry frames: per-rotor rpm + pack bus V/A, protocol quantization | `hw/esc_telem.py` | BLHeli32/KISS ESC telemetry convention (rpm via erpm/pole-pairs, 0.01 V / 0.1 A granularity class). [project knowledge, representative] |

Device parameter file (`hw/params/interceptor_devices.yaml`) is
invented-but-representative: magnitudes sized to the named device classes
(tactical MEMS IMU, multi-band GNSS, MS5611/IST8310-class baro/mag,
BLHeli32-class telemetry), NOT copied from any datasheet, and pinned by
the hw tests.

Known model-validity limitations (P2, kept as-is by design):

- **Mag soft iron neglected** (`hw/mag.py`): only a hard-iron offset is
  modelled; attitude-dependent soft-iron distortion is absent. The P3 EKF
  mag-fusion gates must not be tuned to exploit that absence.
- **ESC telemetry has no temperature channel and pack-level V/I only**
  (`hw/esc_telem.py`): no thermal model exists, and `BatteryEcm` carries
  no per-cell states - per-cell imbalance telemetry arrives with the P5
  CELL_IMBALANCE fault work.
- **Gimbal stabilization assumed ideal** (`hw/seeker_gimbal.py`): no
  coupling of airframe angular rate into the boresight inside the slew
  budget.
- **Baro reads the ISA column, not weather** (`hw/baro.py`): the legacy
  weather model carries no pressure field; if a synoptic pressure offset
  is added later it must enter the baro truth path explicitly.

## P3 CoopFC flight stack - estimation sources and consistency rationale (added 2026-06-12)

Per-equation traceability for `src/coopuavs/coopfc/estimation/` (same
rule as P1/P2: one citation per implemented equation, module docstrings
carry the same references; every equation is additionally pinned by a
unit test, plus the NEES/NIS Monte-Carlo consistency suite
`tests/test_coopfc_ekf_mc.py` (@slow) against the real P2 device
models).

| Model / equation | Implementation | Source |
|---|---|---|
| Error-state 15-state EKF: nominal kinematics, error-state transition F, error injection + reset | `estimation/ekf.py` (`_integrate_nominal`, `_predict_cov`, `_inject`) | J. Sola, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508 (2017), eq. 255-259 (nominal), eq. 270 (transition), eq. 282 (injection). |
| Delayed fusion horizon + IMU ring buffer + output predictor (mainline runs `lag_s` behind now; every sensor, incl. 120 ms-late GNSS, fuses at exactly its stamp; control output = horizon state replayed through buffered IMU) | `ekf.py` (`update`, `_mainline`, `_output`) | PX4-EKF2 / ECL EKF architecture (delayed-time horizon with output complementary predictor), PX4 dev documentation. [standard reference, design pattern] |
| Chi-square innovation gate: reject if NIS > gate^2 * dof; accepted/rejected tallies as CBIT spoof seam | `ekf.py _fuse_block` | Bar-Shalom, Li & Kirubarajan, *Estimation with Applications to Tracking and Navigation* (Wiley 2001), sec. 5.4 innovation tests; gate-in-sigmas convention per PX4 EKF2. [standard reference] |
| Joseph-form covariance update (symmetric PSD under roundoff, exact for ANY gain) | `ekf.py _fuse_block` | Bucy & Joseph (1968), as presented in Brown & Hwang, *Introduction to Random Signals and Applied Kalman Filtering*, 4th ed., ch. 5. [standard reference] |
| Partial (masked-gain) measurement update: baro confined to the vertical channel {dp_z, dv_z, db_a_z} | `ekf.py _fuse_baro` -> `_fuse_block(gain_rows=...)` | Brink, *Partial-Update Schmidt-Kalman Filter*, J. Guidance, Control & Dynamics 40(9), 2017 (zero-gain rows = beta=0 states); consider-state framework per Schmidt 1966. Joseph form keeps P consistent for the deliberately suboptimal gain. |
| Heading-only magnetometer fusion: tilt-leveled field, yaw innovation, H = yaw axis only | `ekf.py _fuse_mag` | PX4 EKF2 mag heading fusion (the default mag mode); leveled-field yaw per Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. (2013), ch. 6. [standard reference] |
| Static coarse alignment: leveling from mean specific force, gyro bias from mean rate, yaw from leveled mag + declination, motion variance gate | `estimation/alignment.py` | Groves 2013, ch. 5 (coarse alignment / leveling). [standard reference] |
| NEES/NIS Monte-Carlo filter consistency methodology | `tests/test_coopfc_ekf_mc.py` | Bar-Shalom, Li & Kirubarajan 2001, ch. 5 (consistency of state estimators; NEES/NIS bounds). [standard reference] |
| Yaw information floor; unmodeled-error budget added to every reported sigma | `ekf.py` (`_fuse_mag` floor, `budget9`) | No external source: ours (static consider-covariance in spirit, Schmidt 1966); rationale below, calibrated against the MC suite. |
| Quaternion attitude P law `rate_sp = 2 kp vec(q^-1 q_sp)`, shortest path, yaw weight | `control/attitude.py` | Brescianini, Hehn & D'Andrea, *Nonlinear quadrocopter attitude control* (ETH Zurich tech report, 2013); PX4 attitude controller convention. [standard reference] |
| Body-rate PID, derivative-on-measurement + LPF; conditional-integration anti-windup | `control/rate.py` | Astrom & Hagglund, *Advanced PID Control* (ISA 2006), ch. 3 (integrator windup; conditional integration). [standard reference] |
| Velocity PI -> specific force -> attitude (thrust-direction flatness map, yaw-frame euler solve) | `control/velocity.py` | Mellinger & Kumar, *Minimum snap trajectory generation and control for quadrotors*, ICRA 2011 (thrust direction = desired body z); PX4 PositionControl structure. [standard reference] |
| Quadratic thrust-curve command linearization `u = u_hover sqrt(\|f\|/g)` | `control/velocity.py` | T ~ omega^2 with omega ~ linear in command (quasi-static armature, P1 motor model); PX4 THR_MDL_FAC convention. [project knowledge] |
| Quad-X mixer + sequential desaturation, priority roll/pitch > collective > yaw; per-axis directional saturation flags for rate-loop anti-windup | `control/mixer.py` | PX4 ControlAllocationSequentialDesaturation order [project knowledge, standard convention]; sign table derived from the `physics/multirotor.py` wrench equations (documented in the module docstring). |
| Position P -> velocity setpoint cascade | `control/position.py` | PX4 PositionControl outer-loop structure. [standard reference] |
| FCU failsafe conventions: battery LOW->RTL / CRITICAL->LAND upward-latching with debounce, link-loss->RTL, offboard setpoint-timeout->POS_HOLD, priority order | `fcu.py`, `battery_monitor.py` | PX4 commander/battery failsafe conventions [project knowledge]; priority order and timelines pinned by `tests/test_coopfc_fcu.py`. |
| Whole-stack flight envelope cross-check (waypoint square) | `tests/test_oracle_ardupilot.py` + `scripts/oracle/export_ardupilot_square.py` | ArduPilot ArduCopter stable SITL (official prebuilt, EKF3) as an independent complete autopilot; offline-oracle policy and envelope-band scoping per tests/fixtures/oracle/README.md. |

### Colored measurement errors and covariance honesty (the P3 EKF contract)

The P2 device suite is deliberately *colored*: GNSS carries GM wander
(sigma 1.2/2.4 m, tau 60 s), the baro a GM drift (~1.25 m, tau 600 s -
effectively one offset per flight), the mag a per-power-up hard iron
(~2 uT = ~5 deg of yaw at 63 deg dip), the IMU GM bias instability
(gyro 4e-5 rad/s = ~8 deg/h class, tau 100 s). A 15-state filter models
white noise + bias random walks only; it provably cannot estimate these
processes. Three mechanisms keep the filter *honest* about that, all
validated by the MC suite:

1. **R inflation** (variance of the colored process added to the
   measurement noise) keeps single-fusion weights right - but N
   repeated fusions of one frozen error still average it down sqrt(N)
   as if white. Hence:
2. **Structural fusion limits** where repetition is the hazard: the mag
   *yaw information floor* (stop fusing once P_yaw reaches the
   hard-iron variance - re-measuring one fixed draw at 50 Hz buys
   nothing) and the baro *partial update* (gain masked to the vertical
   channel; through maneuver-built cross-covariances 15000 baro
   fusions per flight otherwise quietly condition tilt and yaw:
   measured 20x claimed-sigma_vel suppression on the GNSS-denied
   suite, caught by the 4-sigma honesty gate).
3. **The unmodeled-error budget** `budget9` (9-dof variances): the
   residual floors the filter still cannot represent - GNSS GM wander
   on position, baro drift on height, hard-iron on yaw, and the
   hard-iron leak chain into tilt/velocity (coupling factors 0.15x,
   0.25 s, 0.3x calibrated ONCE against the MC suite). Every
   *reported* sigma is sqrt(diag P + budget9); the NEES suite scores
   against P + diag(budget9); on the GNSS-denial transition the
   attitude/velocity floors are injected into P once (they become real
   initial errors that the dynamics double-integrate).

### GNSS-denied 5-minute drift envelope (PHY-UAV-011, partial)

SRS PHY-UAV-011 makes 5-minute GNSS-denied navigation *engagement-grade*
only via a VIO/datalink fallback that is real-system scope (out of this
simulation; docs/TRACEABILITY.md marks the requirement partial). What
the sim therefore validates is (a) the honest free-inertial drift of
the modeled suite — baro holds height, mag holds yaw, nothing holds
horizontal — and (b) *covariance honesty over the whole denial*: the
filter's own 4-sigma claim must contain the true drift at every scored
seed, because that claim is what the P5 CBIT dead-reckoning budget (and
the real system's fallback trigger) acts on.

First-principles horizontal scale over t = 300 s of denial, for the P2
IMU (gyro RW K = 1e-5 rad/s/sqrt(s), GM bias instability
sigma = 4e-5 rad/s ~ 8 deg/h, tau 100 s): the gravity leak of the
random-walking tilt-rate bias integrates to
`sigma_pos = g K sqrt(t^7/252) ~ 2.9 km`, and the GM bias treated as a
coherent ramp adds `g sigma_gm t^3/6 ~ 1.8 km` — ~3.4 km RSS scale.
Measured (MC suite, seeds 0-4, 2026-06-12): worst 5472 m, spread
1.3-5.5 km, all inside the filter's 4-sigma claim; regression gate
7000 m (+28% over worst). The filter over-claims late-denial sigma by
2-8x because it models the bounded GM bias instability as an unbounded
random walk — the conservative side of honesty, accepted.

The baro partial update above is what makes this honest: before it,
15000 full-gain baro fusions during denial suppressed the claimed
sigma_vel 20x below the no-baro covariance (3.1 vs 67.5 m/s at +270 s)
while true drift stayed km-class — the 4-sigma gate failed at 4.2 sigma
and the A/B diagnostic (baro on/off during denial) isolated the
channel. The masked gain trades the small *real* tilt information in
baro z-residuals (order f_horizontal/g) for killing the large fake
component; true drift rises (pure DR), the claim becomes truthful.

### P3-8 hover-accuracy gate semantics (user decision 2026-06-12)

The plan's "hover RMS < 0.15 m calm" is physically unreachable against
TRUTH with this device suite: the GNSS GM wander (sigma_h 1.2 m,
tau 60 s) drags the EKF estimate, and a position-hold loop follows its
estimate — published GNSS (non-RTK) position-hold accuracy is the
1-1.5 m class, and centimeter-level hover requires RTK corrections
(see e.g. RTK-vs-GPS hold comparisons:
https://www.d1store.com.au/lounge/content/rtk-vs-gps-position-hold,
https://thinkrobotics.com/blogs/tutorials/rtk-gps-setup-for-drones-complete-guide-to-centimeter-level-accuracy
[vendor documentation, magnitudes only]). The gate therefore splits:

- **control error** |estimate - hold setpoint|: plan numbers apply
  (< 0.15 m calm, < 1.0 m at 8 m/s + Dryden w20 = 8). Measured
  0.07-0.08 m both — the cascade rejects light-class turbulence to
  the navigation floor.
- **truth error** |truth - truth at capture|: gated at the device
  budget, 2.0 m RMS (measured 0.5-0.9 m; GM wander over a 30 s window
  wanders ~sqrt(2 sigma^2 (1-e^(-t/tau))) ~ 0.9 m at 1 sigma).

The 200 m waypoint-square cross-track gate (< 2 m) stays TRUTH-based:
the GM error is common-mode along a straight segment leg (measured
~1 m class worst).

### P3-8 perf gate re-scope (user decision 2026-06-12)

"1-vehicle RTF >= 20x" predated P1: the batched plant RK4 costs
~0.2 s CPU/sim-s INDEPENDENT of N (numpy small-batch overhead — the
P1/P2 same-bound-for-N=20-and-30 measurements), capping any 1-vehicle
bench near 5x regardless of flight-software cost. Re-scoped to
**>= 3x measured** (3.6-3.7x) plus the requirement that actually
matters for the design envelope, **20-instance projection >= 1x**
per the P4 fleet architecture (one batched plant + device suite, N
python FCUs): C20 = C_phys+dev(N=20) + 20 C_fcu. Passing this needed
the EKF fusion path rewritten in selection-indexed form (every
measurement model's H rows are unit vectors; the dense matmuls only
accumulated exact +0.0 terms) — verified VALUE-IDENTICAL by sha256
over the full state + covariance of a 20 s device-suite run before and
after. Measured: C_fcu 0.023-0.027 s/sim-s direct, projection
0.73-0.81 s/sim-s -> RTF 1.24-1.38x.

P3-R2 follow-ups (2026-06-12 gate review, cut-findings pass): the
one-time sha256 equivalence check is now a COMMITTED default-suite pin
(`test_fuse_sel_matches_dense_joseph_reference` re-derives every sensor
block, incl. the masked baro partial update, against a test-side dense
Joseph reference), and the Joseph update itself is expanded to rank-m
form for selection H ((I-KH)P = P - K P[idx,:]; X(I-KH)^T =
X - X[:,idx] K^T) — an algebraic identity (exact for any gain), ~5x
fewer multiplies than the two dense 15x15 matmuls it replaces. The
output predictor stays a FULL replay (exact prediction) rather than a
PX4-style incremental delta (approximate): fidelity-first, cost bounded
by the lag_s window and covered by the @perf gates.

### P3-5 yaw rate gate (user decision 2026-06-12, gate review)

The plan's "rate rise < 60 ms" is a roll/pitch spec: quad-X yaw is
actuated by rotor drag torque, ~30x weaker authority, and physically
cannot meet it. The interim 0.40 s settle gate carried 2.9x headroom
over the measured 0.138 s (deterministic truth-fed bench) — loose
enough to pass a tripled settle time. Re-stamped per the
fidelity/determinism goal as a REGRESSION gate: **settle < 0.20 s**
(+45% headroom, same style as the GNSS-denied drift gate), overshoot
gate < 20% unchanged.

## P4 fleet integration - engine wiring decisions (added 2026-06-12)

### P4-1 SITL wind coupling (user decision 2026-06-12)

Legacy weather applies an Ornstein-Uhlenbeck gust on the mean wind as a
truth-side *displacement*. A SITL vehicle must feel wind as a *force*
through the plant wrench, so the engine feeds per-vehicle
`WeatherState.mean_wind_at(z)` (the same power-law shear as `wind_at`,
gust-free) plus a MIL-F-8785C Dryden bank (the validated P1 machinery,
per-vehicle child streams) whenever scenario wind is nonzero. The OU
gust term is deliberately excluded for SITL vehicles — Dryden replaces
it; applying both would double-count turbulence. Mapping note: the
Dryden severity knob w20 is the mean wind at 20 ft (6.1 m); the
scenario `wind_speed` is referenced at 10 m. The 10 m figure is used
as w20 directly (≤6% shear-law difference, well inside the turbulence
model's class accuracy). Turbulence sigma/length scales are frozen at
each vehicle's spawn altitude (the bench convention).

### P4-1 IMU acceleration = exact wrench (user decision 2026-06-12)

The P3 bench fed the IMU a dv/dt finite difference (documented
placeholder). The fleet engine threads the exact truth CoM acceleration
`force_world / m` — the plant wrench at the latched rotor speeds and
ZOH wind, gravity included, matching the `hw.Imu.sample` contract —
evaluated at the pre-step state the devices sample. Stand rows read
zero (the unmodeled ground reaction balances them). Real ground
contact stays deferred (same decision): non-ARMED rows are frozen with
zeroed velocity/rates, motors pre-spin to hover at arming.

### P4-4 energy telemetry: voltage-proxy fraction (user decision 2026-06-12)

`UavState.battery` in sitl mode is the FCU's voltage-proxy fraction
(`BatteryMonitor.fraction()`): loaded per-cell voltage mapped linearly
`crit_v_cell (3.30) → 0 .. full_v_cell (4.20) → 1`, shipped in STATUS as
`batt_frac` f32. Deliberately conservative — sag under load reads as less
remaining energy exactly when the MC should break off earlier. The proxy
is noisy under transient load (arming spool-up reads ~0.1 for one
sample), so the MC floor carries a 2 s debounce mirroring the FCU
monitor's. Real SOC estimation (coulomb counting, per-cell) is P5
CELL_IMBALANCE scope. The rearm cycle is physical (land-dock decision):
the pad charger drives ECM SOC directly (boundary condition; the charger
circuit is out of scope) and BATT_RESET carries pack-swap semantics —
ground-only, clears the upward-latched monitor.

### P4-4 touchdown ground recalibration

The stand convention stops the airframe in one micro-tick at touchdown —
a velocity step the IMU stream never expresses (contact dynamics are not
modeled). The EKF's chi-square gates then *defend* the stale velocity
belief against every GPS/baro correction (gate lockout: an 8 m/s
free-running pad drift was measured, `div=False`, `late_meas=0`). The
realistic remedy at our fidelity is the recalibrate-before-flight
doctrine: touchdown drops the EKF and re-runs the static ground
alignment from scratch (the existing BOOT machinery, ~2 s, GPS-seeded);
PBIT holds re-arming until it is green. PX4's equivalent is its
on-ground EKF handling (zero-velocity updates / state resets on land).

### P4-4 OFFBOARD setpoint clamp (PX4 convention)

`cmd_velocity` setpoints are now clamped in the FCU to the same
`fcu.vel_max_h/up/down` envelope params the internal modes obey — an MC
cannot command the airframe past its declared envelope. Scenario
overlays size the envelope per airframe (the racer flies
h=80/up=20/down=20).

### P4 gate-review: vertical-brake loss — root cause + fix (user
### decision 2026-06-12, fidelity-first)

Symptom: braking a fast climb held near-hover average thrust for
seconds (~90 m overshoot). Root cause (instrumented in the fleet
engine): the brake demand is healthy (az clamps to −a_max_down, thrust
≈ 0.2), but the LOW specific force (fz = g − a_down ≈ 1.8 m/s²)
shrinks the tilt cone's lever — any cone-saturating horizontal error
then commands ±tilt_max, and a sign-flipping error steps the attitude
setpoint ±45° at the 50 Hz loop rate. The rate loop slams torque
chasing steps no airframe can follow, and the mixer's rp-priority
desaturation drags average collective back to ~hover: vertical
priority, honored in the demand chain, was lost in the actuator chain.
EKF estimates and gyro-bias were verified healthy throughout.

Fix, two layers, both physical:
1. `VelParams.tilt_slew` (6 rad/s ≈ 344°/s) — the attitude setpoint is
   slew-limited to what the airframe can follow (standard autopilot
   practice; PX4 rate-limits via its attitude-loop time constants). It
   only engages on pathological steps: every P3 maneuver spec passes
   unchanged (30° step in ~90 ms needs ~5.8 rad/s peak).
2. `mc/guidance.approach_velocity` — braking-aware waypoint capture
   (v ≤ √(2·a_brake·d), a_brake 5 m/s² vs the 8 demanded) for posts,
   pads and loiter points in the MC apps. `goto_velocity`'s linear
   taper assumed point-mass 20 m/s² braking; legacy agents keep it.

Verified: deterministic reproducer pinned in `test_coopfc_control.py`
(climbing +15 m/s, −20 commanded, cone-saturating ±3 m/s horizontal
chatter → must reach vz < −5 in 3 s; pre-fix it stays +3.4), fleet
climb-out bounded <30 m in the energy cycle (pre-fix >90), all P3
control/bench acceptance and @slow flights green. The e2e suite was
RE-BASELINED (engagement timing shifted → different adjudicator draw
realizations): 9/10 seed kills (was 10/10; the lost seed is a 5-shot
pk≈0.5 miss streak with healthy vehicles), CI pins killing seeds 1-3,
@slow floor 8/10. Residual honest behavior: sustained full-power
climbs sag the 12S pack toward the (voltage-only, P5 CELL_IMBALANCE
scope) monitor's LOW/CRIT band — the FCU protects, lands, tops up and
retries; the rearm cycle test tolerates one such retry.

### P4-1 fleet-size invariance is a draw-history contract

ORDERING §4 promises that adding a vehicle leaves existing vehicles'
*draw histories* identical — pinned bit-exact at bank level
(test_hw_determinism) and through the engine's Dryden wiring
(test_sil_fleet). Full-trajectory bitwise invariance across batch
sizes does NOT hold: numpy einsum/matmul kernels differ at the last
ULP between n=1 and n=2 shapes (measured 1.6e-14 relative over a
1.5 s hover). The trajectory pin is therefore 1e-9 — any
stream-wiring fault diverges at device-noise scale, five-plus orders
louder. Run-twice determinism at fixed fleet size remains bitwise.
