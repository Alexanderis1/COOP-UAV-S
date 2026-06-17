"""Figure generation for the COOP-UAV-S scientific technical report.

Produces vector PDF figures consumed by ``scripts/build_report.py``. All
figures are drawn deterministically (no simulation run required); the
quantitative results reproduce the documented reference-raid Monte-Carlo.

Run standalone to (re)generate the figures into ``docs/reports/assets/``::

    python scripts/report_figures.py
"""
from __future__ import annotations

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch

# ── House style ────────────────────────────────────────────────────────────
plt.rcParams.update(
    {
        "font.family": "serif",
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "figure.dpi": 300,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.04,
    }
)

INK = "#1b2430"
ACCENT = "#1f5f8b"
SAFE = "#2e8b57"
DANGER = "#d9a441"
CRIT = "#c0392b"
MUTED = "#7a8696"


def _outdir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    d = os.path.join(here, "..", "docs", "reports", "assets")
    os.makedirs(d, exist_ok=True)
    return os.path.normpath(d)


# ── Figure 1 — system architecture / pub-sub dataflow ──────────────────────
def fig_architecture(path: str) -> None:
    fig, ax = plt.subplots(figsize=(7.1, 5.4))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")

    def box(x, y, w, h, text, fc="#eef3f8", ec=ACCENT, fs=8.2, bold=False):
        b = FancyBboxPatch(
            (x, y), w, h,
            boxstyle="round,pad=0.6,rounding_size=2.2",
            linewidth=1.2, edgecolor=ec, facecolor=fc, zorder=2,
        )
        ax.add_patch(b)
        ax.text(
            x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fs, color=INK, zorder=3,
            fontweight="bold" if bold else "normal",
        )
        return (x + w / 2, y, x + w / 2, y + h)  # cx, ybot, cx, ytop

    def arrow(p_from, p_to, label="", color=INK, rad=0.0, off=0.0, ls="-"):
        a = FancyArrowPatch(
            p_from, p_to, arrowstyle="-|>", mutation_scale=11,
            linewidth=1.1, color=color, zorder=1,
            connectionstyle=f"arc3,rad={rad}", linestyle=ls,
        )
        ax.add_patch(a)
        if label:
            mx = (p_from[0] + p_to[0]) / 2 + off
            my = (p_from[1] + p_to[1]) / 2
            ax.text(
                mx, my, label, fontsize=6.6, color=color, style="italic",
                ha="center", va="center",
                bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.85),
                zorder=4,
            )

    # Sim-side band (owns ground truth)
    ax.add_patch(
        FancyBboxPatch(
            (2, 70), 96, 27, boxstyle="round,pad=0.4,rounding_size=2",
            linewidth=1.0, edgecolor=MUTED, facecolor="#f7f4ee",
            linestyle=(0, (4, 3)), zorder=0,
        )
    )
    ax.text(4, 94.4, "SIM SIDE — ground truth quarantined (Gazebo-plugin style)",
            fontsize=7.2, color=MUTED, style="italic")

    world = box(5, 73, 26, 16, "World / Environment\n(clock, RNG, threats,\nrisk map, assets)",
                fc="#efe7d6", ec=MUTED, bold=True)
    sensors = box(37, 73, 30, 16,
                  "Sensors\nradar · RF-DF · EO/IR ·\nacoustic · airborne EW ·\nonboard seekers",
                  fc="#efe7d6", ec=MUTED)
    adj = box(73, 73, 22, 16, "Engagement\nAdjudicator\n(truth Pk · debris)",
              fc="#efe7d6", ec=MUTED, bold=True)

    fusion = box(8, 50, 26, 13, "Fusion\nKF + GNN/scan,\nclass belief, p_decoy",
                 fc="#e8f0ef")
    c2 = box(40, 47, 38, 17,
             "Base Station — C2 (TEWA)\nthreat eval → assignment\n(shooters + Apollonius blockers)\n→ ROE clearance",
             fc="#dbe7f1", ec=ACCENT, bold=True)

    uav = box(14, 24, 44, 14,
              "Interceptor UAVs × N\nmode-FSM · PN guidance ·\ncooperative cutoff/herding · effectors",
              fc="#e8f0ef")
    turret = box(64, 24, 30, 14, "Anti-air turrets\n(shared clearance\ninterlock)", fc="#e8f0ef")

    rec = box(20, 4, 60, 11,
              "Recorder → JSON replay / live WebSocket → Three.js 3D C2 dashboard",
              fc="#f0f0f3", ec=MUTED)

    # dataflow
    arrow((sensors[0], 73), (fusion[0] + 4, 63), "detections", ACCENT, rad=-0.15)
    arrow((fusion[2], 56.5), (c2[0] - 6, 56.5), "tracks", ACCENT)
    arrow((c2[0] - 4, 47), (uav[0] + 6, 38), "engagement/tasks", ACCENT, rad=0.12)
    arrow((uav[0] + 14, 38), (c2[0] - 2, 47), "fire_request", CRIT, rad=0.12, off=-7)
    arrow((c2[0] + 4, 47), (uav[0] + 22, 38), "clearance", SAFE, rad=-0.16, off=8)
    arrow((uav[2] + 8, 31), (adj[0] - 2, 73), "engagement/fire", CRIT, rad=-0.25)
    arrow((adj[0] + 2, 73), (c2[2] + 4, 64), "result", ACCENT, rad=-0.2, off=6)
    arrow((world[2] - 4, 73), (fusion[0], 63), "", MUTED, rad=0.2, ls=(0, (2, 2)))
    arrow((c2[0], 47), (rec[0], 15), "", MUTED, rad=0.0, ls=(0, (2, 2)))
    arrow((uav[0], 24), (rec[0] - 8, 15), "uav/state", MUTED, rad=0.1, ls=(0, (2, 2)))
    arrow((turret[0], 24), (adj[2], 73), "", CRIT, rad=-0.35, ls=(0, (3, 2)))

    fig.savefig(path)
    plt.close(fig)


# ── Figure 2 — Apollonius cooperative-interception geometry ────────────────
def fig_apollonius(path: str) -> None:
    fig, ax = plt.subplots(figsize=(6.6, 4.6))

    # Hostile heading toward a protected asset along its mission corridor.
    T = np.array([2.0, 8.2])
    asset = np.array([8.5, 0.8])
    v_t = 60.0  # m/s, hostile
    v_p = 42.0  # m/s, interceptor (slower)
    k = v_p / v_t  # = 0.70

    # Interceptor positioned ahead of the corridor (the blocker candidate).
    P = np.array([4.7, 5.1])

    # Apollonius circle for the (slower) pursuer vs the evader:
    #   |X-P|/v_p = |X-T|/v_t  =>  |X-P| = k|X-T|.
    # With k<1 the equality locus is a circle; INSIDE it the pursuer reaches
    # X first (its reachable cut-off / first-arrival region).
    #   centre C = (P - k^2 T)/(1-k^2),  radius R = k|P-T|/(1-k^2).
    C = (P - k**2 * T) / (1 - k**2)
    R = k * np.linalg.norm(P - T) / (1 - k**2)

    # shaded reachable region (interceptor-first)
    circ = Circle(C, R, fill=True, ec=ACCENT, fc="#dceaf4", lw=1.6, alpha=0.85,
                  zorder=1)
    ax.add_patch(circ)
    ax.add_patch(Circle(C, R, fill=False, ec=ACCENT, lw=1.6, zorder=3))

    # corridor (hostile -> asset)
    ax.annotate(
        "", xy=asset, xytext=T,
        arrowprops=dict(arrowstyle="-|>", color=CRIT, lw=1.7, ls=(0, (6, 3))),
        zorder=4,
    )

    # blocker post = first corridor point inside the reachable region
    # (intersection of the corridor ray with the Apollonius circle).
    d = asset - T
    d = d / np.linalg.norm(d)
    f = T - C
    b = 2 * np.dot(d, f)
    c = np.dot(f, f) - R**2
    disc = max(b * b - 4 * c, 0.0)
    s = (-b - np.sqrt(disc)) / 2  # near (entry) intersection of corridor & circle
    B = T + s * d

    ax.annotate("", xy=B, xytext=P,
                arrowprops=dict(arrowstyle="-|>", color=ACCENT, lw=1.6), zorder=4)

    pts = [
        (T, CRIT, "Hostile (fast, $v_T$)", (0.25, 0.32), "left"),
        (P, ACCENT, "Interceptor\n(slow, $v_P$)", (-0.30, -0.55), "right"),
        (asset, INK, "Protected\nasset", (0.30, 0.0), "left"),
        (B, SAFE, "Blocker post\n(reached first)", (0.35, 0.45), "left"),
    ]
    for pt, col, lab, (dx, dy), ha in pts:
        ax.plot(*pt, "o", color=col, ms=9, zorder=6)
        ax.text(pt[0] + dx, pt[1] + dy, lab, color=col, fontsize=8.2,
                ha=ha, va="center", zorder=7,
                bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="none",
                          alpha=0.8))

    # circle annotation on-frame (point at the upper arc of the disk)
    ax.annotate(
        "Apollonius circle —\ninterceptor-first region (shaded)",
        xy=(C[0], C[1] + R), xytext=(5.6, 9.7),
        color=ACCENT, fontsize=7.8, ha="left", va="top",
        arrowprops=dict(arrowstyle="-", color=ACCENT, lw=0.8),
        zorder=8,
    )

    ax.text(
        0.15, 0.5,
        r"$v_P/v_T = 0.70 < 1$: the tail chase has no solution — but the"
        "\nhostile is corridor-bound, so a blocker posted where the corridor"
        "\nfirst enters the reachable region meets it head-on."
        "\nGeometry, not airspeed, wins.",
        fontsize=7.4, color=INK,
        bbox=dict(boxstyle="round,pad=0.4", fc="#f3f6fa", ec=MUTED),
        zorder=8,
    )

    ax.set_xlim(-0.3, 12.0)
    ax.set_ylim(-0.4, 10.4)
    ax.set_aspect("equal")
    ax.set_xlabel("east (km, schematic)")
    ax.set_ylabel("north (km, schematic)")
    ax.grid(True, ls=":", lw=0.5, color="#d6dbe2")
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    fig.savefig(path)
    plt.close(fig)


# ── Figure 3 — reference-raid results (documented 10-seed Monte-Carlo) ──────
def fig_results(path: str) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(7.4, 3.0))

    # (a) Raid outcome (reference single-run summary, README)
    ax = axes[0]
    cats = ["Killed", "Leakers", "Armed\nleakers"]
    vals = [6, 3, 1]
    cols = [SAFE, DANGER, CRIT]
    ax.bar(cats, vals, color=cols, edgecolor=INK, linewidth=0.6)
    for i, v in enumerate(vals):
        ax.text(i, v + 0.08, str(v), ha="center", va="bottom", fontsize=8)
    ax.set_title("(a) Raid outcome (9 hostiles)")
    ax.set_ylabel("count")
    ax.set_ylim(0, 7)
    ax.spines[["top", "right"]].set_visible(False)

    # (b) Where the wreckage fell (the ROE invariant)
    ax = axes[1]
    zones = ["SAFE", "DANGEROUS", "CRITICAL"]
    wrecks = [1, 2, 0]
    bars = ax.bar(zones, wrecks, color=[SAFE, DANGER, CRIT],
                  edgecolor=INK, linewidth=0.6)
    for b, v in zip(bars, wrecks):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.06, str(v),
                ha="center", va="bottom", fontsize=8)
    ax.annotate("invariant:\nzero on critical", xy=(2, 0.05), xytext=(1.55, 1.4),
                fontsize=6.8, color=CRIT, ha="center",
                arrowprops=dict(arrowstyle="-|>", color=CRIT, lw=1.0))
    ax.set_title("(b) Wrecks by ground zone")
    ax.set_ylabel("count")
    ax.set_ylim(0, 2.6)
    ax.tick_params(axis="x", labelsize=7)
    ax.spines[["top", "right"]].set_visible(False)

    # (c) Monte-Carlo invariants (10 seeds)
    ax = axes[2]
    labels = ["Critical-zone\nwrecks", "Shots at\nID'd decoys"]
    vals = [0, 0]
    ax.barh(labels, [0.04, 0.04], color="#dfe5ec", edgecolor=INK, linewidth=0.6)
    for i in range(2):
        ax.text(0.06, i, "0", va="center", ha="left", fontsize=11,
                color=SAFE, fontweight="bold")
    ax.set_xlim(0, 1)
    ax.set_xticks([])
    ax.set_title("(c) 10-seed invariants")
    ax.spines[["top", "right", "bottom"]].set_visible(False)
    ax.text(0.5, -0.85, "~50% armed-threat attrition\nunder deliberate saturation",
            transform=ax.transData, fontsize=6.8, ha="center", color=MUTED,
            style="italic")

    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# ── Figure 4 — diving-jet acquisition latency (CAP sentinels) ──────────────
def fig_sentinel_latency(path: str) -> None:
    fig, ax = plt.subplots(figsize=(3.5, 2.7))
    labels = ["Ground set\nonly", "+ forward CAP\nsentinels"]
    vals = [2.15, 0.06]
    bars = ax.bar(labels, vals, color=[MUTED, ACCENT], edgecolor=INK, linewidth=0.6)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.03, f"{v:.2f} s",
                ha="center", va="bottom", fontsize=8)
    ax.set_ylabel("mean acquisition latency (s)")
    ax.set_title("Diving-jet OWA acquisition (~36× earlier)")
    ax.set_ylim(0, 2.5)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main() -> None:
    d = _outdir()
    fig_architecture(os.path.join(d, "fig_architecture.png"))
    fig_apollonius(os.path.join(d, "fig_apollonius.png"))
    fig_results(os.path.join(d, "fig_results.png"))
    fig_sentinel_latency(os.path.join(d, "fig_sentinel.png"))
    print(f"figures written to {d}")


if __name__ == "__main__":
    main()
