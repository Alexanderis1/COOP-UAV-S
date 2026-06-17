"""Build the COOP-UAV-S formal scientific technical report (PDF).

Pure-Python report generator (ReportLab + Matplotlib figures). It depends on
nothing in the simulation package and reproduces the *documented* reference
results, so it runs anywhere the two libraries are installed::

    pip install reportlab matplotlib
    python scripts/report_figures.py     # (re)generate figures
    python scripts/build_report.py       # -> docs/reports/COOP-UAV-S_Technical_Report.pdf

The numbers reported here trace to README.md and docs/ARCHITECTURE.md §4
(10-seed Monte-Carlo of scenarios/residential_raid.yaml) and docs/MARL.md
(diving-jet acquisition latency).
"""
from __future__ import annotations

import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm, mm
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    HRFlowable,
    Image,
    KeepTogether,
    ListFlowable,
    ListItem,
    NextPageTemplate,
    PageBreak,
    PageTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
)

# ── palette ────────────────────────────────────────────────────────────────
INK = colors.HexColor("#1b2430")
ACCENT = colors.HexColor("#1f5f8b")
ACCENT_D = colors.HexColor("#15425f")
MUTED = colors.HexColor("#5b6675")
RULE = colors.HexColor("#c7d0da")
SAFE = colors.HexColor("#2e8b57")
DANGER = colors.HexColor("#bf8b30")
CRIT = colors.HexColor("#c0392b")
ZEBRA = colors.HexColor("#f2f5f8")
BOXBG = colors.HexColor("#f3f6fa")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
ASSETS = os.path.join(ROOT, "docs", "reports", "assets")
OUT = os.path.join(ROOT, "docs", "reports", "COOP-UAV-S_Technical_Report.pdf")

TITLE = "COOP-UAV-S: A Cooperative, Collateral-Damage-Aware Counter-UAS System"
SUBTITLE = "A Simulation Framework for Cooperative Interception of Hostile Drone Swarms over Populated Areas"

# ── styles ─────────────────────────────────────────────────────────────────
ss = getSampleStyleSheet()


def _style(name, **kw):
    kw.setdefault("parent", ss["Normal"])
    return ParagraphStyle(name, **kw)


BODY = _style("Body", fontName="Times-Roman", fontSize=10, leading=14.5,
              alignment=TA_JUSTIFY, textColor=INK, spaceAfter=6)
BODY_T = _style("BodyTight", parent=BODY, spaceAfter=2)
H1 = _style("H1", fontName="Helvetica-Bold", fontSize=15, leading=18,
            textColor=ACCENT_D, spaceBefore=16, spaceAfter=7, keepWithNext=1)
H2 = _style("H2", fontName="Helvetica-Bold", fontSize=11.5, leading=14,
            textColor=ACCENT, spaceBefore=10, spaceAfter=4, keepWithNext=1)
H3 = _style("H3", fontName="Helvetica-BoldOblique", fontSize=10, leading=13,
            textColor=INK, spaceBefore=7, spaceAfter=2, keepWithNext=1)
CAP = _style("Caption", fontName="Helvetica-Oblique", fontSize=8.4, leading=11,
             textColor=MUTED, alignment=TA_CENTER, spaceBefore=4, spaceAfter=10)
ABST = _style("Abstract", parent=BODY, fontSize=9.6, leading=13.5,
              leftIndent=10, rightIndent=10, textColor=INK)
SMALL = _style("Small", fontName="Times-Roman", fontSize=8.6, leading=11.5,
               alignment=TA_JUSTIFY, textColor=INK)
REF = _style("Ref", fontName="Times-Roman", fontSize=8.4, leading=11.2,
             alignment=TA_LEFT, textColor=INK, spaceAfter=4, leftIndent=14,
             firstLineIndent=-14)
TOC_E = _style("Toc", fontName="Times-Roman", fontSize=10, leading=16,
               textColor=INK)
KEYW = _style("Keyw", parent=ABST, fontName="Times-Italic")
COVER_T = _style("CoverT", fontName="Helvetica-Bold", fontSize=23, leading=28,
                 textColor=ACCENT_D, alignment=TA_LEFT)
COVER_S = _style("CoverS", fontName="Helvetica", fontSize=12.5, leading=17,
                 textColor=INK, alignment=TA_LEFT)
COVER_M = _style("CoverM", fontName="Helvetica", fontSize=10, leading=15,
                 textColor=MUTED, alignment=TA_LEFT)


def P(t, st=BODY):
    return Paragraph(t, st)


def bullets(items, st=BODY_T, bullet="•"):
    return ListFlowable(
        [ListItem(P(t, st), leftIndent=14, value=bullet) for t in items],
        bulletType="bullet", start=bullet, leftIndent=10, spaceAfter=6,
    )


def figure(fname, width_cm, caption):
    path = os.path.join(ASSETS, fname)
    img = Image(path)
    iw, ih = img.imageWidth, img.imageHeight
    w = width_cm * cm
    img.drawWidth = w
    img.drawHeight = w * ih / iw
    img.hAlign = "CENTER"
    return KeepTogether([Spacer(1, 2), img, P(caption, CAP)])


def boxed(flowables, bg=BOXBG, border=RULE):
    inner = Table([[f] for f in flowables], colWidths=[16.0 * cm])
    inner.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), bg),
        ("BOX", (0, 0), (-1, -1), 0.7, border),
        ("LEFTPADDING", (0, 0), (-1, -1), 9),
        ("RIGHTPADDING", (0, 0), (-1, -1), 9),
        ("TOPPADDING", (0, 0), (-1, -1), 7),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
    ]))
    return KeepTogether([Spacer(1, 3), inner, Spacer(1, 6)])


def _hdr(cell, color=colors.white):
    return Paragraph(f"<b>{cell}</b>", _style("th", fontName="Helvetica-Bold",
                     fontSize=8.6, leading=10.5, textColor=color))


def _td(cell, bold=False, color=INK, align=TA_LEFT):
    fn = "Times-Bold" if bold else "Times-Roman"
    return Paragraph(str(cell), _style("td", fontName=fn, fontSize=8.6,
                     leading=10.8, textColor=color, alignment=align))


def table(header, rows, col_widths, header_bg=ACCENT, zebra=True,
          align_center_cols=()):
    data = [[_hdr(h) for h in header]]
    for r in rows:
        data.append([_td(c) for c in r])
    t = Table(data, colWidths=[w * cm for w in col_widths], repeatRows=1)
    sty = [
        ("BACKGROUND", (0, 0), (-1, 0), header_bg),
        ("LINEBELOW", (0, 0), (-1, 0), 0.8, header_bg),
        ("LINEBELOW", (0, -1), (-1, -1), 0.6, RULE),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 3.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
    ]
    if zebra:
        for i in range(1, len(data)):
            if i % 2 == 0:
                sty.append(("BACKGROUND", (0, i), (-1, i), ZEBRA))
    t.setStyle(TableStyle(sty))
    return KeepTogether([Spacer(1, 2), t, Spacer(1, 4)])


# ── page furniture ─────────────────────────────────────────────────────────
def _cover_bg(canvas, doc):
    canvas.saveState()
    w, h = A4
    canvas.setFillColor(ACCENT_D)
    canvas.rect(0, h - 4.3 * cm, w, 4.3 * cm, fill=1, stroke=0)
    canvas.setFillColor(ACCENT)
    canvas.rect(0, h - 4.45 * cm, w, 0.16 * cm, fill=1, stroke=0)
    canvas.setFillColor(colors.HexColor("#e9eef3"))
    canvas.setFont("Helvetica-Bold", 9)
    canvas.drawString(2 * cm, h - 1.55 * cm, "TECHNICAL REPORT")
    canvas.drawRightString(w - 2 * cm, h - 1.55 * cm, "C-UAS SIMULATION FRAMEWORK")
    canvas.setFillColor(colors.HexColor("#9fb6c6"))
    canvas.setFont("Helvetica", 8)
    canvas.drawString(2 * cm, h - 2.15 * cm, "Open-source · GPL-3.0 · deterministic, seeded simulation")
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.5)
    canvas.line(2 * cm, 1.7 * cm, w - 2 * cm, 1.7 * cm)
    canvas.setFillColor(MUTED)
    canvas.setFont("Helvetica", 7.5)
    canvas.drawString(2 * cm, 1.25 * cm, "Generated from the COOP-UAV-S repository documentation set.")
    canvas.restoreState()


def _body_frame(canvas, doc):
    canvas.saveState()
    w, h = A4
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.5)
    canvas.line(2 * cm, h - 1.55 * cm, w - 2 * cm, h - 1.55 * cm)
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(MUTED)
    canvas.drawString(2 * cm, h - 1.35 * cm, "COOP-UAV-S — Technical Report")
    canvas.drawRightString(w - 2 * cm, h - 1.35 * cm, "Cooperative collateral-aware C-UAS")
    canvas.line(2 * cm, 1.5 * cm, w - 2 * cm, 1.5 * cm)
    canvas.drawString(2 * cm, 1.1 * cm, "Reproducible · seeded Monte-Carlo")
    canvas.drawRightString(w - 2 * cm, 1.1 * cm, "Page %d" % doc.page)
    canvas.restoreState()


def build_doc():
    doc = BaseDocTemplate(
        OUT, pagesize=A4,
        leftMargin=2 * cm, rightMargin=2 * cm,
        topMargin=2.1 * cm, bottomMargin=2.0 * cm,
        title=TITLE, author="COOP-UAV-S project",
        subject="Cooperative collateral-damage-aware counter-UAS simulation",
    )
    fw = doc.width
    cover = PageTemplate(id="cover", frames=[Frame(
        2 * cm, 2 * cm, fw, A4[1] - 4 * cm, id="cf",
        leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)],
        onPage=_cover_bg)
    body = PageTemplate(id="body", frames=[Frame(
        2 * cm, 1.7 * cm, fw, A4[1] - 3.7 * cm, id="bf",
        leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)],
        onPage=_body_frame)
    doc.addPageTemplates([cover, body])
    return doc


# ── content ────────────────────────────────────────────────────────────────
def story():
    s = []
    fw = A4[0] - 4 * cm  # printable width (2 cm margins both sides)

    # ===== COVER =====
    s.append(Spacer(1, 3.2 * cm))
    s.append(P(TITLE, COVER_T))
    s.append(Spacer(1, 0.4 * cm))
    s.append(P(SUBTITLE, COVER_S))
    s.append(Spacer(1, 1.0 * cm))
    s.append(HRFlowable(width="38%", thickness=1.4, color=ACCENT, hAlign="LEFT"))
    s.append(Spacer(1, 0.7 * cm))
    meta = [
        ("Document type", "Full technical report (system design, methods, results)"),
        ("Subject", "Cooperative multi-UAV counter-drone defence of populated areas"),
        ("System", "COOP-UAV-S — pure-Python, ROS 2-shaped, deterministic simulation"),
        ("Status", "v0.3 simulation baseline · 750+ deterministic tests · GPL-3.0"),
        ("Date", "17 June 2026"),
    ]
    mt = Table([[P(f"<b>{k}</b>", COVER_M), P(v, COVER_M)] for k, v in meta],
               colWidths=[3.6 * cm, fw - 3.6 * cm])
    mt.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 0),
    ]))
    s.append(mt)
    s.append(Spacer(1, 1.0 * cm))
    s.append(boxed([P(
        "<b>Scope note.</b> This report documents the simulation environment (Element 2) and "
        "command interface (Element 3) of a three-element C-UAS specification. The physical "
        "segment (Element 1) is specified but deliberately not built at this stage; it is "
        "simulated at high fidelity. No live-fire, procurement, or flight-test activity is in "
        "scope. All effector kill probabilities are plausible engineering models, not measured "
        "ordnance data.", SMALL)]))

    s.append(NextPageTemplate("body"))
    s.append(PageBreak())

    # ===== ABSTRACT =====
    s.append(P("Abstract", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "Modern one-way-attack (OWA) drone raids defeat point air defences not with sophistication "
        "but with <i>economics</i>: they saturate, they mix in signature-identical decoys to exhaust "
        "interceptor stocks, and each successful kill drops roughly two hundred kilograms of wreckage "
        "onto the neighbourhood being defended. COOP-UAV-S is an open-source simulation framework for "
        "studying an AI-assisted, cooperative counter-UAS (C-UAS) system that treats two normally "
        "neglected constraints as first-class: (i) interceptors that are individually <i>slower</i> than "
        "their targets, and (ii) <i>collateral ground risk</i> from the debris of a successful kill. "
        "The system models a layered sensor network (radar, passive RF, EO/IR, acoustic pickets, "
        "forward high-altitude airborne-early-warning sentinels, and terminal onboard seekers) feeding "
        "a multi-sensor Kalman/GNN tracker with Bayesian threat-class and decoy belief; a "
        "Threat-Evaluation-and-Weapon-Assignment (TEWA) command-and-control loop that reserves "
        "cooperative blocking packages using exact Apollonius-circle rendezvous geometry; and a "
        "rules-of-engagement layer that runs a Monte-Carlo debris footprint against a "
        "SAFE/DANGEROUS/CRITICAL ground-risk raster before authorising any release. The two central "
        "claims — that cooperative <i>geometry</i> defeats a faster evader where airspeed cannot, and "
        "that the predicted debris footprint belongs <i>inside</i> the fire decision — are realised in "
        "code and exercised by a deterministic, seeded, YAML-driven scenario suite. In a ten-seed "
        "Monte-Carlo of the reference residential raid under deliberate saturation, the system holds two "
        "hard invariants — <b>zero wrecks on critical ground</b> and <b>zero rounds spent on identified "
        "decoys</b> — while achieving roughly fifty per cent armed-threat attrition. The allocator is a "
        "clean software seam: a trained multi-agent (MAPPO) cooperation policy drops in behind the same "
        "interface as the classical priority-greedy planner, with every safety gate preserved. The "
        "architecture mirrors ROS 2 topics so that migration to ROS 2 / Gazebo replaces two small "
        "classes rather than the tactical code.", ABST))
    s.append(Spacer(1, 4))
    s.append(P("<b>Keywords —</b> counter-UAS; cooperative pursuit-evasion; Apollonius circle; "
               "weapon-target assignment; multi-target tracking; decoy discrimination; ground-risk "
               "modelling; rules of engagement; multi-agent reinforcement learning; ROS 2.", KEYW))

    # ===== TOC =====
    s.append(Spacer(1, 12))
    s.append(P("Contents", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 5))
    toc = [
        ("1", "Introduction"),
        ("2", "Operational Context and Threat Taxonomy"),
        ("3", "System Architecture"),
        ("4", "Sensing and Perception"),
        ("5", "Command and Control: Threat Evaluation and Weapon Assignment"),
        ("6", "Cooperative Interception Geometry"),
        ("7", "Collateral-Risk-Aware Engagement"),
        ("8", "Learned Cooperative Allocation (MARL)"),
        ("9", "Simulation Methodology and Verification"),
        ("10", "Experimental Results"),
        ("11", "Limitations"),
        ("12", "Related Work"),
        ("13", "Conclusions and Future Work"),
        ("", "References"),
    ]
    rows = [[P(f"<b>{n}</b>" if n else "", TOC_E), P(t, TOC_E)] for n, t in toc]
    tt = Table(rows, colWidths=[1.0 * cm, fw - 1.0 * cm])
    tt.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LINEBELOW", (0, 0), (-1, -2), 0.3, colors.HexColor("#e6ebf0")),
        ("TOPPADDING", (0, 0), (-1, -1), 2),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 2),
    ]))
    s.append(tt)
    s.append(PageBreak())

    # ===== 1. INTRODUCTION =====
    s.append(P("1&nbsp;&nbsp;Introduction", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "The proliferation of cheap, long-range one-way-attack drones has changed the arithmetic of "
        "air defence over populated areas. The operational baseline for this work is the Ukrainian "
        "theatre of 2022&ndash;2026, where raids routinely combine strategic OWA platforms "
        "(Shahed-136 / Geran-2 class) with same-signature decoys (Gerbera class), attack densities "
        "deliberately exhaust interceptor inventories, and the consequence of <i>every</i> successful "
        "intercept is roughly two hundred kilograms of falling wreckage. Conventional missile-based "
        "defences lose the economic exchange and cannot, in any case, control where the debris lands. "
        "COOP-UAV-S studies an alternative: a cooperative team of low-cost kinetic interceptors and a "
        "sensor-fused base station, with probabilistic, collateral-damage-aware engagement enforced as "
        "a hard constraint rather than an afterthought.", BODY))
    s.append(P("1.1&nbsp;&nbsp;The two governing ideas", H2))
    s.append(P(
        "<b>Cooperation beats speed.</b> A lone interceptor slower than its target loses a tail chase "
        "&mdash; provably; the intercept-triangle equation has no positive root. But a hostile OWA is "
        "mission-bound to a predictable corridor toward its target, so a <i>team</i> can post blockers "
        "at corridor points they can reach first and let the target fly into the engagement. The C2 "
        "reserves blocking packages for exactly the targets that outrun the fleet, placing posts with "
        "closed-form Apollonius-circle rendezvous geometry. It is the geometry, not the airspeed, that "
        "wins.", BODY))
    s.append(P(
        "<b>Where the wreck falls is part of the fire decision.</b> The defended area is rasterised "
        "into SAFE / DANGEROUS / CRITICAL ground. Before any munition release the C2 runs a "
        "Monte-Carlo debris footprint of the <i>predicted</i> kill against that map and clears, holds, "
        "or denies the shot &mdash; including a &lsquo;now-or-never&rsquo; rule that authorises an "
        "imperfect shot when the target&rsquo;s own trajectory guarantees every later shot is worse. "
        "Net-gun kills drop wreckage nearly straight down; projectile kills throw it forward &mdash; "
        "and the rules of engagement feel the difference.", BODY))
    s.append(P("1.2&nbsp;&nbsp;Contributions", H2))
    s.append(bullets([
        "A complete, deterministic, message-driven C-UAS battle simulation spanning threats, layered "
        "sensing, multi-sensor fusion, TEWA command-and-control, cooperative interceptor agents, and a "
        "ground-truth engagement adjudicator.",
        "An exact Apollonius-circle formulation of cooperative blocker placement that defeats a faster, "
        "corridor-bound evader with slower pursuers, in relay.",
        "A debris-footprint rules-of-engagement layer that integrates predicted collateral ground cost "
        "directly into weapon-release authorisation, with clearance / hold / deny semantics.",
        "Forward high-altitude combat-air-patrol (CAP) sentinels carrying a look-down airborne "
        "early-warning radar, cutting diving-jet acquisition latency by roughly 36&times;.",
        "A learned multi-agent (MAPPO) cooperation policy that drops in behind the classical allocator "
        "seam without altering any downstream safety gate.",
        "A ROS 2-shaped architecture and a YAML-as-experiment methodology that make every run "
        "reproducible and the migration path concrete.",
    ]))
    s.append(P(
        "The remainder of the report follows the data path: operational context and threat taxonomy "
        "(&sect;2), the architecture (&sect;3), sensing and perception (&sect;4), command and control "
        "(&sect;5), the two innovation pillars &mdash; cooperative geometry (&sect;6) and "
        "collateral-aware engagement (&sect;7) &mdash; the learned allocator (&sect;8), methodology and "
        "verification (&sect;9), results (&sect;10), limitations (&sect;11), related work (&sect;12), "
        "and conclusions (&sect;13).", BODY))

    # ===== 2. OPERATIONAL CONTEXT =====
    s.append(P("2&nbsp;&nbsp;Operational Context and Threat Taxonomy", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "The system is designed around operationally-validated drone threat classes. The defining "
        "tactical patterns are altitude switching (low terrain-masking ingress versus high-altitude "
        "terminal dive), saturation (answered with low cost-per-shot kinetic effectors rather than "
        "missile economics), decoy integration (answered with multi-modal Bayesian discrimination), and "
        "jam-immune fibre-optic FPV guidance (answered kinetically). The nominal operating envelope is "
        "nocturnal, all-weather, &minus;25&nbsp;&deg;C to +45&nbsp;&deg;C, winds to 20&nbsp;m/s, and "
        "altitudes from 50&nbsp;m to 5&nbsp;km above ground level.", BODY))
    s.append(table(
        ["Class", "Exemplar", "Mass", "Speed", "Altitude (AGL)", "Behaviour"],
        [
            ["A — Strategic OWA", "Shahed-136 / Geran-2", "~200 kg", "50–65 m/s", "50 m – 5 km",
             "Swarm saturation, decoy mixing, terminal dive"],
            ["A+ — Jet OWA", "Geran-3 / Shahed-238", "~200 kg", "~100 m/s", "2–5 km",
             "High-speed dive; very low intercept window"],
            ["B — Tactical FPV", "Quadcopter kamikaze", "1–5 kg", "30–40 m/s", "0–200 m",
             "Agile, fibre-optic guided (jam-resistant)"],
            ["C — Loitering munition", "Lancet-3", "12 kg", "~80 m/s", "50–500 m",
             "AI terminal seeker, precision strike"],
            ["D — Decoy", "Gerbera", "~18 kg", "as class A", "as class A",
             "Identical RF/radar signature, no warhead — exists to exhaust interceptor stocks"],
        ],
        col_widths=[2.7, 3.1, 1.5, 1.6, 2.2, 4.9]))
    s.append(P("Table&nbsp;1. Threat taxonomy. Class&nbsp;D decoys share the class&nbsp;A radar and RF "
               "signature and flight profile by design: perception genuinely cannot separate them until "
               "a discriminating sensor earns it.", CAP))
    s.append(P(
        "The decoy is the structurally interesting class. A Gerbera carries no warhead; its sole "
        "purpose is to draw an interceptor. Because it is built to be indistinguishable from a class-A "
        "OWA on radar and RF, a defender that fires on radar tracks alone spends its magazine on empty "
        "airframes &mdash; which is precisely the adversary&rsquo;s objective. Discrimination therefore "
        "cannot be a pre-filter; it must be a belief that the system accumulates over time from the "
        "sensors capable of telling the two apart (EO/IR identification, acoustic engine cues, "
        "kinematic consistency), and it must be fused with the cost of being wrong.", BODY))

    # ===== 3. ARCHITECTURE =====
    s.append(P("3&nbsp;&nbsp;System Architecture", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P("3.1&nbsp;&nbsp;Three-element system definition", H2))
    s.append(P(
        "The governing System Requirements Specification partitions the system into three elements. "
        "<b>E1</b>, the physical segment (interceptor UAVs, ground stations, fixed sensors, anti-air "
        "turrets), is specified but not built at this stage &mdash; its requirements exist as the "
        "fidelity reference. <b>E2</b>, the simulation environment, is a high-fidelity digital twin of "
        "E1, its software, its communications, the threats, and the physical world. <b>E3</b>, the "
        "command interface, is the human-facing 3D real-time C2 console plus its orchestration agent. "
        "The binding principle is that E3 &mdash; and the humans behind it &mdash; cannot functionally "
        "distinguish the simulator from the real system except through explicitly evaluation-only "
        "ground-truth channels. Every physical requirement (PHY-*) has a corresponding simulation "
        "requirement (SIM-*) that recreates it; replacing E2 with real assets requires no change to E3 "
        "beyond disabling the evaluation channel.", BODY))
    s.append(P("3.2&nbsp;&nbsp;Design philosophy", H2))
    s.append(P(
        "Three decisions shape everything else. First, a <b>custom Python simulation with "
        "ROS 2-shaped seams</b>: every software component is a node on a publish/subscribe message bus "
        "with typed dataclass messages &mdash; a one-to-one image of ROS 2 nodes, topics, and "
        "<font face='Courier'>.msg</font> files &mdash; so migration to ROS 2 + Gazebo/PX4 replaces two "
        "small classes and the sim-side plugins while the tactical code is untouched. Second, "
        "<b>probabilistic engagement, not ballistics</b>: effectors expose a kill-probability "
        "<i>envelope</i> over range, off-axis angle and closing speed, and kills produce sampled debris "
        "footprints &mdash; the right fidelity for studying cooperation geometry and collateral risk, "
        "and swappable for higher fidelity later. Third, <b>ground truth is quarantined</b>: only "
        "explicitly sim-side components (sensors and the engagement adjudicator) may read the world&rsquo;s "
        "true state, exactly like Gazebo plugins; perception, C2 and the interceptor agents see nothing "
        "but messages. Decoys are therefore genuinely indistinguishable until a sensor earns the "
        "discrimination.", BODY))
    s.append(figure("fig_architecture.png", 14.6,
                    "Figure&nbsp;1. Component graph and publish/subscribe dataflow. Sim-side nodes "
                    "(beige) own ground truth; tactical nodes (blue) act only on messages. The fire "
                    "request / clearance handshake at the C2 is the human-on-the-loop seam."))
    s.append(P("3.3&nbsp;&nbsp;Topic contract", H2))
    s.append(P(
        "The bus topics are the future ROS 2 message interfaces. The full contract is small enough to "
        "state in one table.", BODY))
    s.append(table(
        ["Topic", "Message", "Producer → Consumer"],
        [
            ["detections", "Detection", "all sensors → Fusion"],
            ["tracks", "TrackArray", "Fusion → C2, UAVs, Recorder"],
            ["uav/state", "UavState", "each UAV → C2, peer UAVs, Recorder"],
            ["engagement/tasks", "list[EngagementTask]", "C2 → UAVs"],
            ["engagement/fire_request", "FireRequest", "shooter UAV → C2"],
            ["engagement/clearance", "FireClearance", "C2 → shooter UAV"],
            ["engagement/fire", "FireRequest", "shooter UAV → Adjudicator"],
            ["engagement/result", "EngagementResult", "Adjudicator → C2"],
        ],
        col_widths=[4.2, 4.2, 7.6]))
    s.append(P("Table&nbsp;2. The topic contract. Each row maps directly onto a ROS 2 topic and "
               "message type.", CAP))

    # ===== 4. SENSING & PERCEPTION =====
    s.append(P("4&nbsp;&nbsp;Sensing and Perception", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P("4.1&nbsp;&nbsp;Layered, mutually-compensating sensing", H2))
    s.append(P(
        "No single sensor solves the problem; each is deliberately imperfect and the layer is designed "
        "so that the weaknesses are complementary. All sensors emit the same "
        "<font face='Courier'>Detection</font> message with a full 3&times;3 covariance, so "
        "bearing-only geometry is encoded as an anisotropic covariance rather than a special case and "
        "the tracker needs no per-sensor logic.", BODY))
    s.append(table(
        ["Sensor", "Strength", "Weakness modelled"],
        [
            ["Radar", "Long range, Doppler", "R⁴ Pd falloff; radar horizon hides low FPVs"],
            ["Passive RF-DF", "Very long range, signature hash", "Bearing-only; decoys share the OWA signature by design"],
            ["EO/IR towers", "The decoy discriminator", "Short range; ID quality ramps with proximity"],
            ["Acoustic pickets", "Hear below the radar horizon", "Short range, coarse bearing"],
            ["Airborne EW (sentinel)", "Look-down on high divers", "Forward deployed; on-station endurance limits"],
            ["Onboard seeker", "Terminal accuracy + close ID", "Only where an interceptor already is"],
        ],
        col_widths=[3.4, 4.6, 8.0]))
    s.append(P("Table&nbsp;3. The sensor layer. The radar and RF set provide volume and persistence; "
               "EO/IR, acoustic and onboard seekers provide the discrimination the volume sensors "
               "cannot.", CAP))
    s.append(P("4.2&nbsp;&nbsp;Tracking, fusion and classification", H2))
    s.append(P(
        "Perception is a three-stage pipeline. <b>Tracking</b> maintains a six-state constant-velocity "
        "Kalman filter per object, with separate filter-time and measurement-time bookkeeping so that "
        "coasting tracks are pruned correctly. <b>Fusion</b> performs per-scan global-nearest-neighbour "
        "association &mdash; the Hungarian algorithm over Mahalanobis-gated costs &mdash; and fuses "
        "sensors sequentially in precision order, so that radar seeds tracks rather than RF bearing "
        "blobs. <b>Classification</b> maintains a Bayesian class belief per track from sensor "
        "likelihoods and RF signature evidence, deliberately joint over {OWA, decoy}; kinematic "
        "consistency is blended idempotently at readout rather than accumulated, because "
        "double-counting the same kinematic evidence every cycle would saturate the posterior. The "
        "output that matters downstream is <font face='Courier'>p_decoy</font>, the probability that a "
        "track is a warhead-less decoy.", BODY))
    s.append(boxed([P(
        "<b>Why decoy belief must be a posterior, not a flag.</b> Because the adversary engineers the "
        "decoy to match the OWA on the volume sensors, the only way to lower <font face='Courier'>"
        "p_decoy</font> is to earn discriminating evidence &mdash; an EO/IR identification at closing "
        "range, an acoustic engine-type cue, or kinematic inconsistency with a warhead profile. The "
        "allocator then trades that belief against the cost of a wrong call: firing on a true OWA you "
        "mistook for a decoy is a leak; firing on a decoy you mistook for an OWA is a wasted "
        "interceptor &mdash; the adversary&rsquo;s goal.", SMALL)]))

    # ===== 5. C2 / TEWA =====
    s.append(P("5&nbsp;&nbsp;Command and Control: Threat Evaluation and Weapon Assignment", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "The base station runs a Threat-Evaluation-and-Weapon-Assignment (TEWA) loop at 1&nbsp;Hz in "
        "three stages.", BODY))
    s.append(P("5.1&nbsp;&nbsp;Threat evaluation", H2))
    s.append(P(
        "Each track is propagated in the horizontal plane to predict impact against the protected-asset "
        "list. Its threat score combines lethality (1&nbsp;&minus;&nbsp;<font face='Courier'>p_decoy"
        "</font>), urgency (time-to-impact), asset value, and a ground-zone factor &mdash; so a "
        "likely-decoy track heading nowhere important scores far below a likely-OWA track diving on a "
        "hospital.", BODY))
    s.append(P("5.2&nbsp;&nbsp;Assignment", H2))
    s.append(P(
        "The allocator is priority-greedy over threat-ordered tracks. Each track receives exactly the "
        "package it needs: a shooter if the fleet can catch it, or a shooter <i>plus reserved "
        "blockers</i> if it outruns the fleet. A budget rule prevents support reservation from starving "
        "queued tracks under saturation; an incumbent discount (0.7&times;) suppresses jitter-driven "
        "shooter swaps. Tracks whose decoy probability exceeds 0.85 are assigned nothing &mdash; "
        "spending interceptors on Gerberas is the enemy&rsquo;s actual objective. The whole allocator "
        "sits behind a single function-call seam (&sect;8), with any allocator exception falling back to "
        "the classical planner for that cycle so that a misbehaving policy can never freeze tasking.", BODY))
    s.append(P("5.3&nbsp;&nbsp;Fire authorisation", H2))
    s.append(P(
        "Every release costs a Monte-Carlo debris footprint against the risk map (&sect;7), evaluated "
        "out-of-band &mdash; immediately, not at the 1&nbsp;Hz planning rate &mdash; because an "
        "engagement window against a 55&nbsp;m/s target lasts only seconds. The rules of engagement "
        "return one of five verdicts:", BODY))
    s.append(bullets([
        "<b>AUTHORIZED (geometry_safe)</b> — footprint cost under the base thresholds.",
        "<b>AUTHORIZED (now_or_never)</b> — above the base threshold, but the footprint cost is minimal "
        "over the target&rsquo;s <i>predicted</i> path: it is flying into the city, so holding only "
        "moves the debris onto worse ground.",
        "<b>AUTHORIZED (last_resort)</b> — impact imminent on a high-value asset.",
        "<b>HOLD</b> — the geometry can still improve; wait.",
        "<b>DENIED</b> — decoy-grade target, or unsafe geometry; disengage.",
    ]))
    s.append(P(
        "There is no release without an explicit clearance message. The fire-request / clearance "
        "handshake is precisely where a human-on-the-loop operator plugs into the message flow; all "
        "engagement authority traces to a human action or a human-pre-approved ROE rule.", BODY))

    # ===== 6. COOPERATIVE GEOMETRY =====
    s.append(P("6&nbsp;&nbsp;Cooperative Interception Geometry", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "The first innovation pillar answers a hard kinematic fact: a propeller interceptor at "
        "~80&nbsp;m/s cannot run down a 100&nbsp;m/s jet OWA, and frequently cannot catch even a "
        "60&nbsp;m/s OWA from a tail-chase start. The intercept-triangle solution &mdash; the quadratic "
        "whose positive root is the time to interception &mdash; simply has no root when the geometry "
        "is unfavourable. That absence of a root is the trigger for cooperation.", BODY))
    s.append(P("6.1&nbsp;&nbsp;Apollonius-circle blocker placement", H2))
    s.append(P(
        "For a pursuer at <b>P</b> with speed v<sub>P</sub> and an evader at <b>T</b> with speed "
        "v<sub>T</sub>, the locus of points the two reach simultaneously satisfies "
        "|X&minus;P|&nbsp;=&nbsp;k|X&minus;T| with k&nbsp;=&nbsp;v<sub>P</sub>/v<sub>T</sub>. For "
        "k&nbsp;&lt;&nbsp;1 (a slower pursuer) this locus is the <b>Apollonius circle</b>, with centre "
        "<b>C</b>&nbsp;=&nbsp;(P&nbsp;&minus;&nbsp;k&sup2;T)/(1&minus;k&sup2;) and radius "
        "R&nbsp;=&nbsp;k|P&minus;T|/(1&minus;k&sup2;); inside the disk the pursuer arrives first. A "
        "lone slower pursuer cannot capture a free evader &mdash; but the OWA is not free. It is "
        "mission-bound to a corridor toward its target, so the defender posts a blocker at the point "
        "where that corridor <i>first enters</i> the reachable disk. The interceptor arrives there "
        "first and meets the target head-on. The C2 reserves these blocking packages only for targets "
        "that outrun the fleet, and chains them in relay so that successive interceptors hand off "
        "containment.", BODY))
    s.append(figure("fig_apollonius.png", 12.2,
                    "Figure&nbsp;2. Apollonius cooperative-interception geometry for a slower pursuer "
                    "(v<sub>P</sub>/v<sub>T</sub>&nbsp;=&nbsp;0.70). The shaded disk is the region the "
                    "interceptor reaches before the corridor-bound hostile; the blocker is posted where "
                    "the predicted corridor first enters it."))
    s.append(P(
        "The implementation (<font face='Courier'>mc/apollonius.py</font>) uses the exact closed-form "
        "rendezvous, replacing an earlier time-stepping search, and extends to the game-theoretic "
        "containment arc and an escape-set (safe-fraction) area objective for manoeuvring evaders. The "
        "interceptor agents themselves run a mode finite-state machine (idle, pursuit, engage, "
        "blocking, herding, return-to-base) with proportional-navigation-style guidance; "
        "blocking-versus-herding is derived downstream from the target-versus-shooter speed comparison "
        "rather than chosen explicitly.", BODY))
    s.append(P("6.2&nbsp;&nbsp;Forward CAP sentinels", H2))
    s.append(P(
        "Cooperative geometry only helps if the target is seen early enough to post the blocker. The "
        "high-altitude diving jet OWA is the worst case: it appears late to the ground sensor set and "
        "leaves a vanishing intercept window. The system answers with forward combat-air-patrol "
        "sentinels &mdash; unarmed patrol UAVs standing on station above the ground radar&rsquo;s "
        "envelope and forward of the defended area, carrying a look-down airborne early-warning radar. "
        "They publish ordinary detections and, in the high-diver scenario, cut mean acquisition latency "
        "from 2.15&nbsp;s to 0.06&nbsp;s &mdash; roughly a 36&times; improvement (Figure&nbsp;3).", BODY))
    s.append(figure("fig_sentinel.png", 8.6,
                    "Figure&nbsp;3. Diving-jet OWA mean acquisition latency in the high-diver scenario, "
                    "ground sensor set versus the set augmented with forward CAP sentinels."))

    # ===== 7. COLLATERAL =====
    s.append(P("7&nbsp;&nbsp;Collateral-Risk-Aware Engagement", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P("7.1&nbsp;&nbsp;The ground-risk raster", H2))
    s.append(P(
        "The defended area is rasterised into a SAFE / DANGEROUS / CRITICAL grid with zone weights "
        "0.02 / 1.0 / 25.0. The default for unknown ground in a residential area is DANGEROUS, not "
        "SAFE: the system assumes civilians are present unless the map says otherwise. With "
        "building-typed urban inputs the raster is painted from building kinds in precedence order "
        "&mdash; a SAFE base, DANGEROUS residential and commercial buffers, SAFE parks, water and "
        "industrial land, and CRITICAL hospital, school and dense-residential buffers &mdash; so that "
        "red means civilians are certainly under the debris, yellow possibly, and green not at all. The "
        "design is SORA/JARUS-inspired and follows the urban ground-risk-map literature.", BODY))
    s.append(P("7.2&nbsp;&nbsp;The debris model", H2))
    s.append(P(
        "A kill produces a sampled ballistic debris footprint. The model is mechanism-dependent: a "
        "net-gun kill retains only ~0.15 of the target&rsquo;s horizontal velocity (the wreck drops "
        "nearly straight down), whereas a projectile-gun kill retains ~0.65 (the wreck is thrown "
        "forward), with terminal-velocity fall time and altitude-growing dispersion. The same model is "
        "used twice: <i>predictively</i> inside the rules of engagement to cost a candidate shot, and "
        "<i>generatively</i> by the adjudicator when a kill actually happens. This is why the choice of "
        "effector is itself part of the collateral calculus, and why the ROE can prefer a net-gun shot "
        "that drops the wreck on safe ground over a projectile shot that throws it onto a school.", BODY))
    s.append(P("7.3&nbsp;&nbsp;Interceptable debris", H2))
    s.append(P(
        "A kill spawns a falling debris object stepped by the world, publishing its predicted impact "
        "point and zone. The C2 turns any non-SAFE-bound debris into intercept tasks for projectile "
        "UAVs and turrets &mdash; CRITICAL before DANGEROUS &mdash; and credits the averted zone cost "
        "when a debris intercept succeeds; SAFE-bound debris is left to fall. The collateral problem is "
        "thus addressed both before the shot (footprint-gated authorisation) and after it (active "
        "debris interception).", BODY))

    # ===== 8. MARL =====
    s.append(P("8&nbsp;&nbsp;Learned Cooperative Allocation (MARL)", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "The weapon-target allocator is a clean software seam: the base station calls it through one "
        "hook, and a trained multi-agent policy can replace the classical priority-greedy planner "
        "behind the <i>same</i> interface. The policy decides which threats to commit which "
        "interceptors to, and in what role; everything downstream &mdash; guidance, the debris-footprint "
        "ROE, fire-control, and the authorisation chain &mdash; is the unchanged, trusted stack. "
        "Critically, the reconciliation step routes the final shooter pick through the classical "
        "best-shooter selector verbatim and re-applies the denied-track, debris-effector and "
        "availability gates, so the Pk-aware, incumbent-discounted choice the ROE geometry assumes "
        "stays authoritative even if the policy mis-commits.", BODY))
    s.append(P("8.1&nbsp;&nbsp;Problem formulation", H2))
    s.append(P(
        "The agents are the interceptors. A single parameter-shared actor runs per agent on an "
        "ego-centric, 176-dimensional observation (own state; the top-K&nbsp;=&nbsp;6 threat tracks by "
        "score, with relative kinematics, threat score, time-to-impact, "
        "<font face='Courier'>p_decoy</font>, impact zone, and caller-owned catchable / incumbent / "
        "denied / debris / valid flags; the nearest M&nbsp;=&nbsp;4 teammates; and a small global "
        "block), so the policy generalises across platforms and fleet sizes and can deploy "
        "decentrally. A centralised critic sees the joint observation in training only (the CTDE / "
        "MAPPO pattern). The action is one masked categorical over 1&nbsp;+&nbsp;2K&nbsp;=&nbsp;13 "
        "choices &mdash; idle, shoot one of the top-K, or block one of them &mdash; with the mask "
        "forbidding empty slots, denied tracks and net-on-debris so that any sampled action reconciles "
        "to a valid task. The reward is a shared team outcome (positive for armed kills; heavily "
        "negative for armed leakers, the real defence failure) plus zone-weighted debris and decoy "
        "penalties, per-agent ammunition and task-churn penalties, and a potential-based "
        "safety-shaping term that is policy-invariant.", BODY))
    s.append(P("8.2&nbsp;&nbsp;Training and deployment", H2))
    s.append(P(
        "Because the simulation is pure-Python and CPU-bound, training throughput scales with worker "
        "<i>processes</i>, not threads, and needs no GPU; the env step, not the network, is the "
        "bottleneck for a small WTA policy. Each episode draws a fresh randomised raid, so the policy "
        "sees a distribution of raids rather than one script. Deployment is a two-line scenario change "
        "pointing the base station at a checkpoint, and an A/B evaluation harness compares the learned "
        "allocator against the classical baseline over a seed sweep. The training and inference "
        "pipeline is complete and verified; the shipped default behaviour remains the classical "
        "planner, with the learned policy opt-in &mdash; a policy that decisively beats a well-tuned "
        "classical baseline requires a full training run, which is honestly noted as outstanding.", BODY))

    # ===== 9. METHODOLOGY =====
    s.append(P("9&nbsp;&nbsp;Simulation Methodology and Verification", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "The simulation is a deterministic fixed-step world (default 20&nbsp;Hz) with per-node rates, "
        "driven by a single seeded random-number generator so that every run is exactly reproducible &mdash; "
        "a property enforced by a dedicated determinism test. Scenarios are pure YAML: each file fully "
        "describes a battle &mdash; map, risk zones, sensor laydown, fleet, raid composition, and ROE "
        "thresholds &mdash; so that experiments are data, not code. The engagement adjudicator referees "
        "every fire event against ground truth, rolling the true kill probability and sampling the "
        "debris footprint, and is one of only two components permitted to read the world&rsquo;s true "
        "state.", BODY))
    s.append(P(
        "Three reference scenarios anchor the experiments. The <b>residential raid</b> is the canonical "
        "case: a 12&times;12&nbsp;km area with two protected assets and a nine-drone three-wave mix of "
        "OWAs, decoys, FPVs and one jet OWA. The <b>high-diver raid</b> isolates the high-altitude "
        "diving-jet problem and the forward CAP sentinel response. The <b>urban raid</b> is a large "
        "procedurally-generated city with twenty interceptors, ten sentinels, line-of-sight occlusion "
        "and live debris. The framework ships with a suite of 750+ deterministic tests, including an "
        "end-to-end raid, and an optional higher-fidelity software-in-the-loop mode that runs the "
        "tactical stack on a virtual flight controller (EKF, mixers, built-in-test fault monitors) over "
        "a modelled datalink.", BODY))
    s.append(table(
        ["Scenario", "What it exercises"],
        [
            ["residential_raid", "Reference raid: 12×12 km, two assets, 9-drone three-wave OWA/decoy/FPV/jet mix"],
            ["high_diver_raid", "High-altitude diving-jet problem + forward CAP sentinels with look-down EW radar"],
            ["urban_raid", "Large procedural city: 20 interceptors, 10 sentinels, occlusion, live debris"],
        ],
        col_widths=[3.6, 12.2]))
    s.append(P("Table&nbsp;4. The reference scenario suite. Scenarios are YAML data; copying and "
               "editing one defines a new experiment with no code change.", CAP))

    # ===== 10. RESULTS =====
    s.append(P("10&nbsp;&nbsp;Experimental Results", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "The headline evaluation is a ten-seed Monte-Carlo of the reference residential raid &mdash; "
        "nine threats including two decoys, defended by six gun and two net interceptors &mdash; run "
        "under deliberate saturation. A representative single run resolves as six kills, three leakers "
        "of which one is armed, and wreckage falling on one SAFE and two DANGEROUS cells with none on "
        "CRITICAL ground (Figure&nbsp;4a&ndash;b). Across all ten seeds the system holds two hard "
        "invariants.", BODY))
    s.append(figure("fig_results.png", 15.6,
                    "Figure&nbsp;4. Reference residential-raid results. (a) Representative single-run "
                    "outcome over nine hostiles. (b) Debris by ground zone &mdash; the ROE invariant of "
                    "zero critical-zone wrecks. (c) The two ten-seed invariants, with overall "
                    "armed-threat attrition."))
    s.append(boxed([
        P("<b>Verified ten-seed invariants (reference residential raid)</b>", BODY_T),
        bullets([
            "<b>Zero critical-zone wrecks</b> across all seeds &mdash; the collateral ROE invariant "
            "holds under saturation.",
            "<b>Zero shots at identified decoys</b> across all seeds &mdash; interceptor economics are "
            "preserved against the Gerbera exhaustion tactic.",
            "<b>~50% overall armed-threat attrition</b>, with a class structure that matches reality: "
            "strategic OWAs and FPVs engaged effectively; the 100&nbsp;m/s jet OWA documented as beyond "
            "the propeller-interceptor tier (motivating the fast-interceptor tier and the forward CAP "
            "sentinels).",
        ], st=SMALL),
    ]))
    s.append(P(
        "These two invariants are the project&rsquo;s central empirical claim: under a saturation raid "
        "engineered to exhaust the magazine and to force collateral damage, a cooperative, "
        "collateral-aware C2 spends nothing on decoys and drops nothing on critical ground, while still "
        "attriting roughly half of the armed threats. The separately measured 36&times; reduction in "
        "diving-jet acquisition latency from the forward CAP sentinels (Figure&nbsp;3) shows that the "
        "remaining hard case &mdash; the fast diver &mdash; is a sensing-and-cooperation problem, which "
        "is the gap the learned allocator and the fast-interceptor tier are designed to close.", BODY))

    # ===== 11. LIMITATIONS =====
    s.append(P("11&nbsp;&nbsp;Limitations", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P("The framework is explicit about the fidelity boundaries of the current baseline.", BODY))
    s.append(bullets([
        "<b>Effector kill probabilities are plausible engineering models, not measured data.</b> No "
        "public Pk data exists for any C-UAS interceptor, so the engagement envelopes are inventions "
        "calibrated for qualitative realism, not validated ordnance performance.",
        "<b>The tracker is constant-velocity</b> and therefore lags manoeuvres; this is mitigated by "
        "the onboard terminal seeker but motivates the planned interacting-multiple-model upgrade.",
        "<b>The battery model is a linear drain</b> with no recovery or rearm cycle in the tactical "
        "core: interceptors return to base and stay there. (A higher-fidelity powertrain and "
        "charging-station cycle exists in the SITL layer.)",
        "<b>The learned policy is not yet a proven win.</b> The pipeline is complete and verified, but "
        "demonstrating a policy that beats the well-tuned classical baseline needs a full training run; "
        "the shipped behaviour is the classical planner.",
        "<b>The three-class risk raster is a coarse proxy</b> for true population exposure; "
        "time-of-day population layers and casualty-expectation units are planned.",
    ]))

    # ===== 12. RELATED WORK =====
    s.append(P("12&nbsp;&nbsp;Related Work", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "COOP-UAV-S draws on six largely separate literatures and integrates them into one closed loop. "
        "<b>Cooperative pursuit-evasion</b> provides the geometric backbone: the multiple-pursuer "
        "differential-game series of Garcia, Casbeer, Von Moll and Pachter [2,3], the faster-evader "
        "treatments of Wang et al. [4] and the Apollonius-with-obstacles work [5], and the "
        "area-optimal safe-reachable-set formulation [6] that motivates the escape-set objective; "
        "Isaacs [1] is the foundational text. <b>Weapon-target assignment and TEWA</b> supply the "
        "allocation layer &mdash; the modern WTA survey [7], consensus-based decentralised auctions "
        "(CBBA) [8] for the link-degraded case, and the Hungarian assignment [9] used in fusion. "
        "<b>Multi-target tracking</b> contributes the filtering and association canon &mdash; MHT [10], "
        "the GM-PHD and labelled-RFS filters [11,12], JPDA/IMM [13] for the planned manoeuvre upgrade, "
        "and the Stone&nbsp;Soup framework [14] as a reuse target. <b>Decoy and drone discrimination</b> "
        "rests on radar micro-Doppler [15] and RF-fingerprinting [16] research, against open-source "
        "intelligence on the Gerbera decoy [17]. <b>Guidance</b> follows proportional-navigation theory "
        "[18] and its multirotor and net-capture adaptations, including the CTU-MRS MBZIRC drone-capture "
        "work [19]. <b>Ground-risk modelling</b> follows JARUS SORA [20] and the urban ground-risk-map "
        "and impact-footprint literature [21,22]. Finally, the <b>multi-agent reinforcement learning</b> "
        "layer uses the MADDPG and MAPPO baselines [23], and the ROS 2 / Aerostack2 / Gazebo stack [24] "
        "defines the migration path. To our knowledge the specific combination &mdash; cooperative "
        "Apollonius blocking in relay, with debris-footprint-gated weapon release against a "
        "civilian-presence raster, evaluated under decoy-saturated raids &mdash; is not addressed as a "
        "closed loop in the open literature.", BODY))

    # ===== 13. CONCLUSIONS =====
    s.append(P("13&nbsp;&nbsp;Conclusions and Future Work", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "COOP-UAV-S demonstrates, in a deterministic and fully reproducible simulation, that the two "
        "constraints usually treated as afterthoughts in counter-drone work &mdash; interceptors slower "
        "than their targets, and the collateral cost of where the wreck falls &mdash; can be made "
        "first-class and still yield an effective defence. Cooperative Apollonius-circle blocker "
        "placement converts a kinematically hopeless tail chase into a head-on engagement against a "
        "corridor-bound evader; a debris-footprint rules-of-engagement layer keeps wreckage off critical "
        "ground; and a decoy-aware allocator refuses to spend the magazine on the adversary&rsquo;s "
        "bait. Under a saturation raid the system holds both hard invariants &mdash; zero critical-zone "
        "wrecks and zero shots at identified decoys &mdash; at roughly fifty per cent armed-threat "
        "attrition.", BODY))
    s.append(P(
        "The near-term roadmap closes the loop the current ROE leaves open: <b>intercept-point "
        "optimisation</b> &mdash; choosing <i>where</i> along the corridor to shoot by minimising "
        "expected debris cost subject to a Pk floor, unifying kill-box selection, herding and ROE &mdash; "
        "is the most defensible novelty and addresses an open-literature gap. Other planned work "
        "includes interacting-multiple-model tracking for dive manoeuvres, decentralised CBBA allocation "
        "for link-degraded operation, population-density risk maps with casualty-expectation units, a "
        "decoy-ratio sweep study, and a full MAPPO training run to test whether the learned policy can "
        "beat the classical baseline. The longer-term path is fidelity and migration: a ROS 2 port that "
        "regenerates the message contract as <font face='Courier'>.msg</font> files, a Gazebo / PX4 "
        "software-in-the-loop slice for a few vehicles, and richer sensor and effector flyout models. "
        "Because the architecture was built ROS 2-shaped from the start, that migration replaces the "
        "bus and node classes &mdash; not the tactical code this report describes.", BODY))

    # ===== REFERENCES =====
    s.append(P("References", H1))
    s.append(HRFlowable(width="100%", thickness=0.7, color=RULE))
    s.append(Spacer(1, 5))
    refs = [
        "R. Isaacs, <i>Differential Games: A Mathematical Theory with Applications to Warfare and "
        "Pursuit, Control and Optimization</i>. Wiley, 1965 (Dover reprint, 1999).",
        "E. Garcia, D. W. Casbeer, A. Von Moll, and M. Pachter, &ldquo;Multiple Pursuer Multiple "
        "Evader Differential Games,&rdquo; <i>IEEE Trans. Automatic Control</i>, 2020.",
        "A. Von Moll, D. Casbeer, E. Garcia, D. Milutinovi&cacute;, and M. Pachter, &ldquo;Cooperative "
        "Pursuit by Multiple Pursuers of a Single Evader,&rdquo; and &ldquo;Multiple-Pursuer, "
        "Single-Evader Border Defense Differential Game,&rdquo; <i>J. Aerospace Information Systems</i>.",
        "X. Wang et al., &ldquo;Cooperative Hunting Strategy with a Superior Evader Based on "
        "Differential Game,&rdquo; <i>Complexity</i>, 2022.",
        "&ldquo;Collaborative pursuit-evasion game of multi-UAVs based on Apollonius circle in the "
        "environment with obstacle,&rdquo; <i>Connection Science</i>, 2023.",
        "&ldquo;Area-Optimal Control Strategies for Heterogeneous Multi-Agent Pursuit,&rdquo; "
        "arXiv:2511.15036, 2025.",
        "&ldquo;A comprehensive survey of weapon target assignment problem: Model, algorithm, and "
        "application,&rdquo; <i>Engineering Applications of Artificial Intelligence</i>, 2024.",
        "H.-L. Choi, L. Brunet, and J. P. How, &ldquo;Consensus-Based Decentralized Auctions for "
        "Robust Task Allocation,&rdquo; <i>IEEE Trans. Robotics</i>, vol. 25, no. 4, 2009.",
        "H. W. Kuhn, &ldquo;The Hungarian Method for the Assignment Problem,&rdquo; <i>Naval Research "
        "Logistics Quarterly</i>, vol. 2, pp. 83&ndash;97, 1955.",
        "D. Reid, &ldquo;An Algorithm for Tracking Multiple Targets,&rdquo; <i>IEEE Trans. Automatic "
        "Control</i>, vol. 24, no. 6, pp. 843&ndash;854, 1979.",
        "B.-N. Vo and W.-K. Ma, &ldquo;The Gaussian Mixture Probability Hypothesis Density Filter,&rdquo; "
        "<i>IEEE Trans. Signal Processing</i>, vol. 54, no. 11, pp. 4091&ndash;4104, 2006.",
        "S. Reuter, B.-T. Vo, B.-N. Vo, and K. Dietmayer, &ldquo;The Labeled Multi-Bernoulli "
        "Filter,&rdquo; <i>IEEE Trans. Signal Processing</i>, vol. 62, no. 12, 2014.",
        "H. A. P. Blom and Y. Bar-Shalom, &ldquo;The IMM algorithm for systems with Markovian switching "
        "coefficients,&rdquo; <i>IEEE Trans. Automatic Control</i>, vol. 33, no. 8, 1988.",
        "Defence Science and Technology Laboratory (dstl), &ldquo;Stone Soup: an open-source framework "
        "for tracking and state estimation,&rdquo; 2017.",
        "M. Rahman and D. A. Robertson, &ldquo;Radar micro-Doppler signatures of drones and birds at "
        "K-band and W-band,&rdquo; <i>Scientific Reports</i>, vol. 8, 2018.",
        "&ldquo;RF-Enabled Deep-Learning-Assisted Drone Detection and Identification: An End-to-End "
        "Approach,&rdquo; <i>Sensors</i>, vol. 23, no. 9, 4202, 2023.",
        "Open-source intelligence on the Gerbera decoy drone (ISIS report; CEPA; Army Recognition "
        "analysis), 2024&ndash;2025.",
        "P. Zarchan, <i>Tactical and Strategic Missile Guidance</i>, 6th/7th ed. AIAA Progress in "
        "Astronautics and Aeronautics, 2012/2019.",
        "M. Vrba et al. (CTU MRS), &ldquo;Autonomous capture of agile flying objects using UAVs: the "
        "MBZIRC 2020 challenge,&rdquo; <i>Robotics and Autonomous Systems</i>, 2022.",
        "JARUS, &ldquo;Guidelines on Specific Operations Risk Assessment (SORA),&rdquo; "
        "JAR-DEL-WG6-D.04.",
        "S. Primatesta, A. Rizzo, and A. la Cour-Harbo, &ldquo;Ground Risk Map for Unmanned Aircraft "
        "in Urban Environments,&rdquo; <i>J. Intelligent &amp; Robotic Systems</i>, 2020.",
        "&ldquo;Accurate Ground Impact Footprints and Probabilistic Maps for Risk Analysis of UAV "
        "Missions,&rdquo; <i>IEEE Aerospace Conference</i>, 2019.",
        "R. Lowe et al., &ldquo;Multi-Agent Actor-Critic for Mixed Cooperative-Competitive "
        "Environments&rdquo; (MADDPG), NeurIPS 2017; C. Yu et al., &ldquo;The Surprising Effectiveness "
        "of PPO in Cooperative Multi-Agent Games&rdquo; (MAPPO), NeurIPS 2022.",
        "M. Fernandez-Cortizas et al., &ldquo;Aerostack2: A Software Framework for Developing "
        "Multi-robot Aerial Systems,&rdquo; arXiv:2303.18237, 2023.",
    ]
    for i, r in enumerate(refs, 1):
        s.append(P(f"[{i}]&nbsp;&nbsp;{r}", REF))
    s.append(Spacer(1, 8))
    s.append(HRFlowable(width="100%", thickness=0.5, color=RULE))
    s.append(Spacer(1, 4))
    s.append(P(
        "<i>Reproducibility.</i> This document and its figures are generated by "
        "<font face='Courier'>scripts/build_report.py</font> and "
        "<font face='Courier'>scripts/report_figures.py</font> from the COOP-UAV-S repository. The "
        "quantitative results reproduce the documented reference Monte-Carlo (README.md; "
        "docs/ARCHITECTURE.md &sect;4) and the diving-jet acquisition latency (docs/MARL.md). Several "
        "references list standard bibliographic details that should be re-verified against the "
        "primary source before formal external citation.", SMALL))
    return s


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    if not os.path.exists(os.path.join(ASSETS, "fig_architecture.png")):
        import report_figures
        report_figures.main()
    doc = build_doc()
    doc.build(story())
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
