# Task 2 — Grid-Company Culture Identification

**Executor:** This task is run by Claude (Research feature). Every finding must be traceable to a named source — no claim, figure, role, crew size, or pain point may appear in any deliverable without a citation. Follow the Sourcing & Verification Protocol below on every work package.

**Objective:** Identify *who* actually inspects transmission towers and power lines today at real grid companies (utilities / transmission system operators), *how* those companies inspect right now in operational detail, and — the primary deliverable — *which people are involved in the inspection process, in depth*: their roles, crew composition, certifications, department placement, and what each physically does.

**Unit of analysis:** The **grid company (end user)**, not the drone-service vendor. Vendors and contractors appear only as far as they explain how a given grid company gets its inspections done.

**Regional scope:** Profile **2–3 major grid companies from each of the following regions**:
- **Pakistan** — the FYP's home context.
- **Saudi Arabia** — regional GCC benchmark.
- **United States** — mature, heavily-documented, regulation-driven inspection culture.
- **Europe** — mature TSO culture, strong standards and public reporting.

**Why this task exists:** Requirements must be derived from real users and their real current practice, not from our design. Until we know *who* plans, flies, climbs, analyses, verifies, and dispatches an inspection at an actual utility — and how that differs by region and maturity — we cannot prioritise design effort or justify the project's value.

**Framing principle (analytical lever):** For each grid company and each role, note what the current process *cannot* do or what it *costs them* today (time, money, safety exposure, missed defects, coverage gaps). Those gaps become the contribution levers for our autonomous system.

**Definitions used below:**
- **Grid company / utility** = the entity that owns the towers and lines and is responsible for inspecting them (transmission system operator, distribution operator, or vertically-integrated utility).
- **Transmission vs distribution** = high-voltage backbone (e.g. 220/500 kV) vs lower-voltage local network; inspection practice and personnel differ between them — record which one each finding refers to.
- **Provider of the inspection** = who physically performs it: **in-house crew** vs **contracted service provider**. Establish this explicitly for every company.
- **Role** = a specific human function in the pipeline (patrolman, tower climber, helicopter pilot, UAS operator, inspection engineer, analyst, asset manager, etc.).
- **Persona** = a profiled role with responsibilities, certification, time, cost, and pains.

---

## Execution Setup

1. **Tool:** Run in the Claude web/desktop app with the **Research** feature enabled (toggle Web Search on first, then Research). Preferred over Claude Code because the deliverable is a cited research document, and Research links every claim to a source by default.
2. **Model:** Use **Opus 4.8** for synthesis-heavy passes (WP 2.4–2.6). If Team-plan usage limits tighten, gather raw sources (WP 2.1–2.3) on **Sonnet 5**, then switch to Opus 4.8 for synthesis.
3. **Agent constraint (usage limit):** Run each work package as a **single controlled research pass**. Do **not** spawn parallel research sub-agents — they multiply token consumption and can exhaust the Team-plan allowance. If a work package is large, split it into sequential passes *within one session* rather than fanning out concurrently. A natural split is **one region per pass** (Pakistan → Saudi → US → Europe).
4. **Scope discipline:** Complete one work package fully, verify its sources, then move to the next. Do not begin synthesis (WP 2.6) until WP 2.1–2.5 are source-complete. The **personnel decomposition (WP 2.3) is the priority** — if time is short, cut breadth of companies before cutting depth of roles.

---

## Sourcing & Verification Protocol (applies to every work package)

1. **No uncited claims.** Every fact, figure, role, cost, crew size, and pain point must carry a citation: source name + direct URL. If a statement has no source, mark it `[ASSUMPTION — unverified]` and keep it visually separate from sourced findings.
2. **Confidence tiering.** Tag each finding as one of:
   - `[WELL-SOURCED]` — corroborated by two or more independent sources.
   - `[SINGLE-SOURCE]` — one source only; flag for later verification.
   - `[THIN/CONTESTED]` — weak, dated, or contradicted evidence; state the contradiction.
3. **Prefer primary sources.** Rank: the grid company's own annual reports / asset-management plans / careers pages / procurement tenders / press releases > regulator or standards bodies (NEPRA, Ofgem, ENTSO-E, ECRA/WERA, FERC/NERC) > market-research firms > news / trade press > blogs / forums. Utility **asset management plans, regulatory filings, tender documents, and job postings** are the richest primary sources for *how they inspect* and *who does it* — go there first.
4. **Record provenance metadata** for each source: publisher, author (if any), publication/update date, date accessed, URL. 
5. **Quote sparingly, paraphrase by default.** Capture findings in our own words with the citation; reserve short verbatim quotes only where exact wording carries meaning (e.g. a regulatory inspection-frequency clause).
6. **Flag contradictions explicitly.** When sources disagree (e.g. crew size, inspection cadence, cost-per-tower), record both figures with their sources rather than silently picking one.
7. **Separate the company's claims from independent evidence.** A utility press release announcing "we now use AI drones" is evidence of what they *claim/pilot*, not proof it is business-as-usual practice; label it as such and look for scale/operational confirmation.
8. **Watch for org-structure changes.** Several target companies were recently restructured or rebranded (see WP 2.1 notes). Confirm the *current* legal entity and which subsidiary owns transmission before attributing practice to it.
9. **Deliverable format.** Every table gets a trailing `Source(s) + confidence tier` column; every narrative finding gets an inline citation. Maintain one consolidated **Source Register** at the end of the document, numbered, so any claim can be audited.

---

## WP 2.1 — Target Grid-Company Selection & Profiling

1. Select **2–3 major grid companies per region** and record for each: transmission vs distribution vs integrated, network size (km of line / number of towers if available), voltage levels, geography/terrain, ownership (state vs private), and regulator.
2. **Candidate companies** (verify current structure before use; substitute if a better-documented company exists):
   - **Pakistan:** **National Grid Company of Pakistan (NGC)** — transmission, 220/500 kV; *note: formerly NTDC, unbundled from 2024–2025 into NGC + EIDMC + ISMO — confirm which entity now owns/inspects transmission assets.* • **K-Electric** — vertically integrated (Karachi). • one DISCO for distribution-level practice: **HESCO / IESCO / LESCO / MEPCO**.
   - **Saudi Arabia:** **National Grid SA (NGSA)** — transmission subsidiary of **Saudi Electricity Company**; *note: SEC rebranded to "Saudi Energy (SE)" in early 2026 — confirm current naming.* • SEC/SE distribution arm for distribution practice. • optionally **Marafiq** (Jubail/Yanbu industrial-city utility).
   - **United States:** pick 2–3 of — **American Electric Power (AEP)** (largest US transmission network), **PG&E** (wildfire-driven inspection programmes — exceptionally well documented), **Duke Energy**, **Dominion**, **Southern Company**, **Xcel Energy**, **ITC Holdings** (pure-play transmission), **TVA**.
   - **Europe:** pick 2–3 TSOs — **National Grid Electricity Transmission (UK)**, **TenneT (NL/DE)**, **RTE (France)**, **Terna (Italy)**, **50Hertz / Amprion (Germany)**, **Red Eléctrica (Spain)**, **Statnett (Norway)**.
3. For each company, record **in-house vs contracted** inspection at a headline level (detail comes in WP 2.2/2.5), and its stated technology-adoption stage (foot-patrol-only → helicopter → manual drone → autonomous/AI drone → LiDAR/robotics).
4. **Deliverable:** grid-company matrix (rows = companies, grouped by region; columns = T/D/integrated · network size · voltage · terrain · ownership · regulator · in-house vs contracted · tech-adoption stage · **Source(s) + confidence tier**). Every classification cites its origin.

---

## WP 2.2 — Current Inspection Practice, in Detail (*how they inspect now*)

*Do this per company. Goal: a precise operational picture of today's process, fully sourced.*

1. **Inspection modalities used** — for each company, which of these are actually in operational use, and for what share / which asset classes:
   - Ground / foot patrol (visual, binoculars).
   - Tower climbing / detailed hands-on inspection.
   - Helicopter (manned aerial visual, LiDAR, thermographic/corona).
   - Manual-piloted drone (pilot flies and frames shots).
   - Semi-/fully-autonomous drone (waypoint or mission-planned).
   - Fixed-wing / LiDAR aerial survey, satellite, robotic line-crawlers.
2. **Inspection types & triggers** — routine visual patrol, detailed/climbing inspection, thermographic (hotspot), corona/UV, vegetation-encroachment survey, post-storm/emergency, commissioning. Record what triggers each.
3. **Cadence & drivers** — how often each inspection type is performed, and *what mandates it*: regulator rule, grid code, internal asset-management-plan cycle, insurance/wildfire requirement. Cite the governing document (e.g. NERC/FERC, Ofgem RIIO, ENTSO-E, NEPRA grid code, ECRA/WERA).
4. **Data → decision flow** — how captured data becomes a maintenance action: manual review vs software/AI platform, who hosts the analytics, report turnaround time, how defects are severity-rated and dispatched.
5. **Technology-adoption reality check** — separate *announced pilots* from *business-as-usual practice* (Protocol §7). Note where a company has publicly committed to drone/AI inspection and at what scale.
6. **Deliverable:** per-company inspection-practice profile + a cross-company modality table (company × modality-in-use × inspection-type × cadence × mandating rule × data-flow × **Source(s) + confidence tier**).

---

## WP 2.3 — Personnel & Role Decomposition — **PRIORITY DELIVERABLE** (*who is involved, in detail*)

*This is the core of the task. For each company (or, where company-specific data is thin, for each region's typical practice), enumerate every human who touches an inspection, stage by stage.*

1. **Draw the end-to-end pipeline** for each modality in use and, at each stage, list **every human role** and **what they physically do**. Stages:
   `mission/patrol planning → outage & permit coordination → site & weather clearance → field mobilisation → capture (climb/fly/patrol/helicopter) → data offload → anomaly/defect detection → report generation → engineering verification & severity rating → maintenance dispatch → close-out & records.`
2. **Enumerate roles explicitly.** Expect (verify per company/region): line patrolman / lineman, tower climber / rigger, **helicopter pilot + aerial observer/inspector**, **UAS/drone pilot (and any standby/backup pilot)**, **ground-control-station operator**, inspection/field engineer, thermographer, data analyst / image reviewer, asset or reliability engineer, GIS/mapping specialist, maintenance planner / scheduler, dispatcher, **safety officer / permit-to-work coordinator**, vegetation-management crew, procurement / asset manager, and any regulator sign-off role.
3. **For each role capture:** what they do at which stage, **crew size / how many per inspection**, **certifications & skill level** required (e.g. lineman qualification, remote-pilot licence, thermography level, helicopter ratings), and **which department / team** they sit in on the org chart.
4. **Resolve the key questions** (answer per company where sourced, else per region):
   - Is the drone pilot also the inspection engineer, or is a separate senior engineer directing what gets photographed?
   - Who decides which components to inspect and when a tower is "done"?
   - Who performs anomaly detection — a person offline, on site, or an AI platform — and who *signs off* the final defect list?
   - For helicopter/climbing: crew composition, safety personnel, and outage/permit coordinators.
   - For any autonomous programme: single operator or GCS team? Is a trained backup pilot retained for manual override? Who monitors telemetry and handles emergencies?
5. **Best primary sources for personnel detail:** company **job/careers postings** (reveal exact titles, certifications, crew structure), **safety manuals & method statements**, **asset-management plans**, **annual reports / org charts**, **procurement tenders** (specify required roles), and recorded **webinars / conference talks** by the utility's asset teams.
6. **Tag each role** per stage as **eliminated / reduced / retained / newly-introduced** under an autonomous system like ours (this feeds WP 2.6).
7. **Deliverable:** one pipeline diagram + a role-per-stage table **per company (or per region where company-level data is thin)**. Each role entry cites the source establishing that the role exists there; label company-claimed roles as claims, not verified fact. This is the deliverable to make most complete and most rigorously sourced.

---

## WP 2.4 — Persona Profiling

1. Consolidate WP 2.3 roles into a **deduplicated master role list**, noting which region(s)/company(ies) each role appears in.
2. Separate roles into three stakeholder classes:
   - **Operators** — hands-on the inspection (patrolman, climber, pilot, GCS operator, analyst).
   - **Decision-makers / buyers** — commission and pay (asset manager, procurement, safety head, reliability engineer).
   - **Beneficiaries** — depend on the output (maintenance crew, grid-reliability/planning teams, regulators).
3. Build a **persona card per key role** capturing: primary responsibility, required skill/certification, time spent per tower/line, approximate cost to employer, and their single biggest frustration today. Note regional variation (a Pakistani DISCO patrolman ≠ a US transmission UAS pilot).
4. Rank personas by how central they are to (a) the daily inspection operation and (b) the buying decision for a new inspection system.
5. **Deliverable:** persona cards + a stakeholder map (operator / buyer / beneficiary). Every attribute on a card must cite a source or be tagged `[ASSUMPTION — unverified]`.

---

## WP 2.5 — In-house vs Contracted, and the Contractor Layer

1. For each grid company, establish **precisely who performs the inspection**: fully in-house crews, a contracted drone/inspection-service provider, or a hybrid — and for which asset classes / regions.
2. Where contracted, identify **the contractor(s)** the utility uses and what that arrangement assumes the utility still staffs (e.g. utility retains verification engineers and dispatch, contractor supplies pilots and analysts). Name contractors only as evidence of *how the grid company operates*.
3. Note **procurement posture**: does the utility buy a service, a platform/software, or drones+training to bring capability in-house? Cite tenders / framework agreements where available.
4. **Deliverable:** in-house-vs-contracted table (company × who-performs × contractor(s) if any × what-utility-still-staffs × service/platform/in-house × **Source(s) + confidence tier**).

---

## WP 2.6 — Cross-Regional Culture Comparison, Pain-Points & Synthesis

1. **Cross-regional comparison.** Compare inspection culture across Pakistan / Saudi / US / Europe on: dominant modality, in-house vs contracted norm, technology-adoption maturity, regulatory pressure, crew sizes, and typical role set. Explicitly locate **where Pakistan sits** relative to the others, and which roles are universal vs region-specific.
2. **Pain-point mapping (ties to the FYP).** For the key personas, enumerate current pains under present methods (time on site, pilot/climber fatigue, safety exposure at height, subjective shot framing, slow report turnaround, missed/edge-case defects, coverage gaps, GPS/mapping dependence). Map each pain to how our autonomous system addresses it — **remove / reduce / no change** — and state the mechanism. Explicitly flag pains we do **not** solve or new burdens we introduce (new GCS-operator skill, RTK dependence, verification expert still required). Cite each *pain* to its source; reference the *project design doc* for each *our-mechanism* claim.
3. **"For whom" statement.** Produce a single statement naming the **primary persona(s)** and the top pains addressed, and rank personas into primary (drives requirements) vs secondary.
4. **Handoff.** Pass the primary persona set + the pain-point→solution matrix to the requirements-definition phase so design priorities are justified against real users.
5. **Audit.** Confirm every claim carried into the summary is cited and confidence-tiered; the summary must contain no `[ASSUMPTION — unverified]` items left unresolved.
6. **Deliverable:** cross-regional comparison table + pain-point→solution matrix + a one-page culture summary feeding directly into user requirements, plus the consolidated Source Register.

---

## Source Register (maintain throughout)

A running, numbered list of every source used, appended to the document. Every citation elsewhere references a number here so any claim can be audited.

| # | Source (publisher / author) | Type (primary / regulator / market-research / press / blog) | Date published | Date accessed | URL | Used in WP(s) |
|---|---|---|---|---|---|---|

**Seed entries (starting points — verify and expand):**

| # | Source | Type | Date | URL | Note |
|---|---|---|---|---|---|
| S1 | Profit / Pakistan Today — "National Grid Company begins restructuring" | Press | May 2025 | https://profit.pakistantoday.com.pk/2025/05/07/national-grid-company-begins-restructuring-under-new-framework/ | Confirms NTDC → NGC unbundling (NGC / EIDMC / ISMO). |
| S2 | Wikipedia — National Transmission & Despatch Company | Reference | 2025 | https://en.wikipedia.org/wiki/National_Transmission_%26_Despatch_Company | NGC owns 220/500 kV assets; network size figures — verify against NGC primary source. |
| S3 | National Grid SA (SEC) — official site | Primary | — | https://www.se.com.sa/en/Whoweare/National-Grid-SA/Introduction/ | NGSA = SEC's wholly-owned transmission operator. |
| S4 | Wikipedia — Saudi Electricity Company / "Saudi Energy" | Reference | Jun 2026 | https://en.wikipedia.org/wiki/Saudi_Electricity_Company | SEC rebranded to "Saudi Energy (SE)" in early 2026 — confirm current entity naming before attribution. |

---

## Notes carried over from the original task

- Local end-user context still matters: NGC (transmission), the DISCOs and K-Electric (distribution) — pin down who inspects their towers today and how (in-house vs contracted).
- If an operator/utility contact can be arranged, use it to validate the personnel picture (WP 2.3–2.4); until then, build from public primary sources (careers pages, tenders, asset-management plans, safety manuals) and flag `[ASSUMPTION — unverified]` where interview confirmation is still needed.
- The competitor/vendor landscape from the previous version is **out of scope here** except where a vendor is named as a grid company's contractor (WP 2.5). Keep that analysis in a separate task if still wanted.
