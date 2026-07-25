# Task 2 — Culture Identification

**Executor:** This task is run by Claude (Research feature). Every finding must be traceable to a named source — no claim, figure, role, or pain point may appear in any deliverable without a citation. Follow the Sourcing & Verification Protocol below on every work package.

**Objective:** Identify *for whom* the project solves pain points — i.e. map every human role that touches a transmission-tower inspection today (across manual-piloted, autonomous, helicopter, and ground/climbing modalities), profile those roles, and pin down which of their pains our autonomous system removes, reduces, or leaves untouched.

**Why this task exists:** Requirements must be derived from real users, not from the design. Until we know who operates, oversees, buys, and benefits from an inspection — and what they suffer today — we cannot prioritise design effort or justify the project's value.

**Framing principle (analytical lever):** For each role and each competitor, note what they *cannot* do or what *costs them* today. Those gaps become the contribution levers for our system.

**Definitions used below:**
- **Provider** = the entity performing the inspection (drone-service company, in-house utility crew, OEM).
- **End user / customer** = the entity commissioning the inspection (utility, transmission/distribution operator).
- **Role** = a specific human function in the pipeline (pilot, field engineer, analyst, etc.).
- **Persona** = a profiled role with responsibilities, cost, and pains.

---

## Execution Setup

1. **Tool:** Run in the Claude web/desktop app with the **Research** feature enabled (toggle Web Search on first, then Research). This is preferred over Claude Code because the deliverable is a cited research document, not a coding task, and Research links every claim to a source by default.
2. **Model:** Use **Opus 4.8** for synthesis-heavy passes (WP 2.3–2.7). If usage limits tighten on the Team plan, gather raw sources (WP 2.1–2.2, 2.5) on **Sonnet 5**, then switch to Opus 4.8 for the final synthesis.
3. **Agent constraint (usage limit):** Run each work package as a **single controlled research pass**. Do **not** spawn multiple parallel research sub-agents. Parallel/recursive agents multiply token consumption and can exhaust the Team-plan allowance quickly. If a work package is large, split it into sequential passes within one session rather than fanning out concurrently.
4. **Scope discipline:** Complete one work package fully, verify its sources, then move to the next. Do not begin synthesis (WP 2.7) until the underlying WPs are source-complete.

---

## Sourcing & Verification Protocol (applies to every work package)

1. **No uncited claims.** Every fact, figure, role, cost, and pain point in a deliverable must carry a citation: source name + direct URL. If a statement has no source, mark it `[ASSUMPTION — unverified]` and keep it visually separate from sourced findings.
2. **Confidence tiering.** Tag each finding as one of:
   - `[WELL-SOURCED]` — corroborated by two or more independent sources.
   - `[SINGLE-SOURCE]` — one source only; flag for later verification.
   - `[THIN/CONTESTED]` — weak, dated, or contradicted evidence; state the contradiction.
3. **Prefer primary sources.** Rank in this order: company's own site/docs/webinars/repos > regulatory or standards bodies > market-research firms (Mordor, Fact.MR, MarketIntelo) > news/trade press > blogs/forums. Do not cite a blog for a claim a primary source can support.
4. **Record provenance metadata** for each source: publisher, author (if any), publication/update date, date accessed, and URL. Flag anything older than ~2 years as potentially stale.
5. **Quote sparingly, paraphrase by default.** Capture findings in our own words with the citation; reserve short verbatim quotes only where exact wording carries meaning (e.g. a regulatory clause).
6. **Flag contradictions explicitly.** When sources disagree (e.g. crew size, cost-per-tower), record both figures with their sources rather than silently picking one.
7. **Separate provider claims from independent evidence.** A vendor's marketing page is evidence of what they *claim*, not proof it is true; label it as such.
8. **Deliverable format.** Every table gets a trailing `Source(s)` column; every narrative finding gets an inline citation. Maintain one consolidated **Source Register** at the end of the document listing every source used, numbered, so claims can be audited.

---

## WP 2.1 — Provider & Inspection-Modality Landscape

1. Compile a list of companies performing transmission-tower inspection, tagged by modality:
   - Manual-piloted drone (pilot flies, manual framing of shots)
   - Semi-/fully-autonomous drone (waypoint or mission-planned, minimal pilot input)
   - Helicopter-based (manned aerial)
   - Ground crew / tower climbing (foot patrol, binoculars, climbers)
   - Robotic / line-crawling
2. For each company, record: modality, level of autonomy, target customer type (transmission vs distribution operator), geography, and whether they sell a service, a platform, or both.
3. Classify each provider as competitor, adjacent, or reference (e.g. AirPelago, Hepta, AirDash, Cyberhawk, and local Pakistan operators).
4. Identify the local end-user context specifically: NTDC (transmission), the DISCOs and K-Electric (distribution) — who inspects their towers today and how (in-house vs contracted).
5. **Deliverable:** provider matrix (rows = companies, columns = modality / autonomy / customer / service model / **Source(s) + confidence tier**). Every row's classification must cite where it came from.

---

## WP 2.2 — Pipeline & Role Decomposition (the "who is in the loop")

*Do this once per modality. The point is to enumerate every human that touches the workflow, stage by stage.*

1. Draw the end-to-end pipeline for each modality across these stages: mission planning → site/weather clearance → flight & data capture → data offload → anomaly detection → report generation → expert verification → maintenance dispatch.
2. At each stage, list every human role involved and what they physically do.
3. For **manual-piloted drone**, resolve the key questions:
   - Is the pilot also the inspection engineer, or is a separate senior engineer on site directing shots?
   - Who decides which components to photograph and when the tower is "done"?
   - Who does anomaly detection — a person offline, on site, or an AI tool?
   - How many people per crew, and what is each one's certification/skill level?
4. For **autonomous drone**, resolve:
   - Who launches and supervises the mission (single operator? GCS team?)
   - Is there a backup/standby pilot for manual override (note: AirPelago appears to keep trained pilots as backup)?
   - Who monitors telemetry / handles emergencies?
   - Who reviews and signs off the AI-generated report?
5. For **helicopter and ground/climbing**, list crew composition, safety personnel, and outage/permit coordinators.
6. Tag each role per stage as: **eliminated**, **reduced**, **retained**, or **newly introduced** under an autonomous system like ours.
7. **Deliverable:** one pipeline diagram + role-per-stage table per modality. Each role entry must cite the source establishing that the role exists in that modality (e.g. company webinar, case study, job posting, industry guide); label vendor-claimed roles as claims, not verified fact.

---

## WP 2.3 — Stakeholder & Persona Profiling

1. Consolidate the roles from WP 2.2 into a deduplicated master role list.
2. Separate roles into three stakeholder classes:
   - **Operators** — hands-on the inspection (pilot, engineer, analyst)
   - **Decision-makers / buyers** — commission and pay (asset manager, procurement, safety head)
   - **Beneficiaries** — depend on the output (maintenance crew, grid-reliability/planning teams, regulators)
3. Build a persona card per key role capturing: primary responsibility, required skill/certification, time spent per tower/line, approximate cost to employer, and their single biggest frustration today.
4. Rank the personas by how central they are to the buying decision and to daily operation.
5. **Deliverable:** persona cards + a stakeholder map (operator / buyer / beneficiary). Every attribute on a card (skill, cost, time-per-tower, frustration) must cite a source or be tagged `[ASSUMPTION — unverified]` for validation in WP 2.6.

---

## WP 2.4 — Pain-Point Analysis (per role)

1. For each persona, enumerate current pain points under the **manual** process (e.g. field engineer's time on site, pilot fatigue, safety exposure at height, subjective shot framing, slow report turnaround, missed defects at distance).
2. For each persona, enumerate pain points that persist even under **existing automated** systems (e.g. selective hotspot imaging misses edge-case anomalies, dependence on prior GPS mapping, no coverage guarantee, offline processing latency).
3. Map each pain point to how our system addresses it — remove / reduce / no change — and state the mechanism (e.g. isocontour coverage removes "which parts were missed"; onboard detection reduces report latency).
4. Explicitly flag pains our system does **not** solve, or new burdens it introduces (e.g. new GCS operator skill, RTK dependency, verification-expert still required).
5. **Deliverable:** pain-point → solution matrix (persona × pain × our mechanism × solved/reduced/unsolved/new). Each *pain* must cite its source; each *our-mechanism* claim references the project design doc rather than an external source.

---

## WP 2.5 — Competitor Culture & Positioning

1. For each competitor (AirPelago, Hepta, AirDash, Cyberhawk, others), document their operating model: on-site pilot vs fixed GCS, service vs platform, backup-pilot policy, onboard vs offline anomaly detection.
2. Identify which stakeholder each competitor primarily sells to and which roles their offering assumes the customer still staffs.
3. Extract the roles/pains each competitor leaves *unaddressed* — these are the contribution levers (e.g. selective hotspot models leave "full-coverage guarantee" unaddressed; GPS-dependent pipelines leave GPS-denied sites unaddressed).
4. Cross-check competitor logic against any public repos (e.g. github.com/airpelago) to validate assumptions about their pipeline.
5. **Deliverable:** competitor positioning table + unaddressed-gap list mapped to our differentiators. Each competitor claim must cite the specific page/webinar/repo; distinguish what the competitor *states* from what independent sources *confirm*.

---


---

## WP 2.7 — Synthesis & Output

1. Produce a single "for whom are we solving pain points" statement naming the primary persona(s) and the top pains addressed.
2. Rank personas into primary (drives requirements) and secondary (nice-to-have) targets.
3. Hand off the primary persona set and the pain-point → solution matrix to the requirements-definition phase so design priorities can be justified against real users.
4. Confirm every claim carried into the summary is cited and confidence-tiered; the summary must contain no `[ASSUMPTION — unverified]` items left unresolved.
5. **Deliverable:** one-page culture summary feeding directly into user requirements, plus the consolidated Source Register.

---

## Source Register (maintain throughout)

A running, numbered list of every source used, appended to the document. Minimum columns:

| # | Source (publisher / author) | Type (primary / market-research / press / blog) | Date published | Date accessed | URL | Used in WP(s) |
|---|---|---|---|---|---|---|

Every citation elsewhere in the document references a number in this register, so any claim can be audited back to its origin.

---

## Cross-references


-start WP 2.1–2.5 in parallel from public sources while the operator contact is being arranged.
