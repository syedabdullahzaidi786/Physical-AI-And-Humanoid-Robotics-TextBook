---
id: paper-001
title: Physical AI & Humanoid Robotics — Research Paper Specification
version: 0.1.0
date: 2025-12-05
authors:
  - Project Team
status: draft
---

# Specification: Research Paper — Physical AI And Humanoid Robotics

Purpose
- Produce a well-scoped, evidence-based research paper synthesizing findings about AI systems in physical K-12 classrooms (especially humanoid robots), with clear guidance for education administrators on workload, student outcomes, ROI, safety, and implementation.

Target audience
- Education administrators, school leaders, district IT/ops leads, and researchers with non-expert familiarity with AI and robotics. Paper MUST include an executive summary and short glossary for non-technical readers.

High-level scope
- Include embodied/physical AI (humanoid robots and simulators), digital AI (tutoring systems, analytics) and VLA/multimodal pipelines (speech→LLM→ROS actions).
- Emphasize simulation-first evidence where appropriate (Gazebo/Unity/NVIDIA Isaac) and document sim-to-real safety considerations.

Definitions & Decisions (clarifying ambiguous terms)
- "Well-researched": minimum 12 peer-reviewed sources; preferred target 15–25. Composition: at least 1 meta-analysis/systematic review, 4–8 empirical studies (preferably RCTs/quasi-experimental), 3–5 robotics/embodiment studies, remainder policy/standards/reviews. Publication timeframe: prefer 2015–2025, allow up to 2 foundational works older than 2015 with justification.
- "Concrete AI applications": include software-only, hardware/embodied, and simulation implementations. Each application description must state: type (software/hardware/sim), components, deployment context (lab/sim/field), measurable outcomes, and primary source(s).
- "Teacher workload reduction" and "student outcome improvement": require BOTH quantitative and qualitative reporting.
  - Quantitative metrics: hours/week, % time saved, effect size (Cohen's d), % improvement on standardized measures, attendance/behavior incident counts, accuracy metrics for operational systems.
  - Qualitative metrics: validated teacher stress scales, interviews, focus groups, and user acceptance surveys.

Citation style & artifacts
- Use APA 7th Edition for in-text citations and reference list.
- Maintain `research/ai-classroom/references.bib` as the canonical BibTeX file.
- Provide DOI/URL for every source and include a claim→source mapping table in `research/ai-classroom/APA-References.md`.

Audience guidance
- Provide an Executive Summary (300–400 words) targeted to administrators, a one-page ROI snapshot, and a 250–400-word plain-language glossary/technical appendix.

Paper structure & word budget (main manuscript)
- Target total: 3,500–5,000 words (exclude appendices and raw data). Suggested allocation:
  - Executive summary & abstract: 300–400 words
  - 1. Introduction & scope (incl. short historical context): 400–600 words
  - 2. Evidence base & related work (search + selection): 600–900 words
  - 3. Methods (search strategy, inclusion/exclusion): 300–500 words
  - 4. Case studies (Tutoring, Behavioral, Accessibility, Operational): 500–1,000 words each (allow longer annexes; produce 500–1000-word condensed versions for main text)
  - 5. ROI & cost-benefit analysis (scenarios + tables): 600–900 words
  - 6. Safety, compliance & ethical considerations: 400–600 words
  - 7. Discussion (synthesis, limitations, policy implications): 600–900 words
  - 8. Conclusion & recommendations: 200–350 words
  - Appendices: simulation artifacts (URDF/SDF), full long-form case studies (if needed), raw data links

Required deliverables
- `specs/paper/spec.md` (this file)
- Manuscript: `research/ai-classroom/AI-Assisted-Classroom-Robotics-Research.md` (main document; updated)
- Condensed case studies in `research/ai-classroom/cases/` (`tutoring.md`, `behavioral.md`, `accessibility.md`, `operational.md`) with 500–1000 words each
- `research/ai-classroom/references.bib` and `APA-References.md` with claim mapping
- Simulation artifacts (if any experimental claims): `simulation/urdf/*`, `simulation/sdf/*`, `simulation/ros2_ws/*`
- PHR files per phase: `history/prompts/ai-classroom/00*-phase*.spec.prompt.md`

Research methodology & inclusion criteria
- Databases: Google Scholar, ERIC, ACM Digital Library, IEEE Xplore, Scopus (at minimum).
- Search strategy: provide keywords and boolean expressions in `research/ai-classroom/search-strategy.md` and record screening funnel (initial → screened → reviewed → selected).
- Inclusion criteria: peer-reviewed, K-12 context, quantified outcomes or reproducible simulation results, English-language (note any non-English exceptions).
- Exclusion criteria: vendor marketing materials, product-only whitepapers (unless peer-reviewed evaluation attached).

Handling conflicting evidence
- Rate each source for quality: Peer-review status, sample size, study design (RCT>quasi>observational), metrics clarity, recency.
- Use weighted synthesis: give more weight to higher-quality studies in narrative and in any aggregated metrics; when conflict persists, present both sides and identify research gaps.

Mandatory tables & figures
- Source inventory table: (ID, citation, year, study type, sample size, key finding, DOI)
- Metrics table: per application domain (learning gain, time saved, effect size, CI)
- ROI scenarios table: assumptions, costs, savings, payback period
- Safety checklist table: standards (ISO/IEC, FERPA, ADA), required actions
- Architecture diagram: VLA (Whisper → GPT → Mapper → ROS2 actions) and simulation-to-real safety gates

Data sharing & reproducibility
- For any new analyses or simulations, include: raw/anonymized CSVs, code snippets/notebooks, URDF/SDF, launch files, and step-by-step reproducibility instructions in `simulation/tests/reproducibility.md`.
- State any IRB/ethics approvals or justify exemption for secondary analyses.

TDD-style validation & acceptance tests (must be recorded as PHR entries)
- Source validation tests: each selected source must pass checks for peer-review, relevance, recency, sample size, and accessibility.
- Claim validation tests: each claim in the manuscript must list supporting source(s) and a test (e.g., reproduce reported effect size extraction or simulate a scenario).
- Simulation reproducibility tests: smoke-launch, URDF parse, sensor topic presence, repeatability across runs (see `simulation/tests/reproducibility.md`).

Review & publication workflow
- Drafts: minimum 2 internal drafts (author review & governance review) + 1 external peer review (educator or robotics researcher) before finalization.
- Track reviewer comments in `history/prompts/` as PHRs and update ADRs if governance changes are needed.

Acceptance criteria (pass/fail)
- At least 12 peer-reviewed sources, all accessible with DOI/URL.
- All major claims traceable to source(s) with claim→source mapping in `APA-References.md`.
- Case studies present measurable metrics and ROI scenarios with transparent assumptions.
- Safety & compliance checklist completed for any sim-to-real claims.
- Reproducibility smoke tests pass for any included simulations.

Governance alignment
- All activities and artifacts must comply with Physical AI & Humanoid Robotics Constitution v1.0.0 (TDD, transparency, modularity, safety). Document compliance in each PHR.

Timeline (suggested)
- Week 0–1: Finalize spec (this file) and confirm source list
- Week 1–2: Write case studies and update manuscript draft
- Week 2–3: ROI analysis, safety section, and simulation reproducibility testing
- Week 3–4: Internal review, external peer review request, revisions
- Week 4+: Finalize manuscript and prepare publication package

PHR locations
- Phase 0: `history/prompts/ai-classroom/002-phase0-research-complete.spec.prompt.md` (existing)
- Phase 1: `history/prompts/ai-classroom/004-phase1-complete.spec.prompt.md` (existing)
- Phase 2: `history/prompts/ai-classroom/005-phase2-scaffolding.spec.prompt.md` (created)
- Create Phase 3/4 PHRs as phases progress in `history/prompts/ai-classroom/`

Open decisions for the authoring team (require confirmation)
1. Final target total word count (choose between 3.5–5k or a narrower journal target).
2. Exact source target (minimum 12 vs preferred 15–25).
3. Whether long-form case studies remain in main text or moved to appendices with 500–1000-word main-text summaries.
4. External peer reviewer selection criteria (educator vs robotics researcher priority).

Next actions (after confirmation of open decisions)
1. Lock `specs/paper/spec.md` (this file) and create a detailed paper outline with per-section source mapping.
2. Generate condensed 500–1000-word case study variants and store them in `research/ai-classroom/cases/`.
3. Implement simulation reproducibility tests in CI and validate locally.

---
End of specification draft (v0.1.0)
