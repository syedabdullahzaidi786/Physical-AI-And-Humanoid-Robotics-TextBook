---
id: paper-plan-001
title: Physical AI & Humanoid Robotics Hackathon — Project Plan
version: 0.1.0
date: 2025-12-05
status: draft
---

# Project Plan — Physical AI & Humanoid Robotics Hackathon

Purpose
- Bridge digital AI and physical humanoid robots by having students simulate, control, and evaluate humanoid robots (Gazebo, Unity, NVIDIA Isaac) integrated with ROS 2, VLA (Vision-Language-Action) pipelines, and GPT-based conversational AI.

High-level Phase-based Workflow
- Phase 0 — Foundation Research
  - Goal: Establish evidence base, choose instruments and simulation platforms, and produce Phase 0 PHR.
  - Key artifacts: `research/ai-classroom/*` (master research doc, search strategy, references.bib), `history/prompts/ai-classroom/002-phase0-research-complete.spec.prompt.md`.

- Phase 1 — Case Studies
  - Goal: Produce four evidence-based case studies (Tutoring, Behavioral, Accessibility, Operational) and condensed variants for main manuscript.
  - Key artifacts: `research/ai-classroom/cases/*.md`, PHR: `history/prompts/ai-classroom/004-phase1-complete.spec.prompt.md`.

- Phase 2 — Robot Simulation
  - Goal: Produce simulation-ready robot models, ROS 2 packages, and reproducible launch scenarios.
  - Map modules: URDF/XACRO, SDF, `simulation/ros2_ws` (ROS 2 nodes), Gazebo/ignition/Unity scenes.
  - Key artifacts: `simulation/urdf/*`, `simulation/sdf/*`, `simulation/ros2_ws/*`, `simulation/tests/reproducibility.md`, PHR: `history/prompts/ai-classroom/005-phase2-scaffolding.spec.prompt.md`.

- Phase 3 — Analysis & AI Integration
  - Goal: Integrate VLA pipelines, GPT conversational agents, perception stacks (Isaac/VSLAM), and run experiments measuring teacher workload and student outcomes.
  - Map modules: NVIDIA Isaac (perception & learning), ROS 2 bridges, VLA mapping components.
  - Key artifacts: experiment scripts, notebooks, `simulation/analysis/` outputs, PHRs for Phase 3.

- Capstone — Field Deployments & Synthesis
  - Goal: Consolidate findings, run small-scale field pilots (if permitted), finalize manuscript, and prepare publication package.
  - Key artifacts: final manuscript, ROI tables, reproducibility archive, external peer-review packet.

Module-to-Phase Mapping
- ROS 2: Phase 2 (simulation runtime + node orchestration), Phase 3 (experiment orchestration and data collection).
- Gazebo/Unity: Phase 2 (robot physics & sensors). Unity optional for advanced visualization / curriculum demos.
- NVIDIA Isaac: Phase 3 (perception, training pipelines, sim-to-real experiments).
- VLA (Vision-Language-Action): Phase 3 (mapping perception outputs to LLM-driven action policies and conversational agents).

Section Structure for Project Report / Research Paper
- Executive Summary (300–400 words)
- 1. Introduction (Physical AI, embodied intelligence, educational relevance) — 400–600 words
- 2. Background (ROS 2, Gazebo, Isaac, VLA, prior work) — 600–900 words
- 3. Case Studies (Tutoring, Behavioral, Accessibility, Operational) — 500–1,000 words each (main text: 500–800; full versions in appendix)
- 4. Simulation Implementation (URDF/SDF, ROS 2 nodes, sensors, launch) — 600–900 words + diagrams
- 5. Analysis & Synthesis (ROI, teacher workload, student outcomes) — 600–900 words
- 6. Safety, Compliance & Ethics — 400–600 words
- 7. Discussion & Limitations — 600–900 words
- 8. Conclusion & Recommendations — 200–350 words
- References & Appendices (APA formatted, source validation, full case studies, simulation artifacts)

Per-section word targets & citation targets (quick table)
- Executive Summary: 300–400 words; cite 3–5 high-level sources.
- Intro: 400–600 words; cite 4–6 sources.
- Background: 600–900 words; cite 8–12 sources (ROS2/Gazebo/Isaac literature + standards).
- Case Studies total: 2,000–4,000 words (500–1,000 ea); each should cite 3–8 domain-specific sources.
- Simulation Implementation: 600–900 words; include code links and cite 3–6 technical references.
- Analysis & Synthesis: 600–900 words; cite empirical studies and include ROI calculations from 4–8 sources.
- Safety & Ethics: 400–600 words; cite policy/standards references (FERPA, ADA, ISO/IEC) and 2–4 ethics papers.

Phase-wise Tasks & Milestones (detailed)
- Phase 0 — Foundation Research (Milestone: Phase 0 PHR + research package)
  - Task 0.1: Finalize search strategy and collect candidate papers (timeline: 3 days)
  - Task 0.2: Screen papers, produce `selected-sources.md` and update `references.bib` (timeline: 3 days)
  - Task 0.3: Produce master research doc and claim→source mapping (timeline: 4 days)
  - Acceptance: 12+ peer-reviewed sources validated; PHR created.

- Phase 1 — Case Studies (Milestone: Four condensed case studies + PHR)
  - Task 1.1: Draft long-form case studies (timeline: 5 days)
  - Task 1.2: Produce condensed 500–1000-word summaries (timeline: 2 days)
  - Task 1.3: Map each case to evidence and ROI scenarios (timeline: 2 days)
  - Acceptance: Each case cites 3+ peer-reviewed sources and includes measurable metrics.

- Phase 2 — Robot Simulation (Milestone: Reproducible simulation runs)
  - Task 2.1: Finalize robot xacro/URDF templates and SDF scenes (timeline: 4 days)
  - Task 2.2: Add sensor links/plugins (camera, IMU, depth/LiDAR) and example configurations (timeline: 3 days)
  - Task 2.3: Create ROS 2 launch files and Gazebo spawn routines (timeline: 3 days)
  - Task 2.4: Write reproducibility smoke tests (URDF parse, launch, topics) and run locally (timeline: 2 days)
  - Acceptance: URDF parses, `robot_state_publisher` runs, camera and IMU topics publish in a smoke run.

- Phase 3 — Analysis & AI Integration (Milestone: Experiment artifacts + analysis notebooks)
  - Task 3.1: Integrate perception stacks (Isaac/VSLAM) and VLA pipeline prototypes (timeline: 7 days)
  - Task 3.2: Implement GPT conversational agent with safe action mapping and logging (timeline: 5 days)
  - Task 3.3: Run experiments, collect metrics on workload and student outcome proxies in simulation (timeline: 7–10 days)
  - Acceptance: Reproducible experiment notebooks and processed metrics with traceable sources.

- Capstone — Field Deployments & Synthesis (Milestone: Final manuscript)
  - Task 4.1: Prepare ROI tables and final manuscript draft (timeline: 5 days)
  - Task 4.2: Internal governance review + external peer review request (timeline: 7–14 days)
  - Acceptance: Manuscript meets acceptance criteria from `specs/paper/spec.md` and PHRs document reviews.

Decision Logs & Trade-offs (templates)
- Decision Log Entry: [id, date, decision, options considered, rationale, owner, PHR link]
- Example decisions to document:
  - Simulator choice: Gazebo vs Unity vs Isaac sim (trade-offs: fidelity vs accessibility vs licensing)
  - Edge device selection: Jetson family vs CPU cloud instances (trade-offs: cost, latency, power)
  - LLM selection: closed-source vs open-source (trade-offs: latency, privacy, cost)

Quality Validation / Testing (QA checkpoints)
- Source validation checks (for each source): peer-review status, sample size, study design, recency, DOI/URL.
- Claim traceability: Each factual claim must point to a source id in `APA-References.md`.
- Word count checks: enforce per-section targets; each case study 500–1000 words in condensed form.
- Simulation smoke tests: URDF parse, `robot_state_publisher` up, topics `/camera/image_raw`, `/imu/data`, `/tf` available, repeatability across 3 runs.
- Reproducibility CI: run smoke tests on push (GitHub Actions or similar); optional containerized simulator jobs for headless tests.

Research Approach & Documentation
- Research-concurrent writing: draft while collecting sources; maintain `research/ai-classroom/search-strategy.md` with search terms and screening funnel.
- Store claim→source mappings in `research/ai-classroom/APA-References.md`.
- Keep `research/ai-classroom/research-notes.md` for ongoing notes and `history/prompts/ai-classroom/` for PHRs.

Technical Details & Infra Notes
- Toolchain: ROS 2 (Humble or later), Python 3.10+, Gazebo/ignition (or Unity), NVIDIA Isaac optional for Phase 3.
- Hardware: development laptop/desktop with GPU for Isaac/Unity optionally; Jetson Orin/Xavier or Nano for edge demos; recommend at least 16GB RAM and GPU with CUDA for Isaac experiments.
- Local vs Cloud: Local for student labs and sensor latency; cloud for heavy model training or shared CI simulation runs. Document required dependencies and container images in `simulation/README.md`.

Validation & QA Checkpoints (summary)
- Pre-Phase 1: At least 12 validated peer-reviewed sources.
- Pre-Phase 2: URDF/XACRO and SDF parse cleanly; initial launch smoke test passes.
- Post-Phase 3: Perception pipeline publishes required topics and VLA prototype completes closed-loop action traces in simulation.

PHR & Tracking
- Record PHRs for each milestone at `history/prompts/ai-classroom/` and link from this plan. Keep decision logs and reviewer comments in PHR entries.

Next Immediate Steps (week 0–1)
1. Confirm this plan (this file) and lock spec decisions or record open decisions as PHRs.
2. Finalize search keywords and run Phase 0 searches; produce `selected-sources.md` and update `references.bib`.
3. Draft condensed case studies (500–1000 words) and place them in `research/ai-classroom/cases/`.

Contact points & ownership (suggested)
- Research lead: @research-lead
- Simulation lead: @simulation-lead
- AI integration lead: @ai-lead
- QA/Governance: @governance-lead

---
End of plan (v0.1.0)
