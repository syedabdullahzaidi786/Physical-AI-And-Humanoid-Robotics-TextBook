---
id: 001
title: Hackathon — Physical AI & Humanoid Robotics Full Workflow
stage: project
date: 2025-12-05
surface: agent
model: GPT-5 mini
feature: hackathon-workflow
branch: 001-hackathon-workflow
user: architect
command: /specify
labels: ["hackathon", "workflow", "phases", "governance"]
links:
  spec: .specify/memory/constitution.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/ai-classroom/spec.md
  - research/ai-classroom/AI-Assisted-Classroom-Robotics-Research.md
  - history/prompts/ai-classroom/002-phase0-research-complete.spec.prompt.md
  - history/prompts/ai-classroom/003-phase1-case-studies.spec.prompt.md
tests: []
---

## Prompt

/specify Hackathon: Physical AI & Humanoid Robotics – Full Workflow

Project context and phased plan recorded. This PHR documents the overall hackathon workflow, governance alignment to Constitution v1.0.0, and references Phase 0 completion and Phase 1 kickoff.

## Summary

- Theme: AI Systems in the Physical World, Embodied Intelligence
- Goal: Bridge digital brain & physical body; simulation-first approach with progressive sim-to-real validation
- Modules: ROS 2, Gazebo, Unity, NVIDIA Isaac, GPT/Whisper, Jetson Orin
- Governance: Adhere to Physical AI & Humanoid Robotics Constitution v1.0.0 (TDD, transparency, open science, safety)

## Immediate status (2025-12-05)

- Phase 0: COMPLETE (research folder and 12 sources validated; PHR: `history/prompts/ai-classroom/002-phase0-research-complete.spec.prompt.md`)
- Phase 1: IN-PROGRESS (case studies being authored; PHR to be created at `history/prompts/ai-classroom/003-phase1-case-studies.spec.prompt.md`)
- Phase 2–4: PLANNED (placeholders created; execution will follow acceptance gates)

## Acceptance Criteria (Workflow)
- Each phase must produce: research artifacts, TDD validation checks, governance compliance notes, APA references, and a Phase PHR
- Simulation reproducible on RTX 4070+ or cloud instance; Jetson Orin deployment artifacts prepared
- All documentation in Markdown; references in `research/ai-classroom/references.bib`

## Next actions
1. Continue Phase 1 case study completion and create Phase 1 PHR
2. Prepare simulation skeletons (URDF/SDF templates) for Phase 2
3. Draft Isaac perception pipeline plan for Phase 3
4. Draft conversational mapping plan (Whisper → GPT → ROS actions) for Phase 4

---

**PHR**: This file records the cross-phase workflow and is intended to be updated as phases complete. If any principle needs amendment, raise an ADR and update constitution accordingly.
