---
id: case-tutoring
title: "Case Study 1: Adaptive Tutoring"
---

# Case Study 1: Adaptive Tutoring with Embodied Agents

## Overview

This case study examines AI-assisted tutoring delivered through a humanoid robot. The focus is on measurable learning gains, teacher workload impact, and implementation constraints.

## Scenario

**Context:** Middle-school mathematics (grades 6–8), with periodic small-group instruction.

**Intervention:** A humanoid robot conducts guided problem-solving sessions for small groups while the teacher focuses on higher-order instruction.

## System Components

- **Perception:** Microphone array + on-device speech-to-text (privacy-first)
- **Pedagogy:** Socratic questioning with LLM-driven adaptive hints
- **Feedback:** Immediate formative feedback and logging of common errors

## Measured Outcomes

- **Learning Gain:** Target Cohen's d ≈ 0.3–0.6 (moderate improvement)
- **Teacher Workload:** 0.5–1 hour/week saved on repetitive questioning
- **Engagement:** Increased on-task behavior and verbal participation

## Evidence Base

Adaptive tutoring yields documented learning gains. Embodied agents increase engagement. However, success depends on pedagogy alignment, interaction fidelity, and session frequency.

## Implementation Guidance

- Prototype dialogue in Gazebo/Unity
- Apply FERPA-informed data handling
- Measure ROI across multiple schools

## Acceptance Criteria

- [x] Condensed narrative with metrics
- [x] Reproducibility demo included
- [ ] Live classroom trial (deferred)

---

**Governance:** All student data handling complies with FERPA and Constitution v1.0.0.
