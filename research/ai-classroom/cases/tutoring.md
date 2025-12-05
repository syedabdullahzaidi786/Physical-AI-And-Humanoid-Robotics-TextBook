Title: Tutoring Case Study (Condensed)

Overview
This case study examines AI-assisted tutoring delivered through an embodied agent (humanoid robot or simulated avatar) and compares it to software-only tutoring tools. The focus is on measurable learning gains, teacher workload impact, and implementation constraints relevant to K–12 classrooms.

Key scenario
- Context: Middle-school mathematics (grades 6–8), mixed-ability classroom with periodic small-group instruction.
- Intervention: A humanoid robot (simulated for curriculum testing) conducts guided problem-solving sessions for small groups while the teacher circulates and focuses on higher-order instruction.

Mechanism and components
- Perception: microphone array + on-device speech-to-text; optional camera for gesture detection (privacy-first configuration: on-device processing, no cloud audio storage).
- Pedagogy: Socratic questioning scaffolded by scripted prompts and adaptive hints driven by an LLM policy tuned on curriculum-specific prompts.
- Feedback: Immediate formative feedback on problem steps, hints calibrated to student proficiency, and logging of common error types for teacher review.

Measured outcomes (example metrics)
- Learning gain: Pre/post standardized test effect size (Cohen's d) — target moderate improvement (d ≈ 0.3–0.6) for students receiving weekly 20–30 minute robot-facilitated sessions over 6–8 weeks.
- Time savings: Teacher preparation and intervention time redistributed — teachers report saving ~0.5–1 hour/week on repetitive question handling, allowing more targeted interventions.
- Engagement: Short-term increases in on-task behavior and verbal participation (classroom observation scales), sustained engagement requires novelty attenuation strategies.

Evidence & constraints
- Evidence indicates that adaptive tutoring yields measurable gains, and embodied agents can increase engagement beyond screen-only tutors. However, learning gains depend strongly on pedagogy alignment, fidelity of interaction, and frequency/duration of sessions.
- Key constraints: reliable speech recognition in classroom noise, curriculum alignment (question bank coverage), and teacher training for integrating robot logs into lesson planning.

Implementation guidance
- Simulation-first: prototype the dialogue and timing in Gazebo/Unity to validate turn-taking, latency, and multimodal prompts before physical deployment.
- Safety & privacy: apply FERPA-informed data handling; anonymize logs and store them under project governance rules.
- ROI snapshot: for a school of 500 students running 20 sessions per week, estimate instructor-hours saved, incremental cost of hardware amortized over 3 years, and projected effect on grade-level progress; include conservative sensitivity ranges.

Acceptance criteria for this case
- Produce a 500–1,000 word condensed narrative with source-backed claims.
- Provide pre/post test results (simulated or empirical) and a teacher workload metric with clear measurement method.
- Include a small reproducibility demo: a recorded simulation run or notebook that shows the tutoring loop and logs produced.

References
- See `research/ai-classroom/APA-References.md` for mapped sources supporting adaptive tutoring, embodied agent engagement, and on-device privacy practices.
