# Physical AI & Humanoid Robotics Constitution

A comprehensive guide, framework, and research platform on humanoid robots integrating AI, simulation, and real-world testing. This constitution defines the non-negotiable principles and practices that govern all development, research, and contributions to this project.

## Core Principles

### I. Hardware-Software Integration

All components MUST be designed with bidirectional hardware-software awareness. Simulation code must validate against physical constraints; hardware implementations must be model-driven from tested simulations. Abstractions MUST expose both simulation and hardware interfaces without loss of fidelity. Integration testing is mandatory before deployment to physical systems.

**Rationale**: Humanoid robotics inherently couples physical dynamics with control software. Decoupling these leads to sim-to-real transfer failures and safety risks. Tight integration ensures reliability and reproducibility.

### II. Safety & Compliance

Safety is non-negotiable. All robot control systems MUST include:
- Failsafe mechanisms (emergency stop, torque limits, velocity bounds)
- Pre-deployment safety validation (simulation, hardware-in-loop, controlled environments)
- Documentation of hazards and mitigations for each module
- Compliance with applicable robotics standards (ISO/IEC, ROS2 best practices)

Untested or unsafe code MUST NOT be merged to production branches. Deployments to physical robots require explicit approval from the project lead.

**Rationale**: Humanoid robots operating in shared spaces pose safety risks to humans and equipment. Compliance and failsafes are foundational to responsible research and operation.

### III. Modularity & Reusability

Every major component (perception, planning, control, simulation) MUST be developed as a standalone, independently testable module with:
- Clear API contracts (inputs, outputs, errors)
- No organizational-only dependencies
- Documented purpose and usage examples
- Minimal coupling to other modules

Modules are composable via ROS2 architecture; breaking changes require major version bumps and ADR documentation.

**Rationale**: Modularity enables reuse across projects, simplifies testing, and accelerates iteration. It supports both simulation and hardware workflows seamlessly.

### IV. Transparency & Documentation

All design decisions, code, and research MUST be documented in Markdown at the point of origin:
- Architecture: ADRs (Architecture Decision Records) in `history/adr/`
- Features: Specifications in `specs/<feature>/spec.md`, plans in `specs/<feature>/plan.md`
- Code: Inline comments for non-obvious logic; docstrings for all functions
- Operations: Runbooks and deployment guides for critical systems

Documentation MUST be kept current; stale docs are removed or flagged as deprecated.

**Rationale**: Transparency enables trust, accelerates onboarding, and supports reproducible research. Public documentation (where applicable) aligns with open science principles.

### V. Open Science Principles

Research findings, datasets, and methodologies MUST be published openly whenever possible:
- Code repositories are public (with secrets managed via `.env` and configuration)
- Datasets are licensed and archived (with provenance tracked)
- Benchmark results are reproducible and version-tracked
- Contributions and contributors are acknowledged

Proprietary or confidential components are isolated, clearly marked, and minimally scoped.

**Rationale**: Open science accelerates innovation, enables external validation, and supports the broader robotics and AI research community. Transparency builds trust.

## Technology Stack & Standards

**Core Stack**:
- Python 3.10+ (primary development language)
- ROS2 (middleware for robot control and simulation)
- FastAPI (REST API services)
- PostgreSQL (persistent data storage)
- Streamlit (UI dashboards and demos)
- Gemini AI (LLM-assisted reasoning and planning)

**Development Standards**:
- Test-first approach (TDD): Tests written → User approved → Tests fail → Implementation → Tests pass
- Type hints required (Python type annotations for all functions)
- Code coverage minimum: 80% for production code
- Linting: black, isort, pylint on all Python code
- CI/CD: Automated tests on all PRs before merge

**Documentation Standards**:
- Markdown format for all architectural and feature documentation
- Docstrings: Google-style format for all public APIs
- README files in every module directory
- CHANGELOG maintained per semantic versioning

## Development Workflow

1. **Specification Phase**: Define feature requirements in `specs/<feature>/spec.md` using the spec template.
2. **Planning Phase**: Create architecture decisions and task breakdown in `specs/<feature>/plan.md` using the plan template.
3. **Task Breakdown**: Generate testable tasks in `specs/<feature>/tasks.md` using the tasks template.
4. **Red-Green-Refactor**: Write failing tests → Implement → Pass tests → Refactor. Repeat for each task.
5. **Integration Testing**: Validate against other modules (especially hardware-software integration).
6. **Review & Approval**: Code review, safety validation (if hardware), and merge approval.
7. **Documentation**: Update constitution and templates if principles or practices change. Create ADRs for significant decisions.

## Quality Gates

- All code MUST pass tests before merge
- All PRs MUST include: failing tests, implementation, passing tests, documentation updates
- Safety-critical code requires additional review and hardware validation
- Breaking changes require ADR and major version bump
- No hardcoded secrets; all sensitive data via `.env` and configuration management

## Governance

**Amendment Procedure**:
- Major changes to principles or practices require an ADR (Architecture Decision Record)
- ADRs are proposed, discussed, and approved by the project lead
- Amendments are ratified by the project lead with documented rationale
- All amendments are tracked in this constitution with dates and version bumps

**Versioning Policy**:
- MAJOR version: Backward-incompatible principle removal/redefinition, governance changes
- MINOR version: New principle, expanded guidance, significant practice addition
- PATCH version: Clarifications, wording fixes, non-semantic refinements
- Constitution version must be updated when any section changes

**Compliance Review**:
- All PRs are validated against this constitution during code review
- Exceptions require documented justification and project lead approval
- Quarterly review of constitution effectiveness and suggested amendments

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
