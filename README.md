# Physical AI and Humanoid Robotics Project

## Project Overview
This repository follows **Spec-Driven Development (SDD)** with Gemini CLI integration for expert-level AI-assisted development.

## Directory Structure

```
.
├── .specify/                    # SpecKit Plus configuration
│   ├── memory/
│   │   └── constitution.md      # Project principles and standards
│   ├── scripts/                 # Build and utility scripts
│   └── templates/               # SDD templates
│       ├── adr-template.md
│       ├── spec-template.md
│       ├── plan-template.md
│       ├── tasks-template.md
│       ├── phr-template.prompt.md
│       └── ...
├── .gemini/                     # Gemini CLI configuration
│   ├── settings.local.json      # API keys and settings
│   └── commands/                # Custom CLI commands
├── history/                     # Audit trail
│   ├── prompts/                 # Prompt History Records (PHRs)
│   │   ├── constitution/        # Constitution-related PHRs
│   │   └── general/             # General PHRs
│   └── adr/                     # Architecture Decision Records
├── specs/                       # Feature specifications
│   └── <feature-name>/
│       ├── spec.md              # Feature requirements
│       ├── plan.md              # Architecture decisions
│       └── tasks.md             # Testable tasks
├── GEMINI.md                    # CLI rules and workflow guidelines
└── README.md                    # This file
```

## Gemini Configuration Status ✓

### Completed Setup
- ✓ `.specify/` directory structure initialized
- ✓ `.gemini/settings.local.json` configured with API key
- ✓ Constitution template available at `.specify/memory/constitution.md`
- ✓ SDD templates loaded
  - ✓ `phr-template.prompt.md` - Prompt History Record template
  - ✓ `spec-template.md` - Specification template
  - ✓ `plan-template.md` - Architecture plan template
  - ✓ `tasks-template.md` - Task breakdown template
  - ✓ `adr-template.md` - Architecture Decision Record template
- ✓ Git repository initialized
- ✓ Directory structure created
  - ✓ `history/prompts/constitution/` - Constitution PHRs
  - ✓ `history/prompts/general/` - General PHRs
  - ✓ `history/adr/` - ADRs
  - ✓ `specs/` - Feature specifications

## Key Workflows

### 1. Creating a Feature Specification
```bash
# Create a new feature directory
mkdir specs/<feature-name>

# Use the spec template
.specify/templates/spec-template.md

# Document the feature plan
.specify/templates/plan-template.md

# Break down into testable tasks
.specify/templates/tasks-template.md
```

### 2. Recording Prompt History (PHR)
Every significant interaction is recorded as a PHR:
- **Location**: `history/prompts/<route>/<ID>-<slug>.<stage>.prompt.md`
- **Routes**: 
  - `constitution/` - Project principle work
  - `<feature-name>/` - Feature-specific work
  - `general/` - General development work
- **Stages**: constitution, spec, plan, tasks, red, green, refactor, explainer, misc, general

### 3. Documenting Architectural Decisions
When significant architectural decisions are made:
```
📋 Architectural decision detected: [brief-description]
Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`
```

Use `history/adr/<filename>.md` with the ADR template.

## Development Guidelines

### Authoritative Source Mandate
- Use MCP tools and CLI commands for all information gathering
- Never assume solutions; verify externally
- Prefer CLI interactions over manual file creation

### Execution Contract
For every request:
1. Confirm surface and success criteria
2. List constraints and non-goals
3. Produce artifacts with acceptance checks
4. Add follow-ups and risks
5. Create PHR in appropriate subdirectory
6. Surface ADR suggestions when applicable

### Minimum Acceptance Criteria
- Clear, testable acceptance criteria
- Explicit error paths and constraints
- Smallest viable change
- Code references to modified files

## Code Standards
See `.specify/memory/constitution.md` for:
- Code quality guidelines
- Testing requirements
- Performance standards
- Security policies
- Architecture principles

## Configuration Files

### `.gemini/settings.local.json`
Contains Gemini API configuration:
- `provider`: "gemini"
- `apiKey`: Your API key (already configured)

### `GEMINI.md`
Complete CLI rules and workflow guidelines including:
- Core guarantees (PHRs, ADRs)
- Development guidelines
- Execution contracts
- Architect guidelines

## Getting Started

1. **Read the Constitution**
   - Review `.specify/memory/constitution.md` to understand project principles

2. **Create First Feature**
   - Use templates in `.specify/templates/` to start a new feature
   - Place specifications in `specs/<feature-name>/`

3. **Record Work**
   - Every significant task creates a PHR in `history/prompts/`
   - Major decisions should suggest ADRs

4. **Reference Guidelines**
   - See `GEMINI.md` for complete CLI rules
   - Follow the execution contract for each request

## Support
For detailed guidelines on specific workflows, refer to:
- **CLI Rules**: `GEMINI.md`
- **Templates**: `.specify/templates/`
- **Constitution**: `.specify/memory/constitution.md`
"# Physical-AI-And-Humanoid-Robotics-TextBook" 
