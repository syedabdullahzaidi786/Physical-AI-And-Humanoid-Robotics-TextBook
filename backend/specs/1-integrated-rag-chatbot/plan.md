# Implementation Plan: Integrated RAG Chatbot Backend (Cohere-based)

**Branch**: `1-integrated-rag-chatbot` | **Date**: 2025-12-16 | **Spec**: [link to spec](../spec.md)
**Input**: Feature specification from `/specs/1-integrated-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a secure, high-accuracy, Cohere-based Retrieval-Augmented Generation (RAG) backend that supports both book-wide RAG queries and user-selected-text queries. The backend enforces strict context-grounded answers with zero hallucination and complies with the project constitution.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: Python 3.11
**Primary Dependencies**: 
- FastAPI
- Cohere Python SDK
- Qdrant Python Client
- asyncpg (for PostgreSQL async operations)
- Neon Serverless Postgres 
- python-jose (for JWT handling)
- passlib (for password hashing)
**Storage**: 
- Neon Serverless Postgres (metadata, sessions, user accounts)
- Qdrant Cloud (vector storage for book content)
**Testing**: 
- pytest
- pytest-asyncio
- httpx (for API testing)
**Target Platform**: Linux server (cloud deployment)
**Project Type**: Backend API service with potential for web application
**Performance Goals**: 
- Response time: < 3 seconds for standard queries
- Handle at least 10 concurrent users
- Efficient vector search and LLM processing
**Constraints**: 
- Must use Cohere LLM APIs exclusively (no OpenAI)
- Temperature ≤ 0.2 for deterministic responses
- Response time target: < 3 seconds
- Free tier limitations for Qdrant and Neon
**Scale/Scope**: 
- Support for single book initially
- Up to 10,000 concurrent users (scalable)
- Multiple books support as future enhancement

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [X] All answers strictly derived from provided context (Groundedness)
- [X] Zero hallucination enforced through system prompts and validation
- [X] User-selected text overrides global book knowledge (Context Authority)
- [X] Clear refusal when answer not present in context (Transparency)
- [X] Deterministic responses using low temperature (≤ 0.2)
- [X] Knowledge hierarchy: User-selected text → Retrieved book chunks → No answer
- [X] Cohere API usage only (LLM Constraints)
- [X] All queries embedded using Cohere Embedding API
- [X] Vector similarity search via Qdrant Cloud
- [X] Top-k retrieval includes metadata (chapter, section, page)
- [X] User-selected text mode ignores vector database completely
- [X] Answers are concise, clear, and factual
- [X] No speculative language or assumptions beyond context
- [X] Correct refusal format: "I cannot answer this question based on the provided text."
- [X] Security constraints maintained (no system prompts exposure)

## Project Structure

### Documentation (this feature)

```text
specs/1-integrated-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   ├── vector/
│   ├── auth/
│   └── core/
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── alembic/
│   └── versions/
├── config/
├── main.py
└── requirements.txt
```

**Structure Decision**: Backend structure selected with modular organization by functionality (models, services, API endpoints, vector operations, authentication, and core utilities).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |