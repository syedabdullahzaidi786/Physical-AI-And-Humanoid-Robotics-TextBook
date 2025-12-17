---
description: "Task list for Integrated RAG Chatbot Backend"
---

# Tasks: Integrated RAG Chatbot Backend

**Input**: Design documents from `/specs/1-integrated-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan in `backend/`
- [X] T002 Initialize Python project with FastAPI, Cohere, Qdrant, and Neon Postgres dependencies in `backend/requirements.txt`
- [ ] T003 [P] Configure linting and formatting tools (black, flake8) for the project

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [X] T004 Setup database schema and alembic migrations framework in `backend/alembic/`
- [X] T005 [P] Implement JWT authentication/authorization framework in `backend/src/auth/`
- [X] T006 [P] Setup API routing and middleware structure in `backend/src/api/` and `backend/main.py`
- [X] T007 Create base data models in `backend/src/models/` that all stories depend on
- [X] T008 Configure error handling and logging infrastructure in `backend/src/core/`
- [X] T009 Setup environment configuration management in `backend/config/`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Full-book RAG Question Answering (Priority: P1) 🎯 MVP

**Goal**: Enable readers to ask questions about the book content and receive accurate answers based on the book's content

**Independent Test**: Can be fully tested by asking various questions about the book content and verifying that the chatbot provides accurate answers based on the book, with no hallucination of facts.

### Implementation for User Story 1

- [X] T010 [P] [US1] Create User model in `backend/src/models/user.py`
- [X] T011 [P] [US1] Create Document model in `backend/src/models/document.py`
- [X] T012 [P] [US1] Create ContentChunk model in `backend/src/models/content_chunk.py`
- [X] T013 [P] [US1] Create UserSession model in `backend/src/models/user_session.py`
- [X] T014 [P] [US1] Create Question model in `backend/src/models/question.py`
- [X] T015 [P] [US1] Create Answer model in `backend/src/models/answer.py`
- [X] T016 [P] [US1] Create InteractionLog model in `backend/src/models/interaction_log.py`
- [X] T017 [US1] Implement document ingestion service in `backend/src/services/document_service.py`
- [X] T018 [US1] Implement content chunking service with 600-token chunks in `backend/src/services/chunking_service.py`
- [X] T019 [US1] Implement vector operations service for Qdrant integration in `backend/src/services/vector_service.py`
- [X] T020 [US1] Implement Cohere embedding service in `backend/src/services/embedding_service.py`
- [X] T021 [US1] Implement Cohere LLM service for RAG generation in `backend/src/services/llm_service.py`
- [X] T022 [US1] Implement RAG retrieval service in `backend/src/services/retrieval_service.py`
- [X] T023 [US1] Implement full-book RAG API endpoint in `backend/src/api/rag_routes.py`
- [X] T024 [US1] Add authentication middleware to RAG endpoints
- [X] T025 [US1] Implement refusal logic with exact message "I cannot answer this question based on the provided text."
- [X] T026 [US1] Add metadata (chapter, section, page) to retrieved results
- [X] T027 [US1] Implement document upload endpoint in `backend/src/api/document_routes.py`
- [X] T028 [US1] Implement document processing status endpoint in `backend/src/api/document_routes.py`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Selected-text Question Answering (Priority: P2)

**Goal**: Enable students to highlight specific text and ask questions about only that text

**Independent Test**: Can be fully tested by selecting text in the book, asking questions about that text, and verifying that the chatbot only uses the selected text to answer, ignoring the broader book content.

### Implementation for User Story 2

- [X] T029 [P] [US2] Enhance Question model to support selected-text mode in `backend/src/models/question.py`
- [X] T030 [US2] Create/Update selected-text only RAG service in `backend/src/services/selected_text_service.py`
- [X] T031 [US2] Implement selected-text RAG endpoint in `backend/src/api/rag_routes.py`
- [X] T032 [US2] Add mode switching logic to handle selected-text vs full-book in `backend/src/services/chat_service.py`
- [X] T033 [US2] Implement bypass for Qdrant vector database when in selected-text mode
- [X] T034 [US2] Ensure selected-text mode only answers from provided text
- [X] T035 [US2] Add mode parameter to chat API endpoint

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Seamless Book UI Integration (Priority: P3)

**Goal**: Enable the chatbot to be seamlessly embedded in the book's UI

**Independent Test**: Can be tested by verifying the chatbot widget is properly integrated into the book interface and accessible during the reading experience.

### Implementation for User Story 3

- [X] T036 [P] [US3] Create embeddable chat widget in `backend/src/api/embed_routes.py`
- [X] T037 [US3] Implement script/iframe endpoint for frontend embedding
- [X] T038 [US3] Add UI integration documentation in `backend/docs/ui_integration.md`
- [X] T039 [US3] Implement mode switching indicator for frontend

**Checkpoint**: All user stories should now be independently functional

---

[Add more user story phases as needed, following the same pattern]

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T040 [P] Documentation updates in `backend/docs/`
- [X] T041 Code cleanup and refactoring
- [X] T042 Performance optimization to achieve < 3s response time
- [X] T043 [P] Add comprehensive unit tests in `backend/tests/unit/`
- [X] T044 Security hardening
- [X] T045 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all models for User Story 1 together:
[US1] Create User model in backend/src/models/user.py
[US1] Create Document model in backend/src/models/document.py
[US1] Create ContentChunk model in backend/src/models/content_chunk.py
[US1] Create UserSession model in backend/src/models/user_session.py
[US1] Create Question model in backend/src/models/question.py
[US1] Create Answer model in backend/src/models/answer.py
[US1] Create InteractionLog model in backend/src/models/interaction_log.py
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Add User Story 1 → Test independently → Deploy/Demo (MVP!)
   - Add User Story 2 → Test independently → Deploy/Demo
   - Add User Story 3 → Test independently → Deploy/Demo
   Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence