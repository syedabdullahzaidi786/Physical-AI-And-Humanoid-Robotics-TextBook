# Feature Specification: Integrated RAG Chatbot for Embedded Book Knowledge

**Feature Branch**: `1-integrated-rag-chatbot`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot for Embedded Book Knowledge Target outcome: Design and implement a production-grade Retrieval-Augmented Generation (RAG) chatbot that is embedded inside a published digital book and answers reader questions strictly from the book's content or from user-selected text. Target users: - Readers of the published book - Students and researchers using the book for reference - Non-technical users expecting accurate, grounded answers Primary capabilities: - Question answering over full book content using vector-based retrieval - Question answering strictly from user-selected/highlighted text - Clear refusal when answer is not present in provided context - Seamless frontend embedding inside the book UI Technology stack (MANDATORY): Backend: - FastAPI (Python) - Cohere LLM APIs (NO OpenAI usage allowed) - Neon Serverless Postgres (metadata, sessions, analytics) - Qdrant Cloud (Free Tier) for vector storage Vector database configuration: - Provider: Qdrant Cloud - Cluster ID: 0248c965-b989-42ca-8a54-869360bb3b4f - Endpoint: https://0248c965-b989-42ca-8a54-869360bb3b4f.europe-west3-0.gcp.cloud.qdrant.io - API Key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Zw89gistyRqpeUSrZEZJF4f5iPrGlRwxs6rnmDDuSJg Database configuration: - Neon Postgres URL: postgresql://neondb_owner:npg_aAOTWlSz0w7P@ep-morning-feather-adczwxp8-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require LLM configuration: - Provider: Cohere - API Key: 20pDxbpyPdPrRHmnml2G3TdzIrDSDw2NY8lnQ09d - Allowed models: command-r, command-r-plus - Temperature: ≤ 0.2 - Responses must be deterministic and context-bound Retrieval behavior: - Book content must be chunked (500–800 tokens, overlapping) - All chunks embedded using Cohere Embeddings - Queries embedded using the same Cohere embedding model - Vector similarity search via Qdrant (top-k configurable) - Retrieved chunks must include metadata (chapter, section, page) Selected-text answering mode: - If user provides highlighted/selected text: - Ignore Qdrant and database retrieval completely - Answer ONLY from the selected text - If answer is not explicitly present, refuse Answering rules: - Answers must rely solely on retrieved or selected context - No external knowledge or training data usage - No assumptions, extrapolation, or speculation - Clear and concise language suitable for general readers Refusal behavior: - If context is insufficient, respond exactly: "I cannot answer this question based on the provided text." Frontend expectations: - Chat widget embeddable via script or iframe - Support for: - Normal chat mode (full-book RAG) - Highlight-and-ask mode (selected-text RAG) - Mode switching handled via request payload Success criteria: - Zero hallucinated responses in evaluation - Accurate answers grounded in book content - Correct refusal in out-of-scope queries - Stable performance on free tiers of Qdrant and Neon - Fully compliant with the defined constitution prompt Explicitly not building: - OpenAI-based agents or APIs - General-purpose chatbot unrelated to the book - Internet or web-search-based answering - Fine-tuning of LLM models - Recommendation systems or analytics dashboards"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Full-book RAG Question Answering (Priority: P1)

As a reader of the published book, I want to ask questions about the book content and receive accurate answers based on the book's content so that I can quickly find information without having to manually search through the entire book.

**Why this priority**: This is the core functionality of the chatbot - enabling readers to ask questions and get answers from the entire book content. Without this, the chatbot has no value.

**Independent Test**: Can be fully tested by asking various questions about the book content and verifying that the chatbot provides accurate answers based on the book, with no hallucination of facts.

**Acceptance Scenarios**:

1. **Given** I am viewing the book with the embedded chatbot widget, **When** I type a question about the book content, **Then** the chatbot responds with a relevant answer strictly based on information in the book.
2. **Given** I have asked a question that cannot be answered with the book's content, **When** I submit the question, **Then** the chatbot responds with "I cannot answer this question based on the provided text."

---

### User Story 2 - Selected-text Question Answering (Priority: P2)

As a student using the book for reference, I want to highlight specific text and ask questions about only that text so that I can get focused answers based on my selected content without getting responses influenced by the entire book.

**Why this priority**: This provides an important alternative mode for users who want to focus specifically on selected passages rather than the full book context.

**Independent Test**: Can be fully tested by selecting text in the book, asking questions about that text, and verifying that the chatbot only uses the selected text to answer, ignoring the broader book content.

**Acceptance Scenarios**:

1. **Given** I have highlighted/select specific text in the book, **When** I ask a question related to that text, **Then** the chatbot responds with an answer based only on the selected text.
2. **Given** I have highlighted text that does not contain the answer to my question, **When** I ask the question, **Then** the chatbot responds with "I cannot answer this question based on the provided text."

---

### User Story 3 - Seamless Book UI Integration (Priority: P3)

As a non-technical book reader, I want the chatbot to be seamlessly embedded in the book's UI so that I can access the question-answering functionality without it disrupting my reading experience.

**Why this priority**: While important for user experience, this is implementation-dependent and can be developed after core functionality is working.

**Independent Test**: Can be tested by verifying the chatbot widget is properly integrated into the book interface and accessible during the reading experience.

**Acceptance Scenarios**:

1. **Given** I am reading the book, **When** I access the chatbot interface, **Then** the widget integrates smoothly without disrupting the reading experience.
2. **Given** I am using the chatbot, **When** I switch between full-book mode and selected-text mode, **Then** the interface clearly indicates the current mode and functions properly.

---

[Add more user stories as needed, each with an assigned priority]

### Edge Cases

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right edge cases.
-->

- What happens when a user asks a question with ambiguous context that could have multiple interpretations in the book?
- How does the system handle extremely long user-selected text passages?
- How does the system handle requests when the Qdrant vector database is temporarily unavailable?
- What happens when the book content has been updated but the vector embeddings haven't been refreshed yet?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST answer user questions based solely on the book content when in full-book RAG mode
- **FR-002**: System MUST answer user questions based solely on user-selected/highlighted text when in selected-text mode
- **FR-003**: System MUST refuse to answer with the exact message "I cannot answer this question based on the provided text." when the required information is not present in the provided context
- **FR-004**: System MUST retrieve relevant book content using vector similarity search from a vector database
- **FR-005**: System MUST provide context metadata (chapter, section, page) with retrieved content when answering questions
- **FR-006**: System MUST support seamless embedding of the chat interface within the book's UI using script or iframe
- **FR-007**: System MUST switch between full-book RAG mode and selected-text RAG mode based on request payload
- **FR-008**: System MUST ensure answers are deterministic and context-bound, with no hallucination of facts
- **FR-009**: System MUST use Cohere LLM APIs exclusively (no OpenAI or other providers)

*Example of marking unclear requirements:*

- **FR-010**: System MUST process book content by chunking it into 600 tokens (within the specified 500-800 token range)
- **FR-011**: System MUST implement response temperature of 0.2 (the maximum allowed value)

### Key Entities *(include if feature involves data)*

- **Question**: A query from the user about the book content, including the query text and mode (full-book RAG vs. selected-text)
- **Book Content Chunk**: Segments of the book content that have been processed and stored in the vector database with metadata (chapter, section, page)
- **User Session**: Information about the current user interaction, including selected text (if in selected-text mode)
- **Answer**: The response generated by the system based on the provided context, including the answer text and source references
- **Context Metadata**: Information about where in the book the relevant content was found (chapter, section, page)

## Clarifications

### Session 2025-12-16

- Q: What authentication requirements exist for chatbot access? → A: User authentication required for chatbot access
- Q: What performance requirements exist for response times? → A: Below 3 seconds for standard queries
- Q: How should the system handle external service unavailability? → A: Graceful degradation with clear user messaging
- Q: What accessibility requirements must be met? → A: WCAG 2.1 AA compliance
- Q: What is the data retention policy for user interactions? → A: Retain for duration of user account, delete upon account deletion

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Zero hallucinated responses in evaluation - 100% of responses must be grounded in the provided book content
- **SC-002**: At least 90% of user questions about book content must receive accurate answers based on the book content
- **SC-003**: When information is not present in the provided context, the system must correctly respond with "I cannot answer this question based on the provided text." in 100% of cases
- **SC-004**: The system must perform reliably on free tiers of Qdrant and Neon without degradation in response quality
- **SC-005**: Users must be able to seamlessly access the chatbot functionality within the book UI without disruption to the reading experience
- **SC-006**: Chatbot responses must be delivered within 3 seconds for standard queries

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST answer user questions based solely on the book content when in full-book RAG mode
- **FR-002**: System MUST answer user questions based solely on user-selected/highlighted text when in selected-text mode
- **FR-003**: System MUST refuse to answer with the exact message "I cannot answer this question based on the provided text." when the required information is not present in the provided context
- **FR-004**: System MUST retrieve relevant book content using vector similarity search from a vector database
- **FR-005**: System MUST provide context metadata (chapter, section, page) with retrieved content when answering questions
- **FR-006**: System MUST support seamless embedding of the chat interface within the book's UI using script or iframe
- **FR-007**: System MUST switch between full-book RAG mode and selected-text RAG mode based on request payload
- **FR-008**: System MUST ensure answers are deterministic and context-bound, with no hallucination of facts
- **FR-009**: System MUST use Cohere LLM APIs exclusively (no OpenAI or other providers)
- **FR-010**: System MUST process book content by chunking it into 600 tokens (within the specified 500-800 token range)
- **FR-011**: System MUST implement response temperature of 0.2 (the maximum allowed value)
- **FR-012**: System MUST require user authentication before allowing access to the chatbot functionality
- **FR-013**: System MUST handle external service unavailability through graceful degradation with clear user messaging
- **FR-014**: System MUST comply with WCAG 2.1 AA accessibility standards
- **FR-015**: System MUST retain user interaction data only for the duration of the user account and delete it upon account deletion

