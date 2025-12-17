---
id: 1
title: integrated-rag-chatbot-spec
stage: spec
date_iso: 2025-12-16
surface: agent
model: Qwen Code
feature: 1-integrated-rag-chatbot
branch: 1-integrated-rag-chatbot
user: user
command: /sp.specify
labels:
  - rag
  - chatbot
  - specification
  - embedded-book
links:
  spec: specs/1-integrated-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/1-integrated-rag-chatbot/spec.md
  - specs/1-integrated-rag-chatbot/checklists/requirements.md
tests_yaml: []
---

# Prompt History Record: Integrated RAG Chatbot Specification

## User Input (PROMPT_TEXT)

Integrated RAG Chatbot for Embedded Book Knowledge Target outcome: Design and implement a production-grade Retrieval-Augmented Generation (RAG) chatbot that is embedded inside a published digital book and answers reader questions strictly from the book's content or from user-selected text. Target users: - Readers of the published book - Students and researchers using the book for reference - Non-technical users expecting accurate, grounded answers Primary capabilities: - Question answering over full book content using vector-based retrieval - Question answering strictly from user-selected/highlighted text - Clear refusal when answer is not present in provided context - Seamless frontend embedding inside the book UI Technology stack (MANDATORY): Backend: - FastAPI (Python) - Cohere LLM APIs (NO OpenAI usage allowed) - Neon Serverless Postgres (metadata, sessions, analytics) - Qdrant Cloud (Free Tier) for vector storage Vector database configuration: - Provider: Qdrant Cloud - Cluster ID: 0248c965-b989-42ca-8a54-869360bb3b4f - Endpoint: https://0248c965-b989-42ca-8a54-869360bb3b4f.europe-west3-0.gcp.cloud.qdrant.io - API Key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Zw89gistyRqpeUSrZEZJF4f5iPrGlRwxs6rnmDDuSJg Database configuration: - Neon Postgres URL: postgresql://neondb_owner:npg_aAOTWlSz0w7P@ep-morning-feather-adczwxp8-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require LLM configuration: - Provider: Cohere - API Key: 20pDxbpyPdPrRHmnml2G3TdzIrDSDw2NY8lnQ09d - Allowed models: command-r, command-r-plus - Temperature: ≤ 0.2 - Responses must be deterministic and context-bound Retrieval behavior: - Book content must be chunked (500–800 tokens, overlapping) - All chunks embedded using Cohere Embeddings - Queries embedded using the same Cohere embedding model - Vector similarity search via Qdrant (top-k configurable) - Retrieved chunks must include metadata (chapter, section, page) Selected-text answering mode: - If user provides highlighted/selected text: - Ignore Qdrant and database retrieval completely - Answer ONLY from the selected text - If answer is not explicitly present, refuse Answering rules: - Answers must rely solely on retrieved or selected context - No external knowledge or training data usage - No assumptions, extrapolation, or speculation - Clear and concise language suitable for general readers Refusal behavior: - If context is insufficient, respond exactly: "I cannot answer this question based on the provided text." Frontend expectations: - Chat widget embeddable via script or iframe - Support for: - Normal chat mode (full-book RAG) - Highlight-and-ask mode (selected-text RAG) - Mode switching handled via request payload Success criteria: - Zero hallucinated responses in evaluation - Accurate answers grounded in book content - Correct refusal in out-of-scope queries - Stable performance on free tiers of Qdrant and Neon - Fully compliant with the defined constitution prompt Explicitly not building: - OpenAI-based agents or APIs - General-purpose chatbot unrelated to the book - Internet or web-search-based answering - Fine-tuning of LLM models - Recommendation systems or analytics dashboards

## Response Summary (RESPONSE_TEXT)

Successfully created the feature specification for the Integrated RAG Chatbot for Embedded Book Knowledge. The specification includes:

- Three prioritized user stories covering full-book RAG, selected-text RAG, and UI integration
- Eleven functional requirements with clear acceptance criteria
- Key entities including Question, Book Content Chunk, User Session, Answer, and Context Metadata
- Five measurable success criteria focused on accuracy, zero hallucination, and proper refusal behavior
- Quality validation checklist with all items completed
- Resolution of clarification requirements with reasonable defaults

The specification is complete and ready for the next phase of development.

## Outcome

The feature specification has been created at specs/1-integrated-rag-chatbot/spec.md with all required sections completed. A quality validation checklist was also created at specs/1-integrated-rag-chatbot/checklists/requirements.md.

## Evaluation

The specification addresses all core requirements from the user input, with clear user scenarios, testable requirements, and measurable success criteria. All validation checklist items have been completed, confirming the specification is ready for planning phase.