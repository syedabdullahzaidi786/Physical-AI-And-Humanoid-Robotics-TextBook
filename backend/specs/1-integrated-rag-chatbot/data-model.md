# Data Model: Integrated RAG Chatbot

## Entity: User
**Description**: Represents an authenticated user of the chatbot system
**Fields**:
- id: UUID (Primary Key)
- email: String (Unique, Required)
- password_hash: String (Required)
- created_at: DateTime (Required, Auto-generated)
- updated_at: DateTime (Required, Auto-generated)
- is_active: Boolean (Default: True)

## Entity: Document
**Description**: Represents a book or document that has been processed for RAG
**Fields**:
- id: UUID (Primary Key)
- title: String (Required)
- source_file_path: String (Required)
- total_chunks: Integer (Required)
- created_at: DateTime (Required, Auto-generated)
- author: String (Optional)
- isbn: String (Optional)
- metadata: JSON (Optional, contains additional document metadata)

## Entity: ContentChunk
**Description**: A segment of document content that has been processed and stored in the vector database
**Fields**:
- id: UUID (Primary Key)
- document_id: UUID (Foreign Key to Document)
- content: Text (Required)
- chunk_index: Integer (Required)
- token_count: Integer (Required)
- metadata: JSON (Contains chapter, section, page information)

## Entity: UserSession
**Description**: Represents a user's interaction session with the chatbot
**Fields**:
- id: UUID (Primary Key)
- user_id: UUID (Foreign Key to User)
- started_at: DateTime (Required, Auto-generated)
- ended_at: DateTime (Optional)
- metadata: JSON (Optional, for additional session info)

## Entity: Question
**Description**: A question asked by a user to the chatbot
**Fields**:
- id: UUID (Primary Key)
- session_id: UUID (Foreign Key to UserSession)
- content: Text (Required)
- mode: Enum (full-book-rag or selected-text)
- selected_text: Text (Optional, for selected-text mode)
- timestamp: DateTime (Required, Auto-generated)
- processed: Boolean (Default: False)

## Entity: Answer
**Description**: The chatbot's response to a user's question
**Fields**:
- id: UUID (Primary Key)
- question_id: UUID (Foreign Key to Question)
- content: Text (Required)
- source_chunks: Array of UUID (Foreign Keys to ContentChunk)
- timestamp: DateTime (Required, Auto-generated)
- is_refusal: Boolean (True if answer is a refusal)

## Entity: InteractionLog
**Description**: Log of user interactions for analytics and compliance
**Fields**:
- id: UUID (Primary Key)
- user_id: UUID (Foreign Key to User)
- question_id: UUID (Foreign Key to Question)
- answer_id: UUID (Foreign Key to Answer)
- timestamp: DateTime (Required, Auto-generated)
- metadata: JSON (For additional logging info)

## Relationships
- User has many UserSessions
- User has many InteractionLogs
- User has many Questions through UserSessions
- Document has many ContentChunks
- UserSession has many Questions
- Question has one Answer
- Question and Answer linked to InteractionLog
- Answer references multiple ContentChunks as sources

## Validation Rules
- User email must be unique and valid
- ContentChunk token_count must be between 500-800 tokens as per spec
- Answer content must be based solely on referenced ContentChunks or provided selected text
- UserSession must be linked to an active User
- Document metadata must include chapter, section, and page information

## State Transitions
- UserSession: active → ended (when session completes)
- Question: pending → processed (when answered)
- Answer: not_generated → generated (when response is created)