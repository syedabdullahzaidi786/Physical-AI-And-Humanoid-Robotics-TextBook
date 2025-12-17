# Quick Start Guide: Integrated RAG Chatbot Backend

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Docker (optional, for containerized deployment)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd physical-ai-and-humanoid-robotics/backend
```

### 2. Create a Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Environment Configuration
Create a `.env` file in the project root with the following variables:

```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=https://0248c965-b989-42ca-8a54-869360bb3b4f.europe-west3-0.gcp.cloud.qdrant.io

# Neon Postgres Configuration
DATABASE_URL=postgresql://neondb_owner:npg_aAOTWlSz0w7P@ep-morning-feather-adczwxp8-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# Security Configuration
SECRET_KEY=your_secret_key_here  # Use a strong secret key
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Application Configuration
DEBUG=false  # Set to true for development
```

### 5. Run Database Migrations
```bash
alembic upgrade head
```

### 6. Start the Application
```bash
uvicorn main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`.

### 7. Run Tests
```bash
pytest
```

## API Usage Examples

### Authentication
```bash
curl -X POST "http://localhost:8000/auth/login" \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "password123"
  }'
```

### Query the Chatbot (Full Book RAG Mode)
```bash
curl -X POST "http://localhost:8000/chat/query" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the main themes of this book?",
    "mode": "full-book-rag"
  }'
```

### Query the Chatbot (Selected Text Mode)
```bash
curl -X POST "http://localhost:8000/chat/query" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What does this text say about climate change?",
    "mode": "selected-text",
    "selected_text": "Climate change is one of the most pressing issues of our time..."
  }'
```

## Project Structure
```
backend/
├── src/
│   ├── models/          # Data models
│   ├── services/        # Business logic
│   ├── api/             # API endpoints
│   ├── vector/          # Vector database operations
│   ├── auth/            # Authentication logic
│   └── core/            # Core utilities
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── alembic/             # Database migrations
├── config/              # Configuration files
├── main.py              # Application entry point
├── requirements.txt     # Python dependencies
└── .env                 # Environment variables (not committed)
```

## Development Commands

### Run with auto-reload
```bash
uvicorn main:app --reload
```

### Run tests with coverage
```bash
pytest --cov=src
```

### Format code
```bash
black src/
```

### Lint code
```bash
flake8 src/
```

## Troubleshooting

### Common Issues

1. **Cohere API Key Missing**: Ensure the `COHERE_API_KEY` environment variable is set correctly.

2. **Qdrant Connection Issues**: Check that the Qdrant host and API key are configured properly in environment variables.

3. **Database Connection Issues**: Verify that the `DATABASE_URL` is set correctly in the environment.

4. **JWT Token Issues**: Ensure the `SECRET_KEY` is set properly in environment variables.

### Getting Help
- Check the API documentation at `/docs` when the server is running
- Review the API contracts in `specs/1-integrated-rag-chatbot/contracts/`
- Check the project specification in `specs/1-integrated-rag-chatbot/spec.md`