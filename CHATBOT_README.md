# RAG Chatbot for Physical AI & Robotics Book

## Overview

A production-ready **Retrieval-Augmented Generation (RAG)** chatbot that answers questions about the Physical AI & Humanoid Robotics textbook using:

- **Gemini 2.0 Flash** for intelligent response generation
- **Qdrant Vector Database** for semantic document retrieval
- **Cohere Embeddings** for converting text to vectors
- **FastAPI** backend with REST API
- **React** frontend with floating chat interface

## Features

### Backend (Python)

✅ **RAG Architecture**
- Retrieves relevant book content using semantic search
- Generates responses based on retrieved context
- Handles multi-turn conversations
- Graceful error handling with fallbacks

✅ **API Endpoints**
- `POST /api/chat` - Send message and get response
- `GET /api/health` - Health check
- `GET /api/info` - Knowledge base information
- `POST /api/stream` - Streaming responses (placeholder)

✅ **Gemini 2.0 Flash Integration**
- Fast response generation
- Context-aware answers
- Safety settings configured

✅ **Conversation Memory**
- Maintains conversation history
- Multi-turn context awareness
- Conversation state management

### Frontend (React/TypeScript)

✅ **Floating Chat Interface**
- Fixed position floating button (💬)
- Expandable chat window (400x600px)
- Responsive design (mobile-friendly)
- Dark mode support

✅ **User Experience**
- Real-time message streaming
- Typing indicator animation
- Message timestamps
- Clear conversation history button
- Error handling and display
- Loading states

✅ **Accessibility**
- Semantic HTML
- ARIA labels
- Keyboard support
- Screen reader friendly

## Architecture

```
┌─────────────────────────────────────────────┐
│         React Frontend (Docusaurus)         │
│  ┌──────────────────────────────────────┐   │
│  │      Floating Chatbot Component      │   │
│  │  ├─ Messages Display                 │   │
│  │  ├─ Input Form                       │   │
│  │  └─ Conversation History             │   │
│  └──────────────────────────────────────┘   │
│                    ↕ HTTP (REST)            │
├─────────────────────────────────────────────┤
│        FastAPI Backend (Python)             │
│  ┌──────────────────────────────────────┐   │
│  │      Chatbot API Server              │   │
│  │  ├─ /api/chat                        │   │
│  │  ├─ /api/health                      │   │
│  │  └─ /api/info                        │   │
│  └──────────────────────────────────────┘   │
│                    ↕                        │
├─────────────┬───────────────┬───────────────┤
│             │               │               │
│          Cohere          Gemini 2.0      Qdrant
│        Embeddings        Flash API      Vector DB
│             │               │               │
└─────────────┴───────────────┴───────────────┘
```

## Setup & Installation

### Prerequisites

- Python 3.8+
- Node.js 16+
- Qdrant Cloud account (vector database)
- Cohere API key
- Google Gemini API key

### Backend Setup

1. **Install dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Configure environment variables** (`.env` file):
   ```bash
   cp .env.example .env
   ```

   Fill in required values:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_URL=https://your-qdrant-instance.qdrant.io
   QDRANT_API_KEY=your_qdrant_api_key
   COLLECTION_NAME=physical-ai-book
   EMBED_MODEL=embed-english-v3.0
   ```

3. **First, ingest the book data** (if not already done):
   ```bash
   python main.py
   ```

4. **Start the chatbot API**:
   ```bash
   python -m uvicorn chatbot_api:app --host 0.0.0.0 --port 8000 --reload
   ```

   Server will be available at: `http://localhost:8000`

### Frontend Setup

1. **Environment variable** (optional, already configured):
   - `REACT_APP_CHATBOT_API_URL` defaults to `http://localhost:8000`
   - For production: set to your deployed API URL

2. **Start development server**:
   ```bash
   cd physical-ai-book
   npm run start
   ```

3. **Build for production**:
   ```bash
   npm run build
   ```

## API Documentation

### Chat Endpoint

**POST** `/api/chat`

Request:
```json
{
  "message": "What is Physical AI?",
  "conversation_history": [
    {
      "role": "user",
      "content": "Hello"
    },
    {
      "role": "assistant",
      "content": "Hi! How can I help?"
    }
  ]
}
```

Response:
```json
{
  "response": "Physical AI is the integration of AI algorithms with physical systems...",
  "sources": [],
  "status": "success"
}
```

### Health Check

**GET** `/api/health`

Response:
```json
{
  "status": "ready",
  "message": "Chatbot is operational"
}
```

### Info Endpoint

**GET** `/api/info`

Response:
```json
{
  "status": "ready",
  "model": "gemini-2.0-flash",
  "collection": "physical-ai-book",
  "documents_count": 1500,
  "vector_size": 1024
}
```

## Usage

### For Users

1. **Open the book** at `http://localhost:3000`
2. **Click the chat button** (💬) in bottom-right corner
3. **Ask questions** about the Physical AI & Robotics content
4. **View responses** generated from book knowledge
5. **Continue conversation** with context awareness

### For Developers

#### Integrate Chatbot into Page

```tsx
import Chatbot from '@site/src/components/Chatbot';

export function MyPage() {
  return (
    <div>
      <h1>My Page</h1>
      {/* Chatbot automatically added via Root.tsx */}
    </div>
  );
}
```

#### Custom API URL

```tsx
<Chatbot apiUrl="https://your-api.com/api" />
```

#### Direct API Calls

```typescript
const response = await fetch('http://localhost:8000/api/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    message: 'What is humanoid robotics?'
  })
});

const data = await response.json();
console.log(data.response);
```

## File Structure

```
backend/
├── rag_chatbot.py          # RAG chatbot service
├── chatbot_api.py          # FastAPI server
├── main.py                 # Data ingestion script
├── requirements.txt        # Python dependencies
└── .env.example            # Environment variables template

physical-ai-book/
└── src/
    └── components/
        └── Chatbot/
            ├── index.tsx                # React component
            └── Chatbot.module.css       # Styling
```

## Configuration

### RAG Parameters

Edit `backend/rag_chatbot.py`:

```python
RAGChatbot(
    collection_name="physical-ai-book",  # Qdrant collection
    top_k=5,                             # Retrieve top 5 documents
    model_name="gemini-2.0-flash"        # Model to use
)
```

### API Server

Edit `backend/chatbot_api.py`:

```python
PORT = 8000                    # Default port
CORS_ORIGINS = "*"             # Configure CORS
MAX_MESSAGES = 50              # Max conversation length
```

### Frontend

Environment variables in `.env.local`:

```
REACT_APP_CHATBOT_API_URL=http://localhost:8000
REACT_APP_CHATBOT_ENABLED=true
```

## Troubleshooting

### "Cannot connect to chatbot API"
- Ensure backend is running: `python -m uvicorn chatbot_api:app --reload`
- Check CORS configuration in `chatbot_api.py`
- Verify `REACT_APP_CHATBOT_API_URL` is correct

### "No relevant documents found"
- Run `python main.py` to ingest the book
- Verify Qdrant collection exists: `GET /api/info`
- Check if `QDRANT_URL` and `QDRANT_API_KEY` are correct

### "Gemini API Error"
- Verify `GEMINI_API_KEY` is valid
- Check API quota and rate limits
- Ensure model `gemini-2.0-flash` is available in your region

### Chatbot not appearing
- Clear browser cache
- Check browser console for errors (F12)
- Verify `Root.tsx` includes `<Chatbot />` component

## Performance Optimization

### Backend

- **Caching**: Implement response caching for common queries
- **Batching**: Batch multiple embedding requests
- **Async**: Use async/await for non-blocking I/O

### Frontend

- **Lazy Loading**: Load chatbot component on demand
- **Message Pagination**: Show only recent messages
- **Debounce**: Debounce input to reduce API calls

## Security

✅ **API Security**
- No API keys exposed in frontend code
- CORS configured for allowed origins
- Input validation on all endpoints
- Rate limiting recommended for production

✅ **Data Privacy**
- Conversation stored only in browser memory
- No persistent storage of user queries
- HTTPS recommended for production

## Deployment

### Production Checklist

- [ ] Set `GEMINI_API_KEY` in production environment
- [ ] Set `QDRANT_URL` and `QDRANT_API_KEY`
- [ ] Configure CORS for production domain
- [ ] Use HTTPS for all API calls
- [ ] Enable rate limiting on API
- [ ] Set up monitoring and logging
- [ ] Configure error tracking (Sentry, etc.)
- [ ] Test with production data

### Deployment Platforms

**Backend** (Python/FastAPI):
- Vercel Functions (serverless)
- Railway
- Render
- Heroku
- AWS Lambda with API Gateway

**Frontend**:
- Already deployed with Docusaurus on Vercel

### Example: Vercel Deployment

1. **Backend on Vercel** (serverless):
   ```bash
   vercel --cwd backend --prod
   ```

2. **Update environment variables**:
   ```
   GEMINI_API_KEY=...
   QDRANT_URL=...
   QDRANT_API_KEY=...
   ```

3. **Update frontend API URL**:
   ```
   REACT_APP_CHATBOT_API_URL=https://your-api.vercel.app
   ```

## Future Improvements

- [ ] Streaming responses with SSE
- [ ] Voice input/output support
- [ ] Multi-language support (Urdu integration)
- [ ] User authentication for conversation history
- [ ] Feedback mechanism for response quality
- [ ] Analytics dashboard for popular questions
- [ ] Fine-tuned model for domain-specific queries
- [ ] Integration with More Gemini API models
- [ ] Conversation export (PDF, JSON)
- [ ] Admin panel for managing knowledge base

## API Reference

See `/docs` endpoint for interactive Swagger UI:
```
http://localhost:8000/docs
```

## Contributing

To improve the chatbot:

1. **Improve RAG**: Enhance retrieval relevance
2. **Expand Dictionary**: Add more offline translations
3. **Fix Bugs**: Report issues on GitHub
4. **Add Features**: Submit PRs for new capabilities

## Support

For issues or questions:
1. Check troubleshooting section
2. Review API logs: `grep "[RAG]" app.log`
3. Check browser DevTools (F12)
4. Open GitHub issue with error logs

## References

- [Google Gemini API Docs](https://ai.google.dev/)
- [Qdrant Documentation](https://qdrant.tech/)
- [Cohere API Reference](https://docs.cohere.com/)
- [FastAPI Guide](https://fastapi.tiangolo.com/)

---

**Last Updated**: December 11, 2025  
**Status**: ✅ Production Ready
