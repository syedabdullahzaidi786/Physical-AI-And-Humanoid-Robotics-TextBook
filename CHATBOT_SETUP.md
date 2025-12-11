# RAG Chatbot Setup & Testing Guide

## Quick Start (5 minutes)

### Step 1: Get API Keys

1. **Gemini API Key**:
   - Go to https://aistudio.google.com/app/apikey
   - Create new API key
   - Copy the key

2. **You should already have**:
   - `COHERE_API_KEY` (from earlier setup)
   - `QDRANT_URL` and `QDRANT_API_KEY` (from vector DB)

### Step 2: Update Environment Variables

```bash
cd backend
cp .env.example .env
```

Edit `.env` and add:
```env
GEMINI_API_KEY=your_new_gemini_key_here
COHERE_API_KEY=your_cohere_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_key
COLLECTION_NAME=physical-ai-book
```

### Step 3: Install Backend Dependencies

```bash
cd backend
pip install -r requirements.txt
```

New packages:
- `google-generativeai` - Gemini API
- `fastapi` - Web framework
- `uvicorn` - ASGI server
- `pydantic` - Data validation
- `python-multipart` - Form data handling

### Step 4: Start Chatbot API

```bash
cd backend
python -m uvicorn chatbot_api:app --host 0.0.0.0 --port 8000 --reload
```

✅ You should see:
```
✅ Chatbot initialized successfully
   Model: gemini-2.0-flash
   Collection: physical-ai-book
   Documents: 1500+ (depending on your ingestion)

INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Step 5: Test API Health

**Option A: Browser**
```
http://localhost:8000/api/health
```

**Option B: PowerShell**
```powershell
curl http://localhost:8000/api/health
```

Expected response:
```json
{
  "status": "ready",
  "message": "Chatbot is operational"
}
```

### Step 6: Start Frontend Dev Server

```bash
cd physical-ai-book
npm run start
```

Visit: http://localhost:3000

### Step 7: Test the Chatbot

1. **Look for the chat button** (💬) in bottom-right corner
2. **Click to open** the chatbot window
3. **Ask a question**: "What is Physical AI?"
4. **Wait for response** (first call may take 3-5 seconds)

## Testing the API Directly

### Test Chat Endpoint

**PowerShell**:
```powershell
$body = @{
    message = "What is humanoid robotics?"
} | ConvertTo-Json

Invoke-WebRequest `
  -Uri "http://localhost:8000/api/chat" `
  -Method POST `
  -ContentType "application/json" `
  -Body $body
```

**Python**:
```python
import requests

response = requests.post(
    "http://localhost:8000/api/chat",
    json={"message": "What is Physical AI?"}
)
print(response.json())
```

### Get Knowledge Base Info

```bash
curl http://localhost:8000/api/info
```

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

## Interactive API Documentation

Open browser to:
```
http://localhost:8000/docs
```

This shows Swagger UI with all endpoints you can test directly!

## Troubleshooting

### Issue: "Cannot connect to API" or CORS Error

**Solution**:
1. Check backend is running:
   ```bash
   curl http://localhost:8000/api/health
   ```
2. If not, start it:
   ```bash
   python -m uvicorn chatbot_api:app --reload
   ```

### Issue: "Gemini API Error" or Timeout

**Check**:
- Gemini API key is valid
- API is accessible: https://aistudio.google.com/app/apikey
- Check quota/rate limits
- Internet connection is working

**Test with Python**:
```python
import google.generativeai as genai

api_key = "your_key_here"
genai.configure(api_key=api_key)

model = genai.GenerativeModel("gemini-2.0-flash")
response = model.generate_content("Test message")
print(response.text)
```

### Issue: "No relevant documents found"

**Ensure**:
1. Knowledge base is populated:
   ```bash
   curl http://localhost:8000/api/info
   # Check documents_count > 0
   ```

2. If count is 0, ingest the book:
   ```bash
   cd backend
   python main.py
   ```

3. Verify Qdrant is accessible with correct credentials

### Issue: Chatbot button not visible

1. **Clear browser cache**: Ctrl+Shift+Delete
2. **Rebuild frontend**: `npm run build`
3. **Check console** (F12 → Console) for JavaScript errors
4. **Verify Root.tsx** has `<Chatbot />` component

### Issue: "OSError: [Errno 48] Address already in use"

Port 8000 is already in use:

**PowerShell**:
```powershell
# Find process using port 8000
Get-NetTCPConnection -LocalPort 8000

# Or use different port
python -m uvicorn chatbot_api:app --port 8001
```

## Performance Testing

### Measure Response Time

**Python**:
```python
import time
import requests

start = time.time()
response = requests.post(
    "http://localhost:8000/api/chat",
    json={"message": "What is robotics?"}
)
elapsed = time.time() - start

print(f"Response time: {elapsed:.2f}s")
print(f"Response: {response.json()['response'][:100]}...")
```

Expected times:
- First request: 3-5 seconds (model initialization)
- Subsequent: 1-3 seconds
- With cache: <1 second

### Load Testing (Light)

```python
import concurrent.futures
import requests

def send_query(query):
    response = requests.post(
        "http://localhost:8000/api/chat",
        json={"message": query}
    )
    return response.status_code

queries = [
    "What is AI?",
    "What is robotics?",
    "What is Physical AI?",
] * 3

with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
    results = list(executor.map(send_query, queries))
    
print(f"Successful: {sum(1 for r in results if r == 200)}/{len(results)}")
```

## Production Deployment Checklist

Before deploying to production:

### Security
- [ ] `GEMINI_API_KEY` set in production environment
- [ ] No `.env` file committed to version control
- [ ] CORS origins configured for your domain only
- [ ] HTTPS enforced for all API calls
- [ ] Rate limiting enabled on API endpoint

### Performance
- [ ] Database connection pooling configured
- [ ] Response caching implemented
- [ ] Load testing completed successfully
- [ ] Error monitoring (Sentry) configured

### Infrastructure
- [ ] API deployed to scalable platform (Vercel, Railway, etc.)
- [ ] Database backups configured
- [ ] Monitoring and alerting set up
- [ ] Logging configured

### Documentation
- [ ] API documentation published
- [ ] Deployment steps documented
- [ ] Error codes documented
- [ ] Support contacts provided

## Development Commands

```bash
# Backend
cd backend

# Install dependencies
pip install -r requirements.txt

# Run API server
python -m uvicorn chatbot_api:app --reload

# Test chatbot locally
python rag_chatbot.py

# Ingest book data
python main.py

# Frontend
cd physical-ai-book

# Start dev server
npm run start

# Build for production
npm run build

# Run tests
npm run test
```

## Next Steps

1. ✅ **Test the chatbot** with sample questions
2. ✅ **Monitor logs** for any errors
3. ✅ **Verify response quality** from Gemini
4. ✅ **Check latency** and optimize if needed
5. 🔜 **Deploy to production** when ready
6. 🔜 **Gather user feedback** and improve

## Support & Resources

- **Gemini API**: https://ai.google.dev/
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Qdrant Docs**: https://qdrant.tech/
- **Cohere Docs**: https://docs.cohere.com/

---

**Once API is running + Frontend loads, you're all set!** 🚀

**Total setup time**: ~5 minutes
**Running services**: API + Frontend + Docs UI
