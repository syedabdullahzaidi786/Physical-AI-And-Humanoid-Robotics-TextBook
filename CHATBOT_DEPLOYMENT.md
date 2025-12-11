# Chatbot Deployment Guide

## Overview

Complete guide to deploy the RAG Chatbot to production on multiple platforms.

## Deployment Architecture

```
Frontend (Docusaurus)     Backend (FastAPI)        Infrastructure
┌────────────────────┐   ┌──────────────────┐     ┌─────────────┐
│ Vercel Hosting     │──→│ Vercel Functions │────→│ Qdrant Cloud│
│ (Docusaurus)       │   │ (Python/FastAPI) │     │ (Vector DB) │
└────────────────────┘   └──────────────────┘     └─────────────┘
                                │
                                ↓
                         ┌──────────────┐
                         │ Gemini API   │
                         │ (Google)     │
                         └──────────────┘
```

## Option 1: Vercel (Recommended)

### Prerequisites
- Vercel account (https://vercel.com)
- GitHub repository connected
- Environment variables configured

### Frontend Deployment (Automatic)

1. **Connect Repository**:
   - Go to https://vercel.com/dashboard
   - Click "Add New..." → "Project"
   - Select your GitHub repository
   - Click "Import"

2. **Configure Environment**:
   ```
   REACT_APP_CHATBOT_API_URL=https://your-api.vercel.app
   ```

3. **Deploy**:
   - Vercel automatically deploys on push to main
   - Build: `npm run build`
   - Output: `build/`

### Backend Deployment (Serverless)

1. **Create API Route** at `api/chat.js`:

```javascript
// api/chat.js
import { spawn } from 'child_process';

export default async function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { message, conversation_history } = req.body;

    // Call Python backend
    const python = spawn('python', [
      'backend/chatbot_api.py',
      JSON.stringify({ message, conversation_history })
    ]);

    let output = '';
    python.stdout.on('data', (data) => {
      output += data.toString();
    });

    python.on('close', (code) => {
      if (code === 0) {
        res.status(200).json(JSON.parse(output));
      } else {
        res.status(500).json({ error: 'Backend error' });
      }
    });
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
}
```

2. **Alternative: Deploy Backend Separately**

Use Vercel Functions or another platform (see below).

## Option 2: Railway (Backend Only)

### Setup

1. **Connect Repository**:
   - Go to https://railway.app
   - Click "New Project" → "Deploy from GitHub"
   - Select repository

2. **Configure**:
   - Framework: Python
   - Root Directory: `backend/`
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn chatbot_api:app --host 0.0.0.0 --port $PORT`

3. **Add Environment Variables**:
   ```
   GEMINI_API_KEY=...
   COHERE_API_KEY=...
   QDRANT_URL=...
   QDRANT_API_KEY=...
   PORT=8000
   ```

4. **Deploy**:
   - Railway automatically deploys on push
   - Get domain: `https://your-app.railway.app`

### Frontend Update

Update `REACT_APP_CHATBOT_API_URL` to Railway domain:
```
REACT_APP_CHATBOT_API_URL=https://your-app.railway.app
```

## Option 3: Render

### Backend Deployment

1. **Connect Repository**:
   - Go to https://render.com
   - Click "New +" → "Web Service"
   - Select GitHub repository
   - Select branch: `main`

2. **Configure**:
   - Name: `physical-ai-chatbot`
   - Runtime: `Python 3.11`
   - Build Command: `pip install -r backend/requirements.txt`
   - Start Command: `cd backend && uvicorn chatbot_api:app --host 0.0.0.0`
   - Plan: Free or Pro

3. **Environment Variables**:
   - `GEMINI_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`

4. **Deploy**:
   - Render deploys automatically
   - Get URL from dashboard

## Option 4: Heroku (Legacy)

### Prerequisites
- Heroku account
- Heroku CLI installed

### Deployment

1. **Login to Heroku**:
   ```bash
   heroku login
   ```

2. **Create App**:
   ```bash
   heroku create your-app-name
   ```

3. **Set Environment Variables**:
   ```bash
   heroku config:set GEMINI_API_KEY=...
   heroku config:set COHERE_API_KEY=...
   heroku config:set QDRANT_URL=...
   heroku config:set QDRANT_API_KEY=...
   ```

4. **Create Procfile** (in root):
   ```
   web: cd backend && uvicorn chatbot_api:app --host 0.0.0.0 --port $PORT
   ```

5. **Deploy**:
   ```bash
   git push heroku main
   ```

## Option 5: Docker (Any Cloud)

### Create Dockerfile

```dockerfile
# backend/Dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy code
COPY . .

# Expose port
EXPOSE 8000

# Run API
CMD ["uvicorn", "chatbot_api:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Build & Push

```bash
# Build image
docker build -f backend/Dockerfile -t your-user/chatbot-api:latest .

# Push to Docker Hub
docker push your-user/chatbot-api:latest

# Run locally
docker run -p 8000:8000 \
  -e GEMINI_API_KEY=... \
  -e COHERE_API_KEY=... \
  -e QDRANT_URL=... \
  -e QDRANT_API_KEY=... \
  your-user/chatbot-api:latest
```

### Deploy to Cloud Run (Google)

```bash
gcloud run deploy chatbot-api \
  --image your-user/chatbot-api:latest \
  --platform managed \
  --region us-central1 \
  --memory 512Mi \
  --set-env-vars GEMINI_API_KEY=...,COHERE_API_KEY=...
```

## Complete Production Setup

### Step 1: Deploy Frontend (Vercel)

```bash
# Automatic on git push
# No additional setup needed
```

### Step 2: Deploy Backend (Railway/Render)

1. Choose platform (Railway recommended)
2. Connect GitHub repo
3. Set environment variables
4. Deploy (automatic)
5. Copy deployment URL

### Step 3: Configure API URL

Update Vercel environment:
```
REACT_APP_CHATBOT_API_URL=https://your-backend-url.app
```

### Step 4: Verify Deployment

```bash
# Frontend
curl https://your-frontend.vercel.app/api/health

# Backend
curl https://your-backend.railway.app/api/health

# Chat endpoint
curl -X POST https://your-backend.railway.app/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"Hello"}'
```

## Production Checklist

### Security
- [ ] No `.env` file in git
- [ ] All secrets in environment variables
- [ ] CORS configured for your domain only
- [ ] HTTPS enforced
- [ ] Rate limiting enabled
- [ ] Input validation enabled

### Performance
- [ ] Database connection pooling
- [ ] Response caching configured
- [ ] CDN enabled (for frontend)
- [ ] Compression enabled
- [ ] Load testing completed

### Monitoring
- [ ] Error tracking (Sentry)
- [ ] Uptime monitoring
- [ ] Performance monitoring
- [ ] Logs aggregation
- [ ] Alert thresholds set

### Documentation
- [ ] API docs published
- [ ] Deployment runbook created
- [ ] Rollback procedure documented
- [ ] Support contacts listed
- [ ] Architecture diagram shared

### Backup & Recovery
- [ ] Database backups configured
- [ ] Backup frequency: Daily
- [ ] Restore procedure tested
- [ ] Disaster recovery plan

## Environment Variables (Production)

```env
# Gemini API
GEMINI_API_KEY=your_production_key

# Cohere API
COHERE_API_KEY=your_cohere_key

# Qdrant
QDRANT_URL=https://your-prod-qdrant.qdrant.io
QDRANT_API_KEY=your_qdrant_key

# Server
HOST=0.0.0.0
PORT=8000
LOG_LEVEL=info

# CORS
CORS_ORIGINS=https://your-domain.com,https://www.your-domain.com

# Database
COLLECTION_NAME=physical-ai-book
EMBED_MODEL=embed-english-v3.0
```

## Monitoring & Alerts

### Error Tracking (Sentry)

1. Create Sentry account
2. Add to backend:

```python
import sentry_sdk
from sentry_sdk.integrations.fastapi import FastApiIntegration

sentry_sdk.init(
    dsn="your_sentry_dsn",
    integrations=[FastApiIntegration()]
)
```

### Uptime Monitoring

Services:
- Uptime Robot (free)
- Freshping
- Datadog

Check: `GET /api/health` endpoint

### Performance Monitoring

Platforms:
- New Relic
- Datadog
- Prometheus + Grafana

## Cost Estimation (Monthly)

| Service | Free Tier | Pro Tier |
|---------|-----------|----------|
| Vercel Frontend | ✅ Included | $20+ |
| Railway Backend | ✅ 500 hrs | $5-50 |
| Qdrant Cloud | ✅ 1GB | $9+ |
| Gemini API | ✅ 50 req/min | $0.075/1M |
| Cohere API | ✅ 100 calls | $0.04/1M |

**Total Free**: $0 (with limitations)  
**Total Pro**: $30-100/month

## Rollback Procedure

If deployment fails:

### Vercel (Frontend)
```
Dashboard → Deployments → Previous Version → Promote
```

### Railway (Backend)
```
Dashboard → Deployments → Previous Version → Deploy
```

## Debugging Production Issues

### Check Logs

**Vercel**:
```
Dashboard → Project → Function Logs
```

**Railway**:
```
Dashboard → Logs tab
```

### Test Endpoints

```bash
# Health check
curl https://your-api.app/api/health

# Knowledge base info
curl https://your-api.app/api/info

# Send test message
curl -X POST https://your-api.app/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"test"}'
```

### Check Environment Variables

```bash
# Railway
railway variables

# Vercel
vercel env list
```

## Post-Deployment

1. **Verify Functionality**:
   - Test chat in production
   - Check response times
   - Verify CORS works

2. **Monitor Metrics**:
   - Response times
   - Error rates
   - API usage
   - User feedback

3. **Optimization**:
   - Cache frequent queries
   - Optimize embeddings
   - Tune model parameters

4. **Feedback Loop**:
   - Collect user feedback
   - Monitor chat logs
   - Improve knowledge base
   - Update model if needed

## Support & SLA

| Metric | Target |
|--------|--------|
| Availability | 99.9% |
| Response Time | <3 seconds |
| Error Rate | <0.1% |
| Support Response | 1 hour |

## References

- Vercel: https://vercel.com/docs
- Railway: https://docs.railway.app/
- Render: https://render.com/docs
- Docker: https://docs.docker.com/
- Google Cloud Run: https://cloud.google.com/run/docs

---

**Estimated Deployment Time**: 30-60 minutes  
**Maintenance**: Daily monitoring, Weekly updates
