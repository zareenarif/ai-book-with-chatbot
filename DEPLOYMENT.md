# Deployment Guide

## Overview

This RAG Chatbot application uses the following deployment architecture:

- **Backend**: Render (FastAPI)
- **Frontend**: Vercel (Docusaurus)
- **Vector DB**: Qdrant Cloud (managed)
- **Relational DB**: Neon Serverless Postgres (managed)
- **CI/CD**: GitHub Actions

---

## Prerequisites

1. **GitHub Repository**: Push your code to GitHub
2. **Accounts**:
   - Render account (https://render.com)
   - Vercel account (https://vercel.com)
   - Qdrant Cloud account (https://cloud.qdrant.io)
   - Neon account (https://neon.tech)
   - SendGrid account (for emails)
   - OpenAI API key

---

## Backend Deployment (Render)

### 1. Create `render.yaml`

Create `backend/render.yaml`:

```yaml
services:
  - type: web
    name: rag-chatbot-api
    env: python
    region: oregon
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn src.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: DATABASE_URL
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: OPENAI_API_KEY
        sync: false
      - key: JWT_SECRET
        sync: false
      - key: SENDGRID_API_KEY
        sync: false
      - key: SENDGRID_FROM_EMAIL
        sync: false
      - key: SENTRY_DSN
        sync: false
      - key: ENV
        value: production
      - key: DEBUG
        value: False
```

### 2. Deploy to Render

1. Go to https://render.com/dashboard
2. Click "New +" → "Web Service"
3. Connect your GitHub repository
4. Select `backend` directory
5. Use the following settings:
   - **Name**: `rag-chatbot-api`
   - **Region**: Oregon (or closest to your users)
   - **Branch**: `main` (or `001-rag-chatbot`)
   - **Root Directory**: `backend`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
6. Add environment variables (see `.env.example`)
7. Click "Create Web Service"

### 3. Run Database Migrations

After first deployment, connect to Render shell and run:

```bash
alembic upgrade head
```

### 4. Initialize Qdrant Collection

```bash
python scripts/init_qdrant_collection.py
```

### 5. Index Book Content

```bash
python scripts/index_book_content.py
```

**Backend URL**: `https://rag-chatbot-api.onrender.com`

---

## Frontend Deployment (Vercel)

### 1. Configure Vercel Project

1. Go to https://vercel.com/dashboard
2. Click "Add New..." → "Project"
3. Import your GitHub repository
4. Configure project:
   - **Framework Preset**: Docusaurus 2
   - **Root Directory**: `./` (leave blank)
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`

### 2. Set Environment Variables

In Vercel project settings → Environment Variables:

```
REACT_APP_API_URL=https://rag-chatbot-api.onrender.com/v1
```

### 3. Deploy

Click "Deploy"

**Frontend URL**: `https://your-project.vercel.app`

---

## Database Setup

### Neon Postgres

1. Go to https://neon.tech/dashboard
2. Create new project: "rag-chatbot"
3. Copy connection string (format: `postgresql://...`)
4. Update `DATABASE_URL` in Render environment variables
5. Run migrations from Render shell (see Backend step 3)

### Qdrant Cloud

1. Go to https://cloud.qdrant.io
2. Create new cluster (free tier)
3. Copy:
   - **URL**: `https://xxxx.cloud.qdrant.io:6333`
   - **API Key**: (from cluster settings)
4. Update `QDRANT_URL` and `QDRANT_API_KEY` in Render
5. Initialize collection (see Backend step 4)

---

## CI/CD with GitHub Actions

### 1. Create `.github/workflows/backend-tests.yml`

```yaml
name: Backend Tests

on:
  pull_request:
    branches: [main]
    paths:
      - 'backend/**'

jobs:
  test:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.11'
      - name: Install dependencies
        run: |
          cd backend
          pip install -r requirements.txt
      - name: Run tests
        run: |
          cd backend
          pytest tests/ -v
```

### 2. Create `.github/workflows/frontend-tests.yml`

```yaml
name: Frontend Tests

on:
  pull_request:
    branches: [main]
    paths:
      - 'src/**'
      - 'package.json'

jobs:
  test:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
      - name: Install dependencies
        run: npm install
      - name: Run tests
        run: npm test
```

### 3. Create `.github/workflows/deploy.yml`

```yaml
name: Deploy

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Trigger Render Deploy
        run: echo "Render auto-deploys on push to main"
      - name: Trigger Vercel Deploy
        run: echo "Vercel auto-deploys on push to main"
```

---

## Monitoring & Error Tracking (Sentry)

### 1. Create Sentry Project

1. Go to https://sentry.io
2. Create new project → Python (backend) and React (frontend)
3. Copy DSN

### 2. Add to Environment Variables

Render (backend):
```
SENTRY_DSN=https://xxxxx@sentry.io/xxxxx
```

Vercel (frontend):
```
REACT_APP_SENTRY_DSN=https://xxxxx@sentry.io/xxxxx
```

---

## Post-Deployment Checklist

### Backend

- [ ] Health check works: `curl https://rag-chatbot-api.onrender.com/v1/health`
- [ ] Database migrations completed
- [ ] Qdrant collection created and indexed
- [ ] Book content indexed (500+ sections)
- [ ] Test chat endpoint:

```bash
curl -X POST https://rag-chatbot-api.onrender.com/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is a ROS 2 node?",
    "session_id": "test123"
  }'
```

### Frontend

- [ ] Frontend loads at Vercel URL
- [ ] ChatWidget appears on all pages
- [ ] Can send messages and receive responses
- [ ] Authentication works (signup/signin)
- [ ] Highlighted text flow works
- [ ] Language toggle works (English ↔ Urdu)
- [ ] Personalization button appears on chapter pages

### Integration

- [ ] Frontend can reach backend API
- [ ] CORS configured correctly
- [ ] JWT authentication works
- [ ] Rate limiting works
- [ ] Email verification emails sent
- [ ] Sentry capturing errors

---

## Load Testing

Test with 100 concurrent users:

```bash
# Install locust
pip install locust

# Create locustfile.py
cat > locustfile.py << 'EOF'
from locust import HttpUser, task, between

class ChatbotUser(HttpUser):
    wait_time = between(1, 3)

    @task
    def chat(self):
        self.client.post("/v1/chat", json={
            "message": "What is a ROS 2 node?",
            "session_id": "load-test"
        })
EOF

# Run load test
locust -f locustfile.py --host https://rag-chatbot-api.onrender.com
```

**Success Criteria**:
- Response time < 3s (p95)
- Error rate < 1%
- Handles 100 concurrent users

---

## Rollback Procedure

### Backend (Render)

1. Go to Render dashboard → rag-chatbot-api
2. Click "Events" tab
3. Find previous successful deploy
4. Click "Rollback to this deploy"

### Frontend (Vercel)

1. Go to Vercel dashboard → project
2. Click "Deployments" tab
3. Find previous deployment
4. Click "..." → "Promote to Production"

---

## Maintenance

### Database Cleanup

Schedule a cron job on Render to clean up old sessions:

```bash
# Add to render.yaml
  - type: cron
    name: cleanup-sessions
    env: python
    schedule: "0 0 * * *"  # Daily at midnight
    buildCommand: pip install -r requirements.txt
    startCommand: python scripts/cleanup_expired_sessions.py
```

Create `backend/scripts/cleanup_expired_sessions.py`:

```python
import asyncio
from datetime import datetime, timedelta
from sqlalchemy import delete
from src.db.session import AsyncSessionLocal
from src.db.models import ChatSession

async def cleanup():
    async with AsyncSessionLocal() as session:
        cutoff = datetime.utcnow() - timedelta(hours=24)
        await session.execute(
            delete(ChatSession).where(ChatSession.expires_at < cutoff)
        )
        await session.commit()
    print("Cleaned up expired sessions")

if __name__ == "__main__":
    asyncio.run(cleanup())
```

---

## Troubleshooting

### Backend won't start

- Check Render logs: Dashboard → rag-chatbot-api → Logs
- Verify all environment variables are set
- Check DATABASE_URL format
- Verify Qdrant and Neon are accessible

### Frontend can't reach backend

- Check CORS settings in `backend/src/main.py`
- Verify `REACT_APP_API_URL` in Vercel
- Check network tab in browser DevTools

### Database connection errors

- Verify Neon database is active
- Check connection string includes `?sslmode=require`
- Test connection: `psql $DATABASE_URL`

### Qdrant errors

- Verify cluster is running
- Check API key is correct
- Verify collection exists

---

## Support

For issues:
- Backend logs: Render dashboard
- Frontend logs: Vercel dashboard
- Errors: Sentry dashboard
- Database: Neon dashboard
- Vector DB: Qdrant Cloud dashboard
