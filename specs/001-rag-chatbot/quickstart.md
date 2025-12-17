# Quickstart Guide: RAG Chatbot Integration

**Feature**: RAG Chatbot Integration
**Branch**: `001-rag-chatbot`
**Date**: 2025-12-10

This guide helps developers set up a local development environment for the RAG Chatbot feature.

---

## Prerequisites

- **Python 3.11+** ([Download](https://www.python.org/downloads/))
- **Node.js 18+** and npm ([Download](https://nodejs.org/))
- **Git** ([Download](https://git-scm.com/))
- **OpenAI API Key** ([Get API Key](https://platform.openai.com/api-keys))
- **Qdrant Cloud Account** ([Sign up](https://cloud.qdrant.io/))
- **Neon Postgres Account** ([Sign up](https://neon.tech/))

---

## Step 1: Clone Repository

```bash
git clone <repository-url>
cd hackathon-claude
git checkout 001-rag-chatbot
```

---

## Step 2: Backend Setup

### 2.1 Create Virtual Environment

```bash
cd backend
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Mac/Linux:
source venv/bin/activate
```

### 2.2 Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**requirements.txt**:
```
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
pydantic-settings==2.1.0
python-dotenv==1.0.0
sqlalchemy[asyncio]==2.0.23
asyncpg==0.29.0
alembic==1.13.0
bcrypt==4.1.1
python-jose[cryptography]==3.3.0
passlib==1.7.4
python-multipart==0.0.6
slowapi==0.1.9
qdrant-client==1.7.0
openai==1.3.0
sendgrid==6.10.0
sentry-sdk[fastapi]==1.38.0
structlog==23.2.0
pytest==7.4.3
pytest-asyncio==0.21.1
httpx==0.25.2
```

### 2.3 Configure Environment Variables

Create `backend/.env`:

```env
# Application
ENV=development
DEBUG=True
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=http://localhost:3000,http://127.0.0.1:3000

# Database (Neon Postgres)
DATABASE_URL=postgresql+asyncpg://user:password@ep-xxxx.neon.tech/dbname?sslmode=require

# Qdrant Cloud
QDRANT_URL=https://xxxx.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_content

# OpenAI
OPENAI_API_KEY=sk-xxxxxxxxxxxxxxxxxxxxxxxx
EMBEDDING_MODEL=text-embedding-3-small
CHAT_MODEL=gpt-4o-mini

# JWT Authentication
JWT_SECRET=your-super-secret-jwt-key-change-in-production
JWT_ALGORITHM=HS256
JWT_EXPIRE_MINUTES=15
REFRESH_TOKEN_EXPIRE_DAYS=7

# SendGrid (Email)
SENDGRID_API_KEY=SG.xxxxxxxxxxxxxxxx
SENDGRID_FROM_EMAIL=noreply@example.com
SENDGRID_FROM_NAME=RAG Chatbot

# Sentry (Error Tracking)
SENTRY_DSN=https://xxxxx@sentry.io/xxxxx

# Rate Limiting
RATE_LIMIT_ENABLED=True
RATE_LIMIT_CHAT=100/minute
RATE_LIMIT_AUTH=5/15minute
```

### 2.4 Initialize Database

```bash
# Run Alembic migrations
alembic upgrade head
```

### 2.5 Run Backend Server

```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Backend API now available at: `http://localhost:8000`
API documentation: `http://localhost:8000/docs` (Swagger UI)

---

## Step 3: Frontend Setup

### 3.1 Install Dependencies

```bash
cd ../  # Return to repo root
npm install
```

**Key Dependencies** (added to package.json):
```json
{
  "dependencies": {
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "react": "^18.2.0",
    "react-dom": "^18.2.0",
    "i18next": "^23.7.0",
    "react-i18next": "^13.5.0",
    "uuid": "^9.0.1"
  },
  "devDependencies": {
    "@testing-library/react": "^14.1.2",
    "@testing-library/jest-dom": "^6.1.5",
    "@testing-library/user-event": "^14.5.1",
    "jest": "^29.7.0",
    "tailwindcss": "^3.3.6"
  }
}
```

### 3.2 Configure Environment Variables

Create `.env.local` in root:

```env
REACT_APP_API_URL=http://localhost:8000/v1
```

### 3.3 Run Development Server

```bash
npm start
```

Docusaurus site now available at: `http://localhost:3000`

---

## Step 4: Initial Data Setup

### 4.1 Index Book Content

Trigger embedding generation for book content:

```bash
curl -X POST http://localhost:8000/v1/embed \
  -H "Content-Type: application/json" \
  -d '{
    "content_chunks": [
      {
        "chapter": "Chapter 1",
        "section": "Introduction",
        "content": "This textbook covers Physical AI and Humanoid Robotics...",
        "url": "/docs/chapter-1/introduction"
      }
    ]
  }'
```

**Automated Indexing Script** (future task):
- Create `backend/scripts/index_book_content.py`
- Parses all Markdown files in `docs/` directory
- Extracts H2/H3 sections
- Calls `/embed` endpoint for each section

### 4.2 Create Test User

```bash
curl -X POST http://localhost:8000/v1/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "TestPass123!"
  }'
```

---

## Step 5: Verify Setup

### 5.1 Test Backend Health Check

```bash
curl http://localhost:8000/v1/health
```

Expected response:
```json
{
  "status": "ok",
  "timestamp": "2025-12-10T10:00:00Z"
}
```

### 5.2 Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is a ROS 2 node?",
    "session_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890"
  }'
```

Expected response:
```json
{
  "message": "A ROS 2 node is a participant in the ROS graph...",
  "role": "assistant",
  "sources": [
    {
      "chapter": "Chapter 3",
      "section": "ROS 2 Architecture",
      "url": "/docs/chapter-3/ros2-architecture",
      "score": 0.89
    }
  ],
  "timestamp": "2025-12-10T10:00:02Z"
}
```

### 5.3 Test Frontend ChatWidget

1. Open `http://localhost:3000/docs/chapter-3/ros2-architecture`
2. Click the chatbot icon (bottom-right corner)
3. Type a question: "What is a ROS 2 node?"
4. Verify response appears with source references

---

## Project Structure

```
hackathon-claude/
├── backend/                    # FastAPI backend
│   ├── src/
│   │   ├── api/               # API route handlers
│   │   │   ├── chat.py
│   │   │   ├── auth.py
│   │   │   └── admin.py
│   │   ├── services/          # Business logic
│   │   │   ├── rag_service.py
│   │   │   ├── auth_service.py
│   │   │   └── embedding_service.py
│   │   ├── models/            # Pydantic schemas
│   │   │   ├── user.py
│   │   │   ├── chat.py
│   │   │   └── embedding.py
│   │   ├── db/                # Database models & config
│   │   │   ├── base.py
│   │   │   ├── models.py
│   │   │   └── session.py
│   │   ├── middleware/        # CORS, rate limiting
│   │   └── main.py            # FastAPI app entry point
│   ├── tests/                 # Backend tests
│   │   ├── unit/
│   │   ├── integration/
│   │   └── conftest.py
│   ├── alembic/               # Database migrations
│   ├── requirements.txt
│   └── .env
├── src/                       # Docusaurus frontend
│   ├── components/
│   │   └── ChatWidget/        # Chatbot UI components
│   │       ├── ChatWidget.jsx
│   │       ├── ChatMessage.jsx
│   │       ├── ChatInput.jsx
│   │       ├── HighlightedTextButton.jsx
│   │       ├── useSelectedText.js
│   │       └── chat.css
│   ├── theme/                 # Docusaurus theme customization
│   └── locales/               # i18next translation files
│       ├── en.json
│       └── ur.json
├── docs/                      # Book content (Markdown)
├── specs/                     # Feature specifications
│   └── 001-rag-chatbot/
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── quickstart.md
│       └── contracts/
│           └── openapi.yaml
├── docusaurus.config.js
├── package.json
└── .env.local
```

---

## Common Issues & Solutions

### Issue: Database connection error

**Error**: `asyncpg.exceptions.InvalidPasswordError`

**Solution**:
1. Verify `DATABASE_URL` in `backend/.env`
2. Ensure Neon Postgres database is active (check Neon dashboard)
3. Confirm connection string includes `?sslmode=require`

---

### Issue: Qdrant connection failed

**Error**: `qdrant_client.exceptions.UnexpectedResponse: 401 Unauthorized`

**Solution**:
1. Verify `QDRANT_API_KEY` in `backend/.env`
2. Ensure Qdrant cluster is running (check Qdrant Cloud dashboard)
3. Check `QDRANT_URL` format: `https://xxxx.cloud.qdrant.io:6333`

---

### Issue: OpenAI API rate limit

**Error**: `openai.error.RateLimitError: Rate limit exceeded`

**Solution**:
1. Implement exponential backoff in embedding service
2. Reduce batch size in `/embed` endpoint (default: 100 → 50)
3. Consider upgrading OpenAI API tier

---

### Issue: Frontend can't reach backend API

**Error**: `NetworkError: Failed to fetch`

**Solution**:
1. Verify backend is running: `curl http://localhost:8000/v1/health`
2. Check `REACT_APP_API_URL` in `.env.local`
3. Ensure CORS is configured in `backend/src/main.py`:
   ```python
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["http://localhost:3000"],
       allow_credentials=True,
       allow_methods=["*"],
       allow_headers=["*"]
   )
   ```

---

## Running Tests

### Backend Tests

```bash
cd backend
pytest tests/ -v --cov=src
```

### Frontend Tests

```bash
npm test
```

---

## Next Steps

1. **Implement Backend API Endpoints**: Start with `/chat` endpoint (highest priority)
2. **Build ChatWidget Component**: Create basic UI with text input and message display
3. **Integrate Authentication**: Implement BetterAuth signup/signin flows
4. **Index Book Content**: Run embedding generation for all Markdown files
5. **Test End-to-End**: Verify complete flow from user query → retrieval → answer

---

## Useful Commands

```bash
# Backend
uvicorn src.main:app --reload                 # Run dev server
alembic revision --autogenerate -m "message"  # Create migration
alembic upgrade head                          # Apply migrations
pytest tests/ -v                              # Run tests
python -m src.scripts.index_content           # Index book content

# Frontend
npm start                                     # Run dev server
npm run build                                 # Build for production
npm test                                      # Run tests
npm run lint                                  # Lint code

# Database
psql $DATABASE_URL                            # Connect to Neon Postgres
psql $DATABASE_URL -c "SELECT * FROM users;"  # Query users table

# Qdrant
curl $QDRANT_URL/collections                  # List collections
curl $QDRANT_URL/collections/book_content     # Get collection info
```

---

## Support

For issues or questions:
- Check `specs/001-rag-chatbot/spec.md` for requirements
- Review `specs/001-rag-chatbot/data-model.md` for database schemas
- See API documentation: `http://localhost:8000/docs`
- Contact: [support@example.com](mailto:support@example.com)
