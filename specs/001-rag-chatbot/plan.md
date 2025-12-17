# Implementation Plan: RAG Chatbot Integration

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics Docusaurus textbook. The chatbot provides instant, grounded answers to student questions using semantic search over vector embeddings of book content. Key features: highlighted text queries, user authentication with chat history persistence, chapter-specific personalization, and Urdu UI translation.

**Technical Approach**: FastAPI backend with OpenAI embeddings + ChatKit SDK, Qdrant Cloud for vector storage, Neon Serverless Postgres for user data, BetterAuth for authentication, React/Docusaurus frontend with custom ChatWidget component, deployed on Render (backend) + Vercel (frontend).

---

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/TypeScript (frontend)
**Primary Dependencies**: FastAPI 0.104+, React 18, OpenAI SDK 1.3+, Qdrant Client 1.7+, SQLAlchemy 2.0 (async), BetterAuth, i18next 23.x
**Storage**: Neon Serverless Postgres (users, chat history), Qdrant Cloud (vector embeddings)
**Testing**: pytest 7.x + pytest-asyncio (backend), Jest + React Testing Library (frontend)
**Target Platform**: Linux server (Render), Static hosting (Vercel), Cloud services (Qdrant Cloud, Neon Postgres)
**Project Type**: Web (split backend/frontend)
**Performance Goals**: <3s response time (95%), 100 concurrent users, <10min embedding generation (500+ sections)
**Constraints**: OpenAI API free tier/budget limits, Qdrant free tier (1GB, 100K vectors), Neon free tier (0.5GB storage)
**Scale/Scope**: 500+ book sections, estimated 1000+ users, 10K+ chat messages/month

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Principles Compliance**:

✅ **I. Academic Excellence & Pedagogical Rigor**:
- Chatbot answers MUST be grounded in book content only (FR-004, SC-003: 100% accuracy)
- Source references included in all responses (FR-005)
- Technical accuracy enforced via answer validation (Skill-Agent)

✅ **II. Production-Ready Code & Deployment**:
- All components fully implemented (no TODOs in final deliverables)
- Automated deployment via Vercel (frontend) + Render (backend)
- Exact dependency versions specified in requirements.txt and package.json

✅ **III. Modular Architecture & Component Reusability**:
- Clean separation: backend API (/api), business logic (/services), data (/models)
- Reusable React components (ChatWidget, ChatMessage, ChatInput)
- Claude Subagents provide modular intelligence (RAG-Agent, UI-Agent, Auth-Agent, Skill-Agent)

✅ **IV. Accessibility & User Experience**:
- Responsive ChatWidget design (mobile, tablet, desktop)
- Keyboard navigation support
- Loading indicators for long queries (FR-007)
- Urdu translation for accessibility (FR-041-045)

✅ **V. Content Completeness & Scope Coverage**:
- All 50 functional requirements addressed in implementation stages
- 5 prioritized user stories mapped to development phases

✅ **VI. Versioning, Maintenance & Evolution**:
- Database migrations via Alembic (semantic versioning)
- API versioning via `/v1/` prefix
- Changelog maintained for backend/frontend releases

✅ **VII. Testing & Quality Assurance**:
- Unit tests for all services (pytest)
- Component tests for React UI (Jest + RTL)
- Integration tests for API endpoints
- Contract tests validate OpenAPI schema compliance

**GATE STATUS**: ✅ PASSED (all principles addressed)

---

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── spec.md              # Feature specification (/sp.specify)
├── plan.md              # This file (/sp.plan)
├── research.md          # Phase 0 output (technology decisions)
├── data-model.md        # Phase 1 output (database schemas)
├── quickstart.md        # Phase 1 output (local dev setup)
├── contracts/           # Phase 1 output (API contracts)
│   └── openapi.yaml
├── checklists/          # Quality validation
│   └── requirements.md
└── tasks.md             # Phase 2 output (/sp.tasks - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
hackathon-claude/
├── backend/                    # FastAPI backend
│   ├── src/
│   │   ├── api/               # API route handlers
│   │   │   ├── __init__.py
│   │   │   ├── chat.py        # POST /chat, POST /search
│   │   │   ├── auth.py        # POST /auth/signup, /signin, etc.
│   │   │   └── admin.py       # POST /embed (admin only)
│   │   ├── services/          # Business logic
│   │   │   ├── __init__.py
│   │   │   ├── rag_service.py         # Qdrant search, retrieval
│   │   │   ├── auth_service.py        # BetterAuth integration
│   │   │   ├── embedding_service.py   # OpenAI embedding generation
│   │   │   └── chat_service.py        # ChatKit integration
│   │   ├── models/            # Pydantic schemas
│   │   │   ├── __init__.py
│   │   │   ├── user.py        # UserCreate, UserLogin, UserResponse
│   │   │   ├── chat.py        # ChatMessageRequest, ChatMessageResponse
│   │   │   └── embedding.py   # EmbedRequest, EmbedResponse
│   │   ├── db/                # Database models & config
│   │   │   ├── __init__.py
│   │   │   ├── base.py        # SQLAlchemy base declarative
│   │   │   ├── models.py      # User, ChatMessage, ChatSession tables
│   │   │   └── session.py     # Async database session factory
│   │   ├── middleware/        # CORS, rate limiting, error handling
│   │   │   ├── __init__.py
│   │   │   ├── cors.py
│   │   │   ├── rate_limit.py  # slowapi integration
│   │   │   └── error_handler.py
│   │   ├── utils/             # Helper functions
│   │   │   ├── __init__.py
│   │   │   ├── password.py    # Bcrypt hashing
│   │   │   ├── jwt.py         # JWT token generation/validation
│   │   │   └── email.py       # SendGrid email sending
│   │   ├── config.py          # Pydantic settings (env vars)
│   │   └── main.py            # FastAPI app entry point
│   ├── tests/                 # Backend tests
│   │   ├── __init__.py
│   │   ├── conftest.py        # Pytest fixtures (mock DB, Qdrant, OpenAI)
│   │   ├── unit/
│   │   │   ├── test_rag_service.py
│   │   │   ├── test_auth_service.py
│   │   │   └── test_embedding_service.py
│   │   ├── integration/
│   │   │   ├── test_chat_api.py
│   │   │   ├── test_auth_api.py
│   │   │   └── test_search_api.py
│   │   └── contract/
│   │       └── test_openapi_schema.py
│   ├── alembic/               # Database migrations
│   │   ├── versions/
│   │   │   └── 001_initial_schema.py
│   │   ├── env.py
│   │   └── alembic.ini
│   ├── scripts/               # Utility scripts
│   │   └── index_book_content.py  # Embed all Markdown files
│   ├── requirements.txt
│   ├── .env.example
│   └── README.md
│
├── src/                       # Docusaurus frontend
│   ├── components/
│   │   └── ChatWidget/        # Chatbot UI components
│   │       ├── ChatWidget.jsx           # Main container
│   │       ├── ChatHeader.jsx           # Title, settings, language toggle
│   │       ├── ChatMessage.jsx          # Single message bubble
│   │       ├── ChatInput.jsx            # Text input + send button
│   │       ├── ChatHistory.jsx          # Scrollable message list
│   │       ├── HighlightedTextButton.jsx # "Ask Chatbot" button
│   │       ├── LoadingIndicator.jsx     # Spinner animation
│   │       ├── SourceReference.jsx      # Clickable chapter/section link
│   │       ├── useSelectedText.js       # Custom hook for text highlighting
│   │       ├── useChatHistory.js        # Custom hook for chat state
│   │       ├── useAuth.js               # Custom hook for auth state
│   │       └── chat.css                 # Tailwind + custom styles
│   ├── context/               # React context providers
│   │   ├── AuthContext.jsx    # User authentication state
│   │   └── ChatContext.jsx    # Chat history state
│   ├── services/              # API client
│   │   └── api.js             # Axios client for backend API
│   ├── theme/                 # Docusaurus theme customization
│   │   └── Root.js            # i18next initialization
│   └── locales/               # i18next translation files
│       ├── en.json            # English UI labels
│       └── ur.json            # Urdu UI labels
│
├── docs/                      # Book content (Markdown)
├── specs/                     # Feature specifications
├── history/                   # Prompt history & ADRs
├── .specify/                  # SpecKit Plus templates
├── docusaurus.config.js
├── package.json
├── .env.local.example
└── README.md
```

**Structure Decision**: Web application structure (Option 2) selected due to clear backend (FastAPI) + frontend (Docusaurus/React) separation. This enables independent deployment, testing, and scaling of each tier.

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

N/A - All constitution principles satisfied without violations.

---

## Implementation Stages

Implementation is broken into 5 stages for systematic development and testing. Each stage builds on the previous one and delivers testable value.

---

### **Stage 1: Backend Foundation** (Priority: P0 - Blocking)

**Goal**: Set up FastAPI backend with database connectivity, API structure, and health checks.

**Tasks**:
1. Initialize FastAPI project structure (`backend/src/`)
2. Configure environment variables (`backend/.env`, `backend/config.py`)
3. Set up Neon Postgres connection (SQLAlchemy async engine + session factory)
4. Create database models (`backend/src/db/models.py`): User, ChatMessage, ChatSession
5. Initialize Alembic and create initial migration (`001_initial_schema.py`)
6. Run migrations on Neon database
7. Implement health check endpoint (`GET /health`)
8. Set up CORS middleware for frontend communication
9. Write unit tests for database models

**Acceptance Criteria**:
- `curl http://localhost:8000/v1/health` returns `{"status": "ok"}`
- Database tables created in Neon Postgres (verify with `psql $DATABASE_URL -c "\dt"`)
- All tests pass: `pytest backend/tests/unit/test_models.py -v`

**Deliverables**:
- `backend/src/main.py` (FastAPI app)
- `backend/src/config.py` (Pydantic settings)
- `backend/src/db/` (models, session)
- `backend/alembic/versions/001_initial_schema.py`
- `backend/tests/unit/test_models.py`

**Dependencies**: Neon Postgres account, DATABASE_URL configured

---

### **Stage 2: RAG Pipeline (Embedding + Search)** (Priority: P0 - Blocking)

**Goal**: Implement embedding generation, Qdrant indexing, and semantic search.

**Tasks**:
1. Set up Qdrant Cloud connection (`backend/src/services/rag_service.py`)
2. Create Qdrant collection with cosine similarity metric
3. Implement `EmbeddingService.embed_text()` (OpenAI API integration)
4. Implement `POST /embed` endpoint (admin-only, triggers indexing)
5. Create script to parse Markdown files and extract H2/H3 sections (`backend/scripts/index_book_content.py`)
6. Implement batch embedding generation (chunks of 100 sections)
7. Implement Qdrant upsert with deduplication (content_hash as UUID seed)
8. Implement `RAGService.search()` (semantic search over Qdrant)
9. Implement `POST /search` endpoint (returns top 5 results)
10. Write unit tests for RAGService and EmbeddingService
11. Write integration test for full indexing + search flow

**Acceptance Criteria**:
- Run `python backend/scripts/index_book_content.py` to index all Markdown files
- Verify embeddings in Qdrant: `curl $QDRANT_URL/collections/book_content/points/count`
- `curl -X POST http://localhost:8000/v1/search -d '{"query": "What is ROS 2?"}' ` returns relevant results
- All tests pass: `pytest backend/tests/ -k rag -v`

**Deliverables**:
- `backend/src/services/rag_service.py`
- `backend/src/services/embedding_service.py`
- `backend/src/api/admin.py` (POST /embed)
- `backend/src/api/chat.py` (POST /search)
- `backend/scripts/index_book_content.py`
- `backend/tests/unit/test_rag_service.py`
- `backend/tests/integration/test_search_api.py`

**Dependencies**: OpenAI API key, Qdrant Cloud URL + API key

---

### **Stage 3: Chat Completion** (Priority: P0 - Blocking)

**Goal**: Implement conversational AI with grounded answer generation.

**Tasks**:
1. Implement `ChatService.generate_answer()` (OpenAI ChatKit SDK integration)
2. Create system prompt template for grounded responses
3. Implement `POST /chat` endpoint:
   - Accept user query
   - Generate embedding for query
   - Search Qdrant (top 5 results)
   - Pass results + query to ChatKit
   - Validate answer is grounded (Skill-Agent logic)
   - Return answer + sources
4. Implement session handling for unauthenticated users (ChatSession table)
5. Implement chat history storage for authenticated users (ChatMessage table)
6. Add rate limiting (100 req/min per user)
7. Write unit tests for ChatService
8. Write integration test for POST /chat endpoint
9. Test highlighted text query flow (priority exact phrase matching)

**Acceptance Criteria**:
- `curl -X POST http://localhost:8000/v1/chat -d '{"message": "What is a ROS 2 node?", "session_id": "uuid"}' ` returns grounded answer with sources
- Response time <3s for 95% of queries (load test with 10 concurrent users)
- All tests pass: `pytest backend/tests/ -k chat -v`

**Deliverables**:
- `backend/src/services/chat_service.py`
- `backend/src/api/chat.py` (POST /chat updated)
- `backend/tests/unit/test_chat_service.py`
- `backend/tests/integration/test_chat_api.py`

**Dependencies**: OpenAI API key, Qdrant indexed with book content

---

### **Stage 4: Authentication & User Management** (Priority: P1)

**Goal**: Implement BetterAuth signup/signin flows with JWT tokens.

**Tasks**:
1. Implement `AuthService.create_user()` (password hashing with Bcrypt)
2. Implement `POST /auth/signup` endpoint (email validation, password strength check)
3. Implement email verification flow (SendGrid integration)
4. Implement `POST /auth/signin` endpoint (JWT access token generation)
5. Implement JWT middleware for protected endpoints
6. Implement Google OAuth flow (BetterAuth social login)
7. Implement `POST /auth/reset-password-request` and `POST /auth/reset-password`
8. Implement `GET /history` endpoint (retrieve chat history for authenticated users)
9. Implement `PUT /auth/preferences` endpoint (language, preferred_chapter)
10. Implement session migration (guest → authenticated user)
11. Write unit tests for AuthService
12. Write integration tests for all auth endpoints

**Acceptance Criteria**:
- User can sign up: `curl -X POST http://localhost:8000/v1/auth/signup -d '{"email": "test@example.com", "password": "Test123!"}'`
- User can sign in and receive JWT token
- Protected endpoint requires valid JWT: `curl -H "Authorization: Bearer <token>" http://localhost:8000/v1/history`
- All tests pass: `pytest backend/tests/ -k auth -v`

**Deliverables**:
- `backend/src/services/auth_service.py`
- `backend/src/api/auth.py`
- `backend/src/middleware/jwt.py`
- `backend/src/utils/password.py`, `backend/src/utils/jwt.py`, `backend/src/utils/email.py`
- `backend/tests/unit/test_auth_service.py`
- `backend/tests/integration/test_auth_api.py`

**Dependencies**: SendGrid API key, JWT_SECRET configured

---

### **Stage 5: Frontend ChatWidget** (Priority: P1)

**Goal**: Build React ChatWidget component with all UI interactions.

**Tasks**:
1. Create `ChatWidget.jsx` main container component
2. Create `ChatHeader.jsx` with language toggle and settings
3. Create `ChatMessage.jsx` for rendering user/assistant messages
4. Create `ChatInput.jsx` with text input + send button
5. Create `ChatHistory.jsx` for scrollable message list
6. Implement `useSelectedText.js` hook (text highlighting detection)
7. Create `HighlightedTextButton.jsx` (appears on text selection)
8. Implement API client (`src/services/api.js`) for backend communication
9. Implement `useAuth.js` hook (signup/signin, JWT storage in localStorage)
10. Implement `useChatHistory.js` hook (manage chat state, call /chat endpoint)
11. Integrate i18next for Urdu translation (locales/en.json, locales/ur.json)
12. Add ChatWidget to Docusaurus theme (`src/theme/Root.js`)
13. Implement loading indicators and error handling
14. Write component tests (Jest + React Testing Library)
15. Test end-to-end flow: highlight text → ask chatbot → receive answer

**Acceptance Criteria**:
- ChatWidget visible on all Docusaurus pages (bottom-right floating button)
- User can type question and receive answer with source references
- User can highlight text on page, click "Ask Chatbot", and query is pre-filled
- Language toggle switches UI to Urdu instantly (<100ms)
- User can sign up/sign in, and chat history persists across sessions
- All component tests pass: `npm test`

**Deliverables**:
- `src/components/ChatWidget/` (all React components)
- `src/services/api.js`
- `src/context/AuthContext.jsx`, `src/context/ChatContext.jsx`
- `src/locales/en.json`, `src/locales/ur.json`
- `src/components/ChatWidget/*.test.jsx` (Jest tests)

**Dependencies**: Backend API running at `REACT_APP_API_URL`

---

### **Stage 6: Personalization & Polish** (Priority: P2)

**Goal**: Implement chapter-specific personalization and final UX enhancements.

**Tasks**:
1. Add "Personalize for this Chapter" button to chapter pages (Docusaurus plugin or theme override)
2. Implement personalization logic (boost Qdrant scores by 2x for preferred chapter)
3. Update `POST /search` to accept `preferred_chapter` parameter
4. Display badge on chatbot answers indicating "Answer from Chapter 5" when personalization active
5. Implement "Reset Personalization" button
6. Add source reference links (ChatMessage component) that scroll to chapter/section
7. Implement pagination for chat history (50 messages per page)
8. Add accessibility features (keyboard navigation, ARIA labels)
9. Optimize performance (lazy-load ChatWidget, debounce input)
10. Write E2E tests for personalization flow

**Acceptance Criteria**:
- User can click "Personalize for Chapter 5" on Chapter 5 page
- Subsequent queries prioritize Chapter 5 content (verify scores in search results)
- User can reset personalization and queries search globally again
- Chatbot meets SC-009: Translation toggle <100ms, SC-010: Personalization improves relevance 90%

**Deliverables**:
- `src/components/PersonalizeButton.jsx`
- Updated `backend/src/services/rag_service.py` (score boosting)
- Updated `src/components/ChatWidget/ChatMessage.jsx` (source links, chapter badge)
- E2E tests in `backend/tests/integration/test_personalization.py`

**Dependencies**: Stages 1-5 complete

---

### **Stage 7: Deployment & Monitoring** (Priority: P2)

**Goal**: Deploy to production (Render + Vercel) and set up monitoring.

**Tasks**:
1. Configure Render backend deployment (connect GitHub repo, set env vars)
2. Configure Vercel frontend deployment (connect GitHub repo, set `REACT_APP_API_URL`)
3. Set up GitHub Actions CI/CD:
   - Run backend tests on PR
   - Run frontend tests on PR
   - Auto-deploy to Render/Vercel on merge to main
4. Set up Sentry for error tracking (backend + frontend)
5. Implement structured logging (structlog for backend)
6. Create periodic cleanup job for expired chat sessions (Render cron or background worker)
7. Write deployment runbook (specs/001-rag-chatbot/deployment.md)
8. Load test with 100 concurrent users (verify SC-002)
9. Monitor OpenAI API costs (set budget alerts)

**Acceptance Criteria**:
- Backend deployed at `https://rag-chatbot-api.onrender.com`
- Frontend deployed at `https://textbook.example.com` (Vercel)
- GitHub Actions CI/CD pipeline passing
- Sentry capturing errors in production
- Cleanup job runs daily, deletes sessions older than 24 hours

**Deliverables**:
- `backend/render.yaml` (Render config)
- `.github/workflows/ci.yml` (GitHub Actions)
- `specs/001-rag-chatbot/deployment.md` (runbook)
- Load test results (documented in deployment.md)

**Dependencies**: Render account, Vercel account, Sentry account, GitHub repo

---

## Architectural Decisions Requiring ADR

The following architecturally significant decisions were made during planning and should be documented:

1. **Vector Database Choice: Qdrant vs. Pinecone vs. FAISS**
   - Decision: Qdrant Cloud
   - Rationale: Free tier capacity, metadata filtering, production-ready
   - See: research.md Section 2

2. **Authentication Strategy: BetterAuth vs. Auth0 vs. Custom JWT**
   - Decision: BetterAuth (Python library)
   - Rationale: FastAPI integration, OAuth2 + JWT, social login support
   - See: research.md Section 4

3. **Frontend State Management: Redux vs. Context API vs. Zustand**
   - Decision: React Context API
   - Rationale: Simple state needs (auth, chat), avoid Redux boilerplate
   - See: research.md Section 7

📋 **Architectural decision detected**: Technology stack selection (FastAPI + Qdrant + Neon + BetterAuth + React)
   Document reasoning and tradeoffs? Run `/sp.adr rag-chatbot-technology-stack`

---

## Risk Mitigation Strategies

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| **OpenAI API costs exceed budget** | Medium | High | Monitor usage via OpenAI dashboard, set billing alerts at $50, implement caching for embeddings (content_hash deduplication), limit free-tier users to 100 queries/day |
| **Qdrant/Neon free tier capacity exceeded** | Low | Medium | Monitor storage via dashboards, implement data retention policy (delete chat sessions >30 days), compress embeddings if needed |
| **Search quality poor for vague queries** | High | Medium | Implement query reformulation (Skill-Agent), suggest related questions, provide feedback mechanism for users to report bad answers |
| **Hallucination despite grounding instructions** | Medium | High | Implement answer validation (Skill-Agent checks answer against retrieved chunks), log all prompts/responses for audit, A/B test different system prompts |
| **Performance degradation under load** | Low | Medium | Implement rate limiting, use CDN for frontend static assets, optimize Qdrant query (reduce `top_k` if needed), cache common queries |
| **Data privacy compliance (GDPR)** | Low | High | Add privacy policy, implement data deletion endpoint (`DELETE /user/{id}`), encrypt sensitive data at rest, obtain user consent for data storage |

---

## Next Steps

1. **Run `/sp.tasks`** to generate actionable implementation tasks from this plan
2. **Create ADR** for technology stack: `/sp.adr rag-chatbot-technology-stack`
3. **Begin Stage 1 implementation**: Set up FastAPI backend foundation
4. **Parallel work streams**:
   - RAG-Agent: Focus on Stages 2-3 (embedding + chat)
   - UI-Agent: Focus on Stage 5 (frontend)
   - Auth-Agent: Focus on Stage 4 (authentication)
   - Skill-Agent: Provide reusable utilities (query reformulation, answer validation)

---

## Appendix: Key Files Reference

- **Specification**: `specs/001-rag-chatbot/spec.md` (50 functional requirements, 10 success criteria)
- **Research**: `specs/001-rag-chatbot/research.md` (15 technology decisions with rationale)
- **Data Model**: `specs/001-rag-chatbot/data-model.md` (Postgres schema, Qdrant schema, Pydantic models)
- **API Contract**: `specs/001-rag-chatbot/contracts/openapi.yaml` (12 endpoints, request/response schemas)
- **Quickstart Guide**: `specs/001-rag-chatbot/quickstart.md` (local development setup)
- **Constitution**: `.specify/memory/constitution.md` (project principles and code standards)
