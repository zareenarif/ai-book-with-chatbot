---
id: 002
title: RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: 001-rag-chatbot
user: DC
command: /sp.plan
labels: ["plan", "rag-chatbot", "fastapi", "qdrant", "neon-db", "react", "architecture", "implementation"]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/plan.md
  - specs/001-rag-chatbot/research.md
  - specs/001-rag-chatbot/data-model.md
  - specs/001-rag-chatbot/quickstart.md
  - specs/001-rag-chatbot/contracts/openapi.yaml
tests:
  - Constitution check (all 7 principles passed)
  - Technology stack validation (15 decisions documented)
---

## Prompt

/sp.plan Create step-by-step implementation plan:

- Backend plan (FastAPI)
- Frontend plan (ChatWidget + hooks)
- Embedding pipeline plan
- Qdrant vector sync plan
- Neon DB plan (users, personalization)
- RAG pipeline plan
- Docusaurus integration plan
- BetterAuth signup/signin flow
- Personalization button logic
- Urdu translation toggle logic
- Claude Subagents + Skills plan
- Deployment pipeline

Break plan into stages for Task generation.

## Response snapshot

Created comprehensive implementation plan for RAG Chatbot Integration with 7 implementation stages:

**Planning Artifacts Created**:

1. **plan.md** (515 lines):
   - Technical context: Python 3.11+, FastAPI, React 18, OpenAI SDK, Qdrant, Neon Postgres
   - Constitution check: All 7 principles ✅ PASSED
   - Project structure: Backend (FastAPI), Frontend (Docusaurus/React)
   - 7 implementation stages with tasks, acceptance criteria, deliverables
   - Risk mitigation strategies (6 risks identified)
   - ADR suggestions for 3 architectural decisions

2. **research.md** (600+ lines):
   - 15 technology decisions with rationale and alternatives
   - FastAPI (backend), Qdrant Cloud (vector DB), Neon Postgres (relational DB)
   - BetterAuth (auth), OpenAI text-embedding-3-small + gpt-4o-mini (AI)
   - Docusaurus + React + Tailwind CSS (frontend), i18next (translation)
   - slowapi (rate limiting), pytest + Jest (testing), Sentry (monitoring)

3. **data-model.md** (450+ lines):
   - 3 Postgres tables: users, chat_messages, chat_sessions
   - 1 Qdrant collection: book_content (1536-dim vectors)
   - 11 Pydantic models for API validation
   - 5 state transition flows
   - Alembic migration scripts

4. **contracts/openapi.yaml** (550+ lines):
   - 12 API endpoints: /chat, /search, /history, /auth/*, /embed, /health
   - Full request/response schemas
   - Security schemes (BearerAuth, AdminAuth)
   - Error responses (400, 401, 429, 500)

5. **quickstart.md** (350+ lines):
   - Prerequisites (Python 3.11+, Node.js 18+, API keys)
   - Backend setup (venv, requirements.txt, Alembic migrations)
   - Frontend setup (npm install, i18next, ChatWidget)
   - Initial data setup (index book content, create test user)
   - Troubleshooting guide (6 common issues with solutions)

**Implementation Stages**:

- **Stage 1: Backend Foundation** (P0 - Blocking)
  - FastAPI app structure, Neon Postgres connection, Alembic migrations
  - Health check endpoint, CORS middleware
  - Acceptance: `curl /health` returns OK, tables created

- **Stage 2: RAG Pipeline** (P0 - Blocking)
  - Qdrant Cloud connection, collection creation
  - OpenAI embedding generation, batch indexing script
  - Semantic search implementation, `/search` endpoint
  - Acceptance: Index 500+ sections, search returns relevant results

- **Stage 3: Chat Completion** (P0 - Blocking)
  - OpenAI ChatKit integration, system prompt template
  - `/chat` endpoint with retrieval + generation
  - Session handling, chat history storage, rate limiting
  - Acceptance: <3s response time, grounded answers with sources

- **Stage 4: Authentication & User Management** (P1)
  - BetterAuth signup/signin, JWT token generation
  - Email verification (SendGrid), password reset flow
  - Google OAuth, `/history` endpoint, preferences API
  - Acceptance: User signup, signin, JWT-protected endpoints work

- **Stage 5: Frontend ChatWidget** (P1)
  - React components (ChatWidget, ChatMessage, ChatInput, etc.)
  - Custom hooks (useSelectedText, useAuth, useChatHistory)
  - i18next integration (English + Urdu), API client
  - Acceptance: ChatWidget on all pages, highlight text flow works

- **Stage 6: Personalization & Polish** (P2)
  - Chapter personalization button, score boosting logic
  - Source reference links, chat history pagination
  - Accessibility (keyboard nav, ARIA labels), performance optimization
  - Acceptance: Personalization boosts chapter relevance by 90%

- **Stage 7: Deployment & Monitoring** (P2)
  - Render (backend) + Vercel (frontend) deployment
  - GitHub Actions CI/CD, Sentry error tracking
  - Structured logging, cleanup jobs, load testing
  - Acceptance: Production deployment, 100 concurrent users supported

**Technology Stack Summary**:
- Backend: FastAPI 0.104+ + Python 3.11+
- Frontend: Docusaurus 3.x + React 18 + Tailwind CSS 3.x
- Vector DB: Qdrant Cloud (free tier)
- Relational DB: Neon Serverless Postgres (free tier)
- AI: OpenAI text-embedding-3-small + gpt-4o-mini
- Auth: BetterAuth + JWT + SendGrid
- Testing: pytest 7.x + Jest + React Testing Library
- Deployment: Render + Vercel + GitHub Actions
- Monitoring: Sentry + structlog

**Claude Subagents Architecture**:
- **RAG-Agent**: Stages 2-3 (embedding, search, retrieval)
- **UI-Agent**: Stage 5 (frontend components, hooks)
- **Auth-Agent**: Stage 4 (authentication, user management)
- **Skill-Agent**: Cross-stage (query reformulation, answer validation)

## Outcome

- ✅ Impact: Complete implementation plan with 7 stages, 15 technology decisions, 4 detailed planning artifacts (plan.md, research.md, data-model.md, quickstart.md), API contracts (OpenAPI 3.0 spec)
- 🧪 Tests: Constitution check validation (7/7 principles passed), Technology stack validation (15 decisions documented with rationale and alternatives)
- 📁 Files: plan.md (515 lines), research.md (600+ lines), data-model.md (450+ lines), contracts/openapi.yaml (550+ lines), quickstart.md (350+ lines)
- 🔁 Next prompts: Run `/sp.tasks` to generate actionable tasks from plan stages, Create ADR for technology stack `/sp.adr rag-chatbot-technology-stack`, Begin Stage 1 implementation
- 🧠 Reflection: Plan breaks down complex RAG chatbot into 7 systematic stages with clear acceptance criteria and deliverables. All 50 functional requirements from spec.md mapped to implementation stages. Constitution principles satisfied without violations. Ready for task generation and parallel development by Claude Subagents.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell script context update extracted placeholder text (needs manual update with actual tech stack), PHR bash script not found (resolved with agent-native creation)
- Graders run and results (PASS/FAIL): Constitution check (PASS - 7/7 principles), Technology stack completeness (PASS - 15 decisions documented), Data model completeness (PASS - 3 tables + 1 collection + 11 Pydantic models), API contracts (PASS - 12 endpoints with full schemas)
- Prompt variant (if applicable): Standard planning workflow with explicit stage breakdown for task generation
- Next experiment (smallest change to try): Run `/sp.tasks` to generate granular, testable tasks for Stage 1 (Backend Foundation) with acceptance tests for each task