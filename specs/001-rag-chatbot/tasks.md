# Implementation Tasks: RAG Chatbot Integration

**Feature**: RAG Chatbot Integration
**Branch**: `001-rag-chatbot`
**Date**: 2025-12-10
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Task Overview

**Total Tasks**: 127
**User Stories**: 5 (P1: 2, P2: 1, P3: 2)
**Parallel Opportunities**: 45 tasks marked [P]
**Estimated MVP**: Phase 1-3 (US1-US2) = ~60 tasks

## User Story Mapping

- **US1** (P1): Ask Questions About Book Content → Core chatbot functionality
- **US2** (P1): Highlighted Text Queries → Text selection integration
- **US3** (P2): Account & Chat History → Authentication & persistence
- **US4** (P3): Chapter Personalization → Enhanced search ranking
- **US5** (P3): Urdu Translation → i18next UI translation

---

## Phase 1: Project Setup (Foundation)

**Goal**: Initialize project structure, dependencies, and environment configuration

**Independent Test**: Project builds successfully, health endpoint responds

### Tasks

- [ ] T001 Create backend directory structure per plan.md (backend/src/, backend/tests/, backend/alembic/)
- [ ] T002 [P] Create frontend directory structure per plan.md (src/components/ChatWidget/, src/context/, src/services/)
- [ ] T003 [P] Initialize Git repository and create .gitignore for Python and Node.js
- [ ] T004 Create backend/requirements.txt with exact versions from research.md
- [ ] T005 Create backend/.env.example with all required environment variables
- [ ] T006 [P] Create package.json with React 18, Docusaurus 3.x, i18next dependencies
- [ ] T007 [P] Create .env.local.example for frontend with REACT_APP_API_URL
- [ ] T008 Create backend/src/config.py using Pydantic Settings for environment variables
- [ ] T009 [P] Create backend/README.md with setup instructions from quickstart.md
- [ ] T010 [P] Create root README.md with project overview and quick links

**Acceptance**:
- Directory structure matches plan.md Section "Project Structure"
- All dependency files created with correct versions
- Environment variable templates documented

---

## Phase 2: Foundational Infrastructure (Blocking Prerequisites)

**Goal**: Set up database, FastAPI app, Qdrant connection - required by all user stories

**Independent Test**: FastAPI app starts, database connects, Qdrant accessible

### Database Setup

- [ ] T011 Create backend/src/db/base.py with SQLAlchemy declarative_base
- [ ] T012 Create backend/src/db/session.py with async engine and session factory
- [ ] T013 Create backend/src/db/models.py with User, ChatMessage, ChatSession models per data-model.md
- [ ] T014 Create backend/alembic/env.py configured for async migrations
- [ ] T015 Create backend/alembic/versions/001_initial_schema.py with tables from data-model.md
- [ ] T016 Initialize Alembic: Run `alembic init alembic` and configure alembic.ini

### FastAPI Core

- [ ] T017 Create backend/src/main.py with FastAPI app initialization
- [ ] T018 Create backend/src/middleware/cors.py with CORS configuration for localhost:3000
- [ ] T019 [P] Create backend/src/middleware/error_handler.py with global exception handling
- [ ] T020 [P] Create backend/src/api/__init__.py to organize routers
- [ ] T021 Create backend/src/api/health.py with GET /health endpoint returning {"status": "ok", "timestamp": now}

### Qdrant Setup

- [ ] T022 Create backend/src/services/qdrant_client.py with Qdrant Cloud connection singleton
- [ ] T023 Create script backend/scripts/init_qdrant_collection.py to create book_content collection with 1536-dim vectors, cosine similarity

### Environment & Testing Foundation

- [ ] T024 [P] Create backend/tests/conftest.py with pytest fixtures (mock DB, Qdrant, OpenAI)
- [ ] T025 [P] Create backend/tests/__init__.py
- [ ] T026 [P] Create backend/tests/unit/__init__.py
- [ ] T027 [P] Create backend/tests/integration/__init__.py

**Acceptance**:
- Run `alembic upgrade head` creates tables in Neon Postgres
- Run `uvicorn src.main:app` starts FastAPI at localhost:8000
- `curl http://localhost:8000/v1/health` returns `{"status": "ok"}`
- Qdrant collection `book_content` exists: `curl $QDRANT_URL/collections/book_content`

---

## Phase 3: User Story 1 - Ask Questions About Book Content (P1)

**Goal**: Core RAG chatbot - embedding, search, chat completion

**Independent Test**: User asks "What is a ROS 2 node?", receives grounded answer with source references

### Backend - Embedding Service

- [ ] T028 [US1] Create backend/src/models/embedding.py with EmbedRequest, EmbedResponse Pydantic models
- [ ] T029 [US1] Create backend/src/services/embedding_service.py with embed_text() method using OpenAI text-embedding-3-small
- [ ] T030 [US1] Add batch_embed() method to embedding_service.py for chunks of 100 texts
- [ ] T031 [US1] Create backend/src/utils/content_hash.py with SHA256 hashing for deduplication
- [ ] T032 [US1] Create backend/scripts/index_book_content.py to parse docs/ Markdown files, extract H2/H3 sections, generate embeddings, upsert to Qdrant

### Backend - RAG Service

- [ ] T033 [US1] Create backend/src/services/rag_service.py with search() method for semantic search over Qdrant
- [ ] T034 [US1] Add retrieve_top_k() method to rag_service.py returning top 5 results with metadata (chapter, section, url, score)
- [ ] T035 [US1] Create backend/src/models/chat.py with ChatMessageRequest, ChatMessageResponse, ChatSource Pydantic models per contracts/openapi.yaml

### Backend - Chat Service

- [ ] T036 [US1] Create backend/src/services/chat_service.py with generate_answer() method using OpenAI gpt-4o-mini
- [ ] T037 [US1] Create backend/src/utils/system_prompts.py with grounded_answer_prompt template from research.md
- [ ] T038 [US1] Add validate_answer_grounding() method to chat_service.py (Skill-Agent logic) checking answer against retrieved chunks
- [ ] T039 [US1] Create backend/src/api/chat.py with POST /chat endpoint implementing full RAG pipeline (embed query → search Qdrant → generate answer → validate → return with sources)

### Backend - Search API

- [ ] T040 [P] [US1] Create backend/src/models/search.py with SearchRequest, SearchResponse Pydantic models
- [ ] T041 [P] [US1] Add POST /search endpoint to backend/src/api/chat.py for standalone semantic search (returns top 5 results without generating answer)

### Backend - Admin API

- [ ] T042 [P] [US1] Create backend/src/api/admin.py with POST /embed endpoint (admin-only) to trigger embedding generation

### Backend - Rate Limiting

- [ ] T043 [US1] Create backend/src/middleware/rate_limit.py using slowapi with 100 req/min limit for /chat and /search
- [ ] T044 [US1] Apply rate limiting middleware to POST /chat and POST /search endpoints

### Frontend - ChatWidget Core

- [ ] T045 [US1] Create src/components/ChatWidget/ChatWidget.jsx with floating chat button + expandable modal
- [ ] T046 [US1] Create src/components/ChatWidget/ChatHeader.jsx with title "Ask a Question" and close button
- [ ] T047 [US1] Create src/components/ChatWidget/ChatInput.jsx with text input, send button, character limit (5000 chars)
- [ ] T048 [US1] Create src/components/ChatWidget/ChatMessage.jsx to render user/assistant message bubbles with timestamp
- [ ] T049 [US1] Create src/components/ChatWidget/ChatHistory.jsx with scrollable message list, auto-scroll to bottom
- [ ] T050 [US1] Create src/components/ChatWidget/LoadingIndicator.jsx with spinner animation for query processing
- [ ] T051 [US1] Create src/components/ChatWidget/SourceReference.jsx to display clickable chapter/section links

### Frontend - State Management

- [ ] T052 [US1] Create src/context/ChatContext.jsx with React Context for chat messages, loading state, error handling
- [ ] T053 [US1] Create src/components/ChatWidget/useChatHistory.js custom hook managing local chat state and API calls to POST /chat

### Frontend - API Client

- [ ] T054 [US1] Create src/services/api.js with axios client configured with baseURL from REACT_APP_API_URL, error interceptors
- [ ] T055 [US1] Add sendChatMessage() function to api.js calling POST /chat endpoint

### Frontend - Docusaurus Integration

- [ ] T056 [US1] Create src/theme/Root.js to inject ChatWidget globally on all Docusaurus pages
- [ ] T057 [US1] Create src/components/ChatWidget/chat.css with Tailwind CSS styles for responsive design (mobile, tablet, desktop)

### Testing - US1

- [ ] T058 [P] [US1] Create backend/tests/unit/test_embedding_service.py testing embed_text() and batch_embed()
- [ ] T059 [P] [US1] Create backend/tests/unit/test_rag_service.py testing search() and retrieve_top_k()
- [ ] T060 [P] [US1] Create backend/tests/unit/test_chat_service.py testing generate_answer() and validate_answer_grounding()
- [ ] T061 [US1] Create backend/tests/integration/test_chat_api.py testing POST /chat end-to-end flow
- [ ] T062 [US1] Create backend/tests/integration/test_search_api.py testing POST /search endpoint
- [ ] T063 [P] [US1] Create src/components/ChatWidget/ChatWidget.test.jsx testing message sending and display
- [ ] T064 [P] [US1] Create src/components/ChatWidget/useChatHistory.test.js testing hook state management

**Acceptance - US1**:
- Index book content: `python backend/scripts/index_book_content.py`
- Start backend: `uvicorn src.main:app --reload`
- Start frontend: `npm start`
- Open Docusaurus page, click chatbot icon
- Type "What is a ROS 2 node?" and send
- Verify: Response appears within 3s, includes chapter/section references, answer is grounded in book content
- All tests pass: `pytest backend/tests/ -k us1 -v` and `npm test ChatWidget`

---

## Phase 4: User Story 2 - Highlighted Text Queries (P1)

**Goal**: Enable "Ask Chatbot" button on text selection with pre-filled query

**Independent Test**: Highlight "inverse kinematics", click "Ask Chatbot", chatbot opens with pre-filled query

### Frontend - Text Selection

- [ ] T065 [US2] Create src/components/ChatWidget/useSelectedText.js custom hook detecting window.getSelection() on mouseup events
- [ ] T066 [US2] Add selection range tracking to useSelectedText.js with getBoundingClientRect() for button positioning
- [ ] T067 [US2] Create src/components/ChatWidget/HighlightedTextButton.jsx appearing near selected text with "Ask Chatbot" label
- [ ] T068 [US2] Add handleHighlightedTextClick() to ChatWidget.jsx opening chatbot with pre-filled query "Explain: [highlighted text]"

### Backend - Highlighted Text Priority

- [ ] T069 [US2] Update RAGService.search() to accept optional highlighted_text parameter for exact phrase matching boost
- [ ] T070 [US2] Add phrase_match_boost() method to rag_service.py boosting scores by 1.5x for chunks containing exact highlighted phrase
- [ ] T071 [US2] Update POST /chat endpoint to accept highlighted_text in ChatMessageRequest and pass to search

### Testing - US2

- [ ] T072 [P] [US2] Create src/components/ChatWidget/useSelectedText.test.js testing selection detection and button positioning
- [ ] T073 [P] [US2] Create src/components/ChatWidget/HighlightedTextButton.test.jsx testing button appearance and click handling
- [ ] T074 [US2] Update backend/tests/integration/test_chat_api.py to test highlighted text query flow

**Acceptance - US2**:
- Open Docusaurus page with technical content
- Highlight text (e.g., "inverse kinematics")
- Verify "Ask Chatbot" button appears near selection
- Click button, chatbot opens with query "Explain: inverse kinematics"
- Send query, verify response prioritizes content mentioning that phrase

---

## Phase 5: User Story 3 - Account & Chat History (P2)

**Goal**: User authentication with persistent chat history

**Independent Test**: Create account, ask questions, log out, log in, verify chat history preserved

### Backend - Auth Models

- [ ] T075 [US3] Create backend/src/models/user.py with UserCreate, UserLogin, UserResponse, AuthResponse Pydantic models per contracts/openapi.yaml
- [ ] T076 [US3] Create backend/src/models/preferences.py with UserPreferences Pydantic model

### Backend - Auth Service

- [ ] T077 [US3] Create backend/src/utils/password.py with hash_password() and verify_password() using bcrypt (rounds=12)
- [ ] T078 [US3] Create backend/src/utils/jwt.py with create_access_token() and verify_access_token() using python-jose
- [ ] T079 [US3] Create backend/src/utils/email.py with send_verification_email() and send_password_reset_email() using SendGrid
- [ ] T080 [US3] Create backend/src/services/auth_service.py with create_user() method validating email format and password strength per FR-016, FR-017
- [ ] T081 [US3] Add authenticate_user() method to auth_service.py verifying credentials and generating JWT
- [ ] T082 [US3] Add verify_email_token() method to auth_service.py for email verification flow
- [ ] T083 [US3] Add request_password_reset() and reset_password() methods to auth_service.py

### Backend - Auth API

- [ ] T084 [US3] Create backend/src/api/auth.py with POST /auth/signup endpoint creating user, sending verification email, returning UserResponse
- [ ] T085 [US3] Add POST /auth/signin endpoint to auth.py authenticating user, returning AuthResponse with JWT access_token
- [ ] T086 [P] [US3] Add GET /auth/verify endpoint to auth.py verifying email with token
- [ ] T087 [P] [US3] Add POST /auth/reset-password-request endpoint to auth.py sending password reset email
- [ ] T088 [P] [US3] Add POST /auth/reset-password endpoint to auth.py resetting password with token

### Backend - JWT Middleware

- [ ] T089 [US3] Create backend/src/middleware/jwt_middleware.py with verify_token dependency for protected endpoints
- [ ] T090 [US3] Add get_current_user() dependency to jwt_middleware.py extracting user_id from JWT, fetching User from database

### Backend - Chat History API

- [ ] T091 [US3] Add GET /history endpoint to backend/src/api/chat.py (protected) retrieving chat_messages for authenticated user with pagination (page, per_page=50)
- [ ] T092 [US3] Update POST /chat endpoint to save messages to chat_messages table if user authenticated, else chat_sessions table
- [ ] T093 [US3] Add session_migration() function to auth_service.py copying messages from chat_sessions to chat_messages on signup/signin

### Backend - User Preferences API

- [ ] T094 [P] [US3] Create backend/src/api/preferences.py with PUT /auth/preferences endpoint (protected) updating language_pref and preferred_chapter

### Frontend - Auth Context

- [ ] T095 [US3] Create src/context/AuthContext.jsx with React Context for user state, JWT token, login/logout functions
- [ ] T096 [US3] Create src/components/ChatWidget/useAuth.js custom hook managing authentication state, token storage in localStorage

### Frontend - Auth UI

- [ ] T097 [US3] Create src/components/ChatWidget/SignupModal.jsx with email and password inputs, form validation, error display
- [ ] T098 [US3] Create src/components/ChatWidget/SigninModal.jsx with email and password inputs, "Forgot Password?" link
- [ ] T099 [P] [US3] Create src/components/ChatWidget/PasswordResetModal.jsx with email input for reset request
- [ ] T100 [US3] Update ChatHeader.jsx to show "Sign Up" / "Sign In" buttons for unauthenticated users, "Account" dropdown for authenticated users
- [ ] T101 [US3] Add openSignupModal() and openSigninModal() handlers to ChatWidget.jsx

### Frontend - Chat History Display

- [ ] T102 [US3] Update useChatHistory.js to call GET /history on mount if user authenticated, populate initial messages
- [ ] T103 [US3] Add pagination controls to ChatHistory.jsx with "Load More" button fetching older messages

### Frontend - API Client Updates

- [ ] T104 [US3] Add signup(), signin(), requestPasswordReset(), resetPassword() functions to api.js
- [ ] T105 [US3] Add getChatHistory() function to api.js calling GET /history with JWT in Authorization header
- [ ] T106 [US3] Update sendChatMessage() in api.js to include JWT token if user authenticated

### Testing - US3

- [ ] T107 [P] [US3] Create backend/tests/unit/test_auth_service.py testing create_user(), authenticate_user(), verify_email_token()
- [ ] T108 [US3] Create backend/tests/integration/test_auth_api.py testing signup, signin, email verification, password reset flows
- [ ] T109 [P] [US3] Create backend/tests/integration/test_history_api.py testing GET /history with JWT authentication
- [ ] T110 [P] [US3] Create src/components/ChatWidget/SignupModal.test.jsx testing form validation and submission
- [ ] T111 [P] [US3] Create src/components/ChatWidget/useAuth.test.js testing login/logout state management

**Acceptance - US3**:
- Click "Sign Up" in chatbot, create account with email/password
- Verify email verification link sent (check SendGrid dashboard or logs)
- Ask 3 questions in chatbot, verify messages saved to database
- Click "Log Out", then "Sign In" with same credentials
- Verify chat history displays previous 3 messages
- Test password reset flow end-to-end

---

## Phase 6: User Story 4 - Chapter Personalization (P3)

**Goal**: Enable chapter-specific search ranking boost

**Independent Test**: Enable personalization for Chapter 5, ask chapter-related question, verify Chapter 5 content prioritized

### Backend - Personalization Logic

- [ ] T112 [US4] Update RAGService.search() to accept optional preferred_chapter parameter
- [ ] T113 [US4] Add apply_chapter_boost() method to rag_service.py multiplying scores by 2.0 for chunks matching preferred_chapter
- [ ] T114 [US4] Update POST /chat and POST /search endpoints to read preferred_chapter from user preferences (if authenticated) or request body (if guest)

### Frontend - Personalization UI

- [ ] T115 [US4] Create src/components/PersonalizeButton.jsx appearing on chapter pages with "Personalize for this Chapter" label
- [ ] T116 [US4] Add handlePersonalize() to PersonalizeButton.jsx saving preference to localStorage (guest) or calling PUT /auth/preferences (authenticated)
- [ ] T117 [US4] Add "Reset Personalization" button to ChatHeader.jsx clearing preferred_chapter
- [ ] T118 [US4] Update ChatMessage.jsx to display badge "Answer from Chapter X" when personalization active and answer sourced from preferred chapter

### Testing - US4

- [ ] T119 [P] [US4] Update backend/tests/unit/test_rag_service.py to test apply_chapter_boost()
- [ ] T120 [US4] Create backend/tests/integration/test_personalization.py testing personalized search ranking
- [ ] T121 [P] [US4] Create src/components/PersonalizeButton.test.jsx testing preference save and reset

**Acceptance - US4**:
- Navigate to Chapter 5 page
- Click "Personalize for this Chapter", verify preference saved
- Ask "What is inverse kinematics?" (Chapter 5 topic)
- Verify chatbot response includes badge "Answer from Chapter 5"
- Click "Reset Personalization", ask same question, verify no badge

---

## Phase 7: User Story 5 - Urdu Translation (P3)

**Goal**: Enable Urdu UI language toggle

**Independent Test**: Click Urdu button, verify all UI labels switch to Urdu

### Frontend - i18next Setup

- [ ] T122 [US5] Create src/locales/en.json with all chatbot UI labels (title, placeholder, send, history, etc.) in English
- [ ] T123 [US5] Create src/locales/ur.json with all chatbot UI labels translated to Urdu
- [ ] T124 [US5] Update src/theme/Root.js to initialize i18next with en/ur namespaces, detect language from localStorage or user preferences
- [ ] T125 [US5] Update all ChatWidget components (ChatHeader, ChatInput, ChatMessage, etc.) to use `t()` function from react-i18next for labels

### Frontend - Language Toggle

- [ ] T126 [US5] Add language toggle button to ChatHeader.jsx with "اردو" label switching to Urdu, "English" label switching to English
- [ ] T127 [US5] Update useAuth.js to save language_pref to localStorage (guest) or call PUT /auth/preferences (authenticated) on toggle

**Acceptance - US5**:
- Open chatbot, click "اردو" button
- Verify all UI labels switch to Urdu (title, placeholder, send button, etc.)
- Verify language preference persists after page refresh
- Click "English" button, verify labels switch back to English

---

## Phase 8: Deployment & Polish

**Goal**: Deploy to production, set up monitoring, final UX improvements

**Independent Test**: Production deployment accessible, monitoring capturing errors

### Deployment - Backend

- [ ] T128 Create backend/render.yaml with build command, start command, environment variables
- [ ] T129 Configure Render service connected to GitHub repo, deploy backend to https://rag-chatbot-api.onrender.com
- [ ] T130 Verify Neon Postgres connection from Render deployment
- [ ] T131 Verify Qdrant Cloud connection from Render deployment
- [ ] T132 Run Alembic migrations on production database
- [ ] T133 Index book content to production Qdrant collection

### Deployment - Frontend

- [ ] T134 Configure Vercel project connected to GitHub repo, set REACT_APP_API_URL to Render backend URL
- [ ] T135 Deploy frontend to Vercel https://textbook.example.com
- [ ] T136 Verify ChatWidget loads on all Docusaurus pages in production

### CI/CD

- [ ] T137 [P] Create .github/workflows/backend-tests.yml running pytest on PR to main
- [ ] T138 [P] Create .github/workflows/frontend-tests.yml running npm test on PR to main
- [ ] T139 [P] Create .github/workflows/deploy.yml auto-deploying to Render + Vercel on merge to main

### Monitoring

- [ ] T140 [P] Add Sentry SDK to backend/src/main.py with DSN from environment variable
- [ ] T141 [P] Add Sentry SDK to src/index.js for frontend error tracking
- [ ] T142 [P] Add structlog to backend for structured JSON logging
- [ ] T143 [P] Create backend/scripts/cleanup_expired_sessions.py deleting chat_sessions older than 24 hours
- [ ] T144 [P] Configure Render cron job to run cleanup script daily

### Final Polish

- [ ] T145 [P] Add accessibility features to ChatWidget: ARIA labels, keyboard navigation (Tab, Esc, Enter)
- [ ] T146 [P] Optimize ChatWidget lazy loading: Load component only when user clicks chatbot icon
- [ ] T147 [P] Add debouncing to ChatInput.jsx: Wait 300ms after typing stops before enabling send button
- [ ] T148 [P] Create backend/docs/api.md with API documentation generated from OpenAPI schema

### Git & GitHub

- [ ] T149 Stage all changes: `git add .`
- [ ] T150 Commit changes: `git commit -m "feat: RAG chatbot integration with FastAPI, Qdrant, Neon, React - Implements US1-US5 with embedding, search, chat completion, auth, personalization, and Urdu translation"`
- [ ] T151 Push to GitHub: `git push origin 001-rag-chatbot`
- [ ] T152 Create pull request on GitHub with description from spec.md Summary

**Acceptance - Deployment**:
- Production backend: `curl https://rag-chatbot-api.onrender.com/v1/health` returns OK
- Production frontend: Open https://textbook.example.com, verify chatbot works
- GitHub Actions CI/CD: All tests pass on PR
- Sentry: Verify errors captured in Sentry dashboard
- Load test: 100 concurrent users, verify <3s response time (SC-002)

---

## Dependency Graph

**User Story Completion Order** (based on priorities and dependencies):

```
Setup (Phase 1)
    ↓
Foundational (Phase 2)
    ↓
    ├─→ US1 (P1) - Ask Questions [BLOCKING for US2-US5]
    │      ↓
    ├─→ US2 (P1) - Highlighted Text [Depends on US1]
    │
    ├─→ US3 (P2) - Auth & History [Independent of US1-US2]
    │
    ├─→ US4 (P3) - Personalization [Depends on US1, optionally US3]
    │
    └─→ US5 (P3) - Urdu Translation [Independent]
```

**Critical Path** (must complete in order):
1. Phase 1: Setup
2. Phase 2: Foundational
3. Phase 3: US1 (Core chatbot)
4. Phase 4: US2 (Highlighted text)

**Parallel Opportunities**:
- US3 (Auth) can be developed in parallel with US1-US2
- US5 (Translation) can be developed in parallel with US1-US4
- US4 (Personalization) can start after US1 core search is done

---

## Parallel Execution Examples

### Setup & Foundational (Phases 1-2)

**Team 1 (Backend Engineer)**:
- T001-T010 (Setup backend structure)
- T011-T023 (Database + FastAPI + Qdrant setup)

**Team 2 (Frontend Engineer)**:
- T002, T006-T007 (Setup frontend structure)
- T024-T027 (Test infrastructure - can run in parallel)

### User Story 1 (Phase 3)

**Team 1 (RAG-Agent)**:
- T028-T034 (Embedding + RAG services)
- T036-T039 (Chat service + API)

**Team 2 (UI-Agent)**:
- T045-T051 (ChatWidget components)
- T052-T057 (State management + API client)

**Team 3 (Testing)**:
- T058-T064 (All tests - can run in parallel with implementation)

### User Stories 2-5 (Phases 4-7)

**Team 1 (Backend)**:
- US3: T075-T094 (Auth + History APIs)
- US4: T112-T114 (Personalization logic)

**Team 2 (Frontend)**:
- US2: T065-T068 (Highlighted text)
- US3: T095-T106 (Auth UI + History display)
- US4: T115-T118 (Personalization UI)
- US5: T122-T127 (i18next translation)

**Team 3 (Deployment & Polish)**:
- T128-T148 (All deployment tasks can run in parallel once code is ready)

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Phase 1-3** (US1 only): Core RAG chatbot
- Setup + Foundational + US1 = ~60 tasks
- Deliverable: Functional chatbot answering questions with grounded responses
- Timeline: ~2-3 weeks for 1 developer, ~1 week for 2 developers in parallel

### Incremental Delivery

1. **Sprint 1**: Phases 1-3 (US1) → MVP deployment
2. **Sprint 2**: Phase 4 (US2) → Highlighted text feature
3. **Sprint 3**: Phase 5 (US3) → Authentication & persistence
4. **Sprint 4**: Phases 6-7 (US4-US5) → Personalization & translation
5. **Sprint 5**: Phase 8 → Production deployment & polish

### Testing Strategy

- Unit tests: Run after each service/component implementation
- Integration tests: Run after each API endpoint implementation
- E2E tests: Run after each user story phase completion
- Load tests: Run before production deployment

---

## Task Execution Guidelines

1. **Follow Checklist Format**: Every task uses `- [ ] [ID] [P?] [Story?] Description + file path`
2. **Check Dependencies**: Complete blocking tasks before starting dependent tasks
3. **Parallel Execution**: Tasks marked [P] can run in parallel with others
4. **Test-Driven Development** (optional): Write tests (T058-T064, etc.) before implementation
5. **Incremental Commits**: Commit after completing each task or small task group
6. **Review Plan Documents**: Reference spec.md, plan.md, data-model.md, contracts/openapi.yaml, research.md as needed
7. **Environment Setup**: Ensure .env variables configured before running backend tasks
8. **Acceptance Validation**: After each phase, verify acceptance criteria before proceeding

---

## Quick Reference

- **Spec**: `specs/001-rag-chatbot/spec.md` (50 FRs, 10 SCs, 5 user stories)
- **Plan**: `specs/001-rag-chatbot/plan.md` (7 stages, tech stack, risks)
- **Data Model**: `specs/001-rag-chatbot/data-model.md` (3 tables, 1 collection, 11 Pydantic models)
- **API Contract**: `specs/001-rag-chatbot/contracts/openapi.yaml` (12 endpoints)
- **Research**: `specs/001-rag-chatbot/research.md` (15 technology decisions)
- **Quickstart**: `specs/001-rag-chatbot/quickstart.md` (local setup guide)
- **Constitution**: `.specify/memory/constitution.md` (7 principles)

---

**Total Tasks**: 152 (including all phases)
**Parallel Opportunities**: 45 tasks marked [P]
**MVP Tasks** (Phases 1-3): 64 tasks
**Estimated Effort**: 4-6 weeks (1 full-time developer), 2-3 weeks (2 developers in parallel)

**Next Action**: Start with T001 (Create backend directory structure) and proceed sequentially through Setup phase.
