# Research & Technology Decisions: RAG Chatbot Integration

**Feature**: RAG Chatbot Integration
**Branch**: 001-rag-chatbot
**Date**: 2025-12-10

## Overview

This document consolidates research findings and technology decisions for implementing a RAG (Retrieval-Augmented Generation) chatbot integrated with a Docusaurus textbook. All decisions are based on the requirements from spec.md and constitution principles.

---

## 1. Backend Framework

### Decision: FastAPI (Python 3.11+)

**Rationale**:
- **Async Support**: Native async/await for concurrent request handling (SC-002: 100 concurrent users)
- **Fast Development**: Automatic API documentation (OpenAPI/Swagger), request validation via Pydantic
- **Python Ecosystem**: Seamless integration with OpenAI SDK, Qdrant client, and data science libraries
- **Performance**: Uvicorn ASGI server provides performance comparable to Node.js Express
- **Type Safety**: Pydantic models enforce type validation at runtime

**Alternatives Considered**:
- **Node.js + Express**: Rejected due to weaker typing and less mature ML/AI library ecosystem
- **Django**: Rejected due to heavier footprint and synchronous-first design
- **Flask**: Rejected due to lack of built-in async support and manual schema validation

**Best Practices**:
- Use dependency injection for database connections and external clients
- Implement middleware for CORS, rate limiting (slowapi library), and error handling
- Structure as: `backend/src/api/` (routers), `backend/src/services/` (business logic), `backend/src/models/` (Pydantic schemas)
- Use `python-dotenv` for environment variable management

---

## 2. Vector Database

### Decision: Qdrant Cloud (Free Tier)

**Rationale**:
- **Purpose-Built for Vectors**: Optimized for similarity search over high-dimensional embeddings
- **Free Tier Capacity**: 1GB storage, ~100K vectors (sufficient for 500+ book sections at 1536 dimensions)
- **Performance**: Sub-100ms search latency for typical queries (SC-001: <3s response time)
- **Metadata Filtering**: Supports filtering by chapter/section during retrieval (FR-038: personalization boost)
- **Cloud-Managed**: No DevOps overhead; auto-scaling and backups included

**Alternatives Considered**:
- **Pinecone**: Rejected due to more restrictive free tier (1M vectors but requires credit card)
- **Weaviate**: Rejected due to higher setup complexity (self-hosting or cloud pricing)
- **FAISS (local)**: Rejected due to lack of persistence and production-readiness concerns

**Best Practices**:
- Create collection with cosine similarity metric (standard for OpenAI embeddings)
- Index metadata fields: `chapter`, `section`, `url`, `content_hash` (for deduplication)
- Use batch upsert for initial indexing (chunks of 100 vectors)
- Implement retry logic with exponential backoff for transient failures

**Collection Schema**:
```python
{
  "vectors": {
    "size": 1536,  # OpenAI text-embedding-3-small dimension
    "distance": "Cosine"
  },
  "payload_schema": {
    "chapter": "keyword",
    "section": "text",
    "content": "text",
    "url": "keyword",
    "content_hash": "keyword"
  }
}
```

---

## 3. Relational Database

### Decision: Neon Serverless Postgres (Free Tier)

**Rationale**:
- **Serverless Architecture**: Auto-pause during inactivity (cost-effective for low-traffic MVP)
- **PostgreSQL Compatibility**: Standard SQL, ACID guarantees, mature Python drivers (psycopg3, asyncpg)
- **Free Tier**: 0.5GB storage, 1 compute instance (sufficient for user accounts and chat history)
- **Connection Pooling**: Built-in connection pooling avoids exhaustion under load
- **Branching**: Git-like database branching for testing schema migrations

**Alternatives Considered**:
- **Supabase**: Rejected due to similar pricing but less focus on serverless auto-pause
- **PlanetScale**: Rejected due to MySQL (not PostgreSQL), less flexible schema migrations
- **SQLite**: Rejected due to lack of multi-user concurrency and cloud deployment challenges

**Best Practices**:
- Use SQLAlchemy 2.0 (async) with asyncpg driver for connection management
- Implement Alembic for database migrations (version control for schema)
- Index frequently queried columns: `user.email`, `chat_message.user_id`, `chat_message.timestamp`
- Use connection pooling via `create_async_engine` (pool_size=5, max_overflow=10)

**Schema Overview** (detailed in data-model.md):
- `users` table: id, email, password_hash, created_at, language_pref, preferred_chapter
- `chat_messages` table: id, user_id, message, timestamp, role (user/assistant), sources (JSONB)
- `chat_sessions` table: id, session_id, messages (JSONB), expires_at (for unauthenticated users)

---

## 4. Authentication

### Decision: BetterAuth (Python Library)

**Rationale**:
- **FastAPI Integration**: Designed for FastAPI with middleware and dependency injection support
- **OAuth2 + JWT**: Standard token-based authentication (secure, stateless)
- **Social Login**: Built-in Google OAuth support (FR-015)
- **Password Security**: Bcrypt hashing with configurable rounds
- **Email Verification**: Built-in email verification flows (FR-018)

**Alternatives Considered**:
- **Authlib**: Rejected due to lower-level API (more manual setup required)
- **Auth0**: Rejected due to external service dependency and potential costs
- **Custom JWT**: Rejected due to security risk of rolling custom auth logic

**Best Practices**:
- Store JWT secret in environment variable (never commit to repo)
- Set token expiration: access token (15 min), refresh token (7 days)
- Implement password strength validation (FR-017): min 8 chars, uppercase, number, special char
- Use HTTPS-only cookies for token storage (mitigate XSS)
- Rate-limit login attempts (5 attempts per 15 min per IP)

**Email Service**:
- Use SendGrid free tier (100 emails/day) for verification and password reset emails
- Fallback: SMTP via Gmail (less reliable, requires app password)

---

## 5. Embedding Model

### Decision: OpenAI text-embedding-3-small

**Rationale**:
- **Cost-Effective**: $0.02 per 1M tokens (~$0.10 for 500 book sections @ 1000 tokens/section)
- **Performance**: 1536 dimensions, optimized for semantic similarity search
- **Speed**: <1s embedding generation for typical paragraph (SC-007: <10 min for 500+ sections)
- **Quality**: High retrieval precision for educational content (tested in benchmarks)

**Alternatives Considered**:
- **text-embedding-3-large**: Rejected due to 2x cost, minimal quality improvement for use case
- **Open-source (Sentence-Transformers)**: Rejected due to lower quality on domain-specific content
- **Cohere Embed**: Rejected due to similar cost but less mature Python SDK

**Best Practices**:
- Batch embed in chunks of 100 sections (API rate limit: 3000 RPM)
- Cache embeddings with content_hash to avoid re-embedding unchanged content
- Normalize vectors before storing in Qdrant (cosine similarity requires unit vectors)
- Handle rate limits with exponential backoff (retry after 60s on 429 errors)

---

## 6. Conversational AI

### Decision: OpenAI Chat Completions API (gpt-4o-mini)

**Rationale**:
- **Grounding Support**: System prompt engineering for retrieval-grounded responses (SC-003: 100% accuracy)
- **Speed**: <2s response time for typical queries (SC-001: <3s for 95% of queries)
- **Cost**: $0.15 per 1M input tokens, $0.60 per 1M output tokens (~$0.01 per chat interaction)
- **Context Window**: 128K tokens (sufficient for 5 retrieved chunks + conversation history)

**Alternatives Considered**:
- **gpt-4o**: Rejected due to 10x cost, overkill for textbook Q&A
- **gpt-3.5-turbo**: Rejected due to lower instruction-following quality
- **Claude 3 Haiku**: Rejected due to less mature Python SDK and API rate limits

**System Prompt Template**:
```python
system_prompt = """
You are a helpful teaching assistant for the Physical AI & Humanoid Robotics textbook.
Answer questions using ONLY the provided book content below. Do not use external knowledge.

Book Content:
{retrieved_chunks}

Rules:
1. If the answer is not in the provided content, say "I couldn't find information about that topic in this textbook."
2. Always cite the chapter and section where you found the answer.
3. Keep answers concise (2-3 sentences) unless a detailed explanation is requested.
"""
```

**Best Practices**:
- Limit retrieved chunks to top 5 (balance context vs. cost)
- Include conversation history (last 5 messages) for multi-turn conversations
- Implement streaming responses for better UX (SC-001: <3s perceived latency)
- Log all prompts and responses for quality monitoring and debugging

---

## 7. Frontend Framework

### Decision: Docusaurus (React 18) + Tailwind CSS

**Rationale**:
- **Already in Use**: Existing Docusaurus site (minimize new dependencies)
- **React Ecosystem**: Mature component libraries, hooks for state management
- **Tailwind CSS**: Utility-first CSS for rapid UI development (Constitution Principle II)
- **TypeScript Support**: Type-safe component development
- **Hot Reload**: Fast development iteration

**Alternatives Considered**:
- **Vue.js**: Rejected to avoid introducing new framework alongside React
- **Vanilla JS**: Rejected due to lack of state management and component structure
- **Svelte**: Rejected due to smaller ecosystem and team familiarity

**Best Practices**:
- Create `src/components/ChatWidget/` directory for chatbot UI components
- Use React Context API for global state (user auth, chat history, preferences)
- Implement custom hooks: `useSelectedText` (FR-008), `useChatHistory`, `useAuth`
- Lazy-load ChatWidget component (reduce initial page load time)
- Use LocalStorage for unauthenticated user preferences (FR-043)

**Component Structure**:
```
src/components/ChatWidget/
├── ChatWidget.jsx           # Main container component
├── ChatHeader.jsx           # Title, settings button, language toggle
├── ChatMessage.jsx          # Single message bubble (user/assistant)
├── ChatInput.jsx            # Text input + send button
├── HighlightedTextButton.jsx  # "Ask Chatbot" button for selections
├── LoadingIndicator.jsx     # Spinner during API calls
└── chat.css                 # Component-specific styles
```

---

## 8. Text Selection Detection

### Decision: Native Browser API (window.getSelection)

**Rationale**:
- **Zero Dependencies**: Built-in browser API (no external library needed)
- **Cross-Browser Support**: Supported in all modern browsers (Chrome, Firefox, Safari, Edge)
- **Performance**: Negligible overhead, instant selection detection
- **Flexibility**: Access to selection range, anchor/focus nodes, text content

**Alternatives Considered**:
- **Rangy**: Rejected due to maintenance concerns (last updated 2016)
- **Selection.js**: Rejected due to added dependency for minimal benefit

**Implementation Approach**:
```javascript
// Custom hook: useSelectedText.js
useEffect(() => {
  const handleSelection = () => {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    if (text.length > 0 && text.length <= 500) {
      // Show "Ask Chatbot" button near selection
      setSelectedText(text);
      setShowButton(true);
    } else {
      setShowButton(false);
    }
  };

  document.addEventListener('mouseup', handleSelection);
  return () => document.removeEventListener('mouseup', handleSelection);
}, []);
```

**Best Practices**:
- Debounce selection events (wait 100ms after mouseup to avoid flicker)
- Position button using `Selection.getRangeAt(0).getBoundingClientRect()`
- Hide button when user clicks away or starts typing in chatbot
- Limit highlighted text to 500 characters (FR-012)

---

## 9. Translation

### Decision: i18next with React-i18next

**Rationale**:
- **Industry Standard**: Widely used for React internationalization
- **Lightweight**: ~20KB gzipped, minimal performance impact (SC-009: <100ms toggle)
- **Namespace Support**: Separate translation files for chatbot UI vs. book content
- **Interpolation**: Dynamic text replacement (e.g., "Welcome, {username}!")
- **localStorage Integration**: Built-in language detection and persistence

**Alternatives Considered**:
- **react-intl**: Rejected due to heavier footprint and more complex API
- **Custom JSON**: Rejected due to lack of pluralization, interpolation, fallback handling

**Implementation Approach**:
- Create `locales/en.json` and `locales/ur.json` for chatbot UI labels
- Only translate UI strings (buttons, labels, messages), not book content (Assumption 7)
- Use language codes: `en` (English), `ur` (Urdu)

**Translation Files**:
```json
// locales/en.json
{
  "chatWidget": {
    "title": "Ask a Question",
    "placeholder": "Type your question here...",
    "send": "Send",
    "history": "History",
    "noResults": "I couldn't find information about that topic in this textbook.",
    "loading": "Searching..."
  }
}

// locales/ur.json
{
  "chatWidget": {
    "title": "سوال پوچھیں",
    "placeholder": "اپنا سوال یہاں ٹائپ کریں...",
    "send": "بھیجیں",
    "history": "تاریخ",
    "noResults": "مجھے اس کتاب میں اس موضوع کے بارے میں معلومات نہیں ملیں۔",
    "loading": "تلاش کر رہے ہیں..."
  }
}
```

**Best Practices**:
- Initialize i18next in `src/theme/Root.js` (Docusaurus custom theme hook)
- Store language preference in localStorage (FR-043) and user account (FR-044)
- Preload both language files (avoid loading delay on toggle)

---

## 10. Personalization Logic

### Decision: Qdrant Metadata Filtering + Score Boosting

**Rationale**:
- **Native Support**: Qdrant supports filtering and score boosting via query parameters
- **Performance**: Filtering happens at query time (no pre-indexing needed)
- **Flexibility**: Easy to adjust boost factor (FR-038: 2x boost for preferred chapter)

**Implementation Approach**:
```python
# Without personalization
results = qdrant_client.search(
    collection_name="book_content",
    query_vector=query_embedding,
    limit=5
)

# With personalization (preferred_chapter = "Chapter 5")
results = qdrant_client.search(
    collection_name="book_content",
    query_vector=query_embedding,
    limit=5,
    query_filter=None,  # Don't hard-filter (still search globally)
    score_threshold=0.5,
    with_payload=True,
    # Boost scores for preferred chapter via custom scoring
)

# Post-processing: boost scores for preferred chapter by 2x
for result in results:
    if result.payload.get("chapter") == preferred_chapter:
        result.score *= 2.0
results = sorted(results, key=lambda x: x.score, reverse=True)[:5]
```

**Best Practices**:
- Store preferred_chapter in user_preferences table (authenticated users) or localStorage (guests)
- Provide "Reset Personalization" button (FR-039) to clear preference
- Display badge on chatbot answers indicating "Answer from Chapter 5" when personalization is active
- Fallback to global search if no results from preferred chapter (FR-040)

---

## 11. Deployment Strategy

### Decision: Vercel (Frontend) + Render (Backend)

**Rationale**:
- **Vercel**: Free tier for static sites, CDN distribution, automatic Git deployments
- **Render**: Free tier for backend services (750 hours/month), auto-deploy from Git
- **Separation of Concerns**: Independent scaling, simpler debugging
- **HTTPS**: Both platforms provide free SSL certificates

**Alternatives Considered**:
- **Netlify + Railway**: Rejected due to Railway's recent pricing changes
- **AWS Amplify + Lambda**: Rejected due to higher complexity (IAM, API Gateway setup)
- **Single Heroku Dyno**: Rejected due to inability to separate frontend/backend concerns

**Deployment Workflow**:
1. **Frontend** (Vercel):
   - Connect GitHub repo to Vercel
   - Build command: `npm run build` (Docusaurus)
   - Output directory: `build/`
   - Environment variables: `REACT_APP_API_URL` (Render backend URL)

2. **Backend** (Render):
   - Connect GitHub repo to Render
   - Build command: `pip install -r requirements.txt`
   - Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - Environment variables: `OPENAI_API_KEY`, `QDRANT_URL`, `NEON_DATABASE_URL`, `JWT_SECRET`

**Best Practices**:
- Use `.env.example` file to document required environment variables
- Implement health check endpoint (`/health`) for monitoring
- Set up GitHub Actions for automated testing before deployment
- Use semantic versioning tags for releases

---

## 12. Claude Subagents + Skills Architecture

### Decision: Task-Based Agent Coordination via REST APIs

**Rationale**:
- **Modularity**: Each agent is an independent, testable unit (Constitution Principle III)
- **Scalability**: Agents can run on separate machines if needed
- **Simplicity**: REST APIs are standard, language-agnostic, easy to debug
- **Claude Code Integration**: Use Claude Code's Task tool to spawn subagents

**Agent Responsibilities**:

### 12.1 RAG-Agent
**Purpose**: Manage embedding generation, Qdrant indexing, semantic search, retrieval

**Endpoints**:
- `POST /rag/embed`: Generate embeddings for new/updated content
- `POST /rag/search`: Perform semantic search over Qdrant
- `GET /rag/status`: Check indexing progress

**Skills**:
- `skill_embed_text(text: str) -> list[float]`: Embed single text chunk
- `skill_batch_embed(texts: list[str]) -> list[list[float]]`: Batch embed multiple chunks
- `skill_search_similar(query_embedding: list[float], top_k: int) -> list[SearchResult]`
- `skill_index_markdown_file(file_path: str)`: Parse Markdown, split into sections, embed, upsert to Qdrant

**Technology**: Python + OpenAI SDK + Qdrant Client

---

### 12.2 UI-Agent
**Purpose**: Build React components, integrate with Docusaurus, manage frontend state

**Tasks**:
- Implement ChatWidget component hierarchy
- Create useSelectedText hook for text highlighting
- Integrate i18next for Urdu translation
- Implement localStorage persistence for unauthenticated users

**Skills**:
- `skill_create_react_component(component_name: str, props: dict)`: Generate boilerplate component
- `skill_create_custom_hook(hook_name: str, dependencies: list)`: Generate custom hook template
- `skill_test_component(component_name: str)`: Run unit tests for component

**Technology**: JavaScript/TypeScript + React + Tailwind CSS

---

### 12.3 Auth-Agent
**Purpose**: Implement BetterAuth flows, Neon DB schema, user operations

**Tasks**:
- Create database schema (users, chat_messages, user_preferences)
- Implement BetterAuth signup/signin endpoints
- Implement password reset flow with SendGrid
- Create user CRUD operations

**Skills**:
- `skill_create_user(email: str, password: str) -> User`: Create user account with hashed password
- `skill_authenticate_user(email: str, password: str) -> Optional[User]`: Verify credentials
- `skill_generate_jwt_token(user_id: int) -> str`: Create access token
- `skill_verify_jwt_token(token: str) -> Optional[int]`: Validate and extract user_id

**Technology**: Python + SQLAlchemy + Alembic + BetterAuth + psycopg3

---

### 12.4 Skill-Agent
**Purpose**: Provide reusable intelligence modules (query reformulation, answer grounding, context merging)

**Skills**:
- `skill_reformulate_query(original_query: str, chat_history: list) -> str`: Enhance query with context from chat history
- `skill_validate_answer(answer: str, retrieved_chunks: list[str]) -> bool`: Verify answer is grounded in retrieved content
- `skill_merge_contexts(chunks: list[str]) -> str`: Combine multiple retrieved chunks into coherent context
- `skill_extract_sources(chunks: list[dict]) -> list[dict]`: Extract chapter/section references from chunks

**Technology**: Python + OpenAI SDK (for reformulation via GPT-4o-mini)

---

## 13. Rate Limiting Strategy

### Decision: slowapi (FastAPI middleware)

**Rationale**:
- **FastAPI Native**: Designed specifically for FastAPI
- **Redis-Optional**: Can use in-memory storage for MVP (no Redis dependency)
- **Flexible Rules**: Per-endpoint, per-user, per-IP rate limits
- **Standard Library**: Uses Python's `functools.lru_cache` for in-memory storage

**Rate Limits** (FR-035):
- `/chat`: 100 requests per minute per user
- `/search`: 100 requests per minute per user
- `/embed`: 10 requests per minute per admin (manual trigger only)
- `/auth/signin`: 5 requests per 15 minutes per IP (prevent brute force)

**Implementation**:
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@app.post("/chat")
@limiter.limit("100/minute")
async def chat_endpoint(request: Request, query: ChatQuery):
    # Handle chat request
    pass
```

---

## 14. Testing Strategy

### Decision: pytest (Backend) + Jest + React Testing Library (Frontend)

**Rationale**:
- **pytest**: Industry standard for Python, excellent async support, fixtures, mocking
- **Jest**: Standard for React testing, built into Create React App, snapshot testing
- **React Testing Library**: Encourages testing user behavior (not implementation details)

**Test Categories**:

### 14.1 Backend Tests
- **Unit Tests** (`tests/unit/`): Test individual functions (embedding, search, auth logic)
- **Integration Tests** (`tests/integration/`): Test API endpoints with mock database and Qdrant
- **Contract Tests** (`tests/contract/`): Validate API responses match OpenAPI schema

**Example Test**:
```python
# tests/unit/test_rag_service.py
import pytest
from src.services.rag_service import RAGService

@pytest.mark.asyncio
async def test_search_returns_top_5_results(mock_qdrant_client):
    rag_service = RAGService(qdrant_client=mock_qdrant_client)
    results = await rag_service.search("What is a ROS 2 node?", top_k=5)
    assert len(results) == 5
    assert all(result.score > 0.5 for result in results)
```

### 14.2 Frontend Tests
- **Component Tests**: Test React components in isolation (props, rendering, state)
- **Hook Tests**: Test custom hooks (useSelectedText, useChatHistory)
- **Integration Tests**: Test full user flows (ask question, highlight text, login)

**Example Test**:
```javascript
// src/components/ChatWidget/ChatWidget.test.jsx
import { render, screen, fireEvent } from '@testing-library/react';
import ChatWidget from './ChatWidget';

test('sends message when send button clicked', async () => {
  render(<ChatWidget />);

  const input = screen.getByPlaceholderText('Type your question here...');
  const sendButton = screen.getByText('Send');

  fireEvent.change(input, { target: { value: 'What is ROS 2?' } });
  fireEvent.click(sendButton);

  expect(await screen.findByText(/ROS 2 is/)).toBeInTheDocument();
});
```

---

## 15. Monitoring & Observability

### Decision: Structured Logging + Sentry (Error Tracking)

**Rationale**:
- **Structured Logging**: JSON logs for easy parsing by log aggregators (Render has built-in log viewer)
- **Sentry**: Free tier (5K events/month), captures exceptions with stack traces, user context, breadcrumbs
- **Minimal Overhead**: Both add <5ms latency to requests

**Implementation**:
```python
# Backend logging
import logging
import structlog

logging.basicConfig(level=logging.INFO)
logger = structlog.get_logger()

@app.post("/chat")
async def chat_endpoint(query: ChatQuery):
    logger.info("chat_request_received", user_id=query.user_id, query_length=len(query.text))
    # Process request
    logger.info("chat_response_sent", response_length=len(response.text), latency_ms=latency)
```

**Sentry Integration**:
```python
import sentry_sdk

sentry_sdk.init(
    dsn=os.getenv("SENTRY_DSN"),
    traces_sample_rate=0.1,  # 10% of requests for performance monitoring
    environment="production"
)
```

---

## Summary of Technology Stack

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| **Backend Framework** | FastAPI | 0.104+ | Async, auto-docs, Pydantic validation |
| **Python Version** | Python | 3.11+ | Async performance, type hints |
| **Vector Database** | Qdrant Cloud | Latest | Free tier, fast similarity search |
| **Relational Database** | Neon Postgres | Latest | Serverless, auto-pause, free tier |
| **Authentication** | BetterAuth | Latest | FastAPI integration, OAuth2/JWT |
| **Embedding Model** | OpenAI | text-embedding-3-small | Cost-effective, 1536 dims |
| **LLM** | OpenAI | gpt-4o-mini | Fast, cost-effective, grounding support |
| **Frontend Framework** | Docusaurus | 3.x | Existing site, React 18 |
| **UI Styling** | Tailwind CSS | 3.x | Utility-first, rapid development |
| **Translation** | i18next | 23.x | Industry standard, lightweight |
| **Selection API** | Browser Native | window.getSelection | Zero dependencies |
| **Backend Deployment** | Render | N/A | Free tier, auto-deploy |
| **Frontend Deployment** | Vercel | N/A | Free tier, CDN, auto-deploy |
| **Testing (Backend)** | pytest | 7.x | Async support, fixtures |
| **Testing (Frontend)** | Jest + RTL | Latest | React standard |
| **Monitoring** | Sentry | Latest | Error tracking, free tier |
| **Rate Limiting** | slowapi | Latest | FastAPI middleware |

---

## Next Steps

1. **Phase 1**: Design detailed data models (data-model.md) for Neon Postgres schema
2. **Phase 1**: Create API contracts (contracts/) with OpenAPI schemas for all endpoints
3. **Phase 1**: Write quickstart.md for local development setup
4. **Phase 1**: Update Claude agent context files with technology decisions
5. **Phase 2**: Proceed to `/sp.tasks` to generate actionable implementation tasks
