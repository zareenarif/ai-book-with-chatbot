# Feature Specification: RAG Chatbot Integration

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Generate full specification for the project: 1. Architecture diagram (text) 2. FastAPI backend design (/embed, /search, /chat) 3. Embedding pipeline from markdown files 4. Qdrant collection schema 5. Neon Postgres schema (users, personalization) 6. BetterAuth signup/signin logic 7. Chatbot frontend widget (ChatWidget.jsx, useSelectedText.js, chat.css) 8. Highlighted-text vs RAG search flow 9. Chat → retrieval → answer flow 10. Claude Subagents + Skills structure 11. Urdu translation module 12. Personalization per chapter button 13. Deployment approach (Render + Vercel + Qdrant + Neon) Make it modular, actionable, ready for Plan phase."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Book Content (Priority: P1)

A student reading the Physical AI & Humanoid Robotics textbook encounters an unfamiliar concept and wants immediate clarification without leaving the page. They open the chatbot widget, type their question, and receive an accurate answer sourced directly from the book content with references to relevant sections.

**Why this priority**: This is the core value proposition of the RAG chatbot - providing instant, contextual answers grounded in book content. Without this, the chatbot provides no value.

**Independent Test**: Can be fully tested by asking sample questions about book content and verifying responses are accurate, grounded in book text, and include source references. Delivers immediate educational value by reducing friction in learning.

**Acceptance Scenarios**:

1. **Given** a student is reading Chapter 3 on ROS 2 architecture, **When** they ask "What is a ROS 2 node?", **Then** the chatbot returns an answer derived from the book's definition with a reference to the specific section
2. **Given** the chatbot has indexed all book content, **When** a user asks a question about any topic covered in the book, **Then** the response cites the relevant chapter and section
3. **Given** a user asks a question not covered in the book, **When** the system searches for relevant content, **Then** the chatbot responds with "I can only answer questions about topics covered in this book. I couldn't find information about that topic in the textbook."
4. **Given** the chatbot is processing a query, **When** retrieval takes longer than 2 seconds, **Then** the user sees a loading indicator
5. **Given** a user types a vague question, **When** the query is submitted, **Then** the chatbot attempts to find the most relevant content or asks for clarification

---

### User Story 2 - Get Instant Answers for Highlighted Text (Priority: P1)

A reader highlights a technical term or sentence they don't understand (e.g., "inverse kinematics"). A small "Ask Chatbot" button appears next to the selection. Clicking it opens the chatbot with a pre-filled query asking for an explanation of the highlighted text, and the chatbot provides context from related sections of the book.

**Why this priority**: This creates a seamless, context-aware learning experience. Highlighting is a natural reading behavior, and this feature transforms passive reading into active learning with zero friction.

**Independent Test**: Can be tested by highlighting any text on any page, clicking the "Ask Chatbot" button, and verifying the chatbot provides relevant explanations. Delivers value by making the book interactive.

**Acceptance Scenarios**:

1. **Given** a user highlights any text on a book page, **When** the selection is complete, **Then** an "Ask Chatbot" button appears near the selection
2. **Given** a user clicks "Ask Chatbot" on highlighted text, **When** the chatbot opens, **Then** the query field is pre-filled with "Explain: [highlighted text]"
3. **Given** the chatbot receives a highlighted-text query, **When** processing the request, **Then** it searches for the highlighted phrase and related concepts in the book content
4. **Given** highlighted text matches exact content in the book, **When** the answer is generated, **Then** it includes the surrounding context from that section
5. **Given** highlighted text is a technical term defined in the glossary, **When** the answer is generated, **Then** it prioritizes the glossary definition

---

### User Story 3 - Create Account and Save Chat History (Priority: P2)

A returning student wants to review previous questions they've asked. They create an account using their email and password (or social login), and all subsequent chat interactions are saved to their account. They can access their chat history from any device after logging in.

**Why this priority**: Persistent chat history transforms the chatbot from a one-off tool into a personalized learning companion. This is essential for serious learners but not required for the chatbot to function.

**Independent Test**: Can be tested by creating an account, asking questions, logging out, logging back in, and verifying chat history is preserved. Delivers value by enabling longitudinal learning tracking.

**Acceptance Scenarios**:

1. **Given** a new user clicks "Sign Up" in the chatbot, **When** they provide email and password, **Then** an account is created and they are logged in automatically
2. **Given** an authenticated user asks questions in the chatbot, **When** each message is sent, **Then** it is saved to their chat history in the database
3. **Given** a user logs out and logs back in, **When** they open the chatbot, **Then** their previous chat history is displayed
4. **Given** an unauthenticated user interacts with the chatbot, **When** they ask questions, **Then** the conversation is stored temporarily but not persisted (session-only)
5. **Given** a user signs up after asking questions as a guest, **When** account creation completes, **Then** their session chat history is migrated to their account

---

### User Story 4 - Enable Chapter-Specific Personalization (Priority: P3)

A student studying Chapter 5 (Locomotion Algorithms) wants the chatbot to prioritize answers from that chapter. They click a "Personalize for this Chapter" button on the chapter page. The chatbot then prioritizes retrieval from Chapter 5 content while still searching other chapters if needed.

**Why this priority**: Personalization improves answer relevance for focused study sessions but is not essential for basic functionality. It's a quality-of-life enhancement.

**Independent Test**: Can be tested by enabling personalization on a chapter page, asking chapter-related questions, and verifying answers prioritize that chapter's content. Delivers value by reducing noise from unrelated chapters.

**Acceptance Scenarios**:

1. **Given** a user is viewing Chapter 5, **When** they click "Personalize for this Chapter", **Then** a preference is saved indicating Chapter 5 as the focus
2. **Given** personalization is enabled for Chapter 5, **When** the user asks a question, **Then** the retrieval system boosts the ranking of Chapter 5 content in search results
3. **Given** a personalized query returns no good matches in the preferred chapter, **When** the system searches globally, **Then** it still retrieves relevant content from other chapters
4. **Given** a user clicks "Reset Personalization", **When** the preference is cleared, **Then** subsequent queries search all chapters equally
5. **Given** personalization is enabled, **When** the chatbot displays an answer, **Then** it indicates whether the response is from the prioritized chapter

---

### User Story 5 - Translate UI and Instructions to Urdu (Priority: P3)

A student whose primary language is Urdu wants to use the chatbot interface in Urdu. They click an "اردو" (Urdu) button in the chatbot settings. All UI labels, buttons, and system messages switch to Urdu, while book content and answers remain in English (or are translated if supported).

**Why this priority**: Language accessibility broadens the textbook's reach but is not core functionality. This is a bonus feature for inclusivity.

**Independent Test**: Can be tested by toggling the Urdu translation button and verifying UI elements switch to Urdu text. Delivers value by making the tool accessible to Urdu-speaking learners.

**Acceptance Scenarios**:

1. **Given** a user clicks the Urdu translation button, **When** the setting is applied, **Then** all chatbot UI labels (e.g., "Ask a question", "Send", "History") appear in Urdu
2. **Given** the UI is in Urdu, **When** a user types a question in English, **Then** the chatbot still processes and responds in English based on book content
3. **Given** the UI is in Urdu, **When** system messages are displayed (e.g., "No results found"), **Then** they appear in Urdu
4. **Given** a user toggles back to English, **When** the setting is changed, **Then** all UI elements revert to English
5. **Given** the Urdu setting is enabled, **When** the user refreshes the page, **Then** the setting persists (stored in localStorage or user preferences)

---

### Edge Cases

- What happens when a user asks a question in a language other than English (e.g., Urdu, Arabic)? → The system should respond in English (since book content is in English) with a message like "I can only answer based on the English textbook content."
- What happens when the embedding service (Qdrant) is temporarily unavailable? → The chatbot displays an error message: "Search service is temporarily unavailable. Please try again in a moment."
- What happens when a user highlights text that spans multiple paragraphs or pages? → The system captures up to 500 characters of highlighted text and processes the query.
- What happens when the database (Neon Postgres) connection fails during login? → The user sees an error message: "Unable to connect to authentication service. Please try again later."
- What happens when a user's chat history exceeds 1000 messages? → Older messages are archived or the system paginates history retrieval to avoid performance degradation.
- What happens when highlighted text is non-textual (e.g., a diagram, code block)? → The "Ask Chatbot" button does not appear, or the system extracts alt-text/code content if available.
- What happens when multiple users are asking questions simultaneously? → The system handles concurrent requests without performance degradation up to the defined capacity (success criteria SC-002).
- What happens when embedding generation for new book content fails? → An admin notification is triggered, and the content is marked as "not indexed" until the issue is resolved.

## Requirements *(mandatory)*

### Functional Requirements

#### Core Chatbot Functionality

- **FR-001**: System MUST provide a chatbot widget accessible on all Docusaurus book pages
- **FR-002**: System MUST accept natural language questions from users via a text input field
- **FR-003**: System MUST retrieve relevant content from the book using semantic search over vector embeddings
- **FR-004**: System MUST generate answers grounded exclusively in book content (no external knowledge)
- **FR-005**: System MUST include source references (chapter, section) in all chatbot responses
- **FR-006**: System MUST display a clear message when no relevant content is found: "I couldn't find information about that topic in this textbook."
- **FR-007**: System MUST display a loading indicator during query processing (exceeding 500ms)

#### Highlighted Text Interaction

- **FR-008**: System MUST detect when a user highlights text on a book page
- **FR-009**: System MUST display an "Ask Chatbot" button near highlighted text selections
- **FR-010**: System MUST pre-fill the chatbot query with the highlighted text when "Ask Chatbot" is clicked
- **FR-011**: System MUST prioritize search results matching the highlighted phrase and surrounding context
- **FR-012**: System MUST support highlighting up to 500 characters of text

#### User Authentication

- **FR-013**: System MUST provide a signup flow via BetterAuth accepting email and password
- **FR-014**: System MUST provide a signin flow via BetterAuth with email/password authentication
- **FR-015**: System MUST support social login (Google OAuth) as an alternative authentication method
- **FR-016**: System MUST validate email format during signup
- **FR-017**: System MUST enforce password strength requirements: minimum 8 characters, at least one uppercase letter, one number, one special character
- **FR-018**: System MUST send email verification links after signup
- **FR-019**: System MUST provide a password reset flow via email

#### Chat History Persistence

- **FR-020**: System MUST save all chat messages for authenticated users to Neon Postgres database
- **FR-021**: System MUST associate chat history with user accounts via user ID
- **FR-022**: System MUST retrieve and display chat history when authenticated users open the chatbot
- **FR-023**: System MUST store session-only chat history for unauthenticated users (not persisted to database)
- **FR-024**: System MUST migrate session chat history to user account upon signup/login
- **FR-025**: System MUST support paginated retrieval of chat history (50 messages per page)

#### Embedding and Indexing

- **FR-026**: System MUST generate vector embeddings from all Markdown book content files
- **FR-027**: System MUST store embeddings in Qdrant Cloud vector database
- **FR-028**: System MUST index each book section (H2/H3 level) as a separate document with metadata (chapter, section title, page URL)
- **FR-029**: System MUST provide an API endpoint (`/embed`) to trigger re-indexing of book content
- **FR-030**: System MUST handle incremental updates when book content is modified

#### Backend API Endpoints

- **FR-031**: System MUST expose a `/chat` POST endpoint accepting user queries and returning answers with sources
- **FR-032**: System MUST expose a `/search` POST endpoint accepting queries and returning ranked relevant content chunks
- **FR-033**: System MUST expose a `/embed` POST endpoint to trigger embedding generation for specified content
- **FR-034**: System MUST expose a `/history` GET endpoint to retrieve authenticated user's chat history
- **FR-035**: System MUST implement rate limiting on all API endpoints (100 requests per minute per user)

#### Personalization

- **FR-036**: System MUST provide a "Personalize for this Chapter" button on each chapter page
- **FR-037**: System MUST store user's chapter preference (if authenticated) or in localStorage (if unauthenticated)
- **FR-038**: System MUST boost ranking of content from the preferred chapter in search results by 2x
- **FR-039**: System MUST provide a "Reset Personalization" option to clear chapter preferences
- **FR-040**: System MUST still search globally if preferred chapter yields no relevant results

#### Translation

- **FR-041**: System MUST provide an Urdu translation toggle button in the chatbot UI
- **FR-042**: System MUST translate all UI labels, buttons, and system messages to Urdu when enabled
- **FR-043**: System MUST store language preference in localStorage for unauthenticated users
- **FR-044**: System MUST store language preference in user account for authenticated users
- **FR-045**: System MUST persist language preference across sessions

#### Agent Architecture

- **FR-046**: System MUST implement a RAG-Agent responsible for embedding generation, semantic search, and retrieval
- **FR-047**: System MUST implement a UI-Agent responsible for frontend components and Docusaurus integration
- **FR-048**: System MUST implement an Auth-Agent responsible for BetterAuth flows and Neon DB user operations
- **FR-049**: System MUST implement a Skill-Agent providing reusable intelligence modules (e.g., query reformulation, answer grounding)
- **FR-050**: System MUST ensure agents communicate via well-defined interfaces (REST APIs or message queues)

### Key Entities

- **User**: Represents a registered user with email, password hash, account creation date, language preference, and personalization settings. Associated with chat history.
- **ChatMessage**: Represents a single message in a conversation with user ID, message text, timestamp, message type (user/assistant), and source references.
- **BookContent**: Represents a section of the book with title, chapter, section, content text, embedding vector, and page URL.
- **Embedding**: Vector representation of book content stored in Qdrant with metadata (chapter, section, URL).
- **UserPreference**: Stores user-specific settings like preferred chapter for personalization and language preference.
- **ChatSession**: Represents a conversation session (for unauthenticated users) with session ID, message list, and expiration timestamp.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive chatbot responses in under 3 seconds for 95% of queries
- **SC-002**: System handles at least 100 concurrent users without response time degradation
- **SC-003**: Chatbot answers are grounded in book content with 100% accuracy (no hallucinations)
- **SC-004**: Users can create an account and complete authentication in under 2 minutes
- **SC-005**: Highlighted text queries return relevant results in under 2 seconds for 95% of cases
- **SC-006**: Chat history is successfully retrieved for authenticated users 99.9% of the time
- **SC-007**: Embedding generation pipeline processes all book content (estimated 500+ sections) in under 10 minutes
- **SC-008**: System achieves 90% user task completion rate for primary flows (ask question, highlight text, login)
- **SC-009**: Translation toggle switches UI language instantly (under 100ms)
- **SC-010**: Personalization feature improves answer relevance by prioritizing the correct chapter in 90% of chapter-specific queries

## Architecture Overview *(mandatory)*

### System Architecture Diagram (Text)

```
┌─────────────────────────────────────────────────────────────────┐
│                         User Browser                             │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Docusaurus Frontend (Vercel)                              │ │
│  │  - Book Pages (Markdown → React)                           │ │
│  │  - ChatWidget.jsx (Chatbot UI)                             │ │
│  │  - useSelectedText.js (Highlight Detection Hook)           │ │
│  │  - Translation Module (Urdu/English Toggle)                │ │
│  │  - Personalization Button (Per-Chapter Preference)         │ │
│  └─────────────┬──────────────────────────────────────────────┘ │
└────────────────┼────────────────────────────────────────────────┘
                 │ HTTPS (REST API Calls)
                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                   FastAPI Backend (Render)                       │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  API Endpoints:                                            │ │
│  │  - POST /chat (conversational queries)                     │ │
│  │  - POST /search (semantic search)                          │ │
│  │  - POST /embed (trigger embedding generation)              │ │
│  │  - GET /history (retrieve user chat history)               │ │
│  │  - POST /auth/signup (BetterAuth)                          │ │
│  │  - POST /auth/signin (BetterAuth)                          │ │
│  │  - POST /auth/reset-password (BetterAuth)                  │ │
│  └─────────────┬─────────────┬────────────────┬───────────────┘ │
└────────────────┼─────────────┼────────────────┼─────────────────┘
                 │             │                │
        ┌────────▼────┐  ┌─────▼──────┐  ┌─────▼──────────┐
        │  Qdrant     │  │   Neon     │  │  OpenAI API    │
        │  Cloud      │  │  Postgres  │  │  (Embeddings + │
        │  (Vector DB)│  │  (Users +  │  │   ChatKit SDK) │
        │             │  │   History) │  │                │
        └─────────────┘  └────────────┘  └────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                     Claude Code Subagents                        │
│  ┌───────────────┐ ┌───────────────┐ ┌───────────────────────┐ │
│  │  RAG-Agent    │ │  UI-Agent     │ │  Auth-Agent           │ │
│  │  - Embedding  │ │  - Frontend   │ │  - BetterAuth logic   │ │
│  │  - Search     │ │  - Docusaurus │ │  - User DB ops        │ │
│  │  - Retrieval  │ │  - Components │ │  - Session mgmt       │ │
│  └───────────────┘ └───────────────┘ └───────────────────────┘ │
│                      ┌───────────────────┐                       │
│                      │  Skill-Agent      │                       │
│                      │  - Query rewrite  │                       │
│                      │  - Answer ground  │                       │
│                      │  - Context merge  │                       │
│                      └───────────────────┘                       │
└─────────────────────────────────────────────────────────────────┘
```

### Component Responsibilities

**Frontend (Docusaurus + React)**:
- Render book content from Markdown files
- Display ChatWidget component on all pages
- Detect text highlighting and show "Ask Chatbot" button
- Handle user authentication UI (signup/signin modals)
- Manage language toggle (Urdu/English)
- Store unauthenticated user preferences in localStorage

**FastAPI Backend**:
- Handle all API requests from frontend
- Integrate with BetterAuth for user authentication
- Query Qdrant for semantic search over embeddings
- Generate embeddings via OpenAI API
- Store and retrieve chat history from Neon Postgres
- Implement rate limiting and error handling
- Coordinate with Claude Subagents for specialized tasks

**Qdrant Cloud**:
- Store vector embeddings for all book content sections
- Provide semantic search API for retrieval
- Maintain metadata (chapter, section, URL) with embeddings

**Neon Postgres**:
- Store user accounts (id, email, password_hash, created_at)
- Store chat history (id, user_id, message, timestamp, role)
- Store user preferences (user_id, language, preferred_chapter)

**OpenAI API / ChatKit SDK**:
- Generate embeddings for book content (text-embedding-3-small)
- Generate conversational responses grounded in retrieved content

**Claude Code Subagents**:
- **RAG-Agent**: Handles embedding pipeline, Qdrant integration, retrieval logic
- **UI-Agent**: Builds React components, integrates with Docusaurus
- **Auth-Agent**: Implements BetterAuth flows, Neon DB schema, user operations
- **Skill-Agent**: Provides reusable logic for query reformulation, answer validation, context merging

### Data Flow: Chat → Retrieval → Answer

1. **User submits query** in ChatWidget (or highlighted text triggers query)
2. **Frontend** sends POST request to `/chat` with query text and user context (authenticated user ID or session ID)
3. **Backend** receives query:
   - If user is authenticated, fetch user preferences (personalization, language)
   - If highlighted text, prioritize exact phrase matching
4. **Embedding generation**: Backend calls OpenAI API to generate embedding for user query
5. **Semantic search**: Backend queries Qdrant with query embedding, retrieves top 5 relevant content chunks
   - If personalization enabled, boost ranking for preferred chapter
6. **Answer generation**: Backend sends retrieved chunks + user query to OpenAI ChatKit SDK with system prompt:
   - "Answer the user's question using ONLY the provided book content. Cite the chapter and section."
7. **Response validation** (Skill-Agent): Verify answer is grounded in retrieved content (no hallucinations)
8. **Save to history**: If user is authenticated, save query + response to Neon Postgres
9. **Return response**: Backend sends answer + source references to frontend
10. **Frontend displays**: Chatbot shows answer with clickable chapter/section references

### Data Flow: Highlighted Text → Query

1. **User highlights text** on a book page
2. **useSelectedText.js hook** detects selection event, captures highlighted text (up to 500 chars)
3. **"Ask Chatbot" button** appears near selection
4. **User clicks button**: ChatWidget opens, query field pre-filled with "Explain: [highlighted text]"
5. **Follows same flow as Chat → Retrieval → Answer** above, with boosted ranking for exact phrase matches

### Deployment Architecture

- **Frontend**: Deployed on **Vercel** (static Docusaurus build with React components)
- **Backend**: Deployed on **Render** (FastAPI app with Uvicorn server)
- **Vector Database**: **Qdrant Cloud Free Tier** (hosted, managed)
- **Relational Database**: **Neon Serverless Postgres** (hosted, managed)
- **Authentication**: **BetterAuth** (library integrated into FastAPI backend)
- **Embeddings**: **OpenAI API** (text-embedding-3-small model)
- **CI/CD**: GitHub Actions for automated deployments (Vercel for frontend, Render for backend)

### Technology Stack Summary

| Component | Technology |
|-----------|-----------|
| Frontend Framework | Docusaurus (React) |
| UI Styling | Tailwind CSS |
| Backend Framework | FastAPI (Python) |
| Authentication | BetterAuth |
| Vector Database | Qdrant Cloud |
| Relational Database | Neon Serverless Postgres |
| Embeddings Model | OpenAI text-embedding-3-small |
| Conversational AI | OpenAI ChatKit SDK |
| Deployment (Frontend) | Vercel |
| Deployment (Backend) | Render |
| Agent Framework | Claude Code Subagents + Skills |

## Assumptions

1. Book content is structured as Markdown files with consistent heading hierarchy (H1 for chapters, H2 for sections)
2. Docusaurus site is already configured and running with existing content
3. Users have stable internet connectivity for real-time chatbot interactions
4. OpenAI API usage stays within free tier or budget limits for embedding + chat generation
5. Qdrant Cloud free tier provides sufficient storage and query capacity (approximately 1GB, 100K vectors)
6. Neon Postgres free tier provides sufficient storage for user accounts and chat history
7. Book content is primarily in English; Urdu translation applies only to UI labels, not content
8. Highlighted text feature works in modern browsers supporting `window.getSelection()` API
9. Users accept standard password-based authentication (no biometric or hardware key support required)
10. Chat history is retained indefinitely (no automatic deletion policy, though this can be added later)

## Out of Scope

- Multi-language support for book content itself (only UI translation to Urdu)
- Voice input/output for chatbot interactions
- Real-time collaborative chat sessions between multiple users
- Mobile native apps (web-only, responsive design)
- Advanced analytics dashboard for user chat patterns
- Integration with external learning management systems (LMS)
- Automated question generation from book content
- Gamification features (badges, points, leaderboards)
- Offline mode for chatbot (requires internet connection)
- Export chat history to PDF or other formats
- Admin panel for managing users or content (manual database operations for now)

## Dependencies

- OpenAI API availability and rate limits
- Qdrant Cloud service uptime and performance
- Neon Postgres service uptime and connection stability
- Vercel deployment pipeline for frontend updates
- Render deployment pipeline for backend updates
- BetterAuth library compatibility with FastAPI and Neon Postgres
- Docusaurus build system and plugin compatibility with custom React components

## Risks

- **Embedding generation cost**: If book content exceeds expectations, OpenAI API costs may increase significantly
- **Search quality**: Semantic search may return irrelevant results for ambiguous or poorly phrased questions
- **Hallucination risk**: Despite grounding instructions, ChatKit may generate responses not fully supported by retrieved content
- **Performance degradation**: High concurrent user load may exceed free-tier limits of Qdrant or Neon
- **Data privacy**: Chat history storage requires compliance with data protection regulations (GDPR, etc.)
- **Authentication security**: Password storage and session management must follow best practices to prevent breaches
- **Highlighted text limitations**: Browser compatibility issues may affect text selection detection on some devices

## Next Steps

1. Proceed to `/sp.plan` phase to design detailed technical architecture and implementation approach
2. Define database schemas for Neon Postgres (users, chat_messages, user_preferences tables)
3. Design Qdrant collection schema with metadata fields (chapter, section, url, content_hash)
4. Specify API contract for all FastAPI endpoints (request/response formats)
5. Design React component hierarchy for ChatWidget and related UI elements
6. Define Claude Subagent responsibilities and inter-agent communication protocols
7. Create implementation tasks in `/sp.tasks` phase with acceptance tests for each functional requirement
