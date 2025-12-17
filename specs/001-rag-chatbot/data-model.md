# Data Models: RAG Chatbot Integration

**Feature**: RAG Chatbot Integration
**Branch**: 001-rag-chatbot
**Date**: 2025-12-10

## Overview

This document defines all data entities, database schemas, and validation rules for the RAG Chatbot system. Data models are split between Neon Postgres (relational data) and Qdrant (vector embeddings).

---

## 1. Neon Postgres Schema

### 1.1 users Table

Stores registered user accounts with authentication credentials and preferences.

```sql
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    language_pref VARCHAR(10) DEFAULT 'en',  -- 'en' or 'ur'
    preferred_chapter VARCHAR(100) NULL,     -- e.g., 'Chapter 5' or NULL
    email_verified BOOLEAN DEFAULT FALSE,
    verification_token VARCHAR(255) NULL,
    reset_token VARCHAR(255) NULL,
    reset_token_expires TIMESTAMP WITH TIME ZONE NULL,
    CONSTRAINT email_format CHECK (email ~* '^[A-Z0-9._%+-]+@[A-Z0-9.-]+\.[A-Z]{2,}$')
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_verification_token ON users(verification_token);
CREATE INDEX idx_users_reset_token ON users(reset_token);
```

**Fields**:
- `id`: Auto-increment primary key
- `email`: User email address (unique, validated format)
- `password_hash`: Bcrypt hashed password (never store plaintext)
- `created_at`: Account creation timestamp
- `updated_at`: Last modification timestamp
- `language_pref`: UI language preference ('en' or 'ur')
- `preferred_chapter`: Chapter for personalized search boost (nullable)
- `email_verified`: Email verification status
- `verification_token`: Token sent via email for verification
- `reset_token`: Token for password reset flow
- `reset_token_expires`: Expiration timestamp for reset token (15 min TTL)

**Validation Rules**:
- Email must match RFC 5322 format
- Password must be hashed with Bcrypt (rounds=12)
- language_pref must be 'en' or 'ur'
- preferred_chapter must match existing chapter titles (validated at application level)

**Relationships**:
- One user → Many chat_messages (via user_id foreign key)

---

### 1.2 chat_messages Table

Stores all chat messages for authenticated users with conversation history.

```sql
CREATE TABLE chat_messages (
    id SERIAL PRIMARY KEY,
    user_id INTEGER NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    message TEXT NOT NULL,
    role VARCHAR(20) NOT NULL,  -- 'user' or 'assistant'
    sources JSONB NULL,  -- Array of {chapter, section, url, score}
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT role_check CHECK (role IN ('user', 'assistant'))
);

CREATE INDEX idx_chat_messages_user_id ON chat_messages(user_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at DESC);
CREATE INDEX idx_chat_messages_user_created ON chat_messages(user_id, created_at DESC);
```

**Fields**:
- `id`: Auto-increment primary key
- `user_id`: Foreign key to users table
- `message`: Message text content
- `role`: Message sender ('user' = student query, 'assistant' = chatbot response)
- `sources`: JSONB array of source references for assistant responses (null for user messages)
- `created_at`: Message timestamp

**JSONB Structure for sources**:
```json
[
  {
    "chapter": "Chapter 3",
    "section": "ROS 2 Architecture",
    "url": "/docs/chapter-3/ros2-architecture",
    "score": 0.89
  },
  {
    "chapter": "Chapter 3",
    "section": "Nodes and Topics",
    "url": "/docs/chapter-3/nodes-topics",
    "score": 0.76
  }
]
```

**Validation Rules**:
- message must be non-empty (max 5000 characters)
- role must be 'user' or 'assistant'
- sources must be valid JSONB array (validated at application level)
- user_id must reference existing user

**Relationships**:
- Many chat_messages → One user (via user_id foreign key)

**Query Patterns**:
```sql
-- Get recent chat history for user (last 50 messages)
SELECT id, message, role, sources, created_at
FROM chat_messages
WHERE user_id = $1
ORDER BY created_at DESC
LIMIT 50;

-- Get conversation context (last 10 messages for multi-turn chat)
SELECT message, role
FROM chat_messages
WHERE user_id = $1
ORDER BY created_at DESC
LIMIT 10;
```

---

### 1.3 chat_sessions Table

Stores temporary chat sessions for unauthenticated users (session-only storage).

```sql
CREATE TABLE chat_sessions (
    id SERIAL PRIMARY KEY,
    session_id VARCHAR(255) UNIQUE NOT NULL,
    messages JSONB NOT NULL DEFAULT '[]',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    CONSTRAINT expires_after_created CHECK (expires_at > created_at)
);

CREATE INDEX idx_chat_sessions_session_id ON chat_sessions(session_id);
CREATE INDEX idx_chat_sessions_expires_at ON chat_sessions(expires_at);
```

**Fields**:
- `id`: Auto-increment primary key
- `session_id`: UUID generated by frontend (stored in localStorage)
- `messages`: JSONB array of all messages in session
- `created_at`: Session creation timestamp
- `expires_at`: Session expiration timestamp (24 hours after creation)

**JSONB Structure for messages**:
```json
[
  {
    "message": "What is a ROS 2 node?",
    "role": "user",
    "timestamp": "2025-12-10T10:30:00Z"
  },
  {
    "message": "A ROS 2 node is...",
    "role": "assistant",
    "sources": [...],
    "timestamp": "2025-12-10T10:30:02Z"
  }
]
```

**Validation Rules**:
- session_id must be UUID v4 format
- messages must be valid JSONB array
- expires_at must be after created_at
- Expired sessions should be deleted via periodic cleanup job (cron or Render background worker)

**Session Migration on Signup**:
When user creates account after using chatbot as guest:
1. Copy messages from `chat_sessions.messages` to `chat_messages` table
2. Associate with newly created `user_id`
3. Delete session from `chat_sessions` table

---

## 2. Qdrant Collection Schema

### 2.1 book_content Collection

Stores vector embeddings for all book content sections with metadata for retrieval.

**Collection Configuration**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

client.create_collection(
    collection_name="book_content",
    vectors_config=VectorParams(
        size=1536,  # OpenAI text-embedding-3-small dimension
        distance=Distance.COSINE
    )
)
```

**Document Schema**:
Each document (point) in Qdrant contains:
- `id`: UUID (generated from content_hash for idempotency)
- `vector`: 1536-dimensional embedding (list of floats)
- `payload`: Metadata dictionary

**Payload Structure**:
```python
{
    "chapter": str,          # e.g., "Chapter 3"
    "section": str,          # e.g., "ROS 2 Architecture"
    "content": str,          # Full text content (for display)
    "url": str,              # Docusaurus page URL
    "content_hash": str,     # SHA256 hash of content (for deduplication)
    "indexed_at": str        # ISO 8601 timestamp
}
```

**Example Document**:
```python
{
    "id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "vector": [0.123, -0.456, ...],  # 1536 floats
    "payload": {
        "chapter": "Chapter 3",
        "section": "ROS 2 Nodes",
        "content": "A ROS 2 node is a participant in the ROS graph...",
        "url": "/docs/chapter-3/ros2-nodes",
        "content_hash": "sha256:a1b2c3...",
        "indexed_at": "2025-12-10T10:00:00Z"
    }
}
```

**Indexing Strategy**:
- Index each H2/H3 section as separate document (FR-028)
- Generate content_hash from section text (SHA256)
- Use content_hash as UUID seed (ensures idempotency: re-indexing same content won't create duplicates)
- Batch upsert in chunks of 100 documents

**Query Patterns**:
```python
# Basic semantic search (no personalization)
results = client.search(
    collection_name="book_content",
    query_vector=query_embedding,
    limit=5,
    score_threshold=0.5
)

# Personalized search (boost scores for preferred chapter)
results = client.search(
    collection_name="book_content",
    query_vector=query_embedding,
    limit=10,  # Fetch more to allow for boosting
    score_threshold=0.4
)
# Post-process: boost scores for preferred chapter by 2x
boosted_results = boost_scores(results, preferred_chapter="Chapter 5")
top_5 = boosted_results[:5]
```

---

## 3. Pydantic Models (API Contracts)

Pydantic models for request/response validation in FastAPI.

### 3.1 User Models

```python
from pydantic import BaseModel, EmailStr, Field
from datetime import datetime
from typing import Optional

class UserCreate(BaseModel):
    """Request model for user signup"""
    email: EmailStr
    password: str = Field(
        ...,
        min_length=8,
        regex="^(?=.*[A-Z])(?=.*[0-9])(?=.*[!@#$%^&*])"
    )

class UserLogin(BaseModel):
    """Request model for user signin"""
    email: EmailStr
    password: str

class UserResponse(BaseModel):
    """Response model for user data"""
    id: int
    email: str
    created_at: datetime
    language_pref: str
    preferred_chapter: Optional[str] = None

    class Config:
        from_attributes = True

class UserPreferences(BaseModel):
    """Request model for updating user preferences"""
    language_pref: Optional[str] = Field(None, regex="^(en|ur)$")
    preferred_chapter: Optional[str] = None
```

---

### 3.2 Chat Models

```python
from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime

class ChatSource(BaseModel):
    """Source reference for chatbot response"""
    chapter: str
    section: str
    url: str
    score: float

class ChatMessageRequest(BaseModel):
    """Request model for /chat endpoint"""
    message: str = Field(..., min_length=1, max_length=5000)
    user_id: Optional[int] = None  # None for unauthenticated users
    session_id: Optional[str] = None  # UUID for guest sessions
    highlighted_text: Optional[str] = None  # If query triggered by text selection

class ChatMessageResponse(BaseModel):
    """Response model for /chat endpoint"""
    message: str
    role: str = "assistant"
    sources: List[ChatSource]
    timestamp: datetime

class ChatHistoryResponse(BaseModel):
    """Response model for /history endpoint"""
    messages: List[dict]
    total_count: int
    page: int
    per_page: int
```

---

### 3.3 Embedding Models

```python
from pydantic import BaseModel
from typing import List

class EmbedRequest(BaseModel):
    """Request model for /embed endpoint"""
    content_chunks: List[dict]  # [{"chapter": "...", "section": "...", "content": "..."}]

class EmbedResponse(BaseModel):
    """Response model for /embed endpoint"""
    success: bool
    indexed_count: int
    failed_count: int
    errors: List[str]
```

---

### 3.4 Search Models

```python
from pydantic import BaseModel
from typing import List, Optional

class SearchRequest(BaseModel):
    """Request model for /search endpoint"""
    query: str = Field(..., min_length=1, max_length=500)
    top_k: int = Field(5, ge=1, le=20)
    preferred_chapter: Optional[str] = None

class SearchResult(BaseModel):
    """Single search result"""
    chapter: str
    section: str
    content: str
    url: str
    score: float

class SearchResponse(BaseModel):
    """Response model for /search endpoint"""
    results: List[SearchResult]
    query: str
```

---

## 4. State Transitions

### 4.1 User Registration Flow

```
[Unregistered]
    → POST /auth/signup (email, password)
    → [Pending Verification] (email_verified=false, verification_token set)
    → Email sent with verification link
    → GET /auth/verify?token={verification_token}
    → [Verified User] (email_verified=true, verification_token cleared)
```

### 4.2 Password Reset Flow

```
[Verified User]
    → POST /auth/reset-password-request (email)
    → [Reset Token Generated] (reset_token set, reset_token_expires = now + 15min)
    → Email sent with reset link
    → POST /auth/reset-password (token, new_password)
    → [Password Updated] (password_hash updated, reset_token cleared)
```

### 4.3 Chat Session Lifecycle

```
[Guest User]
    → Open ChatWidget
    → [Session Created] (session_id generated, stored in localStorage)
    → POST /chat (with session_id)
    → [Messages Stored] in chat_sessions table
    → [Session Expires] after 24 hours (expires_at reached)
    → Periodic cleanup job deletes expired sessions

[Guest → Registered]
    → POST /auth/signup during active session
    → [Session Migrated] (messages copied to chat_messages table)
    → [Session Deleted] from chat_sessions
```

---

## 5. Data Validation Rules

### 5.1 Password Requirements (FR-017)
- Minimum 8 characters
- At least one uppercase letter
- At least one number
- At least one special character (!@#$%^&*)

**Validation Regex**:
```python
import re

def validate_password(password: str) -> bool:
    if len(password) < 8:
        return False
    if not re.search(r"[A-Z]", password):
        return False
    if not re.search(r"[0-9]", password):
        return False
    if not re.search(r"[!@#$%^&*]", password):
        return False
    return True
```

### 5.2 Email Format (FR-016)
- Must match RFC 5322 format
- Use Pydantic's `EmailStr` for validation

### 5.3 Message Length
- User messages: 1-5000 characters
- Highlighted text: 1-500 characters (FR-012)

### 5.4 Language Preference
- Must be 'en' or 'ur' (no other values allowed)

### 5.5 Chapter Name Format
- Must match existing chapter names in book (validated at application level)
- Pattern: `"Chapter {number}"` or specific chapter titles

---

## 6. Database Migrations

Use Alembic for versioned schema migrations.

**Initial Migration** (`alembic/versions/001_initial_schema.py`):
```python
"""Initial schema

Revision ID: 001
Revises:
Create Date: 2025-12-10 10:00:00
"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects.postgresql import JSONB

def upgrade():
    # Create users table
    op.create_table(
        'users',
        sa.Column('id', sa.Integer(), primary_key=True),
        sa.Column('email', sa.String(255), unique=True, nullable=False),
        sa.Column('password_hash', sa.String(255), nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.func.now()),
        sa.Column('updated_at', sa.TIMESTAMP(timezone=True), server_default=sa.func.now()),
        sa.Column('language_pref', sa.String(10), server_default='en'),
        sa.Column('preferred_chapter', sa.String(100), nullable=True),
        sa.Column('email_verified', sa.Boolean(), server_default='false'),
        sa.Column('verification_token', sa.String(255), nullable=True),
        sa.Column('reset_token', sa.String(255), nullable=True),
        sa.Column('reset_token_expires', sa.TIMESTAMP(timezone=True), nullable=True)
    )
    op.create_index('idx_users_email', 'users', ['email'])

    # Create chat_messages table
    op.create_table(
        'chat_messages',
        sa.Column('id', sa.Integer(), primary_key=True),
        sa.Column('user_id', sa.Integer(), sa.ForeignKey('users.id', ondelete='CASCADE'), nullable=False),
        sa.Column('message', sa.Text(), nullable=False),
        sa.Column('role', sa.String(20), nullable=False),
        sa.Column('sources', JSONB, nullable=True),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.func.now())
    )
    op.create_index('idx_chat_messages_user_id', 'chat_messages', ['user_id'])

    # Create chat_sessions table
    op.create_table(
        'chat_sessions',
        sa.Column('id', sa.Integer(), primary_key=True),
        sa.Column('session_id', sa.String(255), unique=True, nullable=False),
        sa.Column('messages', JSONB, server_default='[]'),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), server_default=sa.func.now()),
        sa.Column('expires_at', sa.TIMESTAMP(timezone=True), nullable=False)
    )
    op.create_index('idx_chat_sessions_session_id', 'chat_sessions', ['session_id'])

def downgrade():
    op.drop_table('chat_sessions')
    op.drop_table('chat_messages')
    op.drop_table('users')
```

---

## Summary

This data model provides:
- **3 PostgreSQL tables**: users, chat_messages, chat_sessions
- **1 Qdrant collection**: book_content (vector embeddings)
- **11 Pydantic models**: Request/response validation for all API endpoints
- **5 state transition flows**: User registration, password reset, session lifecycle
- **6 validation rule categories**: Password, email, message length, language, chapter names

All models support the functional requirements (FR-001 to FR-050) and enable the success criteria (SC-001 to SC-010) defined in spec.md.
