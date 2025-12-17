# RAG Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics textbook chatbot.

## Quick Start

### 1. Create Virtual Environment

```bash
python -m venv venv

# Windows
venv\Scripts\activate

# Mac/Linux
source venv/bin/activate
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Configure Environment

Create `.env` file (copy from `.env.example`):

```bash
cp .env.example .env
```

Update the following variables:
- `DATABASE_URL`: Your Neon Postgres connection string
- `QDRANT_URL` and `QDRANT_API_KEY`: Your Qdrant Cloud credentials
- `OPENAI_API_KEY`: Your OpenAI API key
- `JWT_SECRET`: Generate a secure secret key
- `SENDGRID_API_KEY`: Your SendGrid API key (for emails)

### 4. Initialize Database

```bash
# Run Alembic migrations
alembic upgrade head
```

### 5. Initialize Qdrant Collection

```bash
python scripts/init_qdrant_collection.py
```

### 6. Run Development Server

```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

API will be available at:
- **API**: http://localhost:8000/v1
- **Docs**: http://localhost:8000/docs
- **Health**: http://localhost:8000/v1/health

## Project Structure

```
backend/
├── src/
│   ├── api/              # API route handlers
│   │   ├── health.py
│   │   ├── chat.py
│   │   ├── auth.py
│   │   ├── admin.py
│   │   └── preferences.py
│   ├── services/         # Business logic
│   │   ├── rag_service.py
│   │   ├── chat_service.py
│   │   ├── auth_service.py
│   │   ├── embedding_service.py
│   │   └── qdrant_client.py
│   ├── models/           # Pydantic schemas
│   │   ├── user.py
│   │   ├── chat.py
│   │   └── embedding.py
│   ├── db/               # Database models
│   │   ├── base.py
│   │   ├── models.py
│   │   └── session.py
│   ├── middleware/       # Middleware
│   │   └── jwt_middleware.py
│   ├── utils/            # Utilities
│   │   ├── password.py
│   │   ├── jwt.py
│   │   ├── email.py
│   │   └── system_prompts.py
│   ├── config.py         # Configuration
│   └── main.py           # FastAPI app
├── scripts/              # Utility scripts
│   ├── init_qdrant_collection.py
│   └── index_book_content.py
├── alembic/              # Database migrations
├── tests/                # Tests
├── requirements.txt
└── .env.example
```

## API Endpoints

### Chat
- `POST /v1/chat` - Send chat message and get AI response
- `POST /v1/search` - Semantic search over book content

### Authentication
- `POST /v1/auth/signup` - Create new account
- `POST /v1/auth/signin` - Sign in and get JWT
- `GET /v1/auth/verify` - Verify email
- `POST /v1/auth/reset-password-request` - Request password reset
- `POST /v1/auth/reset-password` - Reset password

### Preferences
- `PUT /v1/auth/preferences` - Update language and chapter preferences

### Admin
- `POST /v1/admin/embed` - Trigger embedding generation

### Health
- `GET /v1/health` - Health check

## Indexing Book Content

After setting up the backend, index your book content:

```bash
python scripts/index_book_content.py
```

This script:
1. Parses all Markdown files in `docs/` directory
2. Extracts H2/H3 sections
3. Generates embeddings using OpenAI
4. Upserts vectors to Qdrant

## Testing

Run tests:

```bash
pytest tests/ -v --cov=src
```

## Deployment

See `DEPLOYMENT.md` for production deployment instructions.
