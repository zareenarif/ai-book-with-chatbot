# Physical AI & Humanoid Robotics Textbook with RAG Chatbot

An interactive Docusaurus textbook featuring an AI-powered chatbot that answers questions grounded in book content.

## Features

- **RAG Chatbot**: Ask questions and get answers sourced directly from the textbook
- **Highlighted Text Queries**: Select text and ask the chatbot to explain it
- **User Authentication**: Sign up to save chat history
- **Chapter Personalization**: Prioritize content from specific chapters
- **Urdu Translation**: Toggle UI between English and Urdu
- **Semantic Search**: Vector-based search over book content
- **Source References**: Every answer includes chapter and section references

## Tech Stack

**Backend:**
- FastAPI 0.104+
- Python 3.11+
- OpenAI (text-embedding-3-small, gpt-4o-mini)
- Qdrant Cloud (vector database)
- Neon Serverless Postgres
- BetterAuth (JWT authentication)
- SendGrid (email)

**Frontend:**
- Docusaurus 3.x
- React 18
- i18next (translation)
- Tailwind CSS

**Deployment:**
- Render (backend)
- Vercel (frontend)
- GitHub Actions (CI/CD)

## Quick Start

### Prerequisites

- Python 3.11+
- Node.js 18+
- OpenAI API key
- Qdrant Cloud account
- Neon Postgres account

### Backend Setup

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Mac/Linux
venv\Scripts\activate     # Windows

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your credentials

# Run migrations
alembic upgrade head

# Initialize Qdrant collection
python scripts/init_qdrant_collection.py

# Start server
uvicorn src.main:app --reload
```

Backend runs at: http://localhost:8000

### Frontend Setup

```bash
# Install dependencies
npm install

# Configure environment
cp .env.local.example .env.local
# Edit .env.local with backend URL

# Start development server
npm start
```

Frontend runs at: http://localhost:3000

### Index Book Content

```bash
cd backend
python scripts/index_book_content.py
```

## Project Structure

```
hackathon-claude/
├── backend/                    # FastAPI backend
│   ├── src/
│   │   ├── api/               # API routes
│   │   ├── services/          # Business logic
│   │   ├── models/            # Pydantic schemas
│   │   ├── db/                # Database models
│   │   ├── middleware/        # Auth middleware
│   │   ├── utils/             # Utilities
│   │   └── main.py           # FastAPI app
│   ├── scripts/              # Utility scripts
│   ├── alembic/              # Database migrations
│   ├── requirements.txt
│   └── .env.example
├── src/                       # Frontend source
│   ├── components/
│   │   ├── ChatWidget/       # Chatbot UI
│   │   └── PersonalizeButton.jsx
│   ├── context/              # React context
│   ├── services/             # API client
│   ├── locales/              # i18next translations
│   └── theme/                # Docusaurus theme
├── docs/                     # Book content (Markdown)
├── specs/                    # Feature specifications
│   └── 001-rag-chatbot/
│       ├── spec.md
│       ├── plan.md
│       ├── tasks.md
│       ├── research.md
│       ├── data-model.md
│       └── quickstart.md
├── package.json
├── DEPLOYMENT.md
└── README.md
```

## API Endpoints

### Chat
- `POST /v1/chat` - Send message, get AI response
- `POST /v1/search` - Semantic search

### Authentication
- `POST /v1/auth/signup` - Create account
- `POST /v1/auth/signin` - Sign in (get JWT)
- `GET /v1/auth/verify` - Verify email
- `POST /v1/auth/reset-password-request` - Request password reset

### Preferences
- `PUT /v1/auth/preferences` - Update language/chapter preferences

### Admin
- `POST /v1/admin/embed` - Trigger embedding generation

## Development

### Backend Tests

```bash
cd backend
pytest tests/ -v --cov=src
```

### Frontend Tests

```bash
npm test
```

### Database Migrations

Create new migration:

```bash
cd backend
alembic revision --autogenerate -m "description"
```

Apply migrations:

```bash
alembic upgrade head
```

## Deployment

See [DEPLOYMENT.md](./DEPLOYMENT.md) for detailed deployment instructions.

Quick deployment:
1. Backend → Render
2. Frontend → Vercel
3. Database → Neon Postgres
4. Vector DB → Qdrant Cloud

## Documentation

- **Specification**: `specs/001-rag-chatbot/spec.md`
- **Implementation Plan**: `specs/001-rag-chatbot/plan.md`
- **Tasks**: `specs/001-rag-chatbot/tasks.md`
- **Data Models**: `specs/001-rag-chatbot/data-model.md`
- **Quickstart**: `specs/001-rag-chatbot/quickstart.md`
- **API Contracts**: `specs/001-rag-chatbot/contracts/openapi.yaml`
- **Technology Decisions**: `specs/001-rag-chatbot/research.md`

## Support

For issues or questions:
- Check [quickstart.md](./specs/001-rag-chatbot/quickstart.md) for common setup issues
- Review [spec.md](./specs/001-rag-chatbot/spec.md) for requirements
- See [data-model.md](./specs/001-rag-chatbot/data-model.md) for database schemas
- API docs: http://localhost:8000/docs

## License

MIT