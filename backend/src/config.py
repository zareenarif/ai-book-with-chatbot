from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    # Application
    ENV: str = "development"
    DEBUG: bool = True
    API_HOST: str = "0.0.0.0"
    API_PORT: int = 8000
    CORS_ORIGINS: str = "http://localhost:3000"

    # Database
    DATABASE_URL: str

    # Qdrant
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_NAME: str = "book_content"

    # Embeddings (FREE HuggingFace model - runs locally!)
    EMBEDDING_MODEL: str = "all-MiniLM-L6-v2"

    # Google Gemini (FREE tier available! - for chat generation)
    GEMINI_API_KEY: str
    CHAT_MODEL: str = "gemini-1.5-flash"  # Fast, free tier available

    # JWT
    JWT_SECRET: str
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRE_MINUTES: int = 15
    REFRESH_TOKEN_EXPIRE_DAYS: int = 7

    # SendGrid
    SENDGRID_API_KEY: str
    SENDGRID_FROM_EMAIL: str
    SENDGRID_FROM_NAME: str = "RAG Chatbot"

    # Sentry
    SENTRY_DSN: str = ""

    # Rate Limiting
    RATE_LIMIT_ENABLED: bool = True
    RATE_LIMIT_CHAT: str = "100/minute"
    RATE_LIMIT_AUTH: str = "5/15minute"

    @property
    def cors_origins_list(self) -> List[str]:
        return [origin.strip() for origin in self.CORS_ORIGINS.split(",")]

    class Config:
        env_file = ".env"
        case_sensitive = True


settings = Settings()
