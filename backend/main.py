from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
import sentry_sdk
from datetime import datetime

from src.config import settings
from src.api import health, chat, auth, admin, preferences

# Initialize Sentry if DSN is provi
# ded
if settings.SENTRY_DSN:
    sentry_sdk.init(
        dsn=settings.SENTRY_DSN,
        environment=settings.ENV,
        traces_sample_rate=1.0 if settings.ENV == "development" else 0.1,
    )

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Physical AI & Humanoid Robotics Textbook RAG Chatbot",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add rate limiter
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Include routers
app.include_router(health.router, prefix="/v1", tags=["health"])
app.include_router(chat.router, prefix="/v1", tags=["chat"])
app.include_router(auth.router, prefix="/v1/auth", tags=["auth"])
app.include_router(preferences.router, prefix="/v1/auth", tags=["preferences"])
app.include_router(admin.router, prefix="/v1/admin", tags=["admin"])


@app.on_event("startup")
async def startup_event():
    print(f"Starting RAG Chatbot API - Environment: {settings.ENV}")
    print(f"API Documentation: http://{settings.API_HOST}:{settings.API_PORT}/docs")


@app.on_event("shutdown")
async def shutdown_event():
    print("Shutting down RAG Chatbot API")
