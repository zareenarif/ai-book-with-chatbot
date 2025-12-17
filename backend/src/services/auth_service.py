from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from datetime import datetime, timedelta
from typing import Optional
import secrets

from src.db.models import User
from src.models.user import UserCreate, UserLogin, UserPreferences
from src.utils.password import hash_password, verify_password
from src.utils.jwt import create_access_token
from src.utils.email import send_verification_email, send_password_reset_email


class AuthService:
    """Service for authentication and user management"""

    async def create_user(self, db: AsyncSession, user_data: UserCreate) -> User:
        """Create new user with email verification"""
        # Check if user already exists
        result = await db.execute(
            select(User).where(User.email == user_data.email)
        )
        existing_user = result.scalar_one_or_none()

        if existing_user:
            raise ValueError("Email already registered")

        # Create new user
        verification_token = secrets.token_urlsafe(32)
        hashed_password = hash_password(user_data.password)

        new_user = User(
            email=user_data.email,
            password_hash=hashed_password,
            verification_token=verification_token,
            email_verified=False
        )

        db.add(new_user)
        await db.commit()
        await db.refresh(new_user)

        # Send verification email
        send_verification_email(user_data.email, verification_token)

        return new_user

    async def authenticate_user(
        self,
        db: AsyncSession,
        login_data: UserLogin
    ) -> Optional[User]:
        """Authenticate user and return user object"""
        result = await db.execute(
            select(User).where(User.email == login_data.email)
        )
        user = result.scalar_one_or_none()

        if not user:
            return None

        if not verify_password(login_data.password, user.password_hash):
            return None

        return user

    async def verify_email_token(
        self,
        db: AsyncSession,
        token: str
    ) -> bool:
        """Verify email with token"""
        result = await db.execute(
            select(User).where(User.verification_token == token)
        )
        user = result.scalar_one_or_none()

        if not user:
            return False

        user.email_verified = True
        user.verification_token = None
        await db.commit()

        return True

    async def request_password_reset(
        self,
        db: AsyncSession,
        email: str
    ) -> bool:
        """Generate password reset token and send email"""
        result = await db.execute(
            select(User).where(User.email == email)
        )
        user = result.scalar_one_or_none()

        if not user:
            # Don't reveal if email exists
            return True

        reset_token = secrets.token_urlsafe(32)
        user.reset_token = reset_token
        user.reset_token_expires = datetime.utcnow() + timedelta(hours=1)

        await db.commit()

        send_password_reset_email(email, reset_token)

        return True

    async def reset_password(
        self,
        db: AsyncSession,
        token: str,
        new_password: str
    ) -> bool:
        """Reset password with valid token"""
        result = await db.execute(
            select(User).where(User.reset_token == token)
        )
        user = result.scalar_one_or_none()

        if not user:
            return False

        if user.reset_token_expires < datetime.utcnow():
            return False

        user.password_hash = hash_password(new_password)
        user.reset_token = None
        user.reset_token_expires = None

        await db.commit()

        return True

    async def update_preferences(
        self,
        db: AsyncSession,
        user_id: int,
        preferences: UserPreferences
    ) -> User:
        """Update user preferences"""
        result = await db.execute(
            select(User).where(User.id == user_id)
        )
        user = result.scalar_one_or_none()

        if not user:
            raise ValueError("User not found")

        user.language_pref = preferences.language_pref
        user.preferred_chapter = preferences.preferred_chapter

        await db.commit()
        await db.refresh(user)

        return user


# Singleton instance
auth_service = AuthService()
