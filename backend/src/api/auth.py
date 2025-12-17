from fastapi import APIRouter, HTTPException, Depends, status, Request
from slowapi import Limiter
from slowapi.util import get_remote_address
from sqlalchemy.ext.asyncio import AsyncSession

from src.db.session import get_db
from src.models.user import (
    UserCreate, UserLogin, UserResponse, AuthResponse,
    PasswordResetRequest, PasswordReset
)
from src.services.auth_service import auth_service
from src.utils.jwt import create_access_token
from src.config import settings

router = APIRouter()
limiter = Limiter(key_func=get_remote_address)


@router.post("/signup", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
@limiter.limit(settings.RATE_LIMIT_AUTH)
async def signup(
    request: Request,
    user_data: UserCreate,
    db: AsyncSession = Depends(get_db)
) -> UserResponse:
    """
    Create new user account
    Sends email verification link
    """
    try:
        user = await auth_service.create_user(db, user_data)
        return UserResponse.from_orm(user)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/signin", response_model=AuthResponse)
@limiter.limit(settings.RATE_LIMIT_AUTH)
async def signin(
    request: Request,
    login_data: UserLogin,
    db: AsyncSession = Depends(get_db)
) -> AuthResponse:
    """
    Authenticate user and return JWT token
    """
    user = await auth_service.authenticate_user(db, login_data)

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Create access token
    access_token = create_access_token(data={"sub": str(user.id)})

    return AuthResponse(
        access_token=access_token,
        user=UserResponse.from_orm(user)
    )


@router.get("/verify")
async def verify_email(
    token: str,
    db: AsyncSession = Depends(get_db)
):
    """
    Verify email with token from email link
    """
    success = await auth_service.verify_email_token(db, token)

    if not success:
        raise HTTPException(
            status_code=400,
            detail="Invalid or expired verification token"
        )

    return {"message": "Email verified successfully"}


@router.post("/reset-password-request")
@limiter.limit(settings.RATE_LIMIT_AUTH)
async def request_password_reset(
    request: Request,
    reset_request: PasswordResetRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Request password reset
    Sends email with reset link
    """
    await auth_service.request_password_reset(db, reset_request.email)

    return {"message": "If the email exists, a password reset link has been sent"}


@router.post("/reset-password")
@limiter.limit(settings.RATE_LIMIT_AUTH)
async def reset_password(
    request: Request,
    reset_data: PasswordReset,
    db: AsyncSession = Depends(get_db)
):
    """
    Reset password with token from email
    """
    success = await auth_service.reset_password(
        db,
        reset_data.token,
        reset_data.new_password
    )

    if not success:
        raise HTTPException(
            status_code=400,
            detail="Invalid or expired reset token"
        )

    return {"message": "Password reset successfully"}
