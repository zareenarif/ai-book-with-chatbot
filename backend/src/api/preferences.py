from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession

from src.db.session import get_db
from src.db.models import User
from src.models.user import UserPreferences, UserResponse
from src.services.auth_service import auth_service
from src.middleware.jwt_middleware import get_current_user

router = APIRouter()


@router.put("/preferences", response_model=UserResponse)
async def update_preferences(
    preferences: UserPreferences,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
) -> UserResponse:
    """
    Update user preferences (language, preferred chapter)
    Protected endpoint - requires JWT authentication
    """
    try:
        user = await auth_service.update_preferences(
            db,
            current_user.id,
            preferences
        )
        return UserResponse.from_orm(user)
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
