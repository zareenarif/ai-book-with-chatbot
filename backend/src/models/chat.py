from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class ChatSource(BaseModel):
    chapter: str
    section: str
    url: str
    score: float


class ChatMessageRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=5000)
    session_id: str
    highlighted_text: Optional[str] = None
    preferred_chapter: Optional[str] = None


class ChatMessageResponse(BaseModel):
    message: str
    role: str = "assistant"
    sources: List[ChatSource]
    timestamp: str

    class Config:
        json_schema_extra = {
            "example": {
                "message": "A ROS 2 node is a participant in the ROS graph...",
                "role": "assistant",
                "sources": [
                    {
                        "chapter": "Chapter 3",
                        "section": "ROS 2 Architecture",
                        "url": "/docs/chapter-3/ros2-architecture",
                        "score": 0.89
                    }
                ],
                "timestamp": "2025-12-10T10:00:02Z"
            }
        }
