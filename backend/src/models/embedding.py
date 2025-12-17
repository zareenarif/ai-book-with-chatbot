from pydantic import BaseModel, Field
from typing import List


class ContentChunk(BaseModel):
    chapter: str
    section: str
    content: str
    url: str


class EmbedRequest(BaseModel):
    content_chunks: List[ContentChunk] = Field(..., min_items=1, max_items=100)


class EmbedResponse(BaseModel):
    success: bool
    chunks_processed: int
    message: str
