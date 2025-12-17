from fastapi import APIRouter, HTTPException, Depends, Request
from slowapi import Limiter
from slowapi.util import get_remote_address
from datetime import datetime
from typing import List

from src.models.chat import ChatMessageRequest, ChatMessageResponse, ChatSource
from src.services.rag_service import rag_service
from src.services.chat_service import chat_service
from src.config import settings

router = APIRouter()
limiter = Limiter(key_func=get_remote_address)


@router.post("/chat", response_model=ChatMessageResponse)
@limiter.limit(settings.RATE_LIMIT_CHAT)
async def send_chat_message(
    message_data: ChatMessageRequest,
    request: Request
) -> ChatMessageResponse:
    """
    Send a chat message and receive AI-generated response
    Implements full RAG pipeline:
    1. Embed user query
    2. Search Qdrant for relevant content
    3. Generate grounded answer using OpenAI
    4. Validate answer grounding
    5. Return response with sources
    """
    try:
        # Step 1 & 2: Search for relevant content
        print(f"Searching for: {message_data.message}")
        retrieved_chunks = rag_service.search(
            query=message_data.message,
            limit=5,
            preferred_chapter=message_data.preferred_chapter,
            highlighted_text=message_data.highlighted_text
        )
        print(f"Found {len(retrieved_chunks) if retrieved_chunks else 0} chunks")

        if not retrieved_chunks:
            raise HTTPException(
                status_code=404,
                detail="No relevant content found in the book"
            )

        # Step 3: Generate grounded answer
        print("Generating answer...")
        answer = chat_service.generate_answer(
            user_question=message_data.message,
            retrieved_chunks=retrieved_chunks
        )
        print(f"Answer generated: {answer[:100]}...")

        # Step 4: Validate answer grounding
        print("Validating answer grounding...")
        is_grounded = chat_service.validate_answer_grounding(
            answer=answer,
            retrieved_chunks=retrieved_chunks
        )
        print(f"Is grounded: {is_grounded}")

        if not is_grounded:
            answer = "I don't have enough information in the book to answer this question accurately. Please try rephrasing or asking about a different topic."

        # Step 5: Format sources
        sources = [
            ChatSource(
                chapter=chunk['payload']['chapter'],
                section=chunk['payload']['section'],
                url=chunk['payload']['url'],
                score=round(chunk['score'], 2)
            )
            for chunk in retrieved_chunks[:3]  # Return top 3 sources
        ]

        return ChatMessageResponse(
            message=answer,
            role="assistant",
            sources=sources,
            timestamp=datetime.utcnow().isoformat() + "Z"
        )

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        error_detail = f"{type(e).__name__}: {str(e)}\n{traceback.format_exc()}"
        print(f"ERROR in chat endpoint: {error_detail}")
        raise HTTPException(status_code=500, detail=error_detail)


@router.post("/search")
@limiter.limit(settings.RATE_LIMIT_CHAT)
async def search_content(
    message_data: ChatMessageRequest,
    request: Request
) -> List[ChatSource]:
    """
    Standalone semantic search endpoint
    Returns top 5 results without generating answer
    """
    try:
        results = rag_service.search(
            query=message_data.message,
            limit=5,
            preferred_chapter=message_data.preferred_chapter
        )

        return [
            ChatSource(
                chapter=chunk['payload']['chapter'],
                section=chunk['payload']['section'],
                url=chunk['payload']['url'],
                score=round(chunk['score'], 2)
            )
            for chunk in results
        ]

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
