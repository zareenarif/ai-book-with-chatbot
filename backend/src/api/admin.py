from fastapi import APIRouter, HTTPException
from qdrant_client.models import PointStruct
from typing import List
import uuid

from src.models.embedding import EmbedRequest, EmbedResponse
from src.services.embedding_service import embedding_service
from src.services.qdrant_client import qdrant_service
from src.config import settings

router = APIRouter()


@router.post("/embed", response_model=EmbedResponse)
async def embed_content(request: EmbedRequest) -> EmbedResponse:
    """
    Admin-only endpoint to trigger embedding generation
    Processes content chunks and upserts to Qdrant
    """
    try:
        # Extract texts for batch embedding
        texts = [chunk.content for chunk in request.content_chunks]

        # Generate embeddings
        embeddings = embedding_service.batch_embed(texts)

        # Create Qdrant points
        points: List[PointStruct] = []
        for i, (chunk, embedding) in enumerate(zip(request.content_chunks, embeddings)):
            content_hash = embedding_service.generate_content_hash(chunk.content)

            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "chapter": chunk.chapter,
                    "section": chunk.section,
                    "content": chunk.content,
                    "url": chunk.url,
                    "content_hash": content_hash
                }
            )
            points.append(point)

        # Upsert to Qdrant
        qdrant_service.upsert_vectors(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            points=points
        )

        return EmbedResponse(
            success=True,
            chunks_processed=len(request.content_chunks),
            message=f"Successfully embedded and indexed {len(request.content_chunks)} chunks"
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
