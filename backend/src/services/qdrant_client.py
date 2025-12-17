from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from typing import List, Dict, Optional
from src.config import settings


class QdrantService:
    """Singleton Qdrant client service"""

    _instance: Optional['QdrantService'] = None
    _client: Optional[QdrantClient] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._client is None:
            self._client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
            )

    @property
    def client(self) -> QdrantClient:
        return self._client

    def create_collection(self, collection_name: str, vector_size: int = 1536):
        """Create collection if it doesn't exist"""
        try:
            self._client.get_collection(collection_name)
            print(f"Collection '{collection_name}' already exists")
        except Exception:
            self._client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
            )
            print(f"Created collection '{collection_name}'")

    def upsert_vectors(
        self,
        collection_name: str,
        points: List[PointStruct]
    ):
        """Upsert vectors to collection"""
        self._client.upsert(
            collection_name=collection_name,
            points=points
        )

    def search_vectors(
        self,
        collection_name: str,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: float = 0.7,
        filter_dict: Optional[Dict] = None
    ) -> List[Dict]:
        """Search for similar vectors"""
        results = self._client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=filter_dict
        )

        return [
            {
                "id": hit.id,
                "score": hit.score,
                "payload": hit.payload
            }
            for hit in results
        ]


# Singleton instance
qdrant_service = QdrantService()
