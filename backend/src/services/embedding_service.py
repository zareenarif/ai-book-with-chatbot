from sentence_transformers import SentenceTransformer
from typing import List, Dict
import hashlib
from src.config import settings


class EmbeddingService:
    """Service for generating text embeddings using FREE HuggingFace models"""

    def __init__(self):
        # Load HuggingFace model (runs locally, 100% FREE!)
        # all-MiniLM-L6-v2: 384 dimensions, fast, accurate, perfect for RAG
        self.model = SentenceTransformer(settings.EMBEDDING_MODEL)
        self.dimension = 384  # all-MiniLM-L6-v2 dimension

    def embed_text(self, text: str) -> List[float]:
        """Generate embedding for a single text"""
        embedding = self.model.encode(text, convert_to_numpy=True)
        return embedding.tolist()

    def batch_embed(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts"""
        embeddings = self.model.encode(texts, convert_to_numpy=True, batch_size=32)
        return embeddings.tolist()

    @staticmethod
    def generate_content_hash(content: str) -> str:
        """Generate SHA256 hash for content deduplication"""
        return hashlib.sha256(content.encode()).hexdigest()


# Singleton instance
embedding_service = EmbeddingService()
