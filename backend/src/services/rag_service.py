from typing import List, Dict, Optional
from src.services.embedding_service import embedding_service
from src.services.qdrant_client import qdrant_service
from src.config import settings


class RAGService:
    """Service for Retrieval-Augmented Generation"""

    def __init__(self):
        self.collection_name = settings.QDRANT_COLLECTION_NAME

    def search(
        self,
        query: str,
        limit: int = 5,
        score_threshold: float = 0.5,
        preferred_chapter: Optional[str] = None,
        highlighted_text: Optional[str] = None
    ) -> List[Dict]:
        """
        Semantic search with optional personalization and highlighted text boosting
        """
        # Generate query embedding
        query_vector = embedding_service.embed_text(query)

        # Search Qdrant
        results = qdrant_service.search_vectors(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit * 2,  # Get more results for boosting
            score_threshold=score_threshold
        )

        # Apply chapter boost if personalization is enabled
        if preferred_chapter:
            results = self._apply_chapter_boost(results, preferred_chapter)

        # Apply phrase match boost if highlighted text is provided
        if highlighted_text:
            results = self._apply_phrase_match_boost(results, highlighted_text)

        # Re-sort by score and return top results
        results.sort(key=lambda x: x['score'], reverse=True)
        return results[:limit]

    def retrieve_top_k(
        self,
        query: str,
        k: int = 5,
        preferred_chapter: Optional[str] = None
    ) -> List[Dict]:
        """Retrieve top-k most relevant chunks"""
        return self.search(
            query=query,
            limit=k,
            preferred_chapter=preferred_chapter
        )

    def _apply_chapter_boost(
        self,
        results: List[Dict],
        preferred_chapter: str
    ) -> List[Dict]:
        """Boost scores for chunks from preferred chapter by 2.0x"""
        for result in results:
            chapter = result.get('payload', {}).get('chapter', '')
            if chapter == preferred_chapter:
                result['score'] *= 2.0
        return results

    def _apply_phrase_match_boost(
        self,
        results: List[Dict],
        highlighted_text: str
    ) -> List[Dict]:
        """Boost scores by 1.5x for chunks containing exact highlighted phrase"""
        highlighted_lower = highlighted_text.lower()
        for result in results:
            content = result.get('payload', {}).get('content', '').lower()
            if highlighted_lower in content:
                result['score'] *= 1.5
        return results


# Singleton instance
rag_service = RAGService()
