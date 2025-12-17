"""
Initialize Qdrant collection for book content embeddings
Run this script once to create the collection
"""
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.services.qdrant_client import qdrant_service
from src.config import settings


def main():
    print(f"Creating Qdrant collection: {settings.QDRANT_COLLECTION_NAME}")
    print(f"Collection URL: {settings.QDRANT_URL}")
    print(f"Using FREE HuggingFace embeddings: {settings.EMBEDDING_MODEL}")

    # Create collection with 384-dim vectors (HuggingFace all-MiniLM-L6-v2)
    qdrant_service.create_collection(
        collection_name=settings.QDRANT_COLLECTION_NAME,
        vector_size=384
    )

    print("✅ Collection created successfully!")


if __name__ == "__main__":
    main()
