"""Quick script to insert test content into Qdrant"""
from src.services.embedding_service import embedding_service
from src.services.qdrant_client import qdrant_service
from src.config import settings

# Test content about Physical AI
test_content = """Physical AI refers to artificial intelligence systems that interact with and manipulate the physical world. This includes robotics, autonomous vehicles, and embodied AI agents that can perceive their environment through sensors and act upon it through actuators. Physical AI represents a significant evolution from purely digital AI systems, as it must deal with real-world constraints like physics, safety, and uncertainty."""

# Generate embedding
print("Generating embedding...")
vector = embedding_service.embed_text(test_content)
print(f"Embedding generated: {len(vector)} dimensions")

# Insert into Qdrant
print("Inserting into Qdrant...")
from qdrant_client.models import PointStruct
import uuid

point = PointStruct(
    id=str(uuid.uuid4()),
    vector=vector,
    payload={
        "text": test_content,
        "chapter": "Introduction",
        "section": "What is Physical AI?",
        "url": "/docs/intro"
    }
)

qdrant_service.client.upsert(
    collection_name=settings.QDRANT_COLLECTION_NAME,
    points=[point]
)

print("Content inserted successfully!")
print(f"Collection: {settings.QDRANT_COLLECTION_NAME}")
