"""Debug script to see actual chunk structure"""
from src.services.rag_service import rag_service

# Search
results = rag_service.search("What is Physical AI?", limit=5)

print(f"Found {len(results)} chunks")
print("\n=== First chunk structure ===")
if results:
    import json
    print(json.dumps(results[0], indent=2))
