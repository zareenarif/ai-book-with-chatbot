"""
Index book content from Markdown files
Parses all Markdown files in docs/ directory
Extracts H2/H3 sections and generates embeddings
"""
import sys
import os
import re
from pathlib import Path
import asyncio
import httpx

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import settings


def parse_markdown_file(file_path: Path) -> list:
    """
    Parse Markdown file and extract sections
    Returns list of content chunks
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    chunks = []
    current_chapter = None
    current_section = None
    current_content = []

    lines = content.split('\n')

    for line in lines:
        # Match H1 headers (# Chapter X)
        if line.startswith('# '):
            chapter_match = re.search(r'Chapter\s+(\d+)', line, re.IGNORECASE)
            if chapter_match:
                current_chapter = f"Chapter {chapter_match.group(1)}"

        # Match H2 headers (## Section Name)
        elif line.startswith('## '):
            # Save previous section if exists
            if current_section and current_content:
                chunks.append({
                    "chapter": current_chapter or "Introduction",
                    "section": current_section,
                    "content": '\n'.join(current_content).strip(),
                    "url": f"/docs/{file_path.stem}"
                })

            current_section = line.replace('##', '').strip()
            current_content = []

        # Accumulate content for current section
        elif current_section:
            current_content.append(line)

    # Save last section
    if current_section and current_content:
        chunks.append({
            "chapter": current_chapter or "Introduction",
            "section": current_section,
            "content": '\n'.join(current_content).strip(),
            "url": f"/docs/{file_path.stem}"
        })

    return chunks


async def index_content():
    """Index all Markdown files"""
    print("Scanning for Markdown files in docs/ directory...")

    # Find all .md files in docs directory
    docs_dir = Path(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    docs_path = docs_dir / 'docs'

    if not docs_path.exists():
        print(f"Error: docs directory not found at {docs_path}")
        return

    md_files = list(docs_path.rglob('*.md'))
    print(f"Found {len(md_files)} Markdown files")

    all_chunks = []

    for md_file in md_files:
        print(f"Processing: {md_file.name}")
        chunks = parse_markdown_file(md_file)
        all_chunks.extend(chunks)

    print(f"\nTotal chunks extracted: {len(all_chunks)}")

    # Send to backend API for embedding
    print("\nSending chunks to backend for embedding...")

    async with httpx.AsyncClient(timeout=300.0) as client:
        # Process in batches of 50
        batch_size = 50
        for i in range(0, len(all_chunks), batch_size):
            batch = all_chunks[i:i + batch_size]
            print(f"Processing batch {i // batch_size + 1}/{(len(all_chunks) + batch_size - 1) // batch_size}")

            try:
                response = await client.post(
                    f"http://localhost:8000/v1/admin/embed",
                    json={"content_chunks": batch}
                )

                if response.status_code == 200:
                    result = response.json()
                    print(f"SUCCESS: {result['message']}")
                else:
                    print(f"ERROR: {response.text}")

            except Exception as e:
                print(f"ERROR processing batch: {str(e)}")

    print("\nIndexing complete!")


def main():
    print("=" * 60)
    print("RAG Chatbot - Book Content Indexing")
    print("=" * 60)
    asyncio.run(index_content())


if __name__ == "__main__":
    main()
