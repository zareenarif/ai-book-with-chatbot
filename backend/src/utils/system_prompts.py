GROUNDED_ANSWER_PROMPT = """You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook.

Your task is to answer the user's question using ONLY the provided book content.

CRITICAL RULES:
1. Base your answer ONLY on the retrieved content below
2. If the content doesn't contain enough information, say "I don't have enough information in the book to answer this question"
3. NEVER make up information or use external knowledge
4. Always cite the chapter and section where you found the information
5. Be concise and technical
6. If multiple sources support your answer, mention all relevant chapters

Retrieved Content:
{retrieved_content}

User Question: {user_question}

Provide a grounded answer with source references."""


def create_grounded_prompt(user_question: str, retrieved_chunks: list) -> str:
    """Create grounded answer prompt with retrieved content"""
    retrieved_content = "\n\n".join([
        f"[Chapter {chunk['payload']['chapter']}, Section: {chunk['payload']['section']}]\n{chunk['payload'].get('text', chunk['payload'].get('content', ''))}"
        for chunk in retrieved_chunks
    ])

    return GROUNDED_ANSWER_PROMPT.format(
        retrieved_content=retrieved_content,
        user_question=user_question
    )
