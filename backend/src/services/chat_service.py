import google.generativeai as genai
from typing import List, Dict
from src.config import settings
from src.utils.system_prompts import create_grounded_prompt


class ChatService:
    """Service for chat completion using Google Gemini (FREE tier available!)"""

    def __init__(self):
        # Configure Gemini with API key
        genai.configure(api_key=settings.GEMINI_API_KEY)
        self.model = genai.GenerativeModel(settings.CHAT_MODEL)

    def generate_answer(
        self,
        user_question: str,
        retrieved_chunks: List[Dict]
    ) -> str:
        """Generate grounded answer using Google Gemini"""
        # Create grounded prompt
        system_prompt = create_grounded_prompt(user_question, retrieved_chunks)

        # Combine system prompt and user question for Gemini
        full_prompt = f"{system_prompt}\n\nUser Question: {user_question}"

        # Generate response using Gemini
        response = self.model.generate_content(
            full_prompt,
            generation_config=genai.types.GenerationConfig(
                temperature=0.3,  # Low temperature for factual answers
                max_output_tokens=500,
            )
        )

        # Handle Gemini response structure
        if not response.candidates:
            print("WARNING: No candidates in Gemini response")
            # Fallback answer when no candidates
            return "I don't have enough information in the book to answer this question accurately. Please try rephrasing or asking about a different topic."

        candidate = response.candidates[0]
        print(f"Gemini finish_reason: {candidate.finish_reason} (type: {type(candidate.finish_reason)})")

        # Handle finish reasons - check for successful completion
        # FinishReason enum values: FINISH_REASON_UNSPECIFIED=0, STOP=1, MAX_TOKENS=2, SAFETY=3, RECITATION=4, OTHER=5
        finish_reason_value = int(candidate.finish_reason) if hasattr(candidate.finish_reason, 'value') else candidate.finish_reason

        if finish_reason_value not in [0, 1]:  # 0=UNSPECIFIED, 1=STOP are OK
            print(f"WARNING: Gemini returned finish_reason={finish_reason_value}, using fallback")
            # Return a safe fallback for blocked/incomplete responses
            return "I don't have enough information in the book to answer this question accurately. Please try rephrasing or asking about a different topic."

        if not hasattr(candidate, 'content') or not candidate.content.parts:
            print("WARNING: No content/parts in Gemini response")
            return "I don't have enough information in the book to answer this question accurately. Please try rephrasing or asking about a different topic."

        answer_text = candidate.content.parts[0].text
        print(f"Generated answer length: {len(answer_text)} chars")
        return answer_text

    def validate_answer_grounding(
        self,
        answer: str,
        retrieved_chunks: List[Dict]
    ) -> bool:
        """
        Validate that the answer is grounded in retrieved content
        Skill-Agent logic for answer validation
        """
        # Simple validation: Check if answer mentions at least one chapter
        chapter_mentions = [
            chunk['payload']['chapter']
            for chunk in retrieved_chunks
        ]

        # Check if any chapter is mentioned in the answer
        for chapter in chapter_mentions:
            if chapter.lower() in answer.lower():
                return True

        # Additional check: Look for common hedging phrases that indicate lack of information
        hedging_phrases = [
            "i don't have enough information",
            "the content doesn't contain",
            "not mentioned in the book",
            "cannot find information"
        ]

        for phrase in hedging_phrases:
            if phrase in answer.lower():
                return True  # This is actually a valid grounded response

        return False


# Singleton instance
chat_service = ChatService()
