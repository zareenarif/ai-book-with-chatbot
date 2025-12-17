"""Test Gemini API directly"""
import google.generativeai as genai
from src.config import settings

# Configure Gemini
genai.configure(api_key=settings.GEMINI_API_KEY)
model = genai.GenerativeModel(settings.CHAT_MODEL)

# Test simple prompt
prompt = "What is Physical AI? Answer in 1-2 sentences."

print("Sending prompt to Gemini...")
print(f"Model: {settings.CHAT_MODEL}")

try:
    response = model.generate_content(
        prompt,
        generation_config=genai.types.GenerationConfig(
            temperature=0.3,
            max_output_tokens=500,
        )
    )

    print("\n=== Response Object ===")
    print(f"Response type: {type(response)}")
    print(f"Has candidates: {hasattr(response, 'candidates')}")

    if hasattr(response, 'candidates'):
        print(f"Number of candidates: {len(response.candidates)}")
        if response.candidates:
            candidate = response.candidates[0]
            print(f"\nFirst candidate: {candidate}")
            print(f"Finish reason: {candidate.finish_reason}")
            print(f"Has content: {hasattr(candidate, 'content')}")
            if hasattr(candidate, 'content'):
                print(f"Content: {candidate.content}")
                print(f"Content type: {type(candidate.content)}")
                print(f"Has parts: {hasattr(candidate.content, 'parts')}")
                if hasattr(candidate.content, 'parts'):
                    print(f"Number of parts: {len(candidate.content.parts)}")
                    if candidate.content.parts:
                        print(f"First part: {candidate.content.parts[0]}")
                        print(f"First part text: {candidate.content.parts[0].text}")

    print("\n=== Direct .text access ===")
    try:
        print(f"response.text: {response.text}")
    except Exception as e:
        print(f"Error accessing response.text: {type(e).__name__}: {e}")

except Exception as e:
    print(f"\n=== ERROR ===")
    print(f"{type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()
