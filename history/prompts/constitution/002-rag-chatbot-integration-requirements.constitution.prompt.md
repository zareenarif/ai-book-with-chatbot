---
id: 002
title: RAG Chatbot Integration Requirements
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: DC
command: /sp.constitution update
labels: ["constitution", "rag-chatbot", "architecture", "fastapi", "qdrant", "neon-db", "betterauth", "agents"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.consitution update You are my coding assistant inside this Docusaurus book project.

Goal: Build a fully integrated RAG Chatbot.

Requirements:
- Use FastAPI backend.
- Use OpenAI Agents / ChatKit SDK.
- Qdrant Cloud Free Tier for vector embeddings.
- Neon Serverless Postgres for user accounts and chat history.
- BetterAuth for signup/signin.
- Claude Code Subagents + Agent Skills for reusable intelligence.
- Chatbot must answer only from the book content.
- Highlighted text on a page must be answerable by chatbot.
- Bonus: personalization per chapter and Urdu translation buttons.

Subagents:
- RAG-Agent: embedding, search, retrieval
- UI-Agent: frontend + Docusaurus integration
- Auth-Agent: BetterAuth + Neon DB
- Skill-Agent: reusable intelligence

Rules:
- Clean folder structure
- Executable FastAPI apps
- ROS 2 Humble compatible examples
- Follow Specify → Plan → Tasks → Implement

## Response snapshot

Updated constitution to version 1.1.0 with comprehensive RAG Chatbot integration requirements:

**Added Project Context Section:**
- FastAPI backend architecture
- OpenAI Agents/ChatKit SDK for conversational AI
- Qdrant Cloud Free Tier for vector embeddings and semantic search
- Neon Serverless Postgres for user management and chat history
- BetterAuth for authentication flows
- Claude Code Subagents architecture with 4 specialized agents

**Agent Architecture Defined:**
- RAG-Agent: Embedding generation, semantic search, retrieval from Qdrant
- UI-Agent: Frontend integration with Docusaurus, chatbot UI components
- Auth-Agent: BetterAuth integration, Neon DB operations, user management
- Skill-Agent: Reusable intelligence modules, shared capabilities

**Development Standards:**
- Clean, modular folder structure mandate
- Fully executable FastAPI applications with error handling
- ROS 2 Humble code compatibility requirement
- Specify → Plan → Tasks → Implement workflow enforcement
- Ground-truth requirement: chatbot responses MUST be from book content only

**Key Constraints:**
- No hallucinations: all responses grounded in book content
- Highlighted text on pages must be queryable by chatbot
- Bonus features: per-chapter personalization, Urdu translation

## Outcome

- ✅ Impact: Constitution updated from 1.0.0 to 1.1.0 with RAG Chatbot architecture requirements
- 🧪 Tests: Manual review of constitution structure and semantic completeness
- 📁 Files: Modified .specify/memory/constitution.md
- 🔁 Next prompts: Create feature spec for RAG Chatbot (`/sp.specify`), then architectural plan (`/sp.plan`), then tasks (`/sp.tasks`)
- 🧠 Reflection: Constitution now provides clear guidance for AI agent architecture, tech stack choices (FastAPI, Qdrant, Neon, BetterAuth), and chatbot grounding requirements

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution update successful
- Graders run and results (PASS/FAIL): Constitution structure validated (PASS), version bump verified (PASS)
- Prompt variant (if applicable): Standard constitution update with explicit agent architecture
- Next experiment (smallest change to try): Begin feature specification phase with `/sp.specify` to detail RAG Chatbot implementation requirements
