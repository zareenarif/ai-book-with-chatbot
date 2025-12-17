<!--
Sync Impact Report:
Version change: [None] → 1.0.0
Rationale: Initial constitution for Physical AI & Humanoid Robotics Textbook project
Added sections:
  - All core principles (I-VII)
  - Content Quality Standards
  - Development Workflow
  - Governance
Templates requiring updates:
  ✅ spec-template.md (validated - compatible)
  ✅ plan-template.md (validated - compatible)
  ✅ tasks-template.md (validated - compatible)
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Project Context

This Docusaurus book project includes a fully integrated RAG Chatbot powered by AI agents.

**RAG Chatbot Requirements:**
- FastAPI backend for API endpoints
- OpenAI Agents / ChatKit SDK for conversational AI
- Qdrant Cloud Free Tier for vector embeddings and semantic search
- Neon Serverless Postgres for user accounts and chat history
- BetterAuth for authentication (signup/signin flows)
- Claude Code Subagents + Agent Skills for reusable intelligence
- Chatbot MUST answer only from book content (no hallucinations)
- Highlighted text on any page MUST be answerable by chatbot
- Bonus features: personalization per chapter, Urdu translation buttons

**Agent Architecture:**
- **RAG-Agent**: Handles embedding generation, semantic search, and retrieval from Qdrant
- **UI-Agent**: Manages frontend integration with Docusaurus and chatbot UI components
- **Auth-Agent**: Manages BetterAuth integration and Neon DB operations for user management
- **Skill-Agent**: Provides reusable intelligence modules and shared capabilities

**Development Standards:**
- Clean, modular folder structure
- Fully executable FastAPI applications with proper error handling
- ROS 2 Humble compatible code examples
- Follow Specify → Plan → Tasks → Implement workflow
- All chatbot responses MUST be grounded in book content only

## Core Principles

### I. Academic Excellence & Pedagogical Rigor

Every chapter MUST be academically sound, technically accurate, and pedagogically structured:

- Content follows progressive learning: foundational concepts → intermediate applications → advanced topics
- Each lesson includes: clear learning objectives, explanatory content, visual aids (ASCII diagrams), practical examples, exercises, summary, and glossary
- Technical accuracy is non-negotiable: all code examples must execute, all formulas must be mathematically valid, all citations must be verifiable
- Content MUST be reviewed for correctness before publication

**Rationale**: Academic textbooks serve as authoritative learning resources. Inaccurate or poorly structured content undermines student learning and damages credibility.

### II. Production-Ready Code & Deployment

All code artifacts MUST be production-ready and deployable:

- Every component must have complete, runnable code (no pseudocode or "TODO" placeholders in final deliverables)
- Docusaurus site MUST build without errors or warnings
- Custom React components MUST be fully implemented with Tailwind CSS styling
- GitHub Pages deployment MUST be automated and documented
- All dependencies MUST be specified with exact versions

**Rationale**: Students and educators expect working examples they can run immediately. Incomplete code wastes time and creates frustration.

### III. Modular Architecture & Component Reusability

Technical architecture MUST follow modularity and separation of concerns:

- Content (Markdown) completely separated from presentation (React components)
- Reusable React components for common patterns (lesson layout, diagram containers, code blocks, exercise sections)
- Docusaurus configuration organized into logical modules
- Custom theme cleanly separated from default theme overrides
- Each chapter/lesson independently navigable and testable

**Rationale**: Modular architecture enables easier maintenance, content updates, and potential future extensions (e.g., interactive simulations, video integration).

### IV. Accessibility & User Experience

The textbook MUST be accessible and provide excellent UX:

- Responsive design: mobile, tablet, desktop viewports
- Semantic HTML for screen readers
- Keyboard navigation support
- Clear visual hierarchy and typography
- Fast load times (< 3s initial load, < 1s navigation)
- Search functionality across all content
- Offline capability for core content (progressive web app features)

**Rationale**: Educational resources must be accessible to all learners regardless of device, ability, or network conditions.

### V. Content Completeness & Scope Coverage

The textbook MUST comprehensively cover the defined curriculum:

- Physical AI fundamentals and theory
- Humanoid robotics design and control
- ROS 2 architecture, nodes, topics, services, actions
- Gazebo and Unity simulation environments
- NVIDIA Isaac Sim, Isaac Gym, sensor integration
- Vision-Language-Action (VLA) models and agents
- Conversational AI for human-robot interaction
- Locomotion algorithms, inverse kinematics, dynamics
- Each module mapped to specific chapters; each week mapped to specific lessons

**Rationale**: A textbook is only valuable if it delivers complete coverage of its stated scope. Gaps in content break the learning journey.

### VI. Versioning, Maintenance & Evolution

Content and code MUST follow versioning discipline:

- Semantic versioning (MAJOR.MINOR.PATCH) for content releases:
  - MAJOR: Curriculum restructure, removed chapters, backward-incompatible changes
  - MINOR: New chapters/lessons added, substantial content expansions
  - PATCH: Corrections, clarifications, minor improvements
- Changelog maintained for all content updates
- Deprecated content clearly marked with migration guidance
- Breaking changes to code examples require deprecation notices

**Rationale**: Textbooks evolve as technology and pedagogy advance. Version control enables students and educators to track changes and maintain consistency in courses.

### VII. Testing & Quality Assurance

All deliverables MUST pass quality gates before deployment:

- Markdown linting (no broken links, consistent formatting)
- Build verification (Docusaurus builds successfully)
- Visual regression testing (UI components render correctly)
- Code example validation (all code blocks execute without errors)
- Accessibility audits (WCAG 2.1 AA compliance)
- Performance budgets enforced (<3s load, <1s navigation)

**Rationale**: Quality gates prevent broken deployments and ensure consistent standards across all content.

## Content Quality Standards

### Academic Integrity

- All external sources MUST be cited (IEEE, ACM, or academic journal format)
- Original diagrams and illustrations preferred; third-party visuals require attribution
- Code examples MUST either be original or properly licensed and attributed
- Mathematical derivations MUST show work and cite source theorems

### Technical Accuracy

- All ROS 2 code examples MUST be tested with specified ROS 2 distribution (e.g., Humble, Iron)
- Simulation examples MUST include version compatibility notes (Gazebo Classic vs. Gazebo Sim, Unity version)
- Hardware specifications MUST reflect current commercially available systems
- Algorithm implementations MUST cite authoritative papers and note any modifications

### Pedagogical Structure

Each lesson MUST follow this structure:

1. **Learning Objectives**: 3-5 clear, measurable outcomes
2. **Prerequisites**: Explicitly list required prior knowledge
3. **Conceptual Overview**: Plain-language explanation with analogies
4. **Technical Deep Dive**: Detailed technical content with diagrams
5. **Practical Examples**: Runnable code with explanations
6. **Hands-On Exercises**: Progressive difficulty (beginner → advanced)
7. **Summary**: Key takeaways in bullet form
8. **Glossary**: Terms introduced in this lesson
9. **Further Reading**: 3-5 curated resources for deeper exploration

## Development Workflow

### Content Creation

1. **Research Phase**: Gather authoritative sources, validate technical accuracy
2. **Outline Phase**: Structure lesson according to pedagogical template
3. **Draft Phase**: Write content following constitution principles
4. **Review Phase**: Technical review + pedagogical review
5. **Iteration Phase**: Incorporate feedback, validate examples
6. **Approval Phase**: Final quality gate before merge

### Code Development

1. **Component Design**: Plan React component structure and props API
2. **Implementation**: Develop with TypeScript, Tailwind CSS, accessibility in mind
3. **Testing**: Unit tests for logic, visual regression for UI
4. **Integration**: Verify component works within Docusaurus
5. **Documentation**: Props API, usage examples, customization guide

### Deployment

1. **Build Validation**: `npm run build` succeeds without errors
2. **Preview Deployment**: Deploy to preview environment for review
3. **Accessibility Audit**: Run automated tools + manual keyboard navigation test
4. **Performance Check**: Lighthouse score >90 for all metrics
5. **Production Deploy**: Deploy to GitHub Pages with versioned release tag

## Governance

### Amendment Procedure

1. Proposed changes MUST be documented with rationale
2. Significant changes require approval via GitHub issue discussion
3. All amendments MUST include:
   - Version bump with justification (MAJOR/MINOR/PATCH)
   - Impact analysis on existing content and templates
   - Migration guide if backward-incompatible
4. Amendment merged only after validation that dependent artifacts remain consistent

### Versioning Policy

- Constitution follows semantic versioning
- Each version MUST update `LAST_AMENDED_DATE`
- `RATIFICATION_DATE` remains immutable after initial ratification
- Breaking changes to principles require MAJOR version bump

### Compliance Review

- All pull requests MUST verify compliance with constitution principles
- `/sp.plan` and `/sp.tasks` outputs MUST reference applicable principles
- Complexity violations (e.g., adding unnecessary abstractions, over-engineering) MUST be explicitly justified
- ADRs (Architecture Decision Records) MUST be created for architecturally significant decisions (framework choice, theme architecture, deployment strategy)

### Constitution Authority

- This constitution supersedes all other development practices
- When in conflict, constitution principles take precedence
- Deviations MUST be explicitly documented and approved
- See `CLAUDE.md` for AI assistant runtime guidance (implementation details, execution flow, PHR creation)

**Version**: 1.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-10

**Amendment History:**
- **1.1.0** (2025-12-10): Added RAG Chatbot integration requirements, agent architecture specifications, and development standards for AI-powered features
- **1.0.0** (2025-12-07): Initial constitution ratification
