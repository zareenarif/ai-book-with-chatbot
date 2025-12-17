# How to Run RAG Chatbot - Quick Guide

## EASIEST WAY: One-Click Start 🚀

### Windows Users:

**Option 1 - Batch File (Recommended):**
1. Navigate to `C:\Users\DC\Desktop\hackathon-claude\`
2. Double-click `start-all.bat`
3. Two windows will open automatically

**Option 2 - PowerShell:**
```powershell
cd C:\Users\DC\Desktop\hackathon-claude
.\start-all.ps1
```

## Manual Method: Two Separate Terminals

### Terminal 1 - Backend API:
```bash
cd C:\Users\DC\Desktop\hackathon-claude\backend
.\venv\Scripts\uvicorn src.main:app --reload --host 0.0.0.0
```
Wait until you see: "Application startup complete"

### Terminal 2 - Frontend:
```bash
cd C:\Users\DC\Desktop\hackathon-claude\textbook-platform
npm start
```
Browser will open automatically at http://localhost:3000

## Access Points

- **Frontend (Textbook + Chatbot)**: http://localhost:3000
- **Backend API**: http://localhost:8000
- **API Documentation**: http://localhost:8000/docs

## Finding the Chatbot

1. Open http://localhost:3000 in your browser
2. Look for a **circular chat button (💬)** in the **bottom-right corner**
3. Click it to open the chat window

## Testing Questions

Try asking:
- "What is Physical AI?"
- "How do humanoid robots maintain balance?"
- "Explain reinforcement learning"

## Troubleshooting

### Chatbot button not visible?
1. Hard refresh: `Ctrl + Shift + R` (Windows) or `Cmd + Shift + R` (Mac)
2. Check browser console (F12) for errors
3. Verify both servers are running

### Port already in use?
The scripts will show errors if ports 3000 or 8000 are occupied.
- Kill other processes using those ports
- Or change ports in the commands

### Backend not starting?
Make sure virtual environment and dependencies are installed:
```bash
cd backend
.\venv\Scripts\pip install -r requirements.txt
```

## Stopping the Servers

- **Batch file windows**: Press `Ctrl + C` in each window
- **Manual terminals**: Press `Ctrl + C` in each terminal

---

For detailed documentation, see:
- `CHATBOT_INTEGRATION.md` - Integration details
- `SETUP_COMPLETE.md` - System overview
- `textbook-platform/CHATBOT_INTEGRATION.md` - Troubleshooting guide
