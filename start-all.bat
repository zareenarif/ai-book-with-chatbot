@echo off
echo ========================================
echo Starting RAG Chatbot System
echo ========================================
echo.

echo Starting Backend API...
start "Backend API" cmd /k "cd /d C:\Users\DC\Desktop\hackathon-claude\backend && .\venv\Scripts\uvicorn src.main:app --reload --host 0.0.0.0"

timeout /t 3 /nobreak >nul

echo Starting Frontend...
start "Frontend" cmd /k "cd /d C:\Users\DC\Desktop\hackathon-claude\textbook-platform && npm start"

echo.
echo ========================================
echo Both servers are starting!
echo ========================================
echo Backend API: http://localhost:8000
echo Frontend: http://localhost:3000
echo.
echo Two windows will open - one for each server
echo Close the windows or press Ctrl+C to stop
echo ========================================
