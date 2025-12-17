# Start RAG Chatbot - Backend and Frontend

Write-Host "Starting RAG Chatbot System..." -ForegroundColor Green

# Start Backend
Write-Host "`nStarting Backend API on http://localhost:8000..." -ForegroundColor Yellow
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd C:\Users\DC\Desktop\hackathon-claude\backend; .\venv\Scripts\uvicorn src.main:app --reload --host 0.0.0.0"

# Wait a bit for backend to start
Start-Sleep -Seconds 3

# Start Frontend
Write-Host "Starting Frontend on http://localhost:3000..." -ForegroundColor Yellow
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd C:\Users\DC\Desktop\hackathon-claude\textbook-platform; npm start"

Write-Host "`n✅ Both servers are starting!" -ForegroundColor Green
Write-Host "`nBackend API: http://localhost:8000" -ForegroundColor Cyan
Write-Host "Frontend: http://localhost:3000 (will open automatically)" -ForegroundColor Cyan
Write-Host "`nPress Ctrl+C in each window to stop the servers" -ForegroundColor Gray
