import React, { useState, useRef, useEffect } from 'react';

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const sessionId = useRef(`session-${Date.now()}`);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    setInput('');

    // Add user message
    setMessages(prev => [...prev, {
      role: 'user',
      message: userMessage,
      timestamp: new Date().toISOString()
    }]);

    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage,
          session_id: sessionId.current,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant message
      setMessages(prev => [...prev, {
        role: data.role,
        message: data.message,
        sources: data.sources,
        timestamp: data.timestamp
      }]);
    } catch (error) {
      console.error('Error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        message: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString(),
        error: true
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const buttonStyle = {
    position: 'fixed',
    bottom: '24px',
    right: '24px',
    width: '60px',
    height: '60px',
    borderRadius: '50%',
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    border: 'none',
    fontSize: '28px',
    cursor: 'pointer',
    boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
    transition: 'all 0.3s ease',
    zIndex: 1000,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    color: 'white'
  };

  const windowStyle = {
    position: 'fixed',
    bottom: '100px',
    right: '24px',
    width: '380px',
    height: '600px',
    background: 'white',
    borderRadius: '16px',
    boxShadow: '0 10px 40px rgba(0, 0, 0, 0.2)',
    display: 'flex',
    flexDirection: 'column',
    zIndex: 1000,
    overflow: 'hidden'
  };

  const headerStyle = {
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    color: 'white',
    padding: '20px',
    textAlign: 'center'
  };

  const messagesStyle = {
    flex: 1,
    overflowY: 'auto',
    padding: '20px',
    background: '#f7f9fc'
  };

  const formStyle = {
    display: 'flex',
    padding: '16px',
    background: 'white',
    borderTop: '1px solid #e2e8f0',
    gap: '8px'
  };

  const inputStyle = {
    flex: 1,
    padding: '12px 16px',
    border: '2px solid #e2e8f0',
    borderRadius: '24px',
    fontSize: '14px',
    outline: 'none'
  };

  const sendButtonStyle = {
    width: '44px',
    height: '44px',
    borderRadius: '50%',
    border: 'none',
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    color: 'white',
    fontSize: '18px',
    cursor: 'pointer',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center'
  };

  const userMessageStyle = {
    display: 'flex',
    justifyContent: 'flex-end',
    marginBottom: '16px'
  };

  const assistantMessageStyle = {
    display: 'flex',
    justifyContent: 'flex-start',
    marginBottom: '16px'
  };

  const userBubbleStyle = {
    maxWidth: '80%',
    padding: '12px 16px',
    borderRadius: '16px',
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    color: 'white',
    fontSize: '14px',
    lineHeight: '1.5'
  };

  const assistantBubbleStyle = {
    maxWidth: '80%',
    padding: '12px 16px',
    borderRadius: '16px',
    background: 'white',
    color: '#1e293b',
    fontSize: '14px',
    lineHeight: '1.5',
    boxShadow: '0 2px 8px rgba(0, 0, 0, 0.05)'
  };

  return (
    <>
      {/* Chat Button */}
      <button
        style={buttonStyle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div style={windowStyle}>
          <div style={headerStyle}>
            <h3 style={{ margin: '0 0 8px 0', fontSize: '20px', fontWeight: 600 }}>AI Assistant</h3>
            <p style={{ margin: 0, fontSize: '14px', opacity: 0.9 }}>Ask me anything about the textbook!</p>
          </div>

          <div style={messagesStyle}>
            {messages.length === 0 && (
              <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', height: '100%', textAlign: 'center', color: '#64748b' }}>
                <p>👋 Hello! I'm your AI assistant.</p>
                <p>Ask me anything about Physical AI!</p>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div
                key={idx}
                style={msg.role === 'user' ? userMessageStyle : assistantMessageStyle}
              >
                <div style={msg.role === 'user' ? userBubbleStyle : assistantBubbleStyle}>
                  {msg.message}
                  {msg.sources && msg.sources.length > 0 && (
                    <div style={{ marginTop: '8px', padding: '12px', background: '#f1f5f9', borderRadius: '8px', fontSize: '12px' }}>
                      <strong style={{ display: 'block', marginBottom: '6px', color: '#475569' }}>Sources:</strong>
                      {msg.sources.map((source, sidx) => (
                        <div key={sidx} style={{ padding: '4px 0', color: '#64748b' }}>
                          📚 {source.chapter} - {source.section} (Score: {source.score})
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div style={assistantMessageStyle}>
                <div style={assistantBubbleStyle}>
                  <div style={{ display: 'flex', gap: '4px', padding: '4px 0' }}>
                    <span style={{ width: '8px', height: '8px', background: '#667eea', borderRadius: '50%', animation: 'bounce 1.4s infinite' }}>•</span>
                    <span style={{ width: '8px', height: '8px', background: '#667eea', borderRadius: '50%', animation: 'bounce 1.4s infinite 0.2s' }}>•</span>
                    <span style={{ width: '8px', height: '8px', background: '#667eea', borderRadius: '50%', animation: 'bounce 1.4s infinite 0.4s' }}>•</span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={sendMessage} style={formStyle}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question..."
              style={inputStyle}
              disabled={isLoading}
            />
            <button
              type="submit"
              style={sendButtonStyle}
              disabled={isLoading || !input.trim()}
            >
              ➤
            </button>
          </form>
        </div>
      )}
    </>
  );
}
