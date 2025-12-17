import React, { useState, useRef, useEffect } from 'react';

// ChatWidget component embedded directly
function ChatWidget() {
  const [isClient, setIsClient] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const sessionId = useRef(`session-${Date.now()}`);

  // Ensure component only renders on client side
  useEffect(() => {
    setIsClient(true);
  }, []);

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

    setMessages(prev => [...prev, {
      role: 'user',
      message: userMessage,
      timestamp: new Date().toISOString()
    }]);

    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/v1/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: userMessage,
          session_id: sessionId.current,
        }),
      });

      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);

      const data = await response.json();

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

  // Don't render until client-side
  if (!isClient) {
    return null;
  }

  const styles = {
    button: {
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
      zIndex: 9999,
      color: 'white',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center'
    },
    window: {
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
      zIndex: 9999,
      overflow: 'hidden'
    },
    header: {
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      color: 'white',
      padding: '20px',
      textAlign: 'center'
    },
    messages: {
      flex: 1,
      overflowY: 'auto',
      padding: '20px',
      background: '#f7f9fc'
    },
    form: {
      display: 'flex',
      padding: '16px',
      background: 'white',
      borderTop: '1px solid #e2e8f0',
      gap: '8px'
    },
    input: {
      flex: 1,
      padding: '12px 16px',
      border: '2px solid #e2e8f0',
      borderRadius: '24px',
      fontSize: '14px',
      outline: 'none'
    },
    sendButton: {
      width: '44px',
      height: '44px',
      borderRadius: '50%',
      border: 'none',
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      color: 'white',
      fontSize: '18px',
      cursor: 'pointer'
    },
    userMessage: {
      display: 'flex',
      justifyContent: 'flex-end',
      marginBottom: '16px'
    },
    assistantMessage: {
      display: 'flex',
      justifyContent: 'flex-start',
      marginBottom: '16px'
    },
    userBubble: {
      maxWidth: '80%',
      padding: '12px 16px',
      borderRadius: '16px',
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      color: 'white',
      fontSize: '14px'
    },
    assistantBubble: {
      maxWidth: '80%',
      padding: '12px 16px',
      borderRadius: '16px',
      background: 'white',
      color: '#1e293b',
      fontSize: '14px',
      boxShadow: '0 2px 8px rgba(0, 0, 0, 0.05)'
    }
  };

  return (
    <>
      <button style={styles.button} onClick={() => setIsOpen(!isOpen)} aria-label="Chat">
        {isOpen ? '✕' : '💬'}
      </button>

      {isOpen && (
        <div style={styles.window}>
          <div style={styles.header}>
            <h3 style={{ margin: '0 0 8px 0', fontSize: '20px' }}>AI Assistant</h3>
            <p style={{ margin: 0, fontSize: '14px' }}>Ask me anything about the textbook!</p>
          </div>

          <div style={styles.messages}>
            {messages.length === 0 && (
              <div style={{ textAlign: 'center', padding: '40px', color: '#64748b' }}>
                <p>👋 Hello! I'm your AI assistant.</p>
                <p>Ask me anything about Physical AI!</p>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} style={msg.role === 'user' ? styles.userMessage : styles.assistantMessage}>
                <div style={msg.role === 'user' ? styles.userBubble : styles.assistantBubble}>
                  {msg.message}
                </div>
              </div>
            ))}

            {isLoading && (
              <div style={styles.assistantMessage}>
                <div style={styles.assistantBubble}>Loading...</div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={sendMessage} style={styles.form}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question..."
              style={styles.input}
              disabled={isLoading}
            />
            <button type="submit" style={styles.sendButton} disabled={isLoading || !input.trim()}>
              ➤
            </button>
          </form>
        </div>
      )}
    </>
  );
}

// Root component that wraps Docusaurus
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
