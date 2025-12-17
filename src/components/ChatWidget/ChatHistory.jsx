import React, { useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import LoadingIndicator from './LoadingIndicator';

const ChatHistory = ({ messages }) => {
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  return (
    <div className="chat-history">
      {messages.length === 0 ? (
        <div className="chat-empty">
          <p>Ask a question about the textbook!</p>
        </div>
      ) : (
        messages.map((msg, index) => (
          <ChatMessage key={index} message={msg} />
        ))
      )}
      <div ref={messagesEndRef} />
    </div>
  );
};

export default ChatHistory;
