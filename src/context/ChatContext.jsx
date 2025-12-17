import React, { createContext, useState, useContext } from 'react';

const ChatContext = createContext();

export const useChat = () => {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChat must be used within ChatProvider');
  }
  return context;
};

export const ChatProvider = ({ children }) => {
  const [messages, setMessages] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const addMessage = (message) => {
    setMessages((prev) => [...prev, message]);
  };

  const clearMessages = () => {
    setMessages([]);
  };

  const setMessagesFromHistory = (historyMessages) => {
    setMessages(historyMessages);
  };

  const value = {
    messages,
    loading,
    error,
    addMessage,
    clearMessages,
    setMessagesFromHistory,
    setLoading,
    setError,
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
};
