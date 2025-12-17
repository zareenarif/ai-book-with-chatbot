import { useState, useEffect } from 'react';
import { sendChatMessage, getChatHistory } from '../../services/api';
import { useAuth } from '../../context/AuthContext';
import { useChat } from '../../context/ChatContext';
import { v4 as uuidv4 } from 'uuid';

const useChatHistory = () => {
  const { isAuthenticated, user } = useAuth();
  const { messages, addMessage, setMessagesFromHistory, setLoading, setError } = useChat();
  const [sessionId] = useState(() => {
    // Create or retrieve session ID (SSR-safe)
    if (typeof window === 'undefined') {
      return uuidv4(); // Return a temporary ID during SSR
    }
    const stored = localStorage.getItem('session_id');
    if (stored) return stored;
    const newId = uuidv4();
    localStorage.setItem('session_id', newId);
    return newId;
  });

  useEffect(() => {
    // Load chat history if authenticated
    if (isAuthenticated) {
      loadChatHistory();
    }
  }, [isAuthenticated]);

  const loadChatHistory = async () => {
    try {
      setLoading(true);
      const history = await getChatHistory();

      // Format history messages
      const formattedMessages = history.messages?.map((msg) => ({
        role: msg.role,
        content: msg.message,
        sources: msg.sources,
        timestamp: msg.created_at,
      })) || [];

      setMessagesFromHistory(formattedMessages);
    } catch (err) {
      console.error('Error loading chat history:', err);
      setError('Failed to load chat history');
    } finally {
      setLoading(false);
    }
  };

  const sendMessage = async (message, highlightedText = null) => {
    try {
      setLoading(true);
      setError(null);

      // Add user message
      addMessage({
        role: 'user',
        content: message,
        timestamp: new Date().toISOString(),
      });

      // Get preferred chapter from user preferences
      const preferredChapter = user?.preferred_chapter || null;

      // Send to API
      const response = await sendChatMessage(
        message,
        sessionId,
        highlightedText,
        preferredChapter
      );

      // Add assistant response
      addMessage({
        role: 'assistant',
        content: response.message,
        sources: response.sources,
        timestamp: response.timestamp,
      });

      return response;
    } catch (err) {
      console.error('Error sending message:', err);
      setError(err.response?.data?.detail || 'Failed to send message');

      // Add error message
      addMessage({
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString(),
        isError: true,
      });
    } finally {
      setLoading(false);
    }
  };

  return {
    messages,
    loading,
    error,
    sendMessage,
    loadChatHistory,
  };
};

export default useChatHistory;
