import React from 'react';
import { useAuth } from '../../context/AuthContext';
import SourceReference from './SourceReference';

const ChatMessage = ({ message }) => {
  const { user } = useAuth();
  const isUser = message.role === 'user';
  const isFromPreferredChapter =
    user?.preferred_chapter &&
    message.sources?.some((source) => source.chapter === user.preferred_chapter);

  const formatTimestamp = (timestamp) => {
    if (!timestamp) return '';
    const date = new Date(timestamp);
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={`chat-message ${isUser ? 'user' : 'assistant'}`}>
      <div className="message-bubble">
        {message.isError && (
          <span className="message-error-badge">Error</span>
        )}

        {isFromPreferredChapter && (
          <span className="message-personalization-badge">
            Answer from {user.preferred_chapter}
          </span>
        )}

        <div className="message-content">{message.content}</div>

        {!isUser && message.sources && message.sources.length > 0 && (
          <div className="message-sources">
            <p className="sources-title">Sources:</p>
            {message.sources.map((source, index) => (
              <SourceReference key={index} source={source} />
            ))}
          </div>
        )}

        <div className="message-timestamp">{formatTimestamp(message.timestamp)}</div>
      </div>
    </div>
  );
};

export default ChatMessage;
