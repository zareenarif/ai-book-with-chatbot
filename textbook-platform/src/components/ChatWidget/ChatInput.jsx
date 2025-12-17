import React, { useState, useEffect } from 'react';
import { useTranslation } from 'react-i18next';

const ChatInput = ({ onSend, loading, preFilledQuery }) => {
  const { t } = useTranslation();
  const [message, setMessage] = useState('');

  useEffect(() => {
    if (preFilledQuery) {
      setMessage(preFilledQuery);
    }
  }, [preFilledQuery]);

  const handleSubmit = (e) => {
    e.preventDefault();
    if (message.trim() && !loading) {
      onSend(message);
      setMessage('');
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <form className="chat-input-container" onSubmit={handleSubmit}>
      <textarea
        className="chat-input"
        value={message}
        onChange={(e) => setMessage(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder={t('chatWidget.inputPlaceholder')}
        maxLength={5000}
        rows={3}
        disabled={loading}
      />
      <button
        type="submit"
        className="chat-send-button"
        disabled={!message.trim() || loading}
      >
        {loading ? t('chatWidget.sending') : t('chatWidget.send')}
      </button>
    </form>
  );
};

export default ChatInput;
