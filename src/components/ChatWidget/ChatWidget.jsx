import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import ChatHeader from './ChatHeader';
import ChatHistory from './ChatHistory';
import ChatInput from './ChatInput';
import HighlightedTextButton from './HighlightedTextButton';
import SignupModal from './SignupModal';
import SigninModal from './SigninModal';
import useSelectedText from './useSelectedText';
import useChatHistory from './useChatHistory';
import './chat.css';

const ChatWidget = () => {
  const { t } = useTranslation();
  const [isOpen, setIsOpen] = useState(false);
  const [showSignup, setShowSignup] = useState(false);
  const [showSignin, setShowSignin] = useState(false);
  const [preFilledQuery, setPreFilledQuery] = useState('');

  const { selectedText, selectionPosition, clearSelection } = useSelectedText();
  const { messages, loading, sendMessage } = useChatHistory();

  const handleToggle = () => {
    setIsOpen(!isOpen);
  };

  const handleHighlightedTextClick = () => {
    if (selectedText) {
      setPreFilledQuery(`Explain: ${selectedText}`);
      setIsOpen(true);
      clearSelection();
    }
  };

  const handleSendMessage = async (message) => {
    await sendMessage(message, selectedText || null);
    setPreFilledQuery('');
  };

  return (
    <>
      {/* Highlighted Text Button */}
      {selectedText && selectionPosition && !isOpen && (
        <HighlightedTextButton
          position={selectionPosition}
          onClick={handleHighlightedTextClick}
        />
      )}

      {/* Floating Chat Button */}
      {!isOpen && (
        <button
          className="chat-widget-button"
          onClick={handleToggle}
          aria-label={t('chatWidget.openChat')}
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
            strokeWidth={2}
            stroke="currentColor"
            className="chat-icon"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              d="M8.625 12a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0H8.25m4.125 0a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0H12m4.125 0a.375.375 0 11-.75 0 .375.375 0 01.75 0zm0 0h-.375M21 12c0 4.556-4.03 8.25-9 8.25a9.764 9.764 0 01-2.555-.337A5.972 5.972 0 015.41 20.97a5.969 5.969 0 01-.474-.065 4.48 4.48 0 00.978-2.025c.09-.457-.133-.901-.467-1.226C3.93 16.178 3 14.189 3 12c0-4.556 4.03-8.25 9-8.25s9 3.694 9 8.25z"
            />
          </svg>
        </button>
      )}

      {/* Chat Modal */}
      {isOpen && (
        <div className="chat-widget-modal">
          <ChatHeader
            onClose={handleToggle}
            onOpenSignup={() => setShowSignup(true)}
            onOpenSignin={() => setShowSignin(true)}
          />
          <ChatHistory messages={messages} />
          <ChatInput
            onSend={handleSendMessage}
            loading={loading}
            preFilledQuery={preFilledQuery}
          />
        </div>
      )}

      {/* Auth Modals */}
      {showSignup && (
        <SignupModal
          onClose={() => setShowSignup(false)}
          onSwitchToSignin={() => {
            setShowSignup(false);
            setShowSignin(true);
          }}
        />
      )}

      {showSignin && (
        <SigninModal
          onClose={() => setShowSignin(false)}
          onSwitchToSignup={() => {
            setShowSignin(false);
            setShowSignup(true);
          }}
        />
      )}
    </>
  );
};

export default ChatWidget;
