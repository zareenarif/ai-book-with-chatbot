import React from 'react';
import { useTranslation } from 'react-i18next';

const HighlightedTextButton = ({ position, onClick }) => {
  const { t } = useTranslation();

  if (!position) return null;

  const buttonStyle = {
    position: 'absolute',
    top: `${position.top + position.height + 5}px`,
    left: `${position.left}px`,
    zIndex: 9999,
  };

  return (
    <button
      className="highlighted-text-button"
      style={buttonStyle}
      onClick={onClick}
    >
      {t('chatWidget.askChatbot')}
    </button>
  );
};

export default HighlightedTextButton;
