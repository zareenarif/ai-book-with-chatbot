import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { useAuth } from '../../context/AuthContext';
import { updatePreferences } from '../../services/api';

const ChatHeader = ({ onClose, onOpenSignup, onOpenSignin }) => {
  const { t, i18n } = useTranslation();
  const { user, isAuthenticated, logout, updateUser } = useAuth();
  const [showMenu, setShowMenu] = useState(false);

  const handleLanguageToggle = async () => {
    const newLang = i18n.language === 'en' ? 'ur' : 'en';
    await i18n.changeLanguage(newLang);

    // Update preference if authenticated
    if (isAuthenticated) {
      try {
        const updated = await updatePreferences(newLang, user?.preferred_chapter);
        updateUser(updated);
      } catch (err) {
        console.error('Error updating language preference:', err);
      }
    }
  };

  const handleResetPersonalization = async () => {
    if (isAuthenticated) {
      try {
        const updated = await updatePreferences(user?.language_pref || 'en', null);
        updateUser(updated);
      } catch (err) {
        console.error('Error resetting personalization:', err);
      }
    }
  };

  return (
    <div className="chat-header">
      <h3 className="chat-title">{t('chatWidget.title')}</h3>

      <div className="chat-header-actions">
        {/* Language Toggle */}
        <button
          className="chat-header-btn"
          onClick={handleLanguageToggle}
          title={t('chatWidget.toggleLanguage')}
        >
          {i18n.language === 'en' ? 'اردو' : 'English'}
        </button>

        {/* Auth Buttons / User Menu */}
        {isAuthenticated ? (
          <div className="user-menu">
            <button
              className="chat-header-btn"
              onClick={() => setShowMenu(!showMenu)}
            >
              {user?.email?.substring(0, 2).toUpperCase()}
            </button>

            {showMenu && (
              <div className="user-menu-dropdown">
                <div className="user-email">{user?.email}</div>
                {user?.preferred_chapter && (
                  <button onClick={handleResetPersonalization}>
                    {t('chatWidget.resetPersonalization')}
                  </button>
                )}
                <button onClick={logout}>{t('chatWidget.logout')}</button>
              </div>
            )}
          </div>
        ) : (
          <div className="auth-buttons">
            <button className="chat-header-btn" onClick={onOpenSignin}>
              {t('chatWidget.signin')}
            </button>
            <button className="chat-header-btn" onClick={onOpenSignup}>
              {t('chatWidget.signup')}
            </button>
          </div>
        )}

        {/* Close Button */}
        <button
          className="chat-header-btn"
          onClick={onClose}
          aria-label={t('chatWidget.close')}
        >
          ✕
        </button>
      </div>
    </div>
  );
};

export default ChatHeader;
