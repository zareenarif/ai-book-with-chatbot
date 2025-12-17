import React, { useState, useEffect } from 'react';
import { useTranslation } from 'react-i18next';
import { useAuth } from '../context/AuthContext';
import { updatePreferences } from '../services/api';

const PersonalizeButton = () => {
  const { t } = useTranslation();
  const { user, isAuthenticated, updateUser } = useAuth();
  const [currentChapter, setCurrentChapter] = useState(null);

  useEffect(() => {
    // Extract chapter from URL
    const path = window.location.pathname;
    const chapterMatch = path.match(/chapter-(\d+)/);

    if (chapterMatch) {
      setCurrentChapter(`Chapter ${chapterMatch[1]}`);
    } else {
      setCurrentChapter(null);
    }
  }, [window.location.pathname]);

  const handlePersonalize = async () => {
    if (!currentChapter) return;

    if (isAuthenticated) {
      try {
        const updated = await updatePreferences(user?.language_pref || 'en', currentChapter);
        updateUser(updated);
        alert(`Personalized for ${currentChapter}!`);
      } catch (err) {
        console.error('Error updating personalization:', err);
      }
    } else {
      // For guests, store in localStorage
      localStorage.setItem('preferred_chapter', currentChapter);
      alert(`Personalized for ${currentChapter}!`);
    }
  };

  if (!currentChapter || user?.preferred_chapter === currentChapter) {
    return null;
  }

  return (
    <button className="personalize-button" onClick={handlePersonalize}>
      {t('personalization.personalizeForChapter')}
    </button>
  );
};

export default PersonalizeButton;
