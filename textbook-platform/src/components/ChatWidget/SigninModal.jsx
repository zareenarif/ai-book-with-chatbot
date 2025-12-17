import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { signin } from '../../services/api';
import { useAuth } from '../../context/AuthContext';

const SigninModal = ({ onClose, onSwitchToSignup }) => {
  const { t } = useTranslation();
  const { login } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const response = await signin(email, password);
      login(response.access_token, response.user);
      onClose();
    } catch (err) {
      setError(err.response?.data?.detail || 'Invalid email or password');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-modal-overlay" onClick={onClose}>
      <div className="auth-modal" onClick={(e) => e.stopPropagation()}>
        <div className="auth-modal-header">
          <h3>{t('auth.signin')}</h3>
          <button className="auth-modal-close" onClick={onClose}>
            ✕
          </button>
        </div>

        <form onSubmit={handleSubmit} className="auth-form">
          {error && <div className="auth-error">{error}</div>}

          <div className="auth-form-group">
            <label htmlFor="email">{t('auth.email')}</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              autoComplete="email"
            />
          </div>

          <div className="auth-form-group">
            <label htmlFor="password">{t('auth.password')}</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              autoComplete="current-password"
            />
          </div>

          <button
            type="submit"
            className="auth-submit-button"
            disabled={loading}
          >
            {loading ? t('auth.signingIn') : t('auth.signin')}
          </button>
        </form>

        <div className="auth-switch">
          {t('auth.noAccount')}{' '}
          <button onClick={onSwitchToSignup}>{t('auth.signup')}</button>
        </div>
      </div>
    </div>
  );
};

export default SigninModal;
