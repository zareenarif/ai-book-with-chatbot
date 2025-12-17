import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { signup } from '../../services/api';
import { useAuth } from '../../context/AuthContext';

const SignupModal = ({ onClose, onSwitchToSignin }) => {
  const { t } = useTranslation();
  const { login } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await signup(email, password);
      setSuccess(true);
      setTimeout(() => {
        onClose();
      }, 2000);
    } catch (err) {
      setError(err.response?.data?.detail || 'Signup failed. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-modal-overlay" onClick={onClose}>
      <div className="auth-modal" onClick={(e) => e.stopPropagation()}>
        <div className="auth-modal-header">
          <h3>{t('auth.signup')}</h3>
          <button className="auth-modal-close" onClick={onClose}>
            ✕
          </button>
        </div>

        {success ? (
          <div className="auth-success">
            <p>✅ Account created! Please check your email to verify.</p>
          </div>
        ) : (
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
                minLength={8}
                autoComplete="new-password"
              />
              <small className="auth-hint">
                {t('auth.passwordRequirements')}
              </small>
            </div>

            <button
              type="submit"
              className="auth-submit-button"
              disabled={loading}
            >
              {loading ? t('auth.signingUp') : t('auth.signup')}
            </button>
          </form>
        )}

        <div className="auth-switch">
          {t('auth.alreadyHaveAccount')}{' '}
          <button onClick={onSwitchToSignin}>{t('auth.signin')}</button>
        </div>
      </div>
    </div>
  );
};

export default SignupModal;
