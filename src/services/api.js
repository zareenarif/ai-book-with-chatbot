import axios from 'axios';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/v1';

// Create axios instance with default config
const apiClient = axios.create({
  baseURL: API_URL,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Add JWT token to requests if available
apiClient.interceptors.request.use((config) => {
  const token = localStorage.getItem('access_token');
  if (token) {
    config.headers.Authorization = `Bearer ${token}`;
  }
  return config;
});

// Handle errors globally
apiClient.interceptors.response.use(
  (response) => response,
  (error) => {
    if (error.response?.status === 401) {
      // Clear token and redirect to login
      localStorage.removeItem('access_token');
      localStorage.removeItem('user');
    }
    return Promise.reject(error);
  }
);

// Chat API
export const sendChatMessage = async (message, sessionId, highlightedText = null, preferredChapter = null) => {
  const response = await apiClient.post('/chat', {
    message,
    session_id: sessionId,
    highlighted_text: highlightedText,
    preferred_chapter: preferredChapter,
  });
  return response.data;
};

export const searchContent = async (query, preferredChapter = null) => {
  const response = await apiClient.post('/search', {
    message: query,
    session_id: 'search',
    preferred_chapter: preferredChapter,
  });
  return response.data;
};

// Auth API
export const signup = async (email, password) => {
  const response = await apiClient.post('/auth/signup', {
    email,
    password,
  });
  return response.data;
};

export const signin = async (email, password) => {
  const response = await apiClient.post('/auth/signin', {
    email,
    password,
  });

  // Store token and user
  if (response.data.access_token) {
    localStorage.setItem('access_token', response.data.access_token);
    localStorage.setItem('user', JSON.stringify(response.data.user));
  }

  return response.data;
};

export const requestPasswordReset = async (email) => {
  const response = await apiClient.post('/auth/reset-password-request', {
    email,
  });
  return response.data;
};

export const resetPassword = async (token, newPassword) => {
  const response = await apiClient.post('/auth/reset-password', {
    token,
    new_password: newPassword,
  });
  return response.data;
};

// Preferences API
export const updatePreferences = async (languagePref, preferredChapter) => {
  const response = await apiClient.put('/auth/preferences', {
    language_pref: languagePref,
    preferred_chapter: preferredChapter,
  });
  return response.data;
};

// Chat History API
export const getChatHistory = async (page = 1, perPage = 50) => {
  const response = await apiClient.get('/history', {
    params: { page, per_page: perPage },
  });
  return response.data;
};

// Health Check
export const healthCheck = async () => {
  const response = await apiClient.get('/health');
  return response.data;
};

export default apiClient;
