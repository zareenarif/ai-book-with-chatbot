import React, { useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import ChatWidget from '../components/ChatWidget/ChatWidget';
import { AuthProvider } from '../context/AuthContext';
import { ChatProvider } from '../context/ChatContext';

export default function Root({children}) {
  useEffect(() => {
    // Initialize i18n only in browser
    if (typeof window !== 'undefined') {
      import('../i18n');
    }
  }, []);

  return (
    <AuthProvider>
      <ChatProvider>
        {children}
        <BrowserOnly fallback={<div></div>}>
          {() => <ChatWidget />}
        </BrowserOnly>
      </ChatProvider>
    </AuthProvider>
  );
}
