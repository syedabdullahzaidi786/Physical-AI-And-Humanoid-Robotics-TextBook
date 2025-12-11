import React from 'react';
import Head from '@docusaurus/Head';
import { AuthProvider } from '@site/src/context/AuthContext';
import { TranslationProvider } from '@site/src/context/TranslationContext';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({children}: {children: React.ReactNode}) {
  return (
    <>
      <Head>
        <meta name="algolia-site-verification" content="C92BAA73C916589D" />
      </Head>
      <AuthProvider>
        <TranslationProvider>
          {children}
          <Chatbot apiUrl={process.env.REACT_APP_CHATBOT_API_URL || 'http://localhost:8000'} />
        </TranslationProvider>
      </AuthProvider>
    </>
  );
}
