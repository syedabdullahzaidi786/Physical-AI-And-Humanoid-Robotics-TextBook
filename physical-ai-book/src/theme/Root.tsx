import React from 'react';
import Head from '@docusaurus/Head';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { AuthProvider } from '@site/src/context/AuthContext';

import Chatbot from '@site/src/components/ChatbotComponent';

function DocusaurusRoot({ children }: { children: React.ReactNode }) {
  const { siteConfig } = useDocusaurusContext();

  // Get chatbot API URL from environment (via customFields) or use default
  const chatbotApiUrl = (siteConfig.customFields?.chatbotApiUrl as string) || 'http://localhost:8000';

  return (
    <>
      <Head>
        <meta name="algolia-site-verification" content="C92BAA73C916589D" />
      </Head>
      <AuthProvider>
        {children}
        <Chatbot apiUrl={chatbotApiUrl} />
      </AuthProvider>
    </>
  );
}

export default DocusaurusRoot;
