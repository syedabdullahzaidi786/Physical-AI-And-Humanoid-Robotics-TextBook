import React from 'react';
import Head from '@docusaurus/Head';
import { AuthProvider } from '@site/src/context/AuthContext';

export default function Root({children}: {children: React.ReactNode}) {
  return (
    <>
      <Head>
        <meta name="algolia-site-verification" content="C92BAA73C916589D" />
      </Head>
      <AuthProvider>
        {children}
      </AuthProvider>
    </>
  );
}
