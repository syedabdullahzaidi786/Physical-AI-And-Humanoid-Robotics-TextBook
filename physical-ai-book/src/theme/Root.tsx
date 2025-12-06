import React from 'react';
import Head from '@docusaurus/Head';

export default function Root({children}: {children: React.ReactNode}) {
  return (
    <>
      <Head>
        <meta name="algolia-site-verification" content="C92BAA73C916589D" />
      </Head>
      {children}
    </>
  );
}
