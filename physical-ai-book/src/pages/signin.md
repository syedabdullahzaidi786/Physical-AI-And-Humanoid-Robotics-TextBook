---
title: Sign In
hide_table_of_contents: true
---

import React from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import { useEffect } from 'react';

export function SignInPage() {
  const { user, isLoading, signIn } = useAuth();

  useEffect(() => {
    if (!isLoading && !user) {
      console.log('[SIGNIN] Auto-initiating SSO redirect...');
      signIn();
    } else if (user && !isLoading) {
      window.location.href = '/dashboard';
    }
  }, [user, isLoading, signIn]);

  return (
    <div style={{
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      minHeight: '70vh',
      flexDirection: 'column',
      gap: '24px',
      textAlign: 'center',
      padding: '2rem'
    }}>
      <div style={{ fontSize: '60px', animation: 'bounce 2s infinite' }}>🔐</div>
      <h1 style={{ fontSize: '2.5rem', marginBottom: '0.5rem' }}>SSO Redirection</h1>
      <p style={{ fontSize: '1.2rem', color: 'var(--ifm-color-emphasis-700)', maxWidth: '500px' }}>
        We are taking you to the <strong>AR Devs SSO</strong> to sign in securely.
      </p>
      
      <div style={{ 
        padding: '1.5rem', 
        background: 'rgba(52, 152, 219, 0.1)', 
        borderRadius: '12px',
        border: '1px border var(--ifm-color-primary)',
        marginTop: '1rem'
      }}>
        <p style={{ marginBottom: '1rem', fontWeight: '500' }}>If you are not redirected automatically:</p>
        <button 
          onClick={() => signIn()}
          className="button button--primary button--lg"
          style={{ padding: '12px 32px', fontSize: '1.1rem' }}
        >
          Go to Sign In
        </button>
      </div>

      <style>{`
        @keyframes bounce {
          0%, 20%, 50%, 80%, 100% {transform: translateY(0);}
          40% {transform: translateY(-20px);}
          60% {transform: translateY(-10px);}
        }
      `}</style>
    </div>
  );
}

<SignInPage />
