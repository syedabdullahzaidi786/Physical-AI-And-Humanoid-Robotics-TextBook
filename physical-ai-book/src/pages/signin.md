---
title: Sign In
hide_table_of_contents: true
---

import React from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import { useEffect } from 'react';

export function SignInPage() {
  const { user, isLoading, signIn } = useAuth();

  // If user is already logged in, redirect to dashboard
  useEffect(() => {
    if (user && !isLoading) {
      window.location.href = '/dashboard';
    }
  }, [user, isLoading]);

  if (isLoading) {
    return (
      <div style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '100vh',
        background: 'var(--ifm-background-color)',
      }}>
        <div style={{ textAlign: 'center' }}>
          <div style={{
            fontSize: '32px',
            marginBottom: '1rem',
          }}>⏳</div>
          <p>Loading...</p>
        </div>
      </div>
    );
  }

  if (user) {
    return null; // Will redirect via useEffect
  }

  const handleGoogleSignIn = async () => {
    try {
      console.log('[SIGNIN PAGE] Google Sign In button clicked');
      await signIn();
    } catch (error) {
      console.error('[SIGNIN PAGE] Sign in failed:', error);
    }
  };

  return (
    <div style={{
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      minHeight: 'calc(100vh - 120px)',
      background: 'var(--ifm-background-color)',
      padding: '2rem',
    }}>
      <div style={{
        width: '100%',
        maxWidth: '400px',
        background: 'var(--ifm-color-emphasis-100)',
        borderRadius: '12px',
        padding: '3rem 2rem',
        boxShadow: '0 10px 40px rgba(0, 0, 0, 0.1)',
        border: '1px solid var(--ifm-color-emphasis-200)',
      }}>
        {/* Logo / Icon */}
        <div style={{
          textAlign: 'center',
          marginBottom: '2rem',
        }}>
          <div style={{
            fontSize: '48px',
            marginBottom: '1rem',
          }}>🔐</div>
          <h1 style={{
            margin: '0 0 0.5rem 0',
            fontSize: '28px',
            color: 'var(--ifm-font-color-base)',
          }}>
            Sign In
          </h1>
          <p style={{
            margin: '0',
            color: 'var(--ifm-color-emphasis-700)',
            fontSize: '14px',
          }}>
            Access your Physical AI Dashboard
          </p>
        </div>

        {/* Google Sign In Button */}
        <button
          onClick={handleGoogleSignIn}
          style={{
            width: '100%',
            padding: '12px 16px',
            background: 'var(--ifm-color-primary)',
            color: 'white',
            border: 'none',
            borderRadius: '8px',
            fontSize: '16px',
            fontWeight: '600',
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            gap: '10px',
            transition: 'all 0.2s ease',
            boxShadow: '0 4px 12px rgba(15, 23, 42, 0.15)',
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.transform = 'translateY(-2px)';
            e.currentTarget.style.boxShadow = '0 8px 20px rgba(15, 23, 42, 0.2)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.transform = 'translateY(0)';
            e.currentTarget.style.boxShadow = '0 4px 12px rgba(15, 23, 42, 0.15)';
          }}
        >
          <span style={{ fontSize: '20px' }}>🔵</span>
          <span>Sign In with Google</span>
        </button>

        {/* Divider */}
        <div style={{
          display: 'flex',
          alignItems: 'center',
          gap: '12px',
          margin: '2rem 0',
          color: 'var(--ifm-color-emphasis-700)',
        }}>
          <div style={{ flex: 1, height: '1px', background: 'var(--ifm-color-emphasis-200)' }} />
          <span style={{ fontSize: '12px' }}>OR</span>
          <div style={{ flex: 1, height: '1px', background: 'var(--ifm-color-emphasis-200)' }} />
        </div>

        {/* Guest Access Info */}
        <div style={{
          textAlign: 'center',
          padding: '1rem',
          background: 'rgba(99, 179, 237, 0.1)',
          borderRadius: '8px',
          border: '1px solid rgba(99, 179, 237, 0.2)',
          marginBottom: '1.5rem',
        }}>
          <p style={{
            margin: '0 0 0.5rem 0',
            fontSize: '14px',
            color: 'var(--ifm-color-emphasis-700)',
          }}>
            💡 <strong>Tip:</strong>
          </p>
          <p style={{
            margin: '0',
            fontSize: '13px',
            color: 'var(--ifm-color-emphasis-700)',
            lineHeight: '1.4',
          }}>
            You can also browse the documentation without signing in.
          </p>
        </div>

        {/* Back to Home */}
        <div style={{ textAlign: 'center' }}>
          <a
            href="/"
            style={{
              fontSize: '14px',
              color: 'var(--ifm-color-primary)',
              textDecoration: 'none',
              fontWeight: '500',
              transition: 'opacity 0.2s ease',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.opacity = '0.8';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.opacity = '1';
            }}
          >
            ← Back to Home
          </a>
        </div>
      </div>
    </div>
  );
}

<SignInPage />
