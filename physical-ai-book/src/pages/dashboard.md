---
title: Dashboard
hide_table_of_contents: true
---

import React from 'react';
import { useAuth } from '@site/src/context/AuthContext';

export function DashboardContent() {
  const { user, isLoading } = useAuth();

  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (!user) {
    return (
      <div style={{ textAlign: 'center', padding: '2rem' }}>
        <h1>Access Denied</h1>
        <p>Please sign in to view this page.</p>
        <a href="/" className="button button--primary">
          Back to Home
        </a>
      </div>
    );
  }

  return (
    <div style={{ maxWidth: '900px', margin: '0 auto', padding: '2rem' }}>
      <h1>Welcome, {user.name || user.email}!</h1>

      <section style={{
        background: 'var(--ifm-color-emphasis-100)',
        padding: '2rem',
        borderRadius: '8px',
        marginBottom: '2rem',
      }}>
        <h2>📊 Your Profile</h2>
        <div style={{ display: 'flex', gap: '2rem', alignItems: 'flex-start' }}>
          {user.image && (
            <img
              src={user.image}
              alt={user.name}
              style={{
                width: '120px',
                height: '120px',
                borderRadius: '50%',
                objectFit: 'cover',
              }}
            />
          )}
          <div>
            <p><strong>Name:</strong> {user.name}</p>
            <p><strong>Email:</strong> {user.email}</p>
            <p><strong>ID:</strong> {user.id}</p>
          </div>
        </div>
      </section>

      <section style={{ marginBottom: '2rem' }}>
        <h2>📚 Quick Access</h2>
        <ul>
          <li><a href="/docs/intro">Getting Started</a></li>
          <li><a href="/docs/01-module-ros2">ROS 2 Module</a></li>
          <li><a href="/docs/06-case-studies/tutoring">Case Studies</a></li>
        </ul>
      </section>

      <section style={{
        background: 'var(--ifm-color-emphasis-100)',
        padding: '1.5rem',
        borderRadius: '8px',
        marginBottom: '2rem',
      }}>
        <h2>🔐 Session Info</h2>
        <p>You are securely logged in. Your session expires in 7 days.</p>
      </section>

      <div style={{ textAlign: 'center' }}>
        <a href="/" className="button button--secondary">
          Back to Home
        </a>
      </div>
    </div>
  );
}

<DashboardContent />
