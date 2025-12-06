import React, { useState } from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import styles from './NavbarAuthItem.module.css';

export default function NavbarAuthItem(): React.ReactElement {
  const { user, isLoading, signIn, signOut } = useAuth();
  const [menuOpen, setMenuOpen] = useState(false);

  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.skeleton} />
      </div>
    );
  }

  if (!user) {
    return (
      <div className={styles.container}>
        <button 
          className={styles.signInBtn}
          onClick={() => {
            setMenuOpen(false);
            signIn();
          }}
          title="Sign in with Google"
        >
          <span className={styles.icon}>🔐</span>
          <span className={styles.text}>Sign In</span>
        </button>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.userMenuWrapper}>
        <button
          className={styles.userBtn}
          onClick={() => setMenuOpen(!menuOpen)}
          title={user.email}
          aria-expanded={menuOpen}
          aria-haspopup="true"
        >
          {user.image ? (
            <img src={user.image} alt={user.name || 'User'} className={styles.avatar} />
          ) : (
            <span className={styles.avatarPlaceholder}>
              {(user.name || user.email).charAt(0).toUpperCase()}
            </span>
          )}
          <span className={styles.userName}>
            {user.name ? user.name.split(' ')[0] : user.email.split('@')[0]}
          </span>
          <span className={styles.chevron}>▼</span>
        </button>

        {menuOpen && (
          <div className={styles.dropdown} role="menu">
            <div className={styles.userInfo}>
              <div className={styles.userNameFull}>{user.name || 'User'}</div>
              <div className={styles.userEmail}>{user.email}</div>
            </div>

            <div className={styles.divider} />

            <a 
              href="/dashboard" 
              className={styles.menuItem}
              onClick={() => setMenuOpen(false)}
              role="menuitem"
            >
              <span className={styles.menuIcon}>📊</span>
              <span>Dashboard</span>
            </a>

            <a 
              href="/docs/intro" 
              className={styles.menuItem}
              onClick={() => setMenuOpen(false)}
              role="menuitem"
            >
              <span className={styles.menuIcon}>📚</span>
              <span>Documentation</span>
            </a>

            <div className={styles.divider} />

            <button
              className={styles.signOutBtn}
              onClick={() => {
                setMenuOpen(false);
                signOut();
              }}
              role="menuitem"
            >
              <span className={styles.menuIcon}>🚪</span>
              <span>Sign Out</span>
            </button>
          </div>
        )}
      </div>

      {/* Backdrop to close menu on click outside */}
      {menuOpen && (
        <div 
          className={styles.backdrop}
          onClick={() => setMenuOpen(false)}
          aria-hidden="true"
        />
      )}
    </div>
  );
}
