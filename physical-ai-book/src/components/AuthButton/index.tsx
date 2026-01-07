import React, { useState } from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import styles from './styles.module.css';

export default function AuthButton(): React.ReactElement {
  const { user, isLoading, signIn, signOut } = useAuth();
  const [menuOpen, setMenuOpen] = useState(false);

  if (isLoading) {
    return <div className={styles.authButton}>Loading...</div>;
  }

  if (!user) {
    return (
      <button className={styles.authButton} onClick={signIn} title="Sign in with SSO">
        🔐 Sign In
      </button>
    );
  }

  return (
    <div className={styles.userMenu}>
      <button
        className={styles.userButton}
        onClick={() => setMenuOpen(!menuOpen)}
        title={user.email}
      >
        {user.image ? (
          <img src={user.image} alt={user.name} className={styles.avatar} />
        ) : (
          <span className={styles.avatarPlaceholder}>
            {(user.name || user.email).charAt(0).toUpperCase()}
          </span>
        )}
        <span className={styles.userName}>{user.name || user.email.split('@')[0]}</span>
      </button>

      {menuOpen && (
        <div className={styles.dropdown}>
          <div className={styles.userInfo}>
            <strong>{user.name}</strong>
            <small>{user.email}</small>
          </div>
          <hr className={styles.divider} />
          <a href="/dashboard" className={styles.menuItem}>
            📊 Dashboard
          </a>
          <a href="/docs/intro" className={styles.menuItem}>
            📚 Documentation
          </a>
          <hr className={styles.divider} />
          <button className={styles.signOutBtn} onClick={() => {
            setMenuOpen(false);
            signOut();
          }}>
            🚪 Sign Out
          </button>
        </div>
      )}
    </div>
  );
}
