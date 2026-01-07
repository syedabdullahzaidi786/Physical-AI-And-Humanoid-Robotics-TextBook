import React from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import styles from './NavbarAuthItem.module.css';

export default function NavbarAuthItem(): React.ReactElement {
  const { user, isLoading, signIn } = useAuth();

  const redirectUrl = "/dashboard"; // Change this to your desired URL

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
          onClick={() => signIn()}
          title="Sign in with SSO"
        >
          <span className={styles.icon}>🔐</span>
          <span className={styles.text}>Sign In</span>
        </button>
      </div>
    );
  }

  // If user is signed in, clicking redirects to the target URL
  return (
    <div className={styles.container}>
      <button
        className={styles.userBtn}
        onClick={() => window.location.href = redirectUrl}
        title={user.email}
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
      </button>
    </div>
  );
}
