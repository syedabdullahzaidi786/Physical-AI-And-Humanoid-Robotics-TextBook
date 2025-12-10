import React, { useState } from 'react';
import { useTranslation } from '@site/src/context/TranslationContext';
import styles from './LanguageSwitcher.module.css';

export default function LanguageSwitcher(): React.ReactElement {
  const { isUrdu, toggleLanguage, isReady } = useTranslation();
  const [isLoading, setIsLoading] = useState(false);

  // Don't render until hydration is complete (SSR safety)
  if (!isReady) {
    return (
      <button
        className={styles.languageButton}
        disabled={true}
        title="Loading..."
        style={{ opacity: 0.5 }}
      >
        <span className={styles.flag}>🌐</span>
      </button>
    );
  }

  const handleToggle = async () => {
    setIsLoading(true);
    try {
      toggleLanguage();
      // Small delay to show the toggle action
      await new Promise((resolve) => setTimeout(resolve, 300));
    } catch (error) {
      console.error('[TRANSLATION] Error toggling language:', error);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <button
      className={styles.languageButton}
      onClick={handleToggle}
      disabled={isLoading}
      title={isUrdu ? 'Switch to English' : 'اردو میں سوئچ کریں'}
      aria-label={isUrdu ? 'Switch to English' : 'Switch to Urdu'}
    >
      <span className={styles.flag}>
        {isUrdu ? '🇵🇰' : '🇬🇧'}
      </span>
      <span className={styles.label}>
        {isUrdu ? 'EN' : 'اردو'}
      </span>
    </button>
  );
}
