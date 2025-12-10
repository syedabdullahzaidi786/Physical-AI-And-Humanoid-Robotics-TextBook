import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { translateToUrdu, translateSimple } from '@site/src/lib/translator';

interface TranslationContextType {
  isUrdu: boolean;
  toggleLanguage: () => void;
  translate: (text: string) => Promise<string>;
  translateSync: (text: string) => string; // Simple client-side translation
  currentLanguage: 'en' | 'ur';
  isReady: boolean; // Hydration-safe flag
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

export const TranslationProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [isUrdu, setIsUrdu] = useState(false);
  const [isReady, setIsReady] = useState(false);

  // Load language preference from localStorage on mount (client-side only)
  useEffect(() => {
    try {
      const saved = localStorage.getItem('preferred-language');
      if (saved === 'ur') {
        setIsUrdu(true);
      }
    } catch (e) {
      // localStorage not available
    }
    setIsReady(true);
  }, []);

  const toggleLanguage = useCallback(() => {
    const newIsUrdu = !isUrdu;
    setIsUrdu(newIsUrdu);
    try {
      localStorage.setItem('preferred-language', newIsUrdu ? 'ur' : 'en');
    } catch (e) {
      // localStorage not available
    }
    console.log('[TRANSLATION] Language switched to:', newIsUrdu ? 'Urdu' : 'English');
  }, [isUrdu]);

  const translate = useCallback(async (text: string): Promise<string> => {
    if (!isUrdu || !text) {
      return text;
    }
    return translateToUrdu(text);
  }, [isUrdu]);

  const translateSync = useCallback((text: string): string => {
    if (!isUrdu || !text) {
      return text;
    }
    return translateSimple(text);
  }, [isUrdu]);

  const currentLanguage = isUrdu ? 'ur' : 'en';

  return (
    <TranslationContext.Provider
      value={{
        isUrdu,
        toggleLanguage,
        translate,
        translateSync,
        currentLanguage,
        isReady,
      }}
    >
      {children}
    </TranslationContext.Provider>
  );
};

export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) {
    throw new Error('useTranslation must be used within TranslationProvider');
  }
  return context;
};
