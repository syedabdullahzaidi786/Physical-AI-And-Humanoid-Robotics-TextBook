import React, { useEffect, useState } from 'react';
import { useTranslation } from '@site/src/context/TranslationContext';

interface TranslatableTextProps {
  text: string;
  className?: string;
  children?: never;
}

/**
 * Component to render translatable text content
 * Automatically translates when language is switched
 */
export const TranslatableText: React.FC<TranslatableTextProps> = ({ text, className }) => {
  const { isUrdu, translate, translateSync } = useTranslation();
  const [displayText, setDisplayText] = useState(text);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    if (!isUrdu) {
      setDisplayText(text);
      return;
    }

    // Try sync translation first (from dictionary)
    const syncTranslation = translateSync(text);
    setDisplayText(syncTranslation);

    // If it's different from original, we found a match in dictionary
    if (syncTranslation !== text) {
      return;
    }

    // Otherwise, try async translation (API)
    setIsLoading(true);
    translate(text)
      .then((translated) => {
        setDisplayText(translated);
      })
      .catch(() => {
        setDisplayText(text);
      })
      .finally(() => {
        setIsLoading(false);
      });
  }, [isUrdu, text, translate, translateSync]);

  return (
    <span className={className} style={isLoading ? { opacity: 0.7 } : {}}>
      {displayText}
    </span>
  );
};

export default TranslatableText;
