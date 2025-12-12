import React from 'react';

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
  return (
    <span className={className}>
      {text}
    </span>
  );
};

export default TranslatableText;
