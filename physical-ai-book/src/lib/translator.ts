import axios from 'axios';

const TRANSLATE_API = 'https://api.mymemory.translated.net/get';

export interface TranslationCache {
  [key: string]: string;
}

const translationCache: TranslationCache = {};

// Common English to Urdu translations for offline use
const commonTranslations: TranslationCache = {
  // Navigation & UI
  'Home': 'ہوم',
  'About': 'کے بارے میں',
  'Documentation': 'دستاویزات',
  'Blog': 'بلاگ',
  'Search': 'تلاش',
  'Sign In': 'سائن ان کریں',
  'Sign Out': 'سائن آؤٹ کریں',
  'Welcome': 'خوش آمدید',
  'Dashboard': 'ڈیش بورڈ',
  'Profile': 'پروفائل',
  'Settings': 'سیٹنگز',
  'Logout': 'لاگ آؤٹ',
  
  // Book related
  'Physical AI': 'جسمانی AI',
  'Humanoid Robotics': 'انسان نما روبوٹکس',
  'Introduction': 'تعارف',
  'Module': 'ماڈیول',
  'Case Studies': 'کیس اسٹڈیز',
  'Tutorial': 'سبق',
  'Next': 'اگلا',
  'Previous': 'پچھلا',
  'Back': 'واپس',
  
  // Common words
  'Loading': 'لوڈ ہو رہا ہے',
  'Error': 'خرابی',
  'Success': 'کامیاب',
  'Cancel': 'منسوخ کریں',
  'Submit': 'جمع کریں',
  'Close': 'بند کریں',
  'Open': 'کھولیں',
  'Save': 'محفوظ کریں',
  'Delete': 'حذف کریں',
  'Edit': 'ترمیم کریں',
  'View': 'دیکھیں',
  'Download': 'ڈاؤن لوڈ کریں',
  'Language': 'زبان',
  'English': 'انگریزی',
  'Urdu': 'اردو',
};

/**
 * Simple offline translation using common word dictionary
 */
export function translateSimple(text: string): string {
  if (!text) return text;
  
  // Check if exact match exists
  if (commonTranslations[text]) {
    return commonTranslations[text];
  }
  
  // Try case-insensitive for sentences
  const lowerText = text.toLowerCase();
  for (const [eng, urdu] of Object.entries(commonTranslations)) {
    if (eng.toLowerCase() === lowerText) {
      return urdu;
    }
  }
  
  // Return original if no match
  return text;
}

/**
 * Translate text from English to Urdu using MyMemory Translated API (free)
 * Note: For production, consider using Google Translate API with proper key
 */
export async function translateToUrdu(text: string): Promise<string> {
  if (!text || text.trim().length === 0) {
    return text;
  }

  // Check cache first
  if (translationCache[text]) {
    return translationCache[text];
  }

  try {
    const response = await axios.get(TRANSLATE_API, {
      params: {
        q: text,
        langpair: 'en|ur', // English to Urdu
      },
      timeout: 5000,
    });

    if (response.data && response.data.responseData) {
      const translatedText = response.data.responseData.translatedText;
      
      // Cache the translation
      translationCache[text] = translatedText;
      
      return translatedText;
    }

    // Fallback to original text if translation fails
    return text;
  } catch (error) {
    console.error('[TRANSLATION] Error translating text:', error);
    return text; // Return original text on error
  }
}

/**
 * Translate multiple text blocks in parallel
 */
export async function translateTextsToUrdu(texts: string[]): Promise<string[]> {
  return Promise.all(texts.map((text) => translateToUrdu(text)));
}

/**
 * Clear translation cache (useful for memory management)
 */
export function clearTranslationCache(): void {
  Object.keys(translationCache).forEach((key) => {
    delete translationCache[key];
  });
}
