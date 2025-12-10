# English-to-Urdu Translation Feature

## Overview

The Physical AI & Humanoid Robotics textbook now includes a built-in English-to-Urdu translation feature. Users can switch between English and Urdu with a single click using the language toggle button in the navbar.

## Features

### 🌐 Language Toggle
- Located in the top-right navigation bar (🇬🇧/🇵🇰 button)
- Instant language switching
- Persistent language preference (saved in browser storage)

### 📱 Translation Approaches

**1. Offline Dictionary (Fast)**
- ~50 common UI and book-related words pre-translated
- No network dependency
- Instant results
- Examples: Home, Search, Documentation, Dashboard, Sign In

**2. MyMemory API (Comprehensive)**
- Free tier support for extended content
- Automatic fallback for words not in dictionary
- Caching to minimize API calls
- Graceful degradation if API unavailable

### 🔧 Technical Architecture

```
TranslationContext (React Context)
  ├─ isUrdu: boolean (current language state)
  ├─ isReady: boolean (SSR hydration safety)
  ├─ toggleLanguage(): void
  ├─ translate(text): Promise<string>
  └─ translateSync(text): string (offline only)
       │
       ├─ translator.ts
       │  ├─ commonTranslations{} (offline dictionary)
       │  ├─ translateSimple() (dictionary lookup)
       │  ├─ translateToUrdu() (API + cache)
       │  └─ translationCache{} (API response cache)
       │
       └─ Components
          ├─ LanguageSwitcher
          │  └─ Navbar button with flag emoji
          │
          └─ TranslatableText
             └─ Wraps content for dynamic translation
```

## Usage

### For Users

1. **Switch Language**: Click the language button (🇬🇧 EN / 🇵🇰 اردو) in the navbar
2. **Language Persists**: Your choice is saved in browser storage
3. **Instant Updates**: Page content updates immediately
4. **Works Offline**: Common words work without internet

### For Developers

#### Using the Translation Hook

```tsx
import { useTranslation } from '@site/src/context/TranslationContext';

export function MyComponent() {
  const { isUrdu, toggleLanguage, translate } = useTranslation();

  return (
    <div>
      <p>{isUrdu ? 'اردو' : 'English'}</p>
      <button onClick={toggleLanguage}>Switch Language</button>
    </div>
  );
}
```

#### Using TranslatableText Component

```tsx
import TranslatableText from '@site/src/components/TranslatableText';

export function MyPage() {
  return (
    <div>
      <h1><TranslatableText text="Welcome" /></h1>
      <p><TranslatableText text="This is a demo page" /></p>
    </div>
  );
}
```

#### Sync Translation (for UI strings)

```tsx
const { translateSync } = useTranslation();
const urduLabel = translateSync('Search'); // Instant, no async
```

#### Async Translation (for content)

```tsx
const { translate } = useTranslation();
const urduContent = await translate('This is longer content...');
```

## File Structure

```
physical-ai-book/
├── src/
│   ├── context/
│   │   └── TranslationContext.tsx          # Main state management
│   │
│   ├── lib/
│   │   └── translator.ts                   # Translation service
│   │                                        # - Online: MyMemory API
│   │                                        # - Offline: Dictionary
│   │
│   ├── components/
│   │   ├── LanguageSwitcher/
│   │   │   ├── index.tsx                   # Navbar button
│   │   │   └── LanguageSwitcher.module.css
│   │   │
│   │   └── TranslatableText/
│   │       └── index.tsx                   # Content wrapper
│   │
│   ├── pages/
│   │   └── translation-demo.md             # Demo page
│   │
│   ├── theme/
│   │   ├── Root.tsx                        # Wraps app with TranslationProvider
│   │   └── Navbar/
│   │       └── index.tsx                   # Includes LanguageSwitcher
│   │
│   └── tests/
│       └── translation.test.ts             # Feature tests
```

## Offline Dictionary

Current translations include:

### Navigation & UI
- Home, About, Documentation, Blog, Search
- Sign In, Sign Out, Profile, Settings, Logout
- Dashboard, Welcome

### Book Related
- Physical AI, Humanoid Robotics, Introduction
- Module, Case Studies, Tutorial
- Next, Previous, Back

### Common Words
- Loading, Error, Success, Cancel, Submit
- Close, Open, Save, Delete, Edit
- View, Download, Language, English, Urdu

**To add more:** Edit `src/lib/translator.ts` → `commonTranslations` object

## API Fallback: MyMemory Translated

- **Endpoint**: `https://api.mymemory.translated.net/get`
- **Timeout**: 5 seconds per request
- **Language Pair**: `en|ur` (English to Urdu)
- **Caching**: API responses cached in memory
- **No API Key**: Free tier (no authentication needed)

Example:
```
GET https://api.mymemory.translated.net/get?q=Hello%20World&langpair=en|ur
```

## SSR & Hydration

The feature handles Docusaurus SSR challenges:

1. **isReady Flag**: Prevents render mismatch during hydration
2. **Client-Side Only**: localStorage access guarded with try-catch
3. **Fallback Rendering**: Shows loading state (🌐) until client hydrates

## Testing

### Manual Testing
1. Visit `/translation-demo` page
2. Click language toggle button (top-right navbar)
3. Verify:
   - Page content changes language
   - Button shows correct flag/label
   - Language preference persists on refresh

### Automated Testing
```bash
cd physical-ai-book
npm test -- src/tests/translation.test.ts
```

## Performance

- **Offline Lookup**: ~1ms (dictionary lookup)
- **API Translation**: ~500-2000ms (network dependent)
- **Cache Hit**: ~1ms (cached response)
- **Bundle Size**: +~5KB gzipped (minimal impact)

## Browser Compatibility

- Modern browsers with `localStorage` support
- Fallback for private browsing (no error, just no persistence)
- Works offline for dictionary words

## Limitations & Future Improvements

### Current Limitations
1. Manual offline dictionary (not auto-generated)
2. Single language pair (en→ur only)
3. Word-level translations (not context-aware)
4. No translation memory persistence

### Future Improvements
- [ ] Add more language pairs (en→ur→en bidirectional)
- [ ] Build translation memory from user corrections
- [ ] Context-aware translation (simple ML model)
- [ ] Support for right-to-left (RTL) text layout
- [ ] Performance: Batch API translations
- [ ] Analytics: Track which terms need improvement

## Troubleshooting

### Translation not appearing
- **Check**: Is `isReady` true in TranslationContext?
- **Check**: Is the text in `commonTranslations` dictionary?
- **Check**: Is MyMemory API accessible (no CORS issues)?

### Language preference not persisting
- **Cause**: localStorage unavailable (private browsing)
- **Solution**: Clear browser data and try again (private mode limitation)

### Components not translating
- **Check**: Is component wrapped with `TranslationProvider`?
- **Check**: Is component using `useTranslation()` hook?
- **Check**: Is `TranslatableText` used or is content directly rendered?

### API timeouts
- **Cause**: Network slow or MyMemory API down
- **Fallback**: Component returns original English text
- **Monitor**: Browser console logs `[TRANSLATION] Error...`

## Contributing

To enhance the translation feature:

1. **Add Dictionary Words**: Edit `src/lib/translator.ts`
2. **Improve Components**: Update `src/components/LanguageSwitcher/*`
3. **Fix Bugs**: Report issues on GitHub
4. **Add Tests**: Expand `src/tests/translation.test.ts`

## References

- **MyMemory API Docs**: https://mymemory.translated.net/api/documentation
- **React Context**: https://react.dev/reference/react/useContext
- **Docusaurus SSR**: https://docusaurus.io/docs/guide

---

**Last Updated**: December 10, 2025  
**Feature Status**: ✅ Production Ready
