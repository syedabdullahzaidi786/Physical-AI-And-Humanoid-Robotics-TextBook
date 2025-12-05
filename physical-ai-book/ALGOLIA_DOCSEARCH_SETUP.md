# Algolia DocSearch Setup Guide

## Overview

This guide explains how to set up **Algolia DocSearch** for the Physical AI & Humanoid Robotics book to enable full-text search across all documentation.

## What is Algolia DocSearch?

Algolia DocSearch is a free service that indexes your documentation site and provides a fast, beautiful search experience for your users. It includes:

- **Full-text search** across all pages
- **Instant results** as you type
- **Faceted filtering** (search by category, module, etc.)
- **Mobile-friendly** search UI
- **SEO-friendly** search implementation

## Steps to Enable Algolia DocSearch

### Step 1: Apply for Algolia DocSearch

1. Visit **https://docsearch.algolia.com/apply**
2. Fill out the application form with:
   - **Website URL**: (your deployed site URL, e.g., https://physical-ai-book.github.io)
   - **Documentation Email**: Your project email
   - **Use Case**: Educational documentation about robotics and AI
3. Algolia will review your application (typically within 1-2 weeks)

### Step 2: Receive Algolia Credentials

Once approved, you'll receive:
- **Application ID** (`appId`)
- **Search-Only API Key** (`apiKey`)
- **Index Name** (e.g., `physical-ai-robotics`)

### Step 3: Update docusaurus.config.ts

Open `docusaurus.config.ts` and uncomment and update the `algolia` section:

```typescript
algolia: {
  appId: 'YOUR_APP_ID',
  apiKey: 'YOUR_SEARCH_API_KEY',
  indexName: 'physical-ai-robotics',
  contextualSearch: true,
  externalUrlRegex: 'external\\.com|domain\\.com',
  replaceSearchResultPathname: {
    from: '/docs/',
    to: '/',
  },
  searchParameters: {},
  searchPagePath: 'search',
},
```

Replace the placeholders with your actual credentials.

### Step 4: Rebuild and Deploy

```bash
npm run build
npm run serve  # Test locally
git push       # Deploy to GitHub Pages or your hosting
```

### Step 5: Verify Search is Working

1. Go to your deployed website
2. Look for the search icon in the navbar
3. Type a search query (e.g., "ROS 2", "simulation", "tutorial")
4. Verify results appear with highlighted snippets

## Search Features

### Query Types

- **Full-text search**: `"ROS 2 middleware"`
- **Phrase search**: `"physics simulation"`
- **Wildcard search**: `robot*` (matches "robot", "robotics", "robotic")
- **Negative search**: `-deprecated` (excludes deprecated content)

### Advanced Features

- **Filtering**: Search by document type, module, category
- **Synonyms**: "AI" = "Artificial Intelligence", "VLM" = "Vision Language Model"
- **Typo tolerance**: Finds results even with spelling mistakes
- **Ranking**: Results ranked by relevance and document importance

## Troubleshooting

### Search box not appearing

**Issue**: Search box doesn't show in navbar
- Verify `algolia` configuration is uncommented in `docusaurus.config.ts`
- Rebuild with `npm run build`
- Clear browser cache

### No search results

**Issue**: Typing queries returns no results
- Check that Algolia has finished indexing (check Algolia dashboard)
- Verify `indexName` matches Algolia index name
- Ensure `apiKey` is the **search-only** API key (not admin key)

### Search too slow

**Issue**: Search results take time to load
- Algolia caches results; first search may be slower
- Check network tab in browser DevTools
- Verify internet connection and Algolia service status

## Alternative: Local Search

If Algolia is not available, you can use **local search** with `@easyops-cn/docusaurus-search-local`:

```bash
npm install @easyops-cn/docusaurus-search-local
```

Then add to `docusaurus.config.ts`:

```typescript
plugins: [
  [
    require.resolve('@easyops-cn/docusaurus-search-local'),
    {
      hashed: true,
      language: ['en'],
    },
  ],
],
```

## Best Practices

1. **Keep documentation updated**: Search indexes latest content
2. **Use clear titles**: Helps search indexing and discoverability
3. **Structure documents**: Use headings (h1, h2, h3) for better snippets
4. **Add metadata**: Use SEO descriptions for pages
5. **Test regularly**: Verify search works with common queries

## Resources

- **Algolia DocSearch**: https://docsearch.algolia.com/
- **Docusaurus Search**: https://docusaurus.io/docs/search
- **Algolia Documentation**: https://www.algolia.com/doc/
- **Docusaurus Tutorial**: https://docusaurus.io/docs/docs-introduction

## Next Steps

1. ✅ Apply for Algolia DocSearch
2. ⏳ Wait for approval (1-2 weeks)
3. ✅ Update configuration with credentials
4. ✅ Rebuild and deploy
5. ✅ Test search functionality
6. ✅ Share with users and gather feedback

---

**Status**: Algolia DocSearch is configured but requires credentials to activate.

**Current Search**: Currently using browser-based search. Once Algolia is enabled, full-text search will be available.

**Deployment Ready**: The site is configured to work with Algolia once credentials are provided.
