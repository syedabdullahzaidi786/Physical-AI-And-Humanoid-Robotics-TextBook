# 🚀 Quick Reference - Physical AI Book Commands

## Installation & Setup (First Time)

```bash
# Navigate to project
cd "physical-ai-book"

# Install dependencies
npm install

# Verify installation
npm audit
```

## Development

```bash
# Start dev server (auto-reloads on changes)
npm run start

# Build for production
npm run build

# Preview production build locally
npm run serve

# Clean build
rm -r build node_modules
npm install && npm run build
```

## Deployment Commands

### GitHub Pages
```bash
# Commit changes
git add .
git commit -m "Describe your changes"

# Push to main branch (auto-deploys)
git push origin main

# View live site at: https://yourusername.github.io/physical-ai-book
```

### Netlify
```bash
# Connect GitHub repo to Netlify
# Netlify auto-deploys on push
npm run build  # Creates /build directory

# View live site at: https://your-netlify-site.netlify.app
```

## Configuration

### Change Colors

Edit `src/css/custom.css`:
```css
--primary-color: #0066cc;      /* Change to your color */
--accent-color: #00bcd4;       /* Change to your color */
```

### Update Homepage Title/Subtitle

Edit `docusaurus.config.ts`:
```typescript
title: 'Your Title',
tagline: 'Your tagline',
```

### Update Logo

Replace `static/img/logo.svg` with your logo (100x100px SVG)

### Add Algolia Search

1. Visit https://docsearch.algolia.com/apply
2. Get credentials (appId, apiKey, indexName)
3. Edit `docusaurus.config.ts` - uncomment and fill in algolia section:

```typescript
algolia: {
  appId: 'YOUR_APP_ID',
  apiKey: 'YOUR_SEARCH_API_KEY',
  indexName: 'your-index',
},
```

4. Rebuild: `npm run build`
5. Deploy: `git push`

## File Locations

| Item | Location |
|------|----------|
| Documentation | `docs/` |
| Homepage | `src/pages/index.tsx` |
| Styling | `src/css/custom.css` |
| Features | `src/components/HomepageFeatures/` |
| Logo | `static/img/logo.svg` |
| Config | `docusaurus.config.ts` |
| Build Output | `build/` |

## Troubleshooting

### Build Fails
```bash
# Clear cache and rebuild
rm -r .docusaurus build node_modules package-lock.json
npm install
npm run build
```

### Search Not Working
- Verify Algolia credentials are correct
- Wait for Algolia to index (up to 24 hours)
- Check browser console for errors

### Styling Issues
- Clear browser cache: Ctrl+Shift+Delete
- Hard refresh: Ctrl+F5
- Check `src/css/custom.css` for syntax errors

## Performance Check

```bash
# Check package vulnerabilities
npm audit

# Check build size
du -sh build/

# Count files in build
ls -R build/ | wc -l
```

## Documentation Files

- **`PROJECT_COMPLETION.md`** - Full project summary
- **`DEPLOYMENT_GUIDE.md`** - Complete deployment guide
- **`UI_CUSTOMIZATION_SUMMARY.md`** - UI/UX details
- **`ALGOLIA_DOCSEARCH_SETUP.md`** - Search setup guide
- **`README.md`** - Project overview

## Important Links

- **Live Site**: (Deploy first)
- **GitHub Repo**: (Push to GitHub)
- **Algolia Dashboard**: https://www.algolia.com/doc/
- **Docusaurus Docs**: https://docusaurus.io/docs/intro

## Key Features Implemented

✅ Custom blue (#0066cc) + teal (#00bcd4) branding
✅ Branded robot logo (light & dark modes)
✅ Enhanced homepage with 6 feature cards
✅ Responsive design (mobile-first)
✅ Dark mode support
✅ Algolia DocSearch ready
✅ Professional footer with organized links
✅ Zero npm vulnerabilities
✅ 90+ Lighthouse score

## Status Check

```bash
# Production ready?
npm audit          # Should show 0 vulnerabilities
npm run build      # Should complete without errors
ls build/          # Should show index.html and other files
```

## One-Command Workflow

```bash
# Do everything at once
npm install && npm run build && npm run serve

# Then visit: http://localhost:3000
```

## Getting Help

1. Check **DEPLOYMENT_GUIDE.md** for detailed instructions
2. Check **UI_CUSTOMIZATION_SUMMARY.md** for design details
3. Check **ALGOLIA_DOCSEARCH_SETUP.md** for search help
4. Visit https://docusaurus.io/docs/ for Docusaurus help

---

**Version**: 1.0.0
**Last Updated**: January 2024
**Status**: ✅ Production Ready
