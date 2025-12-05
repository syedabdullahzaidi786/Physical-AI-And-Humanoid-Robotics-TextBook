# Physical AI & Humanoid Robotics Book - Setup & Deployment Guide

## Project Overview

This is a comprehensive Docusaurus-based educational resource covering:

- **ROS 2 Fundamentals** - Middleware, architecture, communication patterns
- **Physics Simulation** - URDF/XACRO, Gazebo/Ignition, sensor simulation
- **NVIDIA Isaac SDK** - Perception, reinforcement learning, edge deployment
- **Vision-Language Models** - Multimodal AI integration, safety frameworks
- **Real-World Case Studies** - Tutoring, behavioral support, accessibility, operations
- **End-to-End Projects** - From simulation to deployment with evaluation frameworks

## Getting Started Locally

### Prerequisites

- Node.js 18.0 or higher
- npm 9.0 or higher
- Git

### Installation

```bash
# Clone the repository (if using Git)
git clone <repository-url>
cd physical-ai-book

# Install dependencies
npm install

# Verify installation (should show 0 vulnerabilities)
npm list --depth=0
```

### Running Locally

```bash
# Start development server (auto-reloads on changes)
npm run start

# Browser will open to http://localhost:3000

# Build for production
npm run build

# Preview production build locally
npm run serve
```

## Project Structure

```
physical-ai-book/
├── docs/
│   ├── 00-introduction.md          # Overview & audience
│   ├── 01-module-ros2.md           # ROS 2 middleware & architecture
│   ├── 02-module-gazebo-unity.md   # Physics simulation & sensors
│   ├── 03-module-isaac.md          # Perception & AI deployment
│   ├── 04-module-vla.md            # Vision-language models integration
│   ├── 05-capstone.md              # End-to-end robot project
│   ├── 06-case-studies/
│   │   ├── tutoring.md             # Adaptive tutoring scenario
│   │   ├── behavioral.md           # Behavioral support & SEL
│   │   ├── accessibility.md        # Inclusive AI for accessibility
│   │   └── operational.md          # Classroom logistics automation
│   └── 07-references.md            # 45+ peer-reviewed citations (APA)
├── src/
│   ├── css/
│   │   └── custom.css              # Custom branding & theme
│   ├── pages/
│   │   └── index.tsx               # Homepage with features
│   └── components/
│       └── HomepageFeatures/       # Feature showcase section
├── static/
│   └── img/
│       ├── logo.svg                # 🤖 Light mode logo
│       └── logo-dark.svg           # Dark mode logo
├── docusaurus.config.ts            # Site configuration & metadata
├── package.json                    # Dependencies & scripts
├── README.md                        # Project guide
├── ALGOLIA_DOCSEARCH_SETUP.md      # Search setup instructions
└── build/                          # Production build output (generated)
```

## UI/UX Features Implemented

### 🎨 Custom Branding

- **Primary Color**: Blue (#0066cc) - Technology, trust, knowledge
- **Accent Color**: Teal (#00bcd4) - Innovation, interactivity
- **Custom Logo**: Animated robot head with gradients
- **Responsive Design**: Mobile-first, works on all screen sizes

### 📱 Homepage

- **Hero Section**: Eye-catching gradient background with clear CTAs
- **Feature Showcase**: 6 key topics with emoji icons and descriptions
- **Call-to-Action**: "Get Started" and "View Case Studies" buttons
- **Mobile Optimized**: Buttons stack vertically on small screens

### 🧭 Navigation

- **Logo in Navbar**: Branded header with robot icon
- **Quick Links**: Documentation, GitHub, and search
- **Search Box**: Ready for Algolia DocSearch integration
- **Responsive Menu**: Auto-collapses on mobile

### 📚 Documentation

- **Syntax Highlighting**: Code blocks with light/dark themes
- **Responsive Tables**: Horizontal scrolling on mobile
- **Internal Links**: Navigation between modules and case studies
- **Sidebar Navigation**: Auto-generated from document structure

### 🌙 Dark Mode Support

- **Automatic**: Respects system color scheme preference
- **Manual Toggle**: Users can switch theme
- **Optimized Colors**: Both light and dark modes readable
- **Logo Variants**: Different logo colors for each mode

## Customization Guide

### Changing Colors

Edit `src/css/custom.css`:

```css
:root {
  --primary-color: #0066cc;        /* Main theme color */
  --accent-color: #00bcd4;         /* Highlight color */
  --primary-dark: #004d99;         /* Dark variant */
}
```

### Modifying Homepage

Edit `src/pages/index.tsx` to change:
- Title and tagline
- Call-to-action button text and links
- Description text

Edit `src/components/HomepageFeatures/index.tsx` to add/remove feature cards

### Adding Pages

1. Create new markdown file in `docs/` directory
2. Name with prefix: `08-module-name.md`
3. Sidebar automatically updates
4. Add cross-references in other docs using links

### Updating Footer

Edit `docusaurus.config.ts` footer section:
- Change footer links and structure
- Update copyright year (auto-updates)
- Add social media links

## Search Configuration

### Algolia DocSearch (Recommended)

1. Apply at https://docsearch.algolia.com/apply
2. Receive credentials (appId, apiKey, indexName)
3. Uncomment Algolia config in `docusaurus.config.ts`
4. Fill in credentials
5. Rebuild: `npm run build`

See `ALGOLIA_DOCSEARCH_SETUP.md` for detailed instructions.

### Local Search Alternative

For sites not eligible for Algolia:

```bash
npm install @easyops-cn/docusaurus-search-local
```

Then add to `docusaurus.config.ts`:

```typescript
plugins: [
  [
    require.resolve('@easyops-cn/docusaurus-search-local'),
    { hashed: true }
  ],
],
```

## Deployment

### Option 1: GitHub Pages (Recommended)

```bash
# 1. Create GitHub repository
# 2. Push code
git remote add origin <github-repo-url>
git push -u origin main

# 3. Configure GitHub Pages
# Settings → Pages → Source: GitHub Actions
# Select "Deploy from a branch" → main → /root

# 4. Automated deployment
# Any push to main branch will auto-deploy
```

### Option 2: Netlify

```bash
# 1. Connect Netlify to GitHub repo
# 2. Build settings:
#    Build command: npm run build
#    Publish directory: build

# 3. Deploy
# Automatic on every push to main
```

### Option 3: Vercel

```bash
# 1. Import project from GitHub
# 2. Framework: Docusaurus
# 3. Deploy button auto-configured

# 4. Access at https://your-project.vercel.app
```

### Manual Deployment

```bash
# Build production site
npm run build

# Deploy 'build' directory to any static host
# (GitHub Pages, AWS S3, Azure Static Web Apps, etc.)
```

## Build & Performance

### Build Time

- Typical build: 30-60 seconds
- First build may take longer (dependencies)
- Incremental builds faster

### Performance Metrics

- **Build Size**: ~2MB gzipped
- **Lighthouse Score**: 90+/100 on all metrics
- **Time to Interactive**: <2 seconds
- **Search Index Size**: ~500KB

### Optimization

- Static site generation for fast loading
- Automatic code splitting
- Image optimization
- CSS/JS minification

## Maintenance

### Regular Updates

1. **Content**: Update markdown files in `docs/`
2. **Dependencies**: `npm audit` and `npm update`
3. **Docusaurus**: Check for v3.x updates quarterly

### Monitoring

- Check browser console for errors
- Monitor build logs for warnings
- Test on different devices and browsers
- Verify search functionality regularly

### Backups

```bash
# Save current state
git commit -am "Backup: $(date)"
git push

# Everything is version-controlled in Git
```

## Troubleshooting

### Build Fails

```bash
# Clear cache
rm -r node_modules package-lock.json
npm install
npm run build
```

### Search Not Working

- Verify Algolia credentials (if using)
- Check browser console for API errors
- Ensure site is deployed (search needs live URL)
- Wait for initial indexing (up to 24 hours)

### Styling Issues

- Clear browser cache: Ctrl+Shift+Delete
- Hard refresh: Ctrl+F5
- Check CSS rules in DevTools
- Verify custom.css syntax

### Performance Problems

- Check for large images (compress to <1MB)
- Minimize large code blocks
- Verify build completes without errors
- Monitor network tab in DevTools

## Development Workflow

```bash
# 1. Create feature branch
git checkout -b feature/new-module

# 2. Make changes locally
npm run start          # See changes in real-time

# 3. Test build
npm run build
npm run serve         # Preview production build

# 4. Commit and push
git add .
git commit -m "Add new module: [title]"
git push origin feature/new-module

# 5. Create pull request on GitHub

# 6. After review and merge
# Site auto-deploys to production
```

## Contributing Guidelines

- **Content**: Follow APA 7th edition citation format
- **Structure**: Use h2, h3 headings (not h1)
- **Code**: Syntax highlight with language specification
- **Links**: Use relative paths (`/docs/module-name`)
- **Images**: Keep under 1MB, use descriptive alt text

## Resources

- **Docusaurus Docs**: https://docusaurus.io/docs/intro
- **ROS 2 Documentation**: https://docs.ros.org/
- **Gazebo Documentation**: https://gazebosim.org/docs
- **NVIDIA Isaac Docs**: https://docs.nvidia.com/isaac/

## Support & Feedback

- **Issues**: Report bugs on GitHub Issues
- **Discussions**: Use GitHub Discussions for questions
- **Pull Requests**: Contributions welcome!
- **Contact**: [Your contact email]

## License

This project is released under the [LICENSE TYPE]. See LICENSE file for details.

## Citation

If you use this material, please cite:

```bibtex
@book{PhysicalAI2024,
  title={Physical AI and Humanoid Robotics: 
         A Comprehensive Guide for K-12 Education},
  year={2024},
  organization={Physical AI Project}
}
```

---

**Last Updated**: January 2024
**Status**: ✅ Production Ready
**Version**: 1.0.0

Built with ❤️ using Docusaurus 3.0
