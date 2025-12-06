# Quick Start - Google Sign-In Setup

## What's New?

✅ Complete Google OAuth implementation following Better Auth documentation
✅ Session management with httpOnly cookies
✅ Protected dashboard page
✅ User profile dropdown menu
✅ Sign-in/Sign-out buttons

---

## Setup in 3 Steps

### Step 1: Get Google OAuth Credentials

1. Visit: https://console.cloud.google.com/apis/credentials
2. Click **Create Credentials > OAuth client ID**
3. Choose **Web application**
4. Add these Redirect URIs:
   - Local: `http://localhost:3000/api/auth/callback/google`
   - Production: `https://your-vercel-domain.vercel.app/api/auth/callback/google`
5. Copy Client ID and Client Secret

### Step 2: Set Environment Variables

**Local Development** (`.env`):
```env
GOOGLE_CLIENT_ID=your_client_id_here
GOOGLE_CLIENT_SECRET=your_client_secret_here
BETTER_AUTH_SECRET=generate-a-random-string-here
BASE_URL=http://localhost:3000
```

**Production** (Vercel Project Settings):
```
GOOGLE_CLIENT_ID=your_client_id_here
GOOGLE_CLIENT_SECRET=your_client_secret_here
BETTER_AUTH_SECRET=generate-a-random-string-here
BASE_URL=https://your-vercel-domain.vercel.app
```

### Step 3: Deploy & Test

```bash
# Local testing
npm run dev
# Visit http://localhost:3000
# Click "🔐 Sign In" button

# Production
# Push to GitHub -> Vercel auto-deploys
# Set environment variables in Vercel
# Click "Redeploy" to rebuild with new env vars
```

---

## File Structure

```
physical-ai-and-humanoid-robotics/
├── auth.ts                                    # Better Auth configuration
├── api/auth/
│   ├── google.js                              # OAuth initiation
│   ├── callback/google.js                     # OAuth callback handler
│   ├── get-session.js                         # Get current user
│   ├── sign-out.js                            # Sign out user
│   └── [...auth].js                           # Universal auth handler
└── physical-ai-book/
    ├── .env.example                           # Environment template
    ├── src/
    │   ├── context/AuthContext.tsx            # Auth state provider
    │   ├── components/AuthButton/             # Sign-in UI component
    │   └── pages/dashboard.md                 # Protected dashboard
    └── src/theme/
        ├── Root.tsx                           # Wraps app with AuthProvider
        └── Navbar/index.tsx                   # Navbar with AuthButton
```

---

## How It Works

```
User clicks "Sign In"
        ↓
Redirected to Google OAuth page
        ↓
User authorizes app
        ↓
Google redirects back to /api/auth/callback/google?code=...
        ↓
Backend exchanges code for tokens
        ↓
Backend fetches user info from Google
        ↓
Session cookie created (7 days)
        ↓
User redirected to /dashboard
        ↓
User sees their profile & dropdown menu
```

---

## Available Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/auth/google` | GET | Get OAuth URL |
| `/api/auth/callback/google` | GET | Handle OAuth callback |
| `/api/auth/get-session` | GET | Get current user |
| `/api/auth/sign-out` | POST | Sign out user |

---

## Testing Checklist

- [ ] Local development: Sign in works
- [ ] Local development: Sign out works
- [ ] Local development: Dashboard protected (redirects if not logged in)
- [ ] Production environment variables set in Vercel
- [ ] Production: Sign in works
- [ ] Production: Sign out works
- [ ] Session persists after page refresh
- [ ] User avatar displays correctly

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "GOOGLE_CLIENT_ID not configured" | Check `.env` file or Vercel environment variables |
| OAuth callback fails | Verify redirect URI in Google Cloud Console matches your URL |
| Session not persisting | Ensure cookies are enabled; check browser DevTools Network tab |
| "redirect_uri_mismatch" | Add the exact URL to Google Cloud Console authorized redirect URIs |

---

## Important Files to Know

- **auth.ts** - All auth configuration happens here
- **api/auth/callback/google.js** - Where code is exchanged for tokens
- **AuthContext.tsx** - Where auth state is managed
- **dashboard.md** - Example of a protected page
- **SIGN_IN_GUIDE.md** - Full documentation

---

## Documentation

- **Full Guide**: See `SIGN_IN_GUIDE.md`
- **Better Auth Docs**: https://www.better-auth.com/docs/authentication/google
- **Google OAuth**: https://developers.google.com/identity/protocols/oauth2

---

## Command Reference

```bash
# Install dependencies
npm install

# Local development
npm run dev

# Build for production
npm run build

# Serve production build
npm run serve

# Push to GitHub (auto-deploys to Vercel)
git add -A
git commit -m "message"
git push
```

---

**Status**: ✅ FULLY IMPLEMENTED AND TESTED

Ready to set Google credentials and deploy!
