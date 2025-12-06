# Sign-In Flow Debugging Guide

## Overview
The sign-in feature uses Google OAuth 2.0 with a Vercel serverless backend. This guide helps identify where the flow might be breaking.

## Sign-In Flow Steps

```
1. User clicks "Sign In" button
   ↓
2. Browser logs: [AUTH] signIn() called
   ↓
3. Fetch `/api/auth/google` → returns OAuth URL
   Browser logs: [AUTH] /api/auth/google response: 200
   Server logs: [OAUTH] /api/auth/google - clientId present: true
   ↓
4. Redirect to Google OAuth
   Browser logs: [AUTH] Redirecting to Google OAuth...
   ↓
5. User signs in with Google
   ↓
6. Google redirects to `/api/auth/callback/google?code=...`
   Server logs: [OAUTH] /api/auth/callback/google - received callback
   ↓
7. Exchange code for tokens
   Server logs: [OAUTH] Token exchange successful
   ↓
8. Fetch user info from Google
   Server logs: [OAUTH] User info fetched from Google
   ↓
9. Create session cookie
   Server logs: [OAUTH] Session cookie created for user: [email]
   ↓
10. Redirect to /dashboard
    Server logs: [OAUTH] /api/auth/callback/google - redirecting to /dashboard
    ↓
11. Page loads & AuthContext checks session
    Browser logs: [AUTH] /api/auth/get-session response: 200
    Browser logs: [AUTH] User restored: [email]
```

## How to Debug

### Step 1: Open Browser DevTools
1. Press `F12` or `Ctrl+Shift+I` (Windows/Linux) or `Cmd+Option+I` (Mac)
2. Go to **Console** tab
3. Leave it open while testing sign-in

### Step 2: Check Client-Side Logs
When you click "Sign In", you should see:

```
[AUTH] signIn() called - fetching OAuth URL
[AUTH] /api/auth/google response: 200
[AUTH] OAuth URL received: true
[AUTH] Redirecting to Google OAuth...
```

**If you don't see these**, the button click handler might not be working:
- Check that the button has the `onClick` event
- Check for JavaScript errors in the console (red error messages)

### Step 3: Check Server Logs (Vercel)
If deployed on Vercel:
1. Go to https://vercel.com → select your project
2. Click **Deployments** → select the latest deployment
3. Click **Runtime Logs**
4. Search for `[OAUTH]` to see server-side logs

If running locally:
1. The dev server should print logs to your terminal

**Expected server logs when clicking sign-in:**
```
[OAUTH] /api/auth/google - baseUrl: http://localhost:3000
[OAUTH] /api/auth/google - clientId present: true
[OAUTH] /api/auth/google - redirectUri: http://localhost:3000/api/auth/callback/google
[OAUTH] /api/auth/google - returning authUrl
```

### Step 4: Verify Environment Variables
Check that these are set in `.env.local` (development) or Vercel (production):

```bash
# Required for sign-in
GOOGLE_CLIENT_ID=your_client_id
GOOGLE_CLIENT_SECRET=your_client_secret
BASE_URL=http://localhost:3000
```

**To check in production (Vercel):**
1. Go to Vercel → Project Settings → Environment Variables
2. Verify all three variables are present

### Step 5: Verify Google OAuth Configuration
1. Go to https://console.cloud.google.com/apis/credentials
2. Click your OAuth 2.0 Client ID (Web application)
3. Verify:
   - **Authorized Redirect URIs** includes:
     - Local: `http://localhost:3000/api/auth/callback/google`
     - Production: `https://your-domain.vercel.app/api/auth/callback/google`
   - **Authorized JavaScript Origins** includes:
     - Local: `http://localhost:3000`
     - Production: `https://your-domain.vercel.app`

## Common Issues & Solutions

### Issue 1: "Sign In button doesn't redirect"
**Symptom:** Click button, nothing happens

**Solution:**
1. Check browser console for errors
2. Verify `/api/auth/google` endpoint is accessible:
   ```bash
   curl http://localhost:3000/api/auth/google
   # Should return: {"url": "https://accounts.google.com/o/oauth2/v2/auth?..."}
   ```
3. Check `GOOGLE_CLIENT_ID` environment variable is set

### Issue 2: "OAuth redirect works, then blank page"
**Symptom:** Redirects to Google, signs in, then shows blank page

**Possible causes:**
- Callback handler error (check `/api/auth/callback/google`)
- Environment variables not set on Vercel
- Session cookie not being set correctly

**Debug:**
1. Check Vercel Runtime Logs for `[OAUTH] OAuth callback error`
2. Verify `GOOGLE_CLIENT_SECRET` is set on Vercel
3. Check that `/dashboard` page exists

### Issue 3: "Signed in but button still shows 'Sign In'"
**Symptom:** OAuth completes, but button doesn't show user name

**Possible causes:**
- Session cookie not being read by `getSession` endpoint
- `AuthContext` not restoring user on page load

**Debug:**
1. Open DevTools Console, look for:
   ```
   [AUTH] /api/auth/get-session response: 200
   [AUTH] User restored: [email]
   ```
2. If response is 401, session cookie wasn't set or expired
3. Check browser cookies (DevTools → Application → Cookies)
   - Should have a `session` cookie for your domain

### Issue 4: "GOOGLE_CLIENT_ID not configured error"
**Symptom:** Console shows: "GOOGLE_CLIENT_ID not configured"

**Solution:**
1. **Local Development:**
   ```bash
   # Create .env.local in physical-ai-book/ directory
   cp .env.example .env.local
   # Edit .env.local and add your credentials
   ```

2. **Production (Vercel):**
   - Go to Vercel → Project Settings → Environment Variables
   - Add `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`, `BASE_URL`
   - Redeploy

## Testing Locally

### Setup Steps
```bash
cd physical-ai-book

# 1. Create .env.local with your credentials
cp .env.example .env.local
# Edit .env.local:
# GOOGLE_CLIENT_ID=your_client_id_here
# GOOGLE_CLIENT_SECRET=your_client_secret_here
# BASE_URL=http://localhost:3000

# 2. Start dev server
npm run start

# 3. Open browser to http://localhost:3000
# 4. Click "Sign In" button
# 5. Check Console for [AUTH] logs
```

### Verifying API Endpoints
```bash
# Test /api/auth/google endpoint
curl -X GET http://localhost:3000/api/auth/google

# Should return:
# {"url":"https://accounts.google.com/o/oauth2/v2/auth?response_type=code&client_id=..."}
```

## Log Locations

### Browser Console Logs
- Open DevTools → Console tab
- Filter by `[AUTH]` prefix
- Shows: client-side auth flow

### Server Logs
- **Local:** Check terminal/PowerShell where you ran `npm run start`
- **Vercel:** Vercel → Deployments → Runtime Logs, search `[OAUTH]`

## Useful Environment Variables for Debugging

Add to `.env.local` for more verbose output:

```bash
# Enable Docusaurus debug mode
DEBUG=*

# Node environment (keeps detailed error messages)
NODE_ENV=development
```

## Still Not Working?

1. **Collect all logs:**
   - Browser console logs (screenshot or paste)
   - Vercel runtime logs (if deployed)
   - `.env.local` file (environment variables set, don't paste secrets)

2. **Provide:**
   - Steps to reproduce
   - Error message (if any)
   - Browser console output (all [AUTH] and [OAUTH] logs)
   - Whether running locally or on Vercel

3. **Common things to verify:**
   - [ ] GOOGLE_CLIENT_ID environment variable is set
   - [ ] GOOGLE_CLIENT_SECRET environment variable is set
   - [ ] BASE_URL matches your domain
   - [ ] Google OAuth redirect URI is registered in Google Cloud Console
   - [ ] JavaScript errors in browser console (red messages)
   - [ ] Network tab shows `/api/auth/google` returning 200 status

## Architecture

```
Client (Browser)
  ↓
AuthContext.tsx (manages user state, sign-in/out)
  ↓ fetch
API Routes (Vercel serverless)
  ├── /api/auth/google.js (returns Google OAuth URL)
  ├── /api/auth/callback/google.js (token exchange, creates session)
  └── /api/auth/get-session.js (returns current user)
  ↓
Google OAuth
  ├── accounts.google.com (user signs in)
  └── Redirects back to /api/auth/callback/google with code
```

Session is stored as:
- **Frontend:** React Context + localStorage (optional)
- **Backend:** HttpOnly Cookie (secure, prevents XSS)

Cookie format: Base64-encoded JSON user object
- Expires: 7 days (604800 seconds)
- Secure: HttpOnly + SameSite=Lax
