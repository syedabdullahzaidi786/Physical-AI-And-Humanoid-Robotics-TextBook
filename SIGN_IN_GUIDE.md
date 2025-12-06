# Sign-In Implementation Guide

## Overview

The authentication system has been fully implemented following the **Better Auth Google OAuth documentation**. The system provides Google Sign-In functionality with session management and protected routes.

**Official Documentation Reference:** https://www.better-auth.com/docs/authentication/google#sign-in-with-google

---

## Architecture

### Server-Side Components

#### 1. **auth.ts** - Better Auth Configuration
```typescript
// Initializes Better Auth with Google OAuth provider
export const auth = betterAuth({
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
      prompt: "select_account",
      accessType: "offline",
    },
  },
  baseURL: process.env.BASE_URL,
});
```

**Key Features:**
- Google OAuth provider configuration
- `prompt: "select_account"` - Always ask user to select account
- `accessType: "offline"` - Get refresh token
- Automatic database management

#### 2. **API Routes Structure**

```
/api/auth/
├── google.js                  # Returns OAuth URL
├── callback/google.js         # Handles OAuth callback
├── get-session.js             # Fetches current session
├── sign-out.js                # Clears session
└── [...auth].js               # Universal auth handler
```

### Client-Side Components

#### 1. **AuthContext.tsx** - React Context Provider
- Global auth state management
- Session fetching on app mount
- Sign-in and sign-out handlers
- User data storage

#### 2. **AuthButton Component**
- UI for signing in/out
- User profile dropdown menu
- Avatar display with fallback

#### 3. **Root.tsx** - App Wrapper
- Wraps entire app with `AuthProvider`
- Ensures all components have access to auth context

---

## OAuth Flow

### Step 1: User Clicks Sign In Button

```typescript
const signIn = async () => {
  const response = await fetch('/api/auth/google');
  const { url } = await response.json();
  window.location.href = url;
};
```

### Step 2: Redirect to Google OAuth

User is sent to: `https://accounts.google.com/o/oauth2/v2/auth?...`

### Step 3: User Authorizes App

Google redirects back to: `/api/auth/callback/google?code=...&state=...`

### Step 4: Backend Exchanges Code for Tokens

```javascript
// callback/google.js
const tokenResponse = await fetch('https://oauth2.googleapis.com/token', {
  method: 'POST',
  body: {
    grant_type: 'authorization_code',
    code,
    client_id,
    client_secret,
    redirect_uri,
  },
});
```

### Step 5: Fetch User Info

```javascript
const userResponse = await fetch(
  'https://www.googleapis.com/oauth2/v2/userinfo',
  { headers: { Authorization: `Bearer ${access_token}` } }
);
```

### Step 6: Create Session Cookie

```javascript
res.setHeader(
  'Set-Cookie',
  `session=${sessionCookie}; HttpOnly; Max-Age=604800; SameSite=Lax`
);
```

### Step 7: Redirect to Dashboard

User is now authenticated and logged in.

---

## Session Management

### Session Cookie Structure

```javascript
{
  id: "user_id",
  email: "user@example.com",
  name: "User Name",
  image: "https://..."
}
```

**Cookie Settings:**
- **HttpOnly**: Not accessible via JavaScript (secure)
- **Max-Age**: 604800 seconds (7 days)
- **SameSite**: Lax (prevent CSRF attacks)
- **Path**: / (accessible site-wide)

### Session Validation

On app mount, `AuthContext` calls `/api/auth/get-session`:

```javascript
const response = await fetch('/api/auth/get-session', {
  credentials: 'include', // Include cookies in request
});
```

If valid session exists, user data is restored automatically.

---

## Protected Pages

### Dashboard Page

```typescript
// src/pages/dashboard.md
const { user, isLoading } = useAuth();

if (!user) {
  return <Redirect to="/" />;
}

return <div>Welcome, {user.name}!</div>;
```

Only authenticated users can access the dashboard. Unauthenticated users are redirected to home.

---

## Environment Variables Required

### For Local Development (.env)

```env
# Google OAuth (from Google Cloud Console)
GOOGLE_CLIENT_ID=YOUR_CLIENT_ID
GOOGLE_CLIENT_SECRET=YOUR_CLIENT_SECRET

# Better Auth Secret (generate a random string)
BETTER_AUTH_SECRET=your-secret-key-change-this

# Application Base URL
BASE_URL=http://localhost:3000
```

### For Production (Vercel)

Set in **Project Settings > Environment Variables**:

```env
GOOGLE_CLIENT_ID=YOUR_PRODUCTION_CLIENT_ID
GOOGLE_CLIENT_SECRET=YOUR_PRODUCTION_CLIENT_SECRET
BETTER_AUTH_SECRET=your-production-secret-key
BASE_URL=https://your-domain.vercel.app
```

---

## Google Cloud Console Setup

### 1. Create OAuth 2.0 Credentials

1. Go to https://console.cloud.google.com/apis/credentials
2. Click **Create Credentials > OAuth client ID**
3. Choose **Web application**

### 2. Configure Authorized Redirect URIs

**For Local Development:**
```
http://localhost:3000/api/auth/callback/google
```

**For Production:**
```
https://your-domain.vercel.app/api/auth/callback/google
```

### 3. Configure Authorized JavaScript Origins

**For Local Development:**
```
http://localhost:3000
```

**For Production:**
```
https://your-domain.vercel.app
```

### 4. Copy Credentials

Copy the Client ID and Client Secret into your `.env` file.

---

## Testing Locally

### 1. Install Dependencies

```bash
npm install
```

### 2. Set Environment Variables

Create `.env` file with Google OAuth credentials:

```bash
cp .env.example .env
# Edit .env and add your Google credentials
```

### 3. Run Development Server

```bash
npm run dev
```

### 4. Test Sign-In

1. Open http://localhost:3000
2. Click the "🔐 Sign In" button (top-right)
3. Select your Google account
4. Authorize the app
5. You should be redirected to the dashboard
6. Your profile information should appear

### 5. Test Sign-Out

1. Click your avatar (top-right)
2. Click "🚪 Sign Out"
3. You should be redirected to home
4. Sign In button should reappear

---

## API Endpoints Reference

### GET /api/auth/google
Returns the Google OAuth authorization URL.

**Response:**
```json
{
  "url": "https://accounts.google.com/o/oauth2/v2/auth?..."
}
```

### GET /api/auth/callback/google?code=...
Handles the OAuth callback from Google.

**Behavior:**
- Exchanges code for tokens
- Fetches user info
- Creates session cookie
- Redirects to `/dashboard`

### GET /api/auth/get-session
Retrieves the current user session.

**Response (Authenticated):**
```json
{
  "session": {
    "user": {
      "id": "123456",
      "email": "user@gmail.com",
      "name": "User Name",
      "image": "https://..."
    }
  }
}
```

**Response (Unauthenticated):**
```json
{
  "error": "Not authenticated"
}
```

### POST /api/auth/sign-out
Signs out the user and clears the session.

**Response:**
```json
{
  "success": true,
  "message": "Signed out successfully"
}
```

---

## Deployment Checklist

- [ ] Set `GOOGLE_CLIENT_ID` in Vercel
- [ ] Set `GOOGLE_CLIENT_SECRET` in Vercel
- [ ] Set `BETTER_AUTH_SECRET` in Vercel (generate random string)
- [ ] Set `BASE_URL` to your Vercel domain
- [ ] Register OAuth callback URL in Google Cloud Console
- [ ] Trigger Vercel redeploy
- [ ] Test sign-in flow on production
- [ ] Test sign-out flow

---

## Troubleshooting

### "GOOGLE_CLIENT_ID not configured"
- Ensure environment variables are set in `.env` (local) or Vercel Project Settings (production)
- Restart dev server after changing `.env`

### OAuth Callback Not Working
- Check that redirect URI in Google Cloud Console matches `/api/auth/callback/google`
- Ensure `BASE_URL` environment variable is correct
- Verify GOOGLE_CLIENT_ID and GOOGLE_CLIENT_SECRET are correct

### Session Cookie Not Persisting
- Ensure browser allows cookies (not in incognito mode)
- Check that requests include `credentials: 'include'`
- Verify `SameSite=Lax` is set on cookie

### "redirect_uri_mismatch" Error
- This means the redirect URI in Google Cloud Console doesn't match
- Go to Google Cloud Console > OAuth consent screen > Edit > Add redirect URIs
- Must match exactly: `https://your-domain.com/api/auth/callback/google`

---

## Security Considerations

1. **HTTPS Required**: Always use HTTPS in production
2. **HttpOnly Cookies**: Session cookie is not accessible to JavaScript
3. **SameSite Protection**: Prevents CSRF attacks
4. **Secure Secrets**: Store `BETTER_AUTH_SECRET` securely in Vercel
5. **Offline Access**: `accessType: "offline"` enables refresh token rotation

---

## Next Steps

1. Get Google OAuth credentials from Google Cloud Console
2. Set environment variables in Vercel
3. Trigger a redeploy
4. Test the sign-in flow on production
5. Monitor for any authentication errors in Vercel logs

For more details, visit: https://www.better-auth.com/docs/authentication/google
