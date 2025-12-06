**Authentication Setup (Better Auth + Google OAuth)**

This document explains how to configure Better Auth (Google provider) and what environment variables to set in Vercel and locally.

1) Authorized JavaScript origins (NO PATH) — register these in your Google OAuth application:

- `https://physical-ai-and-humanoid-robotics-t-peach.vercel.app`
- `http://localhost:3000`

2) Authorized redirect URIs (WITH PATH) — register these in your Google OAuth application:

- `https://physical-ai-and-humanoid-robotics-t-peach.vercel.app/api/auth/callback/google`
- `http://localhost:3000/api/auth/callback/google`

3) Environment variables

Set these in Vercel (Project → Settings → Environment Variables) for production, and in `physical-ai-book/.env` for local development.

- `BETTER_AUTH_CLIENT_ID` = (from Better Auth)
- `BETTER_AUTH_CLIENT_SECRET` = (from Better Auth)
- `BETTER_AUTH_AUTHORIZE_URL` = `https://api.better-auth.com/oauth/authorize` (default)
- `BETTER_AUTH_TOKEN_URL` = `https://api.better-auth.com/oauth/token` (default)
- `BASE_URL` = `https://physical-ai-and-humanoid-robotics-t-peach.vercel.app`

4) Quick local test (development)

- Copy `physical-ai-book/.env.example` to `physical-ai-book/.env` and fill in your Better Auth credentials.
- Run the dev server:

```powershell
cd physical-ai-book
npm run start
```

- Visit `http://localhost:3000` and open `http://localhost:3000/api/auth/google` to start the OAuth flow.

5) Serverless endpoints

This repo includes small stub endpoints under the `api/` folder to start the OAuth flow and to receive the callback. They are intentionally minimal — replace the token-exchange logic with production code or use Better Auth SDKs as documented.

6) Notes

- For security, always set credentials in Vercel environment variables rather than committing them.
- After setting variables in Vercel, trigger a redeploy so the runtime picks up the new values.
