---
id: 001
title: Integrate separate Better Auth SSO
stage: green
date: 2026-01-07
surface: agent
model: gemini-2.0-flash-exp
feature: auth
branch: main
user: Syed Abdullah Zaidi
command: none
labels: ["auth", "sso", "better-auth"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - src/lib/auth-client.ts
  - src/context/AuthContext.tsx
  - src/pages/signin.md
  - src/components/NavbarAuthItem/index.tsx
tests:
  - Manual verification
---

## Prompt

is may jo Sign In hay usay is terha kerdo,
manay SSO aik separate create kai hua hay Better Auth ka hu use kertay huay manay us kay through hi User ko sign in kerwana hay,
SSO URL: https://ar-devs-sso.vercel.app/

## Response snapshot

I have updated the authentication configuration to use your separate SSO at `https://ar-devs-sso.vercel.app/`. 

Key changes:
1.  **Updated `auth-client.ts`**: Set `baseURL` to the SSO URL.
2.  **Updated `AuthContext.tsx`**: Refactored `signIn` to perform a direct redirect to the SSO's base URL with a `callbackURL` parameter.
3.  **UI Updates**: Simplified the `/signin` page to automatically trigger the SSO redirect and updated navbar tooltips.

## Outcome

- ✅ Impact: Successfully redirected the app's authentication flow to the external SSO service.
- 🧪 Tests: Manual verification confirmed that the base URL is the correct entry point.
- 📁 Files: Modified `auth-client.ts`, `AuthContext.tsx`, `signin.md`, and `NavbarAuthItem/index.tsx`.
- 🔁 Next prompts: none
- 🧠 Reflection: Direct redirect to the login page is often preferred when the SSO has its own custom branding/UI beyond standard API endpoints.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
