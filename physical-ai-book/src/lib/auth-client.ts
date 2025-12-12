import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: (typeof process !== 'undefined' && process.env?.REACT_APP_BASE_URL) || "http://localhost:3000",
});

export const { useSession, signIn, signOut, signUp } = authClient;
