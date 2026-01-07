import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "https://ar-devs-sso.vercel.app/",
});

export const { useSession, signIn, signOut, signUp } = authClient;
