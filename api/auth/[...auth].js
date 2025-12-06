// Better Auth API handler for Vercel
// This handles all auth-related requests: /api/auth/[...auth]

import { auth } from "../../auth";

export default async function handler(req, res) {
  const response = await auth.handler(req, res);
  return response;
}
