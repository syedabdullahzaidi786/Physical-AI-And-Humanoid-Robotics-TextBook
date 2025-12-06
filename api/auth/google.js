// Simple redirect to Better Auth authorize endpoint.
// This is a minimal stub to start the OAuth flow. Replace/extend with
// Better Auth SDK calls as needed.

export default function handler(req, res) {
  // Prefer explicit Google client env vars; fall back to Better Auth client if present
  const clientId = process.env.GOOGLE_CLIENT_ID || process.env.BETTER_AUTH_CLIENT_ID;
  const authorizeUrl = process.env.BETTER_AUTH_AUTHORIZE_URL || 'https://api.better-auth.com/oauth/authorize';
  const baseUrl = process.env.BASE_URL || 'http://localhost:3000';
  const redirectUri = `${baseUrl}/api/auth/callback/google`;

  if (!clientId) {
    res.status(500).send('GOOGLE_CLIENT_ID or BETTER_AUTH_CLIENT_ID not configured');
    return;
  }

  // Include prompt=select_account to force account selection
  const params = new URLSearchParams({
    response_type: 'code',
    client_id: clientId,
    redirect_uri: redirectUri,
    scope: 'openid profile email',
    prompt: 'select_account',
    state: 'state',
  });

  res.writeHead(302, { Location: `${authorizeUrl}?${params.toString()}` });
  res.end();
}
