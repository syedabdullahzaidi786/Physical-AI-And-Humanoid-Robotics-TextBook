// Better Auth Google OAuth initialization
// This endpoint returns the Google OAuth URL for the frontend to redirect to

export default function handler(req, res) {
  const baseUrl = process.env.BASE_URL || "http://localhost:3000";
  const clientId = process.env.GOOGLE_CLIENT_ID;

  console.log('[OAUTH] /api/auth/google - baseUrl:', baseUrl);
  console.log('[OAUTH] /api/auth/google - clientId present:', !!clientId);

  if (!clientId) {
    console.error('[OAUTH] GOOGLE_CLIENT_ID not configured');
    res.status(500).json({ error: "GOOGLE_CLIENT_ID not configured" });
    return;
  }

  // Construct the Google OAuth URL
  const params = new URLSearchParams({
    response_type: "code",
    client_id: clientId,
    redirect_uri: `${baseUrl}/api/auth/callback/google`,
    scope: "openid profile email",
    prompt: "select_account",
    access_type: "offline",
  });

  const googleAuthUrl = `https://accounts.google.com/o/oauth2/v2/auth?${params.toString()}`;

  console.log('[OAUTH] /api/auth/google - redirectUri:', `${baseUrl}/api/auth/callback/google`);
  console.log('[OAUTH] /api/auth/google - returning authUrl');

  // Return JSON response that client can use to redirect
  res.setHeader("Content-Type", "application/json");
  res.status(200).json({
    url: googleAuthUrl,
  });
}
