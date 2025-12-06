// Better Auth Google OAuth initialization
// This endpoint returns the Google OAuth URL for the frontend to redirect to

export default function handler(req, res) {
  const baseUrl = process.env.BASE_URL || "http://localhost:3000";
  const clientId = process.env.GOOGLE_CLIENT_ID;

  if (!clientId) {
    res.status(500).send("GOOGLE_CLIENT_ID not configured");
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

  // Return JSON response that client can use to redirect
  res.setHeader("Content-Type", "application/json");
  res.status(200).json({
    url: googleAuthUrl,
  });
}
