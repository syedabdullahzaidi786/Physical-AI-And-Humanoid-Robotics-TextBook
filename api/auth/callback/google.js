// OAuth callback handler for Google OAuth
// Exchanges authorization code for tokens and creates session

export default async function handler(req, res) {
  const { code, state } = req.query || {};

  if (!code) {
    res.status(400).send('Missing authorization code');
    return;
  }

  try {
    const clientId = process.env.GOOGLE_CLIENT_ID;
    const clientSecret = process.env.GOOGLE_CLIENT_SECRET;
    const baseUrl = process.env.BASE_URL || 'http://localhost:3000';
    const redirectUri = `${baseUrl}/api/auth/callback/google`;

    if (!clientId || !clientSecret) {
      console.error('Missing Google OAuth credentials');
      res.status(500).send('OAuth credentials not configured');
      return;
    }

    // Exchange authorization code for tokens
    const tokenResponse = await fetch('https://oauth2.googleapis.com/token', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({
        grant_type: 'authorization_code',
        code,
        client_id: clientId,
        client_secret: clientSecret,
        redirect_uri: redirectUri,
      }).toString(),
    });

    if (!tokenResponse.ok) {
      throw new Error(`Token exchange failed: ${tokenResponse.statusText}`);
    }

    const tokenData = await tokenResponse.json();
    const { access_token } = tokenData;

    // Fetch user info from Google
    const userResponse = await fetch('https://www.googleapis.com/oauth2/v2/userinfo', {
      headers: { Authorization: `Bearer ${access_token}` },
    });

    if (!userResponse.ok) {
      throw new Error('Failed to fetch user info');
    }

    const userData = await userResponse.json();

    // Create session cookie with user data (7 days expiry)
    const sessionData = {
      id: userData.id,
      email: userData.email,
      name: userData.name,
      image: userData.picture,
    };

    const sessionCookie = Buffer.from(JSON.stringify(sessionData)).toString('base64');
    res.setHeader('Set-Cookie', `session=${sessionCookie}; Path=/; HttpOnly; Max-Age=604800; SameSite=Lax`);

    // Redirect to dashboard
    res.writeHead(302, { Location: '/dashboard' });
    res.end();
  } catch (error) {
    console.error('OAuth callback error:', error);
    res.status(500).send(`Authentication failed: ${error.message}`);
  }
}
