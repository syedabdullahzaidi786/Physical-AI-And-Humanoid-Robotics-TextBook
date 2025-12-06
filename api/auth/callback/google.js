// OAuth callback handler for Google - exchanges code for tokens and creates session
export default async function handler(req, res) {
  const { code, state } = req.query || {};

  if (!code) {
    return res.status(400).send('Missing authorization code');
  }

  try {
    const clientId = process.env.GOOGLE_CLIENT_ID;
    const clientSecret = process.env.GOOGLE_CLIENT_SECRET;
    const baseUrl = process.env.BASE_URL || 'http://localhost:3000';
    const redirectUri = `${baseUrl}/api/auth/callback/google`;

    if (!clientId || !clientSecret) {
      console.error('Missing Google OAuth credentials');
      return res.status(500).send('OAuth credentials not configured');
    }

    // Step 1: Exchange authorization code for tokens
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
      const errorData = await tokenResponse.text();
      console.error('Token exchange failed:', errorData);
      return res.status(500).send('Failed to exchange authorization code for tokens');
    }

    const tokenData = await tokenResponse.json();
    const { access_token, id_token } = tokenData;

    if (!access_token) {
      console.error('No access token received');
      return res.status(500).send('Failed to obtain access token');
    }

    // Step 2: Fetch user info from Google
    const userResponse = await fetch('https://www.googleapis.com/oauth2/v2/userinfo', {
      headers: { Authorization: `Bearer ${access_token}` },
    });

    if (!userResponse.ok) {
      console.error('Failed to fetch user info from Google');
      return res.status(500).send('Failed to fetch user information');
    }

    const userData = await userResponse.json();

    // Step 3: Create session cookie with user data
    const sessionData = {
      id: userData.id,
      email: userData.email,
      name: userData.name || userData.email.split('@')[0],
      image: userData.picture,
    };

    const sessionCookie = Buffer.from(JSON.stringify(sessionData)).toString('base64');
    
    // Set the session cookie (7 days expiry)
    res.setHeader(
      'Set-Cookie',
      `session=${sessionCookie}; Path=/; HttpOnly; Max-Age=604800; SameSite=Lax`
    );

    // Step 4: Redirect to dashboard or home
    res.writeHead(302, { Location: '/dashboard' });
    res.end();
  } catch (error) {
    console.error('OAuth callback error:', error);
    return res.status(500).send(`Authentication failed: ${error.message}`);
  }
}
