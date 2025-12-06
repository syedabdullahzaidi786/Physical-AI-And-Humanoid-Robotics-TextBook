// OAuth callback handler for Google - exchanges code for tokens and creates session
export default async function handler(req, res) {
  const { code, state } = req.query || {};

  console.log('[OAUTH] /api/auth/callback/google - received callback');
  console.log('[OAUTH] /api/auth/callback/google - code present:', !!code);

  if (!code) {
    console.error('[OAUTH] Missing authorization code in callback');
    return res.status(400).json({ error: 'Missing authorization code' });
  }

  try {
    const clientId = process.env.GOOGLE_CLIENT_ID;
    const clientSecret = process.env.GOOGLE_CLIENT_SECRET;
    const baseUrl = process.env.BASE_URL || 'http://localhost:3000';
    const redirectUri = `${baseUrl}/api/auth/callback/google`;

    console.log('[OAUTH] /api/auth/callback/google - credentials check');
    console.log('[OAUTH] clientId present:', !!clientId, 'clientSecret present:', !!clientSecret);

    if (!clientId || !clientSecret) {
      console.error('[OAUTH] Missing Google OAuth credentials');
      return res.status(500).json({ error: 'OAuth credentials not configured' });
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
      console.error('[OAUTH] Token exchange failed:', errorData);
      return res.status(500).json({ error: 'Failed to exchange authorization code for tokens', details: errorData });
    }

    console.log('[OAUTH] Token exchange successful');

    const tokenData = await tokenResponse.json();
    const { access_token, id_token } = tokenData;

    if (!access_token) {
      console.error('[OAUTH] No access token received');
      return res.status(500).json({ error: 'Failed to obtain access token' });
    }

    console.log('[OAUTH] Access token obtained');

    // Step 2: Fetch user info from Google
    const userResponse = await fetch('https://www.googleapis.com/oauth2/v2/userinfo', {
      headers: { Authorization: `Bearer ${access_token}` },
    });

    if (!userResponse.ok) {
      const errText = await userResponse.text();
      console.error('[OAUTH] Failed to fetch user info from Google:', errText);
      return res.status(500).json({ error: 'Failed to fetch user information', details: errText });
    }

    console.log('[OAUTH] User info fetched from Google');

    const userData = await userResponse.json();

    // Step 3: Create session cookie with user data
    const sessionData = {
      id: userData.id,
      email: userData.email,
      name: userData.name || userData.email.split('@')[0],
      image: userData.picture,
    };

    const sessionCookie = Buffer.from(JSON.stringify(sessionData)).toString('base64');
    
    console.log('[OAUTH] Session cookie created for user:', sessionData.email);
    
    // Set the session cookie (7 days expiry)
    res.setHeader(
      'Set-Cookie',
      `session=${sessionCookie}; Path=/; HttpOnly; Max-Age=604800; SameSite=Lax`
    );

    console.log('[OAUTH] /api/auth/callback/google - redirecting to /dashboard');
    
    // Step 4: Redirect to dashboard or home
    res.writeHead(302, { Location: '/dashboard' });
    res.end();
  } catch (error) {
    console.error('[OAUTH] OAuth callback error:', error);
    return res.status(500).json({ error: 'Authentication failed', details: error.message });
  }
}
