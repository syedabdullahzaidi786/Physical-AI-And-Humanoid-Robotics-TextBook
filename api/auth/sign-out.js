// Sign out endpoint - clears session and cookies
export default async function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Clear the session cookie
    res.setHeader(
      'Set-Cookie',
      'session=; Path=/; HttpOnly; Max-Age=0; SameSite=Lax'
    );

    return res.status(200).json({ 
      success: true,
      message: 'Signed out successfully' 
    });
  } catch (error) {
    console.error('Sign out error:', error);
    return res.status(500).json({ error: 'Failed to sign out' });
  }
}
