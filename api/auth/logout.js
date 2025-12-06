// Sign out endpoint - clears session cookie
export default function handler(req, res) {
  if (req.method !== 'POST') {
    res.status(405).json({ error: 'Method not allowed' });
    return;
  }

  // Clear session cookie
  res.setHeader('Set-Cookie', 'session=; Path=/; HttpOnly; Max-Age=0');
  res.status(200).json({ message: 'Signed out successfully' });
}
