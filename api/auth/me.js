// Endpoint to get current user session from cookie
export default function handler(req, res) {
  const cookie = req.headers.cookie || '';
  const sessionMatch = cookie.match(/session=([^;]+)/);

  if (!sessionMatch || !sessionMatch[1]) {
    res.status(401).json({ error: 'Not authenticated' });
    return;
  }

  try {
    const sessionJson = Buffer.from(sessionMatch[1], 'base64').toString('utf8');
    const session = JSON.parse(sessionJson);
    res.status(200).json(session);
  } catch (error) {
    res.status(401).json({ error: 'Invalid session' });
  }
}
