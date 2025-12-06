// OAuth callback handler stub for Better Auth (Google)
// This file receives the `code` from the provider. In production you
// should exchange the code for tokens using the Better Auth token endpoint
// and create a session (cookie, JWT, etc.).

export default async function handler(req, res) {
  const { code, state } = req.query || {};

  if (!code) {
    res.status(400).send('Missing code');
    return;
  }

  // Example: echo the code for manual testing. Replace with token exchange.
  res.setHeader('Content-Type', 'application/json');
  res.status(200).send(JSON.stringify({
    message: 'Received authorization code. To complete sign-in, exchange this code at the token endpoint.',
    code,
    state,
  }));
}
