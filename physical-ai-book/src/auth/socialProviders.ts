// socialProviders config for client/server use
export const socialProviders = {
  google: {
    prompt: 'select_account',
    clientId: process.env.GOOGLE_CLIENT_ID as string,
    clientSecret: process.env.GOOGLE_CLIENT_SECRET as string,
  },
};

export default socialProviders;
