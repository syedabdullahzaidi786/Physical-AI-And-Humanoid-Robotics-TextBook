import { betterAuth } from "better-auth";
import { nodePlugin } from "better-auth/node";

export const auth = betterAuth({
  database: undefined, // Using in-memory for now, can be changed to a real database
  secret: process.env.BETTER_AUTH_SECRET || "your-secret-key-change-this",
  plugins: [nodePlugin()],
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
      prompt: "select_account",
      accessType: "offline",
    },
  },
  baseURL: process.env.BASE_URL || "http://localhost:3000",
});
