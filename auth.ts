import { betterAuth } from "better-auth";
import { memoryDB } from "better-auth/db";
import { nodePlugin } from "better-auth/node";

export const auth = betterAuth({
  database: memoryDB(),
  secret: process.env.BETTER_AUTH_SECRET || "your-secret-key-change-this-in-production",
  plugins: [nodePlugin()],
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
      prompt: "select_account",
    },
  },
  baseURL: process.env.BASE_URL || "http://localhost:3000",
  trustedOrigins: [
    process.env.BASE_URL || "http://localhost:3000",
  ],
});
