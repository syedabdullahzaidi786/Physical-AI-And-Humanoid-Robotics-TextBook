import React, { createContext, useContext } from 'react';
import { authClient } from '@site/src/lib/auth-client';

export interface User {
  id: string;
  email: string;
  name?: string;
  image?: string;
}

interface AuthContextType {
  user: User | null;
  isLoading: boolean;
  signIn: () => Promise<void>;
  signOut: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { data: session, isPending } = authClient.useSession();

  const user: User | null = session?.user ? {
    id: session.user.id,
    email: session.user.email,
    name: session.user.name,
    image: session.user.image || undefined,
  } : null;

  const isLoading = isPending;

  const signIn = async () => {
    try {
      console.log('[AUTH] Redirecting to SSO Login Page...');
      const ssoUrl = "https://ar-devs-sso.vercel.app/";
      const callbackUrl = encodeURIComponent(window.location.origin + '/dashboard');
      // Redirecting to the base URL with callbackURL parameter
      window.location.href = `${ssoUrl}?callbackURL=${callbackUrl}`;
    } catch (error) {
      console.error('[AUTH] Redirect failed:', error);
    }
  };

  const signOut = async () => {
    try {
      await authClient.signOut();
      window.location.href = '/';
    } catch (error) {
      console.error('[AUTH] Sign out failed:', error);
    }
  };

  return (
    <AuthContext.Provider value={{ user, isLoading, signIn, signOut }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
};
