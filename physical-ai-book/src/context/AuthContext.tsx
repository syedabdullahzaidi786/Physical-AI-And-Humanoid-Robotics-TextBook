import React, { createContext, useContext, useState, useEffect } from 'react';

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
  initializeGoogleOAuth: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Check if user is logged in on mount
  useEffect(() => {
    const checkAuth = async () => {
      try {
        console.log('[AUTH] checkAuth() - checking for existing session');
        const response = await fetch('/api/auth/get-session', {
          credentials: 'include', // Important: include cookies
        });
        console.log('[AUTH] /api/auth/get-session response:', response.status);
        
        if (response.ok) {
          const data = await response.json();
          console.log('[AUTH] Session data received:', !!data.session?.user);
          if (data.session && data.session.user) {
            console.log('[AUTH] User restored:', data.session.user.email);
            setUser({
              id: data.session.user.id,
              email: data.session.user.email,
              name: data.session.user.name,
              image: data.session.user.image,
            });
          }
        } else {
          console.log('[AUTH] No active session');
        }
      } catch (error) {
        console.error('[AUTH] checkAuth error:', error);
      } finally {
        console.log('[AUTH] Auth check complete - isLoading set to false');
        setIsLoading(false);
      }
    };

    checkAuth();
  }, []);

  const signIn = async () => {
    try {
      console.log('[AUTH] signIn() called - redirecting to /signin page');
      // Redirect to sign-in page where user can click "Sign In With Google"
      window.location.href = '/signin';
    } catch (error) {
      console.error('[AUTH] Failed to redirect to sign-in:', error);
    }
  };

  const signOut = async () => {
    try {
      await fetch('/api/auth/sign-out', {
        method: 'POST',
        credentials: 'include',
      });
      setUser(null);
      window.location.href = '/';
    } catch (error) {
      console.error('Failed to sign out:', error);
    }
  };

  const initializeGoogleOAuth = async () => {
    try {
      console.log('[AUTH] initializeGoogleOAuth() called - fetching OAuth URL');
      // Get the OAuth URL from the endpoint
      const response = await fetch('/api/auth/google');
      console.log('[AUTH] /api/auth/google response:', response.status);
      
      if (!response.ok) {
        const errText = await response.text();
        console.error('[AUTH] Failed to get OAuth URL:', errText);
        return;
      }
      
      const data = await response.json();
      console.log('[AUTH] OAuth URL received:', !!data.url);
      
      if (data.url) {
        console.log('[AUTH] Redirecting to Google OAuth...');
        // Redirect to Google OAuth
        window.location.href = data.url;
      } else {
        console.error('[AUTH] No URL in response:', data);
      }
    } catch (error) {
      console.error('[AUTH] Failed to initiate Google OAuth:', error);
    }
  };

  return (
    <AuthContext.Provider value={{ user, isLoading, signIn, signOut, initializeGoogleOAuth }}>
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
