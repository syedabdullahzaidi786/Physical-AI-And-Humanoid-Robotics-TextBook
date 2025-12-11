import React, { useState, useRef, useEffect } from 'react';
import { useTranslation } from '@site/src/context/TranslationContext';
import styles from './Chatbot.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

interface ChatbotProps {
  apiUrl?: string;
}

export default function Chatbot({ apiUrl }: ChatbotProps): React.ReactElement {
  const { translateSync } = useTranslation();
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [apiStatus, setApiStatus] = useState<'unknown' | 'checking' | 'ready' | 'unavailable'>('unknown');
  const [finalApiUrl, setFinalApiUrl] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Initialize API URL and check health
  useEffect(() => {
    const initializeApi = async () => {
      // Determine API URL
      let url = apiUrl;
      
      // If no URL provided, try to auto-detect
      if (!url) {
        if (typeof window !== 'undefined') {
          // For production: use the same domain
          const protocol = window.location.protocol;
          const hostname = window.location.hostname;
          
          // Try production URL first (same domain)
          url = `${protocol}//${hostname}`;
          
          // For Vercel deployments, append /api
          if (hostname.includes('vercel.app') || hostname.includes('herokuapp.com')) {
            url = `${protocol}//${hostname}`;
          }
        }
      }
      
      // Default fallback
      if (!url) {
        url = 'http://localhost:8000';
      }
      
      setFinalApiUrl(url);
      
      // Check API health
      setApiStatus('checking');
      try {
        const response = await fetch(`${url}/api/health`, {
          method: 'GET',
          headers: { 'Content-Type': 'application/json' },
        });
        
        if (response.ok) {
          setApiStatus('ready');
          setError(null);
        } else {
          setApiStatus('unavailable');
          setError('Chatbot API is unavailable. Please try again later.');
        }
      } catch (err) {
        setApiStatus('unavailable');
        setError('Cannot connect to chatbot API. Please check if the service is running.');
        console.error('[CHATBOT] Health check failed:', err);
      }
    };
    
    initializeApi();
  }, [apiUrl]);

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Add initial welcome message
  useEffect(() => {
    if (messages.length === 0) {
      const welcomeMessage: Message = {
        id: '0',
        role: 'assistant',
        content: translateSync('Welcome') + ' to the Physical AI & Robotics Chatbot! ' +
                 'I can help you with questions about humanoid robotics, AI, and more. ' +
                 'What would you like to know?',
        timestamp: new Date(),
      };
      setMessages([welcomeMessage]);
    }
  }, []);

  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!input.trim()) return;
    
    if (apiStatus !== 'ready') {
      setError('Chatbot API is not available. Please try again later.');
      return;
    }

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      // Send to chatbot API
      const response = await fetch(`${finalApiUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage.content,
          conversation_history: messages.map((msg) => ({
            role: msg.role,
            content: msg.content,
          })),
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.response || 'No response received',
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Unknown error occurred';
      console.error('[CHATBOT] Error:', errorMsg);
      setError(errorMsg);

      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: `⚠️ Error: ${errorMsg}\n\nThe chatbot API may not be running. If you're on a hosted site, the backend needs to be deployed separately.`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const clearHistory = () => {
    const welcomeMessage: Message = {
      id: '0',
      role: 'assistant',
      content: 'Chat cleared. How can I help you?',
      timestamp: new Date(),
    };
    setMessages([welcomeMessage]);
  };

  return (
    <div className={styles.chatbotContainer}>
      {/* Chat Button (Floating) */}
      <button
        className={styles.chatButton}
        onClick={toggleChat}
        title="Open Chatbot"
        aria-label="Open Chatbot"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>{translateSync('Physical AI Chatbot')}</h3>
            <div className={styles.headerActions}>
              <button
                className={styles.clearBtn}
                onClick={clearHistory}
                title="Clear conversation"
                aria-label="Clear conversation"
              >
                🔄
              </button>
              <button
                className={styles.closeBtn}
                onClick={toggleChat}
                title="Close chatbot"
                aria-label="Close chatbot"
              >
                ✕
              </button>
            </div>
          </div>

          {/* Messages Area */}
          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.role]}`}
              >
                <div className={styles.messageContent}>{message.content}</div>
                <div className={styles.messageTime}>
                  {message.timestamp.toLocaleTimeString([], {
                    hour: '2-digit',
                    minute: '2-digit',
                  })}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            {error && (
              <div className={`${styles.message} ${styles.error}`}>
                <div className={styles.messageContent}>
                  <strong>⚠️ Error:</strong> {error}
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <form className={styles.inputForm} onSubmit={handleSendMessage}>
            <input
              type="text"
              placeholder={apiStatus === 'ready' ? translateSync('Ask about AI, robotics, and more...') : 'API unavailable...'}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              disabled={isLoading || apiStatus !== 'ready'}
              className={styles.input}
              aria-label="Chat message input"
            />
            <button
              type="submit"
              disabled={isLoading || !input.trim() || apiStatus !== 'ready'}
              className={styles.sendBtn}
              aria-label="Send message"
              title={apiStatus !== 'ready' ? 'Chatbot API is unavailable' : 'Send message'}
            >
              {isLoading ? '⏳' : apiStatus === 'ready' ? '📤' : '⚠️'}
            </button>
          </form>

          {/* Status Indicator */}
          <div className={styles.statusBar}>
            <span className={`${styles.statusIndicator} ${styles[`status-${apiStatus}`]}`}></span>
            <small>
              {apiStatus === 'ready' && '✅ API Connected'}
              {apiStatus === 'checking' && '🔄 Checking API...'}
              {apiStatus === 'unavailable' && '❌ API Unavailable'}
              {apiStatus === 'unknown' && '⏳ Initializing...'}
            </small>
          </div>
        </div>
      )}
    </div>
  );
}
