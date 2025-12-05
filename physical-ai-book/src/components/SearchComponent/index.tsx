import React, { useState, useMemo } from 'react';
import Fuse from 'fuse.js';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface SearchResult {
  title: string;
  path: string;
  excerpt: string;
}

interface DocumentItem {
  title: string;
  path: string;
  content: string;
}

// Search index - populated from your docs
const searchIndex: DocumentItem[] = [
  {
    title: 'Introduction',
    path: '/docs/intro',
    content: 'Physical AI humanoid robotics K-12 education embodied AI overview'
  },
  {
    title: 'ROS 2 Fundamentals',
    path: '/docs/01-module-ros2',
    content: 'ROS 2 middleware publish subscribe nodes topics services actions launch'
  },
  {
    title: 'Physics Simulation',
    path: '/docs/02-module-gazebo-unity',
    content: 'Gazebo URDF XACRO physics simulation sensors domain randomization'
  },
  {
    title: 'NVIDIA Isaac SDK',
    path: '/docs/03-module-isaac',
    content: 'Isaac perception AI reinforcement learning Jetson edge deployment'
  },
  {
    title: 'Vision-Language Models',
    path: '/docs/04-module-vla',
    content: 'Whisper GPT vision language models safety guardrails decision making'
  },
  {
    title: 'Capstone Project',
    path: '/docs/05-capstone',
    content: 'End-to-end project design evaluation deployment implementation'
  },
  {
    title: 'Case Study: Adaptive Tutoring',
    path: '/docs/06-case-studies/tutoring',
    content: 'Adaptive tutoring personalized learning student engagement education'
  },
  {
    title: 'Case Study: Behavioral Support',
    path: '/docs/06-case-studies/behavioral',
    content: 'Behavioral support emotional learning SEL classroom management'
  },
  {
    title: 'Case Study: Accessibility',
    path: '/docs/06-case-studies/accessibility',
    content: 'Accessibility inclusive design disabilities ADA support'
  },
  {
    title: 'Case Study: Classroom Operations',
    path: '/docs/06-case-studies/operational',
    content: 'Classroom logistics automation administration operations management'
  },
  {
    title: 'References',
    path: '/docs/07-references',
    content: 'Citations bibliography peer-reviewed research academic sources APA'
  },
];

export default function SearchComponent(): React.ReactElement {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<SearchResult[]>([]);
  const [isOpen, setIsOpen] = useState(false);

  // Initialize Fuse.js
  const fuse = useMemo(
    () =>
      new Fuse(searchIndex, {
        keys: ['title', 'content'],
        threshold: 0.3,
        includeScore: true,
        minMatchCharLength: 2,
      }),
    []
  );

  // Handle search
  const handleSearch = (value: string) => {
    setQuery(value);

    if (value.trim().length > 0) {
      const searchResults = fuse.search(value);
      const formattedResults: SearchResult[] = searchResults
        .slice(0, 8)
        .map((result) => ({
          title: result.item.title,
          path: result.item.path,
          excerpt: result.item.content.substring(0, 100) + '...',
        }));
      setResults(formattedResults);
      setIsOpen(true);
    } else {
      setResults([]);
      setIsOpen(false);
    }
  };

  return (
    <div className={styles.searchContainer}>
      <div className={styles.searchBox}>
        <input
          type="text"
          placeholder="🔍 Search topics, modules, case studies..."
          value={query}
          onChange={(e) => handleSearch(e.target.value)}
          onFocus={() => query && setIsOpen(true)}
          className={styles.searchInput}
        />
      </div>

      {isOpen && (
        <div className={styles.searchResults}>
          {results.length > 0 ? (
            <>
              {results.map((result, index) => (
                <Link
                  key={index}
                  to={result.path}
                  onClick={() => setIsOpen(false)}
                  className={styles.resultItem}
                >
                  <div className={styles.resultTitle}>{result.title}</div>
                  <div className={styles.resultExcerpt}>{result.excerpt}</div>
                </Link>
              ))}
            </>
          ) : query.length > 0 ? (
            <div className={styles.noResults}>
              No results found for "{query}". Try different keywords!
            </div>
          ) : null}
        </div>
      )}
    </div>
  );
}
