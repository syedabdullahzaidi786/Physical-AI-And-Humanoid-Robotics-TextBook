import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Embodied AI in K–12 Education',
  favicon: 'img/logo.svg',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-and-humanoid-robotics-t-peach.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'syedabdullahzaidi786', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-And-Humanoid-Robotics-TextBook', // Usually your repo name.

  onBrokenLinks: 'ignore',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/syedabdullahzaidi786/Physical-AI-And-Humanoid-Robotics-TextBook/tree/main/physical-ai-book/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/syedabdullahzaidi786/Physical-AI-And-Humanoid-Robotics-TextBook/tree/main/physical-ai-book/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
      disableSwitch: false,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      hideOnScroll: false,
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
        srcDark: 'img/logo-dark.svg',
        width: 32,
        height: 32,
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Documentation',
        },
        {
          to: '/search',
          label: '🔍 Search',
          position: 'right',
        },
        {
          href: 'https://github.com/syedabdullahzaidi786/Physical-AI-And-Humanoid-Robotics-TextBook',
          label: 'GitHub',
          position: 'right',
          target: '_blank',
          rel: 'noopener noreferrer',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learning Modules',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'ROS 2 Fundamentals',
              to: '/docs/01-module-ros2',
            },
            {
              label: 'Simulation & Physics',
              to: '/docs/02-module-gazebo-unity',
            },
            {
              label: 'Perception & AI',
              to: '/docs/03-module-isaac',
            },
          ],
        },
        {
          title: 'Case Studies',
          items: [
            {
              label: 'Adaptive Tutoring',
              to: '/docs/06-case-studies/tutoring',
            },
            {
              label: 'Behavioral Support',
              to: '/docs/06-case-studies/behavioral',
            },
            {
              label: 'Accessibility',
              to: '/docs/06-case-studies/accessibility',
            },
            {
              label: 'Classroom Operations',
              to: '/docs/06-case-studies/operational',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'References',
              to: '/docs/07-references',
            },
            {
              label: 'GitHub Repository',
              href: 'https://github.com/syedabdullahzaidi786/Physical-AI-And-Humanoid-Robotics-TextBook',
              target: '_blank',
              rel: 'noopener noreferrer',
            },
            {
              label: 'GIAIC Q4 Hackathon',
              href: 'https://www.giaic.pk',
              target: '_blank',
              rel: 'noopener noreferrer',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Project. All rights reserved. | Built with Docusaurus & 💙 by AI Educators | Educational Use Only`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    // Algolia DocSearch Configuration
    // To use DocSearch, you need an Algolia account with a search index
    // Visit: https://docsearch.algolia.com/ to apply for a free account
    // Once approved, uncomment and fill in your credentials below:
    /*
    algolia: {
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'physical-ai-robotics',
      contextualSearch: true,
      externalUrlRegex: 'external\\.com|domain\\.com',
      replaceSearchResultPathname: {
        from: '/docs/',
        to: '/',
      },
      searchParameters: {},
      searchPagePath: 'search',
    },
    */
  } satisfies Preset.ThemeConfig,
};

export default config;
