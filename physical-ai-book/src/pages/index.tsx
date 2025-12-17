import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

import useBaseUrl from '@docusaurus/useBaseUrl';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={clsx('container', styles.heroContainer)}>
        <div className={styles.heroImage}>
          <img
            src={useBaseUrl('/img/hero-1.png')}
            alt="Physical AI & Humanoid Robotics"
            className={styles.floatingImage}
          />
        </div>
        <div className={styles.heroContent}>
          <Heading as="h1" className="hero__title">
            Where Digital Brains Meet <br />
            <span className={styles.highlight}>Physical Bodies</span>
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <p className={styles.subtitle_extra}>
            Learn how to build intelligent humanoid robots for K-12 education using ROS 2, Gazebo simulation, NVIDIA Isaac, and vision-language models.
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/introduction">
              Connect Dapp
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} - Educational AI & Robotics`}
      description="A comprehensive guide to physical AI and humanoid robotics in K-12 educational settings">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
