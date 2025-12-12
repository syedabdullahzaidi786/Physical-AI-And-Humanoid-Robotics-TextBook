import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  emoji: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 & Middleware',
    emoji: '🔗',
    description: (
      <>
        Master Robot Operating System (ROS 2) fundamentals, including publish-subscribe architectures,
        service calls, and action servers for building scalable robotics applications.
      </>
    ),
  },
  {
    title: 'Physics Simulation',
    emoji: '🎮',
    description: (
      <>
        Learn URDF/XACRO robot modeling, Gazebo/Ignition physics engines, sensor simulation,
        and domain randomization for robust sim-to-real transfer in robotics.
      </>
    ),
  },
  {
    title: 'AI & Perception',
    emoji: '🤖',
    description: (
      <>
        Explore NVIDIA Isaac SDK, computer vision, reinforcement learning, and
        deployment on edge hardware like NVIDIA Jetson Orin for real-world applications.
      </>
    ),
  },
  {
    title: 'Vision-Language Models',
    emoji: '🗣️',
    description: (
      <>
        Integrate Whisper (speech-to-text), GPT models, and VLMs for natural language interactions,
        safety guardrails, and decision-making in embodied AI systems.
      </>
    ),
  },
  {
    title: 'Real-World Case Studies',
    emoji: '📚',
    description: (
      <>
        Study adaptive tutoring, behavioral support, accessibility solutions, and classroom logistics
        with evidence-based research and implementation guidelines.
      </>
    ),
  },
  {
    title: 'End-to-End Projects',
    emoji: '🚀',
    description: (
      <>
        Build complete humanoid robots from simulation through deployment, including evaluation frameworks,
        safety protocols, and reproducible research methodologies for K-12 education.
      </>
    ),
  },
];

function Feature({ title, emoji, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className={styles.featureCard}>
        <div className={styles.featureIcon}>{emoji}</div>
        <div className={styles.featureContent}>
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
