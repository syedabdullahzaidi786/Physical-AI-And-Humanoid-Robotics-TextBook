---
title: Search
hide_table_of_contents: true
---

import SearchComponent from '@site/src/components/SearchComponent';
import styles from './styles.module.css';

<div className={styles.searchPage}>
  <div className={styles.searchHeader}>
    <h1>🔍 Search Documentation</h1>
    <p>Find topics, modules, and case studies instantly</p>
  </div>
  
  <div className={styles.searchWidget}>
    <SearchComponent />
  </div>

  <div className={styles.searchTips}>
    <h2>💡 Search Tips</h2>
    <ul>
      <li>Type keywords like "ROS 2", "simulation", "Isaac", "tutoring"</li>
      <li>Use partial words: "control" finds "control systems"</li>
      <li>Search is case-insensitive</li>
      <li>Results appear as you type</li>
    </ul>
  </div>

  <div className={styles.searchIndex}>
    <h2>📚 Searchable Topics</h2>
    <div className={styles.topicsGrid}>
      <div className={styles.topic}>
        <strong>Modules</strong>
        <ul>
          <li>Introduction</li>
          <li>ROS 2</li>
          <li>Simulation</li>
          <li>Isaac SDK</li>
          <li>VLM Integration</li>
          <li>Capstone</li>
        </ul>
      </div>
      <div className={styles.topic}>
        <strong>Case Studies</strong>
        <ul>
          <li>Adaptive Tutoring</li>
          <li>Behavioral Support</li>
          <li>Accessibility</li>
          <li>Operations</li>
        </ul>
      </div>
      <div className={styles.topic}>
        <strong>Resources</strong>
        <ul>
          <li>References</li>
          <li>Keywords</li>
          <li>Topics</li>
        </ul>
      </div>
    </div>
  </div>
</div>
