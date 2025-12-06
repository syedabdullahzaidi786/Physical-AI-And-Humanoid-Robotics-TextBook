import React from 'react';
import Navbar from '@theme-original/Navbar';
import NavbarAuthItem from '@site/src/components/NavbarAuthItem';
import styles from './NavbarWrapper.module.css';
import type NavbarType from '@theme/Navbar';

type NavbarProps = React.ComponentProps<typeof NavbarType>;

export default function NavbarWrapper(props: NavbarProps): React.ReactElement {
  return (
    // Wrap the original Navbar in a positioned container so the auth
    // item can be positioned relative to the navbar (no longer fixed)
    <div className={styles.container} style={{ position: 'relative' }}>
      <Navbar {...props} />

      {/* Right-aligned auth item: vertically centered within the navbar */}
      <div
        style={{
          position: 'absolute',
          top: '50%',
          right: '1rem',
          transform: 'translateY(-50%)',
          zIndex: 999,
          display: 'flex',
          alignItems: 'center',
        }}
        aria-hidden={false}
      >
        <NavbarAuthItem />
      </div>
    </div>
  );
}
