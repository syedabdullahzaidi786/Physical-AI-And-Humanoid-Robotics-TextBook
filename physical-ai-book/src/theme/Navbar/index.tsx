import React from 'react';
import Navbar from '@theme-original/Navbar';
import NavbarAuthItem from '@site/src/components/NavbarAuthItem';
import type NavbarType from '@theme/Navbar';
import styles from './styles.module.css';

type NavbarProps = React.ComponentProps<typeof NavbarType>;

export default function NavbarWrapper(props: NavbarProps): React.ReactElement {
  return (
    <div className={styles.navbarWrapper}>
      <Navbar {...props} />
      <div className={styles.authContainer}>
        <NavbarAuthItem />
      </div>
    </div>
  );
}
