import React from 'react';
import Navbar from '@theme-original/Navbar';
import AuthButton from '@site/src/components/AuthButton';
import type NavbarType from '@theme/Navbar';
import styles from './styles.module.css';

type NavbarProps = React.ComponentProps<typeof NavbarType>;

export default function NavbarWrapper(props: NavbarProps): React.ReactElement {
  return (
    <>
      <Navbar {...props} />
      <div className={styles.authButtonContainer}>
        <AuthButton />
      </div>
    </>
  );
}
