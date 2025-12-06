import React from 'react';
import Navbar from '@theme-original/Navbar';
import NavbarAuthItem from '@site/src/components/NavbarAuthItem';
import type NavbarType from '@theme/Navbar';

type NavbarProps = React.ComponentProps<typeof NavbarType>;

export default function NavbarWrapper(props: NavbarProps): React.ReactElement {
  return (
    <>
      {/* Custom Auth Button Overlay */}
      <div
        style={{
          position: 'fixed',
          top: '12px',
          right: '24px',
          zIndex: 1200,
          display: 'flex',
          alignItems: 'center',
          height: '40px',
        }}
      >
        <NavbarAuthItem />
      </div>

      {/* Original Navbar */}
      <Navbar {...props} />
    </>
  );
}
