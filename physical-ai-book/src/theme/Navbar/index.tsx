import React from 'react';
import Navbar from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';

type NavbarProps = React.ComponentProps<typeof NavbarType>;

export default function NavbarWrapper(props: NavbarProps): React.ReactElement {
  return <Navbar {...props} />;
}
