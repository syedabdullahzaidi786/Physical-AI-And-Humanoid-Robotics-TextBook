import React from "react";
import Head from "@docusaurus/Head";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import { AuthProvider } from "@site/src/context/AuthContext";

function DocusaurusRoot({ children }: { children: React.ReactNode }) {
  const { siteConfig } = useDocusaurusContext();

  return (
    <>
      <Head>
        <meta name="algolia-site-verification" content="C92BAA73C916589D" />
      </Head>
      <AuthProvider>{children}</AuthProvider>
    </>
  );
}

export default DocusaurusRoot;
