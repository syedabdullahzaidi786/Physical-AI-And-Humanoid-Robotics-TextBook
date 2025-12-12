import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/context/AuthContext';
import { Redirect } from '@docusaurus/router';
import styles from './index.module.css'; // Reusing hero styles for consistency or create new ones

export default function Dashboard() {
    const { user, isLoading, signOut } = useAuth();

    if (isLoading) {
        return (
            <Layout title="Dashboard">
                <div style={{
                    display: 'flex',
                    justifyContent: 'center',
                    alignItems: 'center',
                    minHeight: '60vh',
                    fontSize: '1.5rem'
                }}>
                    Loading...
                </div>
            </Layout>
        );
    }

    if (!user) {
        return <Redirect to="/signin" />;
    }

    return (
        <Layout title="Dashboard" description="User Dashboard">
            <div className="container margin-vert--xl">
                <div className="row">
                    <div className="col col--8 col--offset-2">
                        <div className="card shadow--md">
                            <div className="card__header">
                                <div className="avatar avatar--vertical">
                                    {user.image ? (
                                        <img
                                            className="avatar__photo avatar__photo--xl"
                                            src={user.image}
                                            alt={user.name}
                                        />
                                    ) : (
                                        <div className="avatar__photo avatar__photo--xl" style={{ background: '#ccc', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                                            {user.name?.charAt(0) || user.email.charAt(0)}
                                        </div>
                                    )}
                                    <div className="avatar__intro">
                                        <div className="avatar__name margin-top--md">{user.name}</div>
                                        <small className="avatar__subtitle">{user.email}</small>
                                    </div>
                                </div>
                            </div>
                            <div className="card__body">
                                <div className="text--center margin-bottom--lg">
                                    <h2>Welcome back!</h2>
                                    <p>You have full access to the Physical AI & Humanoid Robotics learning resources.</p>
                                </div>

                                <div className="row">
                                    <div className="col col--6 margin-bottom--md">
                                        <div className="card shadow--lw" style={{ height: '100%', padding: '1rem', border: '1px solid var(--ifm-color-emphasis-200)' }}>
                                            <h3>📚 My Progress</h3>
                                            <p>Track your reading and completed tutorials.</p>
                                            <button className="button button--outline button--primary button--block">View Progress</button>
                                        </div>
                                    </div>
                                    <div className="col col--6 margin-bottom--md">
                                        <div className="card shadow--lw" style={{ height: '100%', padding: '1rem', border: '1px solid var(--ifm-color-emphasis-200)' }}>
                                            <h3>🤖 AI Chat History</h3>
                                            <p>Review your past conversations with the AI Tutor.</p>
                                            <button className="button button--outline button--primary button--block">View Chats</button>
                                        </div>
                                    </div>
                                </div>
                            </div>
                            <div className="card__footer">
                                <button
                                    onClick={signOut}
                                    className="button button--danger button--block"
                                >
                                    Sign Out
                                </button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </Layout>
    );
}
