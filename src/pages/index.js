import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started with the Textbook 📚
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const features = [
    {
      title: '🤖 AI-Powered Chatbot',
      description: (
        <>
          Ask questions about any topic in the textbook and get instant,
          accurate answers powered by RAG (Retrieval-Augmented Generation).
        </>
      ),
    },
    {
      title: '📝 Interactive Learning',
      description: (
        <>
          Highlight any text on the page and ask the chatbot for explanations.
          Personalize your learning experience for each chapter.
        </>
      ),
    },
    {
      title: '🌐 Multilingual Support',
      description: (
        <>
          Toggle between English and Urdu (اردو) to learn in your preferred
          language. Content and chatbot support both languages.
        </>
      ),
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <div key={idx} className={clsx('col col--4')}>
              <div className="text--center padding-horiz--md">
                <h3>{feature.title}</h3>
                <p>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Interactive textbook for Physical AI and Humanoid Robotics with AI-powered chatbot">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <div className="container" style={{marginTop: '3rem', marginBottom: '3rem'}}>
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className="card">
                <div className="card__header">
                  <h2>🚀 Features</h2>
                </div>
                <div className="card__body">
                  <ul>
                    <li><strong>Real-time Chat:</strong> Click the chat icon in the bottom-right corner</li>
                    <li><strong>Smart Answers:</strong> Responses grounded in textbook content with source citations</li>
                    <li><strong>Chapter Personalization:</strong> Prioritize content from specific chapters</li>
                    <li><strong>100% FREE:</strong> Powered by Google Gemini and HuggingFace embeddings</li>
                  </ul>
                </div>
                <div className="card__footer">
                  <Link
                    className="button button--primary button--block"
                    to="/docs/intro">
                    Start Learning Now →
                  </Link>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
