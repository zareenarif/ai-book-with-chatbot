import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ChapterCard from '@site/src/components/ChapterCard';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome`}
      description="Physical AI & Humanoid Robotics - A Complete Textbook">
      <header className="bg-gradient-to-r from-primary-600 to-secondary-600 text-white py-20">
        <div className="container mx-auto px-4">
          <h1 className="text-5xl font-bold mb-4">
            Physical AI & Humanoid Robotics
          </h1>
          <p className="text-2xl mb-8">
            A Complete Guide to ROS 2, Simulation, and Advanced Robotics
          </p>
          <div className="flex space-x-4">
            <Link
              className="bg-white text-primary-600 px-6 py-3 rounded-lg font-semibold hover:bg-gray-100 transition-colors no-underline"
              to="/docs/book/module-1-ros2/">
              Start Learning
            </Link>
            <Link
              className="border-2 border-white text-white px-6 py-3 rounded-lg font-semibold hover:bg-white hover:text-primary-600 transition-colors no-underline"
              to="https://github.com/zareenarif/ai-book">
              View on GitHub
            </Link>
          </div>
        </div>
      </header>

      <main className="container mx-auto px-4 py-16">
        <section className="mb-16">
          <h2 className="text-4xl font-bold text-center mb-4">Course Overview</h2>
          <p className="text-xl text-center text-gray-600 dark:text-gray-400 max-w-3xl mx-auto mb-12">
            This comprehensive textbook covers 13 weeks of Physical AI and Humanoid Robotics curriculum,
            organized into 4 modules with hands-on exercises and real-world applications.
          </p>
        </section>

        <section className="grid grid-cols-1 md:grid-cols-2 gap-8 mb-16">
          <ChapterCard
            moduleNumber={1}
            title="ROS 2 Foundations"
            description="Introduction to Physical AI, Humanoid Robotics, and ROS 2 architecture. Learn core concepts, nodes, topics, and communication patterns."
            lessonCount={4}
            href="/docs/book/module-1-ros2/"
            topics={['Physical AI', 'ROS 2', 'Pub/Sub', 'Services']}
          />

          <ChapterCard
            moduleNumber={2}
            title="Simulation Environments"
            description="Master Gazebo, Unity, and NVIDIA Isaac Sim for robotics simulation. Build and test robots in virtual environments."
            lessonCount={3}
            href="/docs/book/module-2-simulation/"
            topics={['Gazebo', 'Unity', 'Isaac Sim', 'Physics']}
          />

          <ChapterCard
            moduleNumber={3}
            title="Sensors & AI Integration"
            description="Integrate LiDAR, cameras, and IMU sensors with Vision-Language-Action models for intelligent robotics."
            lessonCount={2}
            href="/docs/book/module-3-sensors/"
            topics={['LiDAR', 'Computer Vision', 'VLA Models', 'Perception']}
          />

          <ChapterCard
            moduleNumber={4}
            title="Advanced Humanoid Robotics"
            description="Explore conversational AI, locomotion algorithms, inverse kinematics, and capstone integration projects."
            lessonCount={4}
            href="/docs/book/module-4-advanced/"
            topics={['Conversational AI', 'Locomotion', 'IK/Dynamics', 'Capstone']}
          />
        </section>

        <section className="bg-gray-50 dark:bg-gray-800 p-8 rounded-lg">
          <h2 className="text-3xl font-bold mb-6">What You'll Learn</h2>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            <div>
              <h3 className="text-xl font-semibold mb-2 text-primary-600">Foundational Skills</h3>
              <ul className="space-y-2 text-gray-700 dark:text-gray-300">
                <li>✓ ROS 2 architecture and core concepts</li>
                <li>✓ Python programming for robotics</li>
                <li>✓ Simulation environment setup</li>
              </ul>
            </div>
            <div>
              <h3 className="text-xl font-semibold mb-2 text-primary-600">Applied Techniques</h3>
              <ul className="space-y-2 text-gray-700 dark:text-gray-300">
                <li>✓ Sensor integration and processing</li>
                <li>✓ Vision-Language-Action pipelines</li>
                <li>✓ Humanoid locomotion algorithms</li>
              </ul>
            </div>
            <div>
              <h3 className="text-xl font-semibold mb-2 text-primary-600">Advanced Projects</h3>
              <ul className="space-y-2 text-gray-700 dark:text-gray-300">
                <li>✓ Conversational robot interfaces</li>
                <li>✓ Inverse kinematics solvers</li>
                <li>✓ Full-stack robotics integration</li>
              </ul>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
