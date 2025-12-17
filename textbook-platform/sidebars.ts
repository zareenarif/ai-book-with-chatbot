import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Foundations',
      link: {
        type: 'doc',
        id: 'book/module-1-ros2/index',
      },
      items: [
        'book/module-1-ros2/week-01-intro-physical-ai',
        'book/module-1-ros2/week-02-humanoid-fundamentals',
        'book/module-1-ros2/week-03-ros2-architecture',
        'book/module-1-ros2/week-04-ros2-nodes-topics',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      link: {
        type: 'doc',
        id: 'book/module-2-simulation/index',
      },
      items: [
        'book/module-2-simulation/week-05-gazebo-basics',
        'book/module-2-simulation/week-06-unity-robotics',
        'book/module-2-simulation/week-07-nvidia-isaac',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Sensors & AI Integration',
      link: {
        type: 'doc',
        id: 'book/module-3-sensors/index',
      },
      items: [
        'book/module-3-sensors/week-08-sensor-integration',
        'book/module-3-sensors/week-09-vision-language-action',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Advanced Humanoid Robotics',
      link: {
        type: 'doc',
        id: 'book/module-4-advanced/index',
      },
      items: [
        'book/module-4-advanced/week-10-conversational-ai',
        'book/module-4-advanced/week-11-humanoid-locomotion',
        'book/module-4-advanced/week-12-inverse-kinematics',
        'book/module-4-advanced/week-13-capstone-project',
      ],
    },
  ],
};

export default sidebars;
