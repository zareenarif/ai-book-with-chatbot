---
id: week-04-ros2-nodes-topics
title: 'Week 4: ROS 2 Nodes and Topics'
sidebar_label: 'Week 4: Nodes & Topics'
---

# Week 4: ROS 2 Nodes and Topics

This lesson focuses on the **core communication system of ROS 2**, which is built around **Nodes and Topics**. These two components form the foundation of how different parts of a robot communicate with each other in real time.

Understanding nodes and topics is **mandatory** before moving toward robot control, navigation, perception, and AI-based automation.

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. **Define** what a Node is in ROS 2 and explain its role in distributed systems
2. **Explain** what a Topic is and how the Publisher-Subscriber model works
3. **Create** ROS 2 nodes in Python with proper lifecycle management
4. **Implement** publishers and subscribers to exchange data between nodes
5. **Visualize** topic data flow using ROS 2 command-line tools and RViz
6. **Apply** node-topic architecture to humanoid robotics systems

---

## Prerequisites

- ROS 2 Humble installed on Ubuntu 22.04 or Docker container
- Python 3.8+ with basic understanding of classes and functions
- Completed Week 3: ROS 2 Architecture & Core Concepts
- Terminal/command-line proficiency
- Text editor or IDE (VS Code recommended)

---

## 1. Introduction

Imagine a humanoid robot trying to walk. Its camera needs to detect obstacles, its IMU needs to track balance, its motors need position commands, and its AI brain needs to coordinate everything. How do these components communicate without creating a tangled mess of dependencies?

**ROS 2 solves this with Nodes and Topics** - a elegant publish-subscribe architecture that keeps components loosely coupled, independently testable, and easily replaceable. This lesson explores how these building blocks enable modular robotics systems.

---

## 2. Conceptual Overview

### What is a Node?

A **Node** is a single executable process that performs one specific task in your robot system. Think of nodes as specialized workers in a factory:

- **Camera Node**: Captures images from robot's camera
- **Motor Controller Node**: Sends commands to motors
- **AI Perception Node**: Detects objects in images
- **Balance Controller Node**: Maintains humanoid stability

Each node runs independently and communicates with others through topics.

### What is a Topic?

A **Topic** is a named communication channel where nodes publish and subscribe to messages. It's like a radio frequency:

- Publishers broadcast messages on a topic (like a radio transmitter)
- Subscribers listen for messages on that topic (like a radio receiver)
- Multiple nodes can publish and subscribe to the same topic

**Key Principle**: Publishers and subscribers don't know about each other directly - they only know the topic name. This decoupling is what makes ROS 2 so flexible.

---

## 3. Technical Deep Dive

### Node Architecture

Every ROS 2 node inherits from the `Node` class and includes:

```
┌───────────────────────────────────┐
│         ROS 2 Node                │
│                                   │
│  ┌─────────────────────────────┐ │
│  │   Publishers                │ │
│  │   - Send messages           │ │
│  └─────────────────────────────┘ │
│                                   │
│  ┌─────────────────────────────┐ │
│  │   Subscribers               │ │
│  │   - Receive messages        │ │
│  └─────────────────────────────┘ │
│                                   │
│  ┌─────────────────────────────┐ │
│  │   Timers                    │ │
│  │   - Periodic callbacks      │ │
│  └─────────────────────────────┘ │
│                                   │
│  ┌─────────────────────────────┐ │
│  │   Parameters                │ │
│  │   - Runtime configuration   │ │
│  └─────────────────────────────┘ │
└───────────────────────────────────┘
```

### Topic Communication Flow

```
Publisher Node                    Subscriber Node
┌──────────────┐                 ┌──────────────┐
│              │                 │              │
│  publish()   │                 │  callback()  │
│      │       │                 │      ▲       │
│      ▼       │                 │      │       │
│  ┌────────┐  │                 │  ┌────────┐  │
│  │Message │  │   Topic: /data  │  │Message │  │
│  └────────┘  │ ─────────────►  │  └────────┘  │
│              │                 │              │
└──────────────┘                 └──────────────┘
        DDS Middleware (Data Distribution Service)
```

### Message Types

ROS 2 uses strongly-typed messages. Common types include:

- `std_msgs/String`: Simple text messages
- `std_msgs/Int32`: Integer values
- `geometry_msgs/Twist`: Velocity commands (linear/angular)
- `sensor_msgs/Image`: Camera images
- `sensor_msgs/JointState`: Robot joint positions

---

## 4. Diagrams

### Pub-Sub Communication Pattern

```
┌────────────────────────────────────────────────────────────┐
│                    ROS 2 Graph                             │
│                                                            │
│  ┌──────────────┐         Topic: /robot/cmd_vel          │
│  │ Teleop Node  ├──────────────────────►                 │
│  │ (Publisher)  │      geometry_msgs/Twist               │
│  └──────────────┘                          │              │
│                                             │              │
│                                             ▼              │
│                                   ┌──────────────────┐    │
│                                   │  Motor Driver    │    │
│                                   │  (Subscriber)    │    │
│                                   └──────────────────┘    │
│                                                            │
│  ┌──────────────┐         Topic: /camera/image           │
│  │ Camera Node  ├──────────────────────►                 │
│  │ (Publisher)  │      sensor_msgs/Image                 │
│  └──────────────┘                          │              │
│                                             │              │
│                                             ├──────►┌─────────────┐
│                                             │       │ Vision Node │
│                                             │       │(Subscriber) │
│                                             │       └─────────────┘
│                                             │              │
│                                             └──────►┌─────────────┐
│                                                     │  Recorder   │
│                                                     │(Subscriber) │
│                                                     └─────────────┘
└────────────────────────────────────────────────────────────┘
```

---

## 5. Code Examples

### Example 1: Simple Publisher Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create publisher on topic '/robot/status'
        self.publisher_ = self.create_publisher(
            String,
            '/robot/status',
            10  # Queue size
        )

        # Create timer to publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        self.get_logger().info('Publisher node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot status update #{self.counter}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**ROS 2 Humble Compatible** ✅

### Example 2: Simple Subscriber Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # Create subscriber on topic '/robot/status'
        self.subscription = self.create_subscription(
            String,
            '/robot/status',
            self.listener_callback,
            10
        )

        self.get_logger().info('Subscriber node started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**ROS 2 Humble Compatible** ✅

### Example 3: Humanoid Joint Publisher

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class HumanoidJointPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_joint_publisher')

        self.publisher_ = self.create_publisher(
            JointState,
            '/humanoid/joint_states',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.angle = 0.0

        self.get_logger().info('Humanoid joint publisher started')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Define joints for humanoid robot
        msg.name = ['left_hip', 'left_knee', 'left_ankle',
                    'right_hip', 'right_knee', 'right_ankle']

        # Simulate walking motion with sine wave
        self.angle += 0.05
        offset = math.sin(self.angle)

        msg.position = [
            offset * 0.3,      # left hip
            abs(offset) * 0.6, # left knee
            -offset * 0.2,     # left ankle
            -offset * 0.3,     # right hip
            abs(-offset) * 0.6,# right knee
            offset * 0.2       # right ankle
        ]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint states (angle: {self.angle:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**ROS 2 Humble Compatible** ✅

---

## 6. Hands-On Exercises

<ExerciseSection
  title="Exercise 1: Create Your First Publisher"
  difficulty="Beginner"
  description="Create a ROS 2 publisher node that publishes a counter message every second. Run it and verify the messages using command-line tools."
  hints={[
    "Use `ros2 topic echo /your_topic_name` to see published messages",
    "Remember to source your ROS 2 workspace: `source /opt/ros/humble/setup.bash`",
    "Use `ros2 node list` to verify your node is running"
  ]}
  expectedOutcome="You should see incrementing counter messages in the terminal when running `ros2 topic echo`. The node should publish at 1 Hz frequency."
/>

<ExerciseSection
  title="Exercise 2: Build a Subscriber Node"
  difficulty="Beginner"
  description="Create a subscriber node that listens to the counter topic from Exercise 1 and prints received messages with a custom format."
  hints={[
    "Use the same message type as your publisher (std_msgs/String)",
    "The subscriber callback function is triggered automatically when messages arrive",
    "Run both publisher and subscriber in separate terminals"
  ]}
  expectedOutcome="Both nodes should run simultaneously. The subscriber should display formatted messages showing it's receiving data from the publisher."
/>

<ExerciseSection
  title="Exercise 3: Temperature Sensor Simulator"
  difficulty="Intermediate"
  description="Create a publisher that simulates a temperature sensor publishing Float32 values (20-30°C range) every 0.5 seconds, and a subscriber that triggers a warning if temperature exceeds 28°C."
  hints={[
    "Use `std_msgs/Float32` for the message type",
    "Use `random.uniform(20.0, 30.0)` to generate random temperatures",
    "Add conditional logic in the subscriber callback to check temperature threshold"
  ]}
  expectedOutcome="The publisher should send varying temperature values. The subscriber should display normal readings and WARNING messages when temperature exceeds 28°C."
/>

<ExerciseSection
  title="Exercise 4: Humanoid Balance Monitor"
  difficulty="Advanced"
  description="Create a system with: (1) A publisher simulating IMU data (roll, pitch, yaw), (2) A subscriber that calculates if the humanoid is stable (roll < 5°, pitch < 5°), and logs balance status."
  hints={[
    "Use `geometry_msgs/Vector3` for roll, pitch, yaw angles",
    "Simulate IMU noise with small random variations around 0°",
    "Create a threshold checker in the subscriber that evaluates stability"
  ]}
  expectedOutcome="The IMU simulator should publish orientation data. The balance monitor should analyze the data and print 'STABLE' or 'UNSTABLE' with specific angle violations."
/>

---

## 7. Quiz

<QuizComponent
  title="Week 4 Knowledge Check"
  questions={[
    {
      id: 1,
      question: "What is a ROS 2 Node?",
      options: [
        "A single executable process that performs a specific task",
        "A data structure for storing robot configuration",
        "A type of message format",
        "A hardware component of the robot"
      ],
      correctAnswer: 0,
      explanation: "A Node is a single executable process (program) that performs one specific task in a ROS 2 system. Nodes are the building blocks of distributed robotics applications."
    },
    {
      id: 2,
      question: "What communication pattern do Topics use in ROS 2?",
      options: [
        "Request-Response",
        "Publish-Subscribe",
        "Peer-to-Peer",
        "Client-Server"
      ],
      correctAnswer: 1,
      explanation: "Topics use the Publish-Subscribe pattern where publishers send messages to a topic and subscribers receive messages from that topic. This decouples senders and receivers."
    },
    {
      id: 3,
      question: "Can multiple nodes subscribe to the same topic?",
      options: [
        "No, only one subscriber per topic is allowed",
        "Yes, but only if they're on the same computer",
        "Yes, any number of nodes can subscribe to the same topic",
        "No, it would cause data corruption"
      ],
      correctAnswer: 2,
      explanation: "Yes! One of the key benefits of the pub-sub model is that multiple subscribers can listen to the same topic simultaneously, each receiving a copy of the published messages."
    },
    {
      id: 4,
      question: "What does the 'queue size' parameter (10) mean in create_publisher()?",
      options: [
        "Maximum number of nodes that can subscribe",
        "Maximum number of messages to buffer if sending faster than network can transmit",
        "Number of topics the publisher can create",
        "The publishing frequency in Hz"
      ],
      correctAnswer: 1,
      explanation: "Queue size specifies how many messages to buffer if the publisher is producing messages faster than the network can send them. A queue of 10 holds up to 10 pending messages."
    },
    {
      id: 5,
      question: "What command lists all active topics in a ROS 2 system?",
      options: [
        "ros2 node list",
        "ros2 topic list",
        "ros2 msg list",
        "ros2 param list"
      ],
      correctAnswer: 1,
      explanation: "`ros2 topic list` displays all currently active topics in the system. Use `ros2 topic echo /topic_name` to see the messages being published."
    },
    {
      id: 6,
      question: "In the Publisher-Subscriber model, do publishers and subscribers know about each other directly?",
      options: [
        "Yes, they must establish a direct connection first",
        "No, they only know the topic name they're communicating on",
        "Yes, but only the publisher knows about subscribers",
        "It depends on the message type being used"
      ],
      correctAnswer: 1,
      explanation: "No! This is the power of decoupling. Publishers and subscribers only know the topic name. They don't know about each other's existence, making the system modular and flexible."
    },
    {
      id: 7,
      question: "What happens if a subscriber callback function takes too long to execute?",
      options: [
        "The node crashes immediately",
        "Incoming messages may be queued or dropped",
        "Other nodes stop publishing",
        "Nothing, ROS 2 waits indefinitely"
      ],
      correctAnswer: 1,
      explanation: "If the callback takes too long, incoming messages will queue up (up to the queue size) and then start dropping. This is why callbacks should be fast and offload heavy processing to separate threads if needed."
    },
    {
      id: 8,
      question: "Which message type would you use to send velocity commands to a robot?",
      options: [
        "std_msgs/String",
        "sensor_msgs/JointState",
        "geometry_msgs/Twist",
        "std_msgs/Float32"
      ],
      correctAnswer: 2,
      explanation: "`geometry_msgs/Twist` is the standard message type for velocity commands, containing linear (x, y, z) and angular (roll, pitch, yaw) velocity components."
    }
  ]}
/>

---

## 8. Summary

**Key Takeaways**:

- **Nodes** are independent executable processes that perform specific tasks in a ROS 2 system
- **Topics** are named communication channels using the Publish-Subscribe pattern
- **Publishers** send messages to topics; **Subscribers** receive messages from topics
- Publishers and subscribers are **loosely coupled** - they don't know about each other directly
- **Multiple publishers and subscribers** can use the same topic simultaneously
- ROS 2 uses **strongly-typed messages** (std_msgs, sensor_msgs, geometry_msgs)
- **Queue size** determines how many messages to buffer when sending/receiving
- **DDS middleware** handles the underlying network communication automatically
- Node-topic architecture enables **modular, distributed, fault-tolerant** robotics systems
- Essential command-line tools: `ros2 topic list`, `ros2 topic echo`, `ros2 node list`

---

## 9. Glossary

- **Node**: A single executable process in ROS 2 that performs a specific task
- **Topic**: A named communication channel for message exchange between nodes
- **Publisher**: A node component that sends messages to a topic
- **Subscriber**: A node component that receives messages from a topic
- **Publish-Subscribe (Pub-Sub)**: A messaging pattern where senders (publishers) and receivers (subscribers) are decoupled through an intermediary (topic)
- **Message Type**: The data structure format for messages (e.g., String, Int32, Twist)
- **Queue Size**: The number of messages to buffer when publishing/subscribing faster than processing
- **Callback Function**: A function automatically called when a subscriber receives a message
- **DDS (Data Distribution Service)**: The middleware layer ROS 2 uses for network communication
- **rclpy**: The ROS 2 Client Library for Python
- **Spin**: The function that keeps a node running and processing callbacks

---

## 10. Further Reading

1. **ROS 2 Humble Official Documentation - Nodes**
   - [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
   - Official tutorial on ROS 2 nodes with command-line tools

2. **ROS 2 Humble Official Documentation - Topics**
   - [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
   - Comprehensive guide to topics, including visualization and introspection

3. **Writing a Simple Publisher and Subscriber (Python)**
   - [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
   - Step-by-step tutorial with complete code examples

4. **About ROS 2 Interfaces (Messages, Services, Actions)**
   - [https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
   - Deep dive into message types and custom interface creation

5. **DDS and ROS 2 Middleware**
   - [https://design.ros2.org/articles/ros_on_dds.html](https://design.ros2.org/articles/ros_on_dds.html)
   - Technical details on how ROS 2 uses DDS for communication

---

**Version**: ROS 2 Humble
**License**: CC BY-SA 4.0
