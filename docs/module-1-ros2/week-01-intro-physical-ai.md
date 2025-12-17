---
id: week-01-intro-physical-ai
title: 'Week 1: Introduction to Physical AI'
sidebar_label: 'Week 1: Intro to Physical AI'
---

# Week 1: Introduction to Physical AI

## Learning Objectives

By the end of this lesson, you will be able to:

1. **Define** Physical AI and explain how it differs from traditional AI
2. **Describe** the sensor-motor loop and its role in robotic systems
3. **Identify** real-world applications of Physical AI across industries
4. **Implement** a basic ROS 2 publisher-subscriber node demonstrating sensor-motor concepts
5. **Analyze** the challenges unique to Physical AI (latency, safety, real-world uncertainty)

## Prerequisites

- Basic understanding of artificial intelligence concepts (machine learning, neural networks)
- Familiarity with Python 3.8+
- Linux command-line basics (recommended: Ubuntu 22.04)
- No prior robotics experience required

---

## 1. Introduction

Imagine driving a car. You continuously observe the road (visual sensors), feel the steering wheel (tactile sensors), hear the engine (audio sensors), and adjust your hands and feet (motor commands) in a tight feedback loop. This **sensor-motor loop** is the essence of physical intelligence—the ability to perceive, reason, and act in the physical world.

Traditional AI excels at pattern recognition in digital domains: classifying images, translating languages, or playing chess. However, these systems lack a physical body and cannot interact with the real world. **Physical AI** bridges this gap by embedding intelligence into robots, drones, autonomous vehicles, and humanoid platforms that can perceive their environment, make decisions, and execute actions in real time.

In this lesson, we explore the foundational concepts of Physical AI, the sensor-motor loop architecture, and how ROS 2 (Robot Operating System 2) serves as the backbone for building these intelligent embodied systems.

---

## 2. What is Physical AI?

**Physical AI** (also called **Embodied AI**) refers to artificial intelligence systems that:
1. **Perceive** the environment through sensors (cameras, LiDAR, IMU, tactile)
2. **Reason** about sensory input using algorithms (classical control, machine learning, planning)
3. **Act** on the environment through actuators (motors, grippers, wheels)
4. **Learn** from interaction to improve performance over time

Unlike traditional AI, which operates on static datasets or simulated environments, Physical AI must handle:
- **Real-time constraints**: Decisions must be made within milliseconds
- **Uncertainty**: Sensors are noisy, actuators are imprecise
- **Safety**: Errors can cause physical harm
- **Generalization**: Robots encounter infinite variations of real-world scenarios

---

## 3. The Sensor-Motor Loop

The sensor-motor loop is the core computational pattern in robotics:

```
┌──────────────────────────────────────────────────┐
│                                                  │
│   ┌─────────┐     ┌──────────┐     ┌─────────┐ │
│   │ Sensors ├────►│ Reasoning├────►│Actuators│ │
│   └────▲────┘     │  (AI/    │     └────┬────┘ │
│        │          │  Control)│          │      │
│        │          └──────────┘          │      │
│        │                                 │      │
│        └────────── Environment ◄─────────┘      │
│                                                  │
└──────────────────────────────────────────────────┘

Figure 1: Sensor-Motor Loop Architecture
```

**Steps**:
1. **Perception**: Sensors capture environmental state (e.g., camera images, LiDAR point clouds)
2. **Processing**: AI algorithms process sensor data (e.g., object detection, SLAM)
3. **Decision**: Control logic determines motor commands (e.g., "turn left", "grasp object")
4. **Action**: Actuators execute commands, changing the environment
5. **Feedback**: New sensor readings reflect the changed state → loop repeats

---

## 4. ROS 2 Code Example

### Minimal Publisher (Sensor Simulator)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Sensor reading {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage**:
```bash
ros2 run <package_name> minimal_publisher.py
```

---

## 5. Hands-On Exercises

### Exercise 1: Echo ROS 2 Topics (Beginner)
**Tasks**:
1. Install ROS 2 Humble on Ubuntu 22.04
2. Source ROS 2: `source /opt/ros/humble/setup.bash`
3. List topics: `ros2 topic list`
4. Echo a topic: `ros2 topic echo /rosout`

### Exercise 2: Modify Publisher Frequency (Intermediate)
**Tasks**:
1. Change timer period from 1.0s to 0.1s (10 Hz)
2. Run publisher and check frequency: `ros2 topic hz /sensor_data`
3. Experiment with 100 Hz (0.01s)

### Exercise 3: Sensor-Motor Feedback Loop (Advanced)
**Tasks**:
1. Create `feedback_controller.py` node
2. Subscribe to `/sensor_data`, publish to `/control_signal`
3. Implement proportional control based on sensor threshold
4. Test closed-loop stability

---

## 6. Knowledge Check Quiz

**Question 1**: What is the primary difference between traditional AI and Physical AI?

- A) Traditional AI uses neural networks, Physical AI does not
- B) Physical AI interacts with the real world through sensors and actuators ✓
- C) Physical AI cannot learn from data
- D) Traditional AI is faster than Physical AI

**Answer**: B. Physical AI embodies intelligence in robots that perceive and act in the physical world.

---

## 7. Summary

In this lesson, we established the foundational concepts of **Physical AI** and the **sensor-motor loop**, the core architecture enabling robots to perceive, reason, and act in the real world. We explored how ROS 2 provides a modular, real-time framework for building these systems through **nodes**, **topics**, **publishers**, and **subscribers**.

**Key Takeaways**:
1. Physical AI extends traditional AI by embedding intelligence into embodied systems
2. The sensor-motor loop (perception → reasoning → action → feedback) is the fundamental pattern
3. ROS 2's publisher-subscriber model enables distributed, real-time communication
4. Python `rclpy` provides an accessible API for building ROS 2 nodes
5. Real-world constraints (latency, safety, uncertainty) distinguish Physical AI challenges

---

## 8. Glossary

- **Physical AI**: AI systems embodied in robots that perceive, reason, and act in the physical world
- **Sensor-Motor Loop**: Cyclical process of sensing → reasoning → acting → feedback
- **ROS 2**: Robot Operating System 2, an open-source middleware for building robotic applications
- **Node**: A process in ROS 2 that performs computation
- **Topic**: A named channel for asynchronous message passing between nodes
- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic

---

## 9. Further Reading

1. **Official ROS 2 Documentation** - [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
2. **Siciliano, B., & Khatib, O. (2016).** *Springer Handbook of Robotics* (2nd ed.). Springer. DOI: 10.1007/978-3-319-32552-1
3. **Macenski, S., et al. (2022).** "Robot Operating System 2: Design, architecture, and uses in the wild." *Science Robotics*, 7(66). DOI: 10.1126/scirobotics.abm6074

---

**Version**: ROS 2 Humble Hawksbill | **Last Updated**: 2025-12-07
**License**: Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
