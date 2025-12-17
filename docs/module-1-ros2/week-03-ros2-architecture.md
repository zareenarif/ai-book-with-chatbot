---
id: week-03-ros2-architecture
title: 'week 3 : ROS 2 Architecture'
sidebar_label: 'week 3 : ROS 2 Architecture'
---

# ROS 2 Architecture

ROS 2 (Robot Operating System 2) is a modern, flexible, and high-performance framework used for building robotic systems. This lesson explains the **core architecture of ROS 2**, how different components communicate, and how real-world robotic applications are structured using ROS 2.

This module is essential for understanding **how humanoid robots, autonomous vehicles, drones, and AI-powered robots are built and controlled**.

---

## Learning Objectives

By the end of this lesson, students will be able to:

- Understand what **ROS 2 is** and why it is used  
- Explain the **core components of ROS 2 architecture**  
- Understand **nodes, topics, services, and actions**  
- Describe the role of **DDS (Data Distribution Service)**  
- Understand the **ROS 2 communication model**  
- Differentiate between **ROS 1 and ROS 2**  
- Understand how ROS 2 is used in **real humanoid robotics systems**  

---

## Prerequisites

- Completed Week 1: Introduction to Physical AI
- Completed Week 2: Fundamentals of Humanoid Robotics
- Basic understanding of software architecture and communication patterns
- Familiarity with robotics terminology (sensors, actuators, nodes)
- Python 3.8+ or C++ knowledge (optional but helpful)
- Linux or Windows development environment

---

## 1. What is ROS 2?

ROS 2 is an **open-source robotic middleware framework** that helps developers build, control, and simulate robots easily.

It acts as a **bridge between hardware and software**, allowing different robot components to communicate efficiently.

### Key Uses of ROS 2:
- Robot control
- Sensor data processing
- Autonomous navigation
- Computer vision
- AI-powered robotics
- Simulation with Gazebo

---

## 2. Core ROS 2 Architecture Overview

ROS 2 follows a **distributed architecture**, meaning:

- There is **no single central master**
- Each component runs independently
- Communication happens **peer-to-peer**
- Highly **scalable and fault tolerant**

### Main Architectural Layers:

1. Application Layer (Robot Programs)
2. ROS 2 Client Libraries (Python, C++)
3. ROS 2 Middleware (DDS)
4. Operating System (Linux/Windows)
5. Hardware Layer (Sensors, Motors, Cameras)

---

## 3. Nodes in ROS 2

A **node** is a **small independent program** that performs one specific task.

### Examples of ROS 2 Nodes:
- Camera node → Publishes images  
- Motor node → Controls wheel movement  
- Lidar node → Publishes distance data  
- AI node → Performs object detection  

✅ Each robot system contains **multiple nodes running together**.

---

## 4. Topics (Publish–Subscribe Model)

Topics allow **asynchronous communication** between nodes.

- A **publisher** sends data  
- A **subscriber** receives data  
- Data flows continuously in real-time  

### Example:
- Camera Node → Publishes image data on `/camera/image`
- Vision Node → Subscribes to `/camera/image`

✅ Used for:
- Sensor data streaming
- Real-time telemetry
- Continuous robot feedback

---

## 🔁 5. Services (Request–Response Model)

Services allow **two-way communication**.

- Client sends a **request**
- Server sends a **response**

### Example:
- Turn on motor
- Reset robot position
- Start scanning

✅ Used for **short, instant commands**

---

## ⏳ 6. Actions (Long-Running Tasks)

Actions are used for **long-running tasks with continuous feedback**.

### Example:
- Navigate to a location
- Pick an object
- Walk to a target

Actions provide:
- Feedback
- Result
- Cancel option

---

## 7. DDS (Data Distribution Service)

ROS 2 uses **DDS as its communication backbone**.

### DDS Provides:
- Real-time communication
- High reliability
- Secure data transfer
- Auto-discovery of nodes
- Quality of Service (QoS) control

✅ This is a **major upgrade from ROS 1**, which used a central master.

---

## 🔐 8. Security in ROS 2

ROS 2 includes built-in **security features**, which were missing in ROS 1:

- 🔐 Encrypted communication
- 👤 Node authentication
- Permission-based access control

This makes ROS 2 suitable for:
- Military robots  
- Medical robots  
- Industrial automation  
- Autonomous vehicles  

---

## 9. ROS 1 vs ROS 2 (Quick Comparison)

| Feature | ROS 1 | ROS 2 |
|--------|--------|--------|
| Master Node | Required | ❌ Not required |
| Security | ❌ No | ✅ Built-in |
| Real-time Support | ❌ Poor | ✅ Excellent |
| Middleware | Custom | ✅ DDS |
| Windows Support | ❌ Limited | ✅ Full |
| Multi-Robot Systems | ❌ Weak | ✅ Strong |

---

## 10. ROS 2 in Humanoid Robotics

ROS 2 is widely used in humanoid robots for:

- 🚶 Walking & gait control
- 👁️ Vision and perception
- 🦾 Arm manipulation
- 🧠 AI decision-making
- Sensor integration
- ⚖️ Balance and posture control

Popular humanoid robots using ROS 2:
- NASA Valkyrie
- SoftBank Pepper (new systems)
- Research humanoid platforms

---

## 11. Programming Languages in ROS 2

ROS 2 supports multiple programming languages:

- 🐍 Python → Easy & beginner-friendly
- ⚙️ C++ → High performance & real-time control
- 🧪 Java → Experimental use

---

## 🧪 12. Practical Applications of ROS 2

- Humanoid robots  
- Autonomous vehicles  
- Drones  
- Smart factories  
- Medical robotics  
- AI research  

---

## 13. Tools Used with ROS 2

- ROS 2 CLI
- Gazebo Simulator
- RViz Visualization Tool
- OpenCV
- TensorFlow / PyTorch
- Arduino & Microcontrollers

---

## 🧪 14. Hands-On Activities

### Exercise 1: Create Your First ROS 2 Node (Beginner)
**Objective**: Create a simple ROS 2 node that publishes messages

**Steps**:
1. Install ROS 2 Humble: `sudo apt install ros-humble-desktop`
2. Source ROS 2: `source /opt/ros/humble/setup.bash`
3. Create a workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_node
```
4. Create a Python publisher node in `my_first_node/my_first_node/`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(2.0, self.publish_status)
        self.counter = 0

    def publish_status(self):
        msg = String()
        msg.data = f'Robot status update #{self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
5. Build and run: `colcon build && source install/setup.bash && ros2 run my_first_node simple_publisher`

---

### Exercise 2: Publisher-Subscriber Demo (Intermediate)
**Objective**: Create two nodes that communicate via topics

**Tasks**:
1. Create a subscriber node that listens to `/robot_status`
2. Print received messages with timestamps
3. Run both publisher and subscriber in separate terminals
4. Use `ros2 topic list` and `ros2 topic echo /robot_status` to observe communication

**Expected Output**:
```
[Publisher] Publishing: Robot status update #5
[Subscriber] Received: Robot status update #5 at 12:45:30
```

---

### Exercise 3: Service-Based Motor Control (Advanced)
**Objective**: Create a service for controlling robot motor speed

**Tasks**:
1. Define a custom service interface `SetMotorSpeed.srv`:
```
float32 target_speed
---
bool success
string message
```
2. Create a service server that accepts speed requests
3. Create a client that sends speed commands
4. Test with: `ros2 service call /set_motor_speed example_interfaces/srv/SetMotorSpeed "{target_speed: 5.5}"`

---

### Exercise 4: Visualize Topics in RViz (Intermediate)
**Objective**: Learn to visualize sensor data

**Steps**:
1. Launch RViz: `ros2 run rviz2 rviz2`
2. Add topic displays for camera, lidar, or robot state
3. Configure coordinate frames
4. Save configuration for reuse

---

## 15. Knowledge Check Quiz

**Question 1**: What is the main architectural difference between ROS 1 and ROS 2?

- A) ROS 2 uses Python while ROS 1 uses C++
- B) ROS 2 eliminates the central master node and uses DDS ✓
- C) ROS 2 is slower than ROS 1
- D) ROS 2 cannot run on Windows

**Answer**: B. ROS 2 uses a distributed architecture with DDS, removing the single point of failure from ROS 1's master node.

---

**Question 2**: Which communication pattern is best for long-running tasks with feedback?

- A) Topics
- B) Services
- C) Actions ✓
- D) Parameters

**Answer**: C. Actions provide goal, feedback, and result mechanisms ideal for long-duration tasks like navigation.

---

**Question 3**: What does DDS stand for in ROS 2?

- A) Direct Data Service
- B) Data Distribution Service ✓
- C) Dynamic Device System
- D) Distributed Data Sensor

**Answer**: B. DDS (Data Distribution Service) is the middleware layer providing real-time, reliable communication in ROS 2.

---

**Question 4**: Which ROS 2 component would you use for real-time sensor data streaming?

- A) Services
- B) Actions
- C) Topics ✓
- D) Parameters

**Answer**: C. Topics use the publish-subscribe pattern, perfect for continuous sensor data streams.

---

**Question 5**: What quality does QoS (Quality of Service) control in ROS 2?

- A) Code quality
- B) Communication reliability, latency, and durability ✓
- C) Graphics quality
- D) CPU performance

**Answer**: B. QoS policies in ROS 2 allow fine-tuned control over message delivery guarantees, history depth, and reliability.

---

## 16. Glossary

- **Node:** An independent process performing a specific computational task in ROS 2
- **Topic:** A named bus for asynchronous message streaming using publish-subscribe pattern
- **Service:** Synchronous request-response communication between client and server nodes
- **Action:** Asynchronous goal-based communication for long-running tasks with feedback
- **DDS:** Data Distribution Service - the middleware providing real-time pub-sub communication
- **QoS:** Quality of Service policies controlling message delivery characteristics
- **Publisher:** A node that sends messages to a topic
- **Subscriber:** A node that receives messages from a topic
- **Package:** A collection of ROS 2 nodes, libraries, configuration files, and dependencies
- **Workspace:** A directory containing one or more ROS 2 packages

---

## 17. Further Reading

### Official Documentation
1. **ROS 2 Documentation** - [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
   - Complete ROS 2 tutorials, API references, and best practices

2. **ROS 2 Design Documentation** - [https://design.ros2.org/](https://design.ros2.org/)
   - Architectural decisions and design rationale behind ROS 2

### DDS and Middleware
3. **OMG DDS Specification** - [https://www.omg.org/spec/DDS/](https://www.omg.org/spec/DDS/)
   - Official Data Distribution Service standard specification

4. **Macenski, S., et al. (2022).** "Robot Operating System 2: Design, architecture, and uses in the wild." *Science Robotics*, 7(66). DOI: 10.1126/scirobotics.abm6074
   - Comprehensive academic paper on ROS 2 architecture and real-world deployments

### Security
5. **ROS 2 Security Enclaves** - [https://design.ros2.org/articles/ros2_security_enclaves.html](https://design.ros2.org/articles/ros2_security_enclaves.html)
   - Security architecture and access control in ROS 2

### Humanoid Robotics Applications
6. **NASA Valkyrie ROS 2 Integration** - Research papers on humanoid control architectures
7. **Quigley, M., et al. (2015).** "ROS: an open-source Robot Operating System." *ICRA Workshop on Open Source Software*
   - Historical context from ROS 1 to understand ROS 2 evolution

### Video Tutorials
8. **The Construct ROS 2 Courses** - [https://www.theconstructsim.com/](https://www.theconstructsim.com/)
9. **Articulated Robotics YouTube Channel** - Practical ROS 2 tutorials for beginners

---

## Lesson Summary

This lesson explained the **complete ROS 2 architecture**, including nodes, topics, services, actions, and DDS. Students learned how ROS 2 enables **real-time, secure, and scalable communication** between robot components. This foundation is extremely important for building **humanoid robots, AI-driven robotic systems, and autonomous machines**.

---

📌 *This chapter prepares you for writing real ROS 2 programs, controlling robot hardware, and building full robotic systems.*
