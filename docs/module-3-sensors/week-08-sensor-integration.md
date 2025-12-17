---
id: week-08-sensor-integration
title: 'Week 8: Sensor Integration'
sidebar_label: 'Week 8: Sensor Integration'
---

# Week 8: Sensor Integration

Sensor Integration is a fundamental part of robotics that allows a robot to **sense, understand, and interact with the real or simulated world**. Sensors collect real-time data from the environment, and through ROS 2, this data is processed for perception, navigation, manipulation, and AI decision-making.

This lesson focuses on **integrating different sensors with ROS 2 Humble**, publishing sensor data, visualizing it, and using it for intelligent robot behavior.

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. **Understand** what sensor integration means in robotics and why it's critical
2. **Identify** different types of robotic sensors and their applications
3. **Integrate** common sensors (LiDAR, cameras, IMU) with ROS 2 Humble
4. **Publish** sensor data through ROS 2 topics using standard message types
5. **Visualize** sensor data using RViz2 for debugging and analysis
6. **Apply** sensor fusion techniques for robust perception

---

## Prerequisites

- ROS 2 Humble installed on Ubuntu 22.04
- Completed Week 4: ROS 2 Nodes and Topics
- Understanding of Publisher-Subscriber pattern
- Python 3.8+ with NumPy library
- Basic understanding of sensor physics (optional but helpful)

---

## 1. Introduction

Imagine a humanoid robot navigating a crowded room. Its cameras detect humans and obstacles, its LiDAR scans the environment for accurate distances, its IMU maintains balance while walking, and force sensors in its feet provide ground contact feedback. Without sensors, the robot would be blind, unbalanced, and unable to interact safely.

**Sensor integration transforms raw physical measurements into actionable data** that enables robots to perceive, reason, and act intelligently. This lesson explores how ROS 2's standardized message types and topic-based architecture make sensor integration modular and reusable.

---

## 2. Conceptual Overview

### What is Sensor Integration?

Sensor integration is the process of:

1. **Connecting** physical sensors to your robot
2. **Reading** raw sensor data
3. **Converting** data to ROS 2 messages
4. **Publishing** data on topics for other nodes to consume
5. **Fusing** multiple sensors for robust perception

Think of sensors as the robot's senses - vision (cameras), touch (tactile), hearing (microphones), and proprioception (IMU, encoders).

### Types of Sensors in Robotics

**Vision Sensors:**
- RGB cameras for object detection
- Depth cameras for 3D perception
- Stereo cameras for distance estimation

**Distance Sensors:**
- LiDAR for precise distance measurements
- Ultrasonic for close-range obstacle detection
- ToF (Time-of-Flight) for depth sensing

**Motion & Balance Sensors:**
- IMU (Inertial Measurement Unit): Accelerometer + Gyroscope
- Magnetometer for compass orientation
- Odometry from wheel encoders

**Interaction Sensors:**
- Force/Torque sensors for grasping
- Tactile sensors for touch detection
- Microphones for audio input

---

## 3. Technical Deep Dive

### Sensor Communication in ROS 2

ROS 2 uses three communication patterns for sensors:

```
Sensor → Driver Node → ROS 2 Topic → Processing Node → Control/AI
```

**Topics (most common)**: Continuous data streams (camera frames, LiDAR scans)
**Services**: On-demand sensor queries (take a single photo)
**Actions**: Long-running sensor tasks (SLAM mapping)

### Standard Sensor Message Types

ROS 2 provides `sensor_msgs` package with standardized types:

| Message Type | Purpose | Example Topic |
|--------------|---------|---------------|
| `sensor_msgs/Image` | Camera RGB/grayscale images | `/camera/image_raw` |
| `sensor_msgs/LaserScan` | 2D LiDAR scan data | `/scan` |
| `sensor_msgs/PointCloud2` | 3D point clouds | `/points` |
| `sensor_msgs/Imu` | Acceleration + gyroscope | `/imu/data` |
| `sensor_msgs/JointState` | Joint positions/velocities | `/joint_states` |
| `sensor_msgs/Temperature` | Temperature readings | `/temperature` |

---

## 4. Diagrams

### Sensor Data Flow Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  Sensor Integration System                  │
│                                                             │
│  Hardware Layer                                             │
│  ┌───────────┐  ┌───────────┐  ┌───────────┐              │
│  │  Camera   │  │  LiDAR    │  │    IMU    │              │
│  │  (USB)    │  │  (Serial) │  │   (I2C)   │              │
│  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘              │
│        │              │              │                      │
│  Driver Layer                                               │
│        │              │              │                      │
│   ┌────▼────┐    ┌───▼────┐    ┌───▼────┐                 │
│   │ Camera  │    │ LiDAR  │    │  IMU   │                 │
│   │ Driver  │    │ Driver │    │ Driver │                 │
│   │  Node   │    │  Node  │    │  Node  │                 │
│   └────┬────┘    └───┬────┘    └───┬────┘                 │
│        │             │             │                       │
│  ROS 2 Topics                                              │
│        │             │             │                       │
│   /camera/image  /scan       /imu/data                     │
│        │             │             │                       │
│   ┌────▼─────────────▼─────────────▼────┐                 │
│   │      Sensor Fusion Node              │                 │
│   │  (Combines sensor data for           │                 │
│   │   robust perception)                 │                 │
│   └──────────────┬───────────────────────┘                 │
│                  │                                         │
│   ┌──────────────▼───────────────┐                        │
│   │   Robot Control / AI         │                        │
│   │   (Navigation, Manipulation)  │                        │
│   └──────────────────────────────┘                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. Code Examples

### Example 1: Camera Image Publisher (Simulated)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')

        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_image)  # 10 Hz
        self.bridge = CvBridge()
        self.counter = 0

        self.get_logger().info('Camera simulator started')

    def publish_image(self):
        # Create simulated 640x480 RGB image
        img_array = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Add text to image to show it's updating
        import cv2
        cv2.putText(img_array, f'Frame {self.counter}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img_array, encoding='rgb8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'

        self.publisher_.publish(img_msg)
        self.counter += 1

        if self.counter % 10 == 0:
            self.get_logger().info(f'Published image #{self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**ROS 2 Humble Compatible** ✅
**Dependencies**: `sudo apt install ros-humble-cv-bridge python3-opencv`

### Example 2: LiDAR Scanner (Simulated)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')

        self.publisher_ = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        self.angle = 0.0

        self.get_logger().info('LiDAR simulator started')

    def publish_scan(self):
        scan = LaserScan()

        # Header
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        # Scan parameters
        scan.angle_min = -3.14159  # -180 degrees
        scan.angle_max = 3.14159   # +180 degrees
        scan.angle_increment = 0.0174533  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0

        # Generate 360 simulated range measurements
        # Simulate circular wall at 5m with an object at 2m
        num_readings = 360
        scan.ranges = []

        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment

            # Simulate object in front (between -30 and +30 degrees)
            if -0.5 < angle < 0.5:
                distance = 2.0 + 0.1 * math.sin(self.angle)
            else:
                distance = 5.0 + 0.2 * math.sin(angle * 3 + self.angle)

            scan.ranges.append(distance)

        self.angle += 0.1
        self.publisher_.publish(scan)
        self.get_logger().info(f'Published {len(scan.ranges)} laser readings')

def main(args=None):
    rclpy.init(args=args)
    node = LidarSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**ROS 2 Humble Compatible** ✅

### Example 3: IMU Data Publisher

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import random

class ImuSimulator(Node):
    def __init__(self):
        super().__init__('imu_simulator')

        self.publisher_ = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        self.timer = self.create_timer(0.01, self.publish_imu)  # 100 Hz
        self.time = 0.0

        self.get_logger().info('IMU simulator started')

    def publish_imu(self):
        imu = Imu()

        # Header
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Simulate humanoid walking motion (oscillating acceleration)
        # Linear acceleration (m/s²)
        imu.linear_acceleration.x = 0.0 + 0.5 * math.sin(self.time * 2)
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81 + 0.1 * math.sin(self.time * 3)

        # Angular velocity (rad/s) - small swaying motion
        imu.angular_velocity.x = 0.05 * math.sin(self.time * 1.5)
        imu.angular_velocity.y = 0.03 * math.cos(self.time * 2)
        imu.angular_velocity.z = 0.01 * math.sin(self.time)

        # Add realistic sensor noise
        imu.linear_acceleration.x += random.gauss(0, 0.01)
        imu.linear_acceleration.z += random.gauss(0, 0.01)

        # Orientation (quaternion) - upright position with slight tilt
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = 1.0

        self.publisher_.publish(imu)
        self.time += 0.01

def main(args=None):
    rclpy.init(args=args)
    node = ImuSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**ROS 2 Humble Compatible** ✅

---

## 6. Hands-On Exercises

<ExerciseSection
  title="Exercise 1: Run and Visualize Sensors"
  difficulty="Beginner"
  description="Run all three simulator nodes (camera, LiDAR, IMU) simultaneously and visualize them using RViz2. Add appropriate displays for each sensor type."
  hints={[
    "Launch RViz2: `rviz2`",
    "Add Image display for camera, LaserScan display for LiDAR, TF for IMU frame",
    "Set Fixed Frame to 'laser_frame' or 'imu_link'",
    "Use `ros2 topic hz /topic_name` to check publishing rate"
  ]}
  expectedOutcome="RViz2 should display simulated camera images, LiDAR scan visualization showing circular pattern with object, and IMU data being published at 100Hz."
/>

<ExerciseSection
  title="Exercise 2: Temperature Sensor Integration"
  difficulty="Intermediate"
  description="Create a temperature sensor node that publishes Float32 messages. Add safety logic: if temperature exceeds threshold, publish a warning message to /robot/alerts topic."
  hints={[
    "Use `std_msgs/Float32` for temperature",
    "Use `std_msgs/String` for alert messages",
    "Simulate temperature variation with sine wave + noise",
    "Create a subscriber node that monitors alerts"
  ]}
  expectedOutcome="Temperature sensor publishes readings at 1Hz. When threshold exceeded, alert messages appear on /robot/alerts topic."
/>

<ExerciseSection
  title="Exercise 3: Multi-Sensor Data Logger"
  difficulty="Advanced"
  description="Create a node that subscribes to /scan, /imu/data, and /camera/image_raw. Log timestamps and basic statistics (e.g., min/max/mean range for LiDAR) to a CSV file."
  hints={[
    "Use multiple subscriptions in one node",
    "Store data in Python lists or NumPy arrays",
    "Use Python's csv module to write data",
    "Synchronize timestamps to match sensor data"
  ]}
  expectedOutcome="CSV file contains timestamped sensor statistics. Data shows synchronized measurements from all three sensors."
/>

<ExerciseSection
  title="Exercise 4: Sensor Fusion - Obstacle Detection"
  difficulty="Advanced"
  description="Combine LiDAR scan data and IMU orientation to create an obstacle detection system. If an obstacle is detected within 1m and the robot is tilting (IMU shows roll > 10°), publish emergency stop command."
  hints={[
    "Subscribe to both /scan and /imu/data",
    "Parse LaserScan ranges to find minimum distance",
    "Convert IMU quaternion to Euler angles to get roll",
    "Publish Bool or Twist message with zero velocity for emergency stop"
  ]}
  expectedOutcome="System detects close obstacles from LiDAR. When combined with unstable IMU reading, emergency stop is triggered and logged."
/>

---

## 7. Quiz

<QuizComponent
  title="Week 8 Knowledge Check"
  questions={[
    {
      id: 1,
      question: "What is the primary purpose of sensor integration in robotics?",
      options: [
        "To make robots look more advanced",
        "To allow robots to perceive and interact with their environment",
        "To increase the robot's power consumption",
        "To reduce the cost of robots"
      ],
      correctAnswer: 1,
      explanation: "Sensor integration enables robots to perceive their environment through sensors (vision, distance, motion) and make intelligent decisions based on that data."
    },
    {
      id: 2,
      question: "Which ROS 2 message type is used for camera images?",
      options: [
        "std_msgs/Image",
        "sensor_msgs/Image",
        "geometry_msgs/Image",
        "vision_msgs/Image"
      ],
      correctAnswer: 1,
      explanation: "`sensor_msgs/Image` is the standard message type for camera images in ROS 2, containing pixel data, encoding, and metadata."
    },
    {
      id: 3,
      question: "What does an IMU (Inertial Measurement Unit) measure?",
      options: [
        "Distance to objects",
        "Temperature and humidity",
        "Acceleration and angular velocity",
        "Light intensity"
      ],
      correctAnswer: 2,
      explanation: "An IMU combines an accelerometer (linear acceleration) and gyroscope (angular velocity) to measure motion and orientation."
    },
    {
      id: 4,
      question: "What is the typical publishing frequency for IMU data in robotics?",
      options: [
        "1 Hz (once per second)",
        "10 Hz",
        "100 Hz or higher",
        "0.1 Hz (once every 10 seconds)"
      ],
      correctAnswer: 2,
      explanation: "IMUs typically publish at 100 Hz or higher for accurate motion tracking, as humanoid balance and control require high-frequency updates."
    },
    {
      id: 5,
      question: "What is sensor fusion?",
      options: [
        "Physically combining multiple sensors into one device",
        "Combining data from multiple sensors to improve accuracy",
        "Connecting sensors with fusion welding",
        "A type of sensor calibration"
      ],
      correctAnswer: 1,
      explanation: "Sensor fusion combines data from multiple sensors (e.g., camera + LiDAR) to create a more accurate and robust understanding of the environment."
    },
    {
      id: 6,
      question: "Which tool is used to visualize sensor data in ROS 2?",
      options: [
        "Gazebo",
        "RViz2",
        "RQT",
        "Plotjuggler"
      ],
      correctAnswer: 1,
      explanation: "RViz2 (ROS Visualization 2) is the primary tool for visualizing sensor data, robot models, and environmental information in real-time."
    },
    {
      id: 7,
      question: "What does LaserScan message's 'ranges' array contain?",
      options: [
        "Angles of detected objects",
        "Distances to obstacles at different angles",
        "Colors of detected objects",
        "Temperature readings"
      ],
      correctAnswer: 1,
      explanation: "The 'ranges' array in LaserScan contains distance measurements (in meters) at different angles around the robot."
    },
    {
      id: 8,
      question: "Why is sensor noise a concern in robotics?",
      options: [
        "It makes robots louder",
        "It can lead to incorrect perception and control decisions",
        "It drains battery faster",
        "It's not a concern"
      ],
      correctAnswer: 1,
      explanation: "Sensor noise introduces uncertainty in measurements, which can cause robots to make incorrect decisions. Filtering and sensor fusion help mitigate this."
    }
  ]}
/>

---

## 8. Summary

**Key Takeaways**:

- **Sensors enable robots to perceive their environment** through vision, distance, motion, and touch
- **ROS 2 standardizes sensor data** through `sensor_msgs` package (Image, LaserScan, Imu, etc.)
- **Driver nodes publish raw sensor data** on topics for processing nodes to consume
- **Common sensors**: Cameras (RGB/depth), LiDAR (2D/3D), IMU (acceleration/gyroscope), encoders
- **Sensor fusion combines multiple sensors** for robust perception (e.g., camera + LiDAR)
- **RViz2 visualizes sensor data** in real-time for debugging and analysis
- **High-frequency sensors** (IMU at 100Hz) require efficient message handling
- **Sensor noise is inevitable** - filtering and probabilistic methods address uncertainty
- **Standard topics**: `/camera/image_raw`, `/scan`, `/imu/data`, `/joint_states`
- **Humanoid robots require multi-sensor integration** for balance, navigation, and manipulation

---

## 9. Glossary

- **Sensor Integration**: Process of connecting sensors to robots and processing their data in ROS 2
- **IMU (Inertial Measurement Unit)**: Sensor measuring acceleration and angular velocity
- **LiDAR (Light Detection and Ranging)**: Laser-based distance measurement sensor
- **LaserScan**: ROS 2 message type for 2D LiDAR data (ranges at different angles)
- **PointCloud2**: ROS 2 message type for 3D point cloud data
- **Sensor Fusion**: Combining data from multiple sensors for improved accuracy
- **RViz2**: ROS 2 visualization tool for viewing sensor data and robot state
- **cv_bridge**: ROS 2 package for converting between OpenCV and ROS Image messages
- **Frame ID**: Coordinate frame name used to identify sensor position/orientation
- **Sensor Noise**: Random variations in sensor measurements due to hardware limitations
- **Depth Camera**: Camera that measures distance to each pixel (RGB-D)

---

## 10. Further Reading

1. **ROS 2 Humble sensor_msgs Documentation**
   - [https://github.com/ros2/common_interfaces/tree/humble/sensor_msgs](https://github.com/ros2/common_interfaces/tree/humble/sensor_msgs)
   - Complete reference for all sensor message types

2. **RViz2 User Guide**
   - [https://github.com/ros2/rviz/blob/ros2/README.md](https://github.com/ros2/rviz/blob/ros2/README.md)
   - How to visualize sensor data effectively

3. **cv_bridge Tutorials**
   - [https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge)
   - Converting between ROS images and OpenCV

4. **Robot Localization Package (Sensor Fusion)**
   - [https://github.com/cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)
   - EKF-based sensor fusion for odometry, IMU, and GPS

5. **LIDAR Drivers for ROS 2**
   - [https://github.com/ros-drivers](https://github.com/ros-drivers)
   - Hardware-specific LiDAR drivers compatible with ROS 2

---

**Version**: ROS 2 Humble
**License**: CC BY-SA 4.0
