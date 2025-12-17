---
slug: getting-started-with-ros2
title: Getting Started with ROS 2 - Your First Robot Application
authors: [ros2_expert]
tags: [ros2, tutorial, robotics, middleware, beginner]
---

ROS 2 (Robot Operating System 2) is the industry-standard middleware for building modern robot applications. Whether you're building a simple mobile robot or a complex humanoid system, ROS 2 provides the tools you need.

<!-- truncate -->

## Why ROS 2?

ROS 2 is a complete rewrite of the original ROS, designed for production robotics with:

- **Real-time performance** with DDS middleware
- **Multi-robot systems** with native network support
- **Security** with DDS-Security integration
- **Cross-platform** support (Linux, Windows, macOS)
- **Production-ready** for commercial applications

### ROS 2 vs ROS 1

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom TCP/UDP | DDS (Data Distribution Service) |
| Real-time | Limited | Native support |
| Security | Minimal | DDS-Security |
| Multi-robot | Requires workarounds | Native |
| Platforms | Mainly Linux | Linux, Windows, macOS |

## Installation

### Ubuntu 22.04 (Recommended)

Install ROS 2 Humble Hawksbill (LTS release):

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
```

### Source your environment

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Your First ROS 2 Node

Let's create a simple publisher-subscriber system.

### Create a workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Create a Python package

```bash
ros2 pkg create --build-type ament_python my_first_node
cd my_first_node/my_first_node
```

### Publisher Node (`talker.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0
        self.get_logger().info('Talker node started')

    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
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

### Listener Node (`listener.py`)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.message_callback,
            10
        )
        self.get_logger().info('Listener node started')

    def message_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
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

### Update `setup.py`

```python
entry_points={
    'console_scripts': [
        'talker = my_first_node.talker:main',
        'listener = my_first_node.listener:main',
    ],
},
```

### Build and run

```bash
# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run talker (in one terminal)
ros2 run my_first_node talker

# Run listener (in another terminal)
source ~/ros2_ws/install/setup.bash
ros2 run my_first_node listener
```

## Key ROS 2 Concepts

### 1. Nodes
Independent processes that perform specific tasks. In our example:
- `talker` - publishes messages
- `listener` - subscribes to messages

### 2. Topics
Named channels for message passing:
- **Pub/Sub pattern**: One-to-many communication
- **Asynchronous**: Publishers don't wait for subscribers
- **Type-safe**: Messages have defined types

### 3. Messages
Data structures passed between nodes:
- Standard messages: `std_msgs`, `geometry_msgs`, `sensor_msgs`
- Custom messages: Define your own

### 4. Quality of Service (QoS)
Configure reliability, durability, and history:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher = self.create_publisher(String, 'topic', qos)
```

## Useful ROS 2 Commands

```bash
# List active nodes
ros2 node list

# Get node info
ros2 node info /talker

# List topics
ros2 topic list

# Echo topic messages
ros2 topic echo /chatter

# Show topic info
ros2 topic info /chatter

# Publish from command line
ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello from CLI'}"

# Run rqt for visualization
rqt_graph
```

## Next Steps

Now that you have ROS 2 basics down, explore:

1. **Services** - Synchronous request/response communication
2. **Actions** - Long-running tasks with feedback
3. **Parameters** - Runtime configuration
4. **Launch files** - Start multiple nodes together
5. **Simulation** - Gazebo integration for testing

Check out our [ROS 2 Architecture lesson](/docs/book/chapters/module-1-physical-ai/week-03-ros2-architecture) for a deeper dive!

## Resources

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Design Principles](https://design.ros2.org/)
- [ROS Discourse Forum](https://discourse.ros.org/)

Happy coding with ROS 2!
