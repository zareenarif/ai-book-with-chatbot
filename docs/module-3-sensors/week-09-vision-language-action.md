---
id: week-09-vision-language-action
title: ' Vision-Language-Action Models'
sidebar_label: 'week 9 : Vision-Language-Action Models'
---

# Vision-Language-Action Models

Vision-Language-Action (VLA) models are **AI systems that integrate computer vision, natural language processing, and robotic action** to enable robots to understand visual input, interpret human commands, and execute appropriate physical actions. This integration is crucial for building **intelligent, autonomous humanoid robots** capable of complex real-world interactions.

This lesson introduces **VLA model architecture, ROS 2 integration, and practical applications** in humanoid robotics.

---

## Learning Objectives

By the end of this lesson, students will be able to:

- Understand what **Vision-Language-Action (VLA) models** are
- Explain how **perception, language, and action modules** work together
- Integrate VLA models with **ROS 2 Humble**
- Apply VLA for **autonomous task execution** in humanoid robots
- Implement vision processing, command parsing, and action planning
- Use VLA models in simulation environments (Gazebo, Unity, Isaac Sim)

---

## Prerequisites

- Completed Week 1-7: ROS 2 and simulation fundamentals
- Completed Week 8: Sensor Integration
- Python 3.8+ with PyTorch or TensorFlow installed
- OpenCV for computer vision tasks
- Basic understanding of machine learning and neural networks
- Familiarity with NLP concepts (tokenization, embeddings)
- ROS 2 Humble workspace configured

---

## 1. VLA Model Architecture

### 1. Perception Module
- CNNs, Vision Transformers, or Depth sensors  
- Detects objects, humans, and environment features  

### 2. Language Module
- NLP models (BERT, GPT, LLaMA, etc.)  
- Converts natural language into actionable commands  

### 3. Action Module
- Motion planning  
- Control commands  
- Robotics kinematics  

✅ This modular architecture allows **flexible integration with ROS 2 nodes**.

---

## 2. Integration with ROS 2

VLA Models interact with ROS 2 as follows:

- **Vision** → Camera node publishes `/camera/image`  
- **Language** → Text instructions sent via `/instructions` topic  
- **Action** → Action node subscribes and publishes `/cmd_vel` or `/joint_states`  

✅ Enables **real-time perception, instruction understanding, and robot motion**.

---

## 3. Practical Examples

### Example 1: Pick-and-Place
- Robot receives: `"Move the green object to the red zone"`  
- Vision detects object  
- NLP module parses command  
- Action module plans trajectory  
- Robot executes motion  

### Example 2: Navigation
- Robot receives: `"Go to the charging station"`  
- Vision detects obstacles  
- Action module generates path  
- Robot moves safely  

### Example 3: Human-Robot Interaction
- Robot answers questions: `"Where is the blue box?"`  
- Uses camera input and NLP to respond  
- Can manipulate objects if required  

---

## 4. Tools & Technologies Used

- ROS 2 Humble  
- Python / C++  
- PyTorch / TensorFlow  
- OpenCV  
- NLP Models (BERT, GPT, LLaMA)  
- Robotics libraries (MoveIt, Isaac Sim, Unity)  
- Gazebo / Unity / Isaac Sim for simulation  

---

## Hands-On Exercises

### Exercise 1: Create a Basic Camera Node in ROS 2 (Beginner)

**Objective**: Set up a ROS 2 camera node that publishes image data for vision processing.

**Steps**:
1. Install required dependencies:
   ```bash
   sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv python3-opencv
   ```

2. Create a camera publisher node (`camera_publisher.py`):
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge
   import cv2

   class CameraPublisher(Node):
       def __init__(self):
           super().__init__('camera_publisher')
           self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
           self.bridge = CvBridge()
           self.timer = self.create_timer(0.1, self.publish_image)  # 10 Hz
           self.cap = cv2.VideoCapture(0)  # Default camera

       def publish_image(self):
           ret, frame = self.cap.read()
           if ret:
               msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
               self.publisher_.publish(msg)
               self.get_logger().info('Publishing camera frame')

   def main():
       rclpy.init()
       node = CameraPublisher()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Run the node and verify output:
   ```bash
   ros2 run <package_name> camera_publisher.py
   ros2 topic hz /camera/image_raw
   ```

4. Visualize in RViz2:
   ```bash
   rviz2
   # Add -> By Topic -> /camera/image_raw -> Image
   ```

**Expected Outcome**: Camera feed published at 10 Hz, visible in RViz2.

---

### Exercise 2: Object Detection with OpenCV (Intermediate)

**Objective**: Process camera images to detect colored objects and publish detection results.

**Steps**:
1. Create an object detector node (`object_detector.py`):
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from std_msgs.msg import String
   from cv_bridge import CvBridge
   import cv2
   import numpy as np

   class ObjectDetector(Node):
       def __init__(self):
           super().__init__('object_detector')
           self.subscription = self.create_subscription(
               Image, '/camera/image_raw', self.image_callback, 10)
           self.detection_pub = self.create_publisher(String, '/detections', 10)
           self.bridge = CvBridge()

       def image_callback(self, msg):
           frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
           hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

           # Detect green objects (adjust HSV range as needed)
           lower_green = np.array([40, 50, 50])
           upper_green = np.array([80, 255, 255])
           mask = cv2.inRange(hsv, lower_green, upper_green)

           contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)

           if contours:
               largest = max(contours, key=cv2.contourArea)
               if cv2.contourArea(largest) > 500:
                   M = cv2.moments(largest)
                   cx = int(M['m10'] / M['m00'])
                   cy = int(M['m01'] / M['m00'])
                   detection_msg = String()
                   detection_msg.data = f"Green object detected at ({cx}, {cy})"
                   self.detection_pub.publish(detection_msg)
                   self.get_logger().info(detection_msg.data)

   def main():
       rclpy.init()
       node = ObjectDetector()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Test detection:
   ```bash
   ros2 run <package_name> object_detector.py
   ros2 topic echo /detections
   ```

3. Place green objects in camera view and observe detections.

**Expected Outcome**: Real-time detection of green objects with position coordinates.

---

### Exercise 3: Simple NLP Command Parser (Intermediate)

**Objective**: Parse natural language commands and extract robot actions.

**Steps**:
1. Install spaCy (lightweight NLP library):
   ```bash
   pip3 install spacy
   python3 -m spacy download en_core_web_sm
   ```

2. Create command parser node (`command_parser.py`):
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   import spacy

   class CommandParser(Node):
       def __init__(self):
           super().__init__('command_parser')
           self.subscription = self.create_subscription(
               String, '/voice_commands', self.command_callback, 10)
           self.action_pub = self.create_publisher(String, '/parsed_actions', 10)
           self.nlp = spacy.load("en_core_web_sm")

       def command_callback(self, msg):
           command = msg.data
           doc = self.nlp(command)

           # Extract verbs (actions) and nouns (objects)
           actions = [token.lemma_ for token in doc if token.pos_ == "VERB"]
           objects = [token.text for token in doc if token.pos_ == "NOUN"]
           colors = [token.text for token in doc if token.text.lower() in
                    ['red', 'green', 'blue', 'yellow', 'black', 'white']]

           parsed = {
               'action': actions[0] if actions else 'none',
               'object': objects[0] if objects else 'none',
               'color': colors[0] if colors else 'none'
           }

           action_msg = String()
           action_msg.data = f"Action: {parsed['action']}, Object: {parsed['object']}, Color: {parsed['color']}"
           self.action_pub.publish(action_msg)
           self.get_logger().info(f'Parsed: {action_msg.data}')

   def main():
       rclpy.init()
       node = CommandParser()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Test parsing:
   ```bash
   ros2 run <package_name> command_parser.py
   ros2 topic pub /voice_commands std_msgs/msg/String "data: 'pick the green box'" --once
   ros2 topic echo /parsed_actions
   ```

**Expected Outcome**: Commands like "pick the green box" parsed into action=pick, object=box, color=green.

---

### Exercise 4: VLA Integration - Vision + Language + Action (Advanced)

**Objective**: Combine vision, language, and action modules into a complete VLA pipeline.

**Steps**:
1. Create VLA controller node (`vla_controller.py`):
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist

   class VLAController(Node):
       def __init__(self):
           super().__init__('vla_controller')
           # Subscribers
           self.detection_sub = self.create_subscription(
               String, '/detections', self.detection_callback, 10)
           self.action_sub = self.create_subscription(
               String, '/parsed_actions', self.action_callback, 10)
           # Publisher
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

           self.current_detection = None
           self.current_action = None

       def detection_callback(self, msg):
           self.current_detection = msg.data
           self.get_logger().info(f'Detection: {self.current_detection}')
           self.execute_vla()

       def action_callback(self, msg):
           self.current_action = msg.data
           self.get_logger().info(f'Action: {self.current_action}')
           self.execute_vla()

       def execute_vla(self):
           if not (self.current_detection and self.current_action):
               return

           # Simple logic: if "move" action and object detected, navigate toward it
           if 'move' in self.current_action.lower() and 'detected' in self.current_detection:
               twist = Twist()
               twist.linear.x = 0.2  # Move forward
               self.cmd_vel_pub.publish(twist)
               self.get_logger().info('Executing: Move toward detected object')
           elif 'stop' in self.current_action.lower():
               twist = Twist()
               twist.linear.x = 0.0
               self.cmd_vel_pub.publish(twist)
               self.get_logger().info('Executing: Stop')

   def main():
       rclpy.init()
       node = VLAController()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Run all nodes in separate terminals:
   ```bash
   # Terminal 1
   ros2 run <package_name> camera_publisher.py

   # Terminal 2
   ros2 run <package_name> object_detector.py

   # Terminal 3
   ros2 run <package_name> command_parser.py

   # Terminal 4
   ros2 run <package_name> vla_controller.py
   ```

3. Send command and observe robot motion:
   ```bash
   ros2 topic pub /voice_commands std_msgs/msg/String "data: 'move to the green object'" --once
   ```

**Expected Outcome**: Robot detects green object, parses "move" command, and navigates toward the object.

---

### Exercise 5: Simulation in Gazebo with VLA Pipeline (Advanced)

**Objective**: Deploy VLA pipeline in Gazebo simulation with a TurtleBot3.

**Steps**:
1. Install TurtleBot3 packages:
   ```bash
   sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-simulations
   export TURTLEBOT3_MODEL=waffle_pi
   ```

2. Launch Gazebo world with colored objects:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

3. Modify `camera_publisher.py` to subscribe to Gazebo camera:
   ```python
   # Change topic to Gazebo camera
   self.subscription = self.create_subscription(
       Image, '/camera/image_raw', self.image_callback, 10)
   ```

4. Run complete VLA stack (camera, detector, parser, controller)

5. Test pick-and-place simulation:
   - Place colored blocks in Gazebo
   - Send command: `"pick the red block and place it in the blue zone"`
   - Observe VLA pipeline execution

**Expected Outcome**: TurtleBot3 autonomously navigates to colored objects based on natural language commands in simulation.

---

## Knowledge Check Quiz

### Question 1: What are the three core modules of a Vision-Language-Action (VLA) model?

- A) Perception, Communication, Navigation
- B) Vision, Language, Action
- C) Sensors, Processors, Actuators
- D) Camera, Speaker, Motors

**Answer**: B) Vision, Language, Action

**Explanation**: VLA models consist of three integrated modules: (1) **Vision/Perception** for processing visual input from cameras and sensors, (2) **Language** for understanding and parsing natural language commands using NLP, and (3) **Action** for planning and executing motor commands. While options C and D mention related hardware components, VLA specifically refers to the AI model architecture that processes perception, interprets language, and generates actions.

---

### Question 2: In a ROS 2 VLA pipeline, which topic typically carries visual sensor data?

- A) /cmd_vel
- B) /camera/image_raw
- C) /joint_states
- D) /voice_commands

**Answer**: B) /camera/image_raw

**Explanation**: In ROS 2, camera nodes publish image data to topics like `/camera/image_raw` (or `/camera/image_color`, `/camera/depth_image`, etc.). The `/cmd_vel` topic carries velocity commands for robot motion (action output), `/joint_states` publishes joint positions (action feedback), and `/voice_commands` would carry text instructions (language input). Understanding ROS 2 topic naming conventions is essential for integrating VLA components.

---

### Question 3: Which of the following best describes how VLA models differ from traditional robot control systems?

- A) VLA models use sensors while traditional systems do not
- B) VLA models integrate multimodal AI (vision + language) to enable autonomous decision-making from natural instructions
- C) VLA models are faster than traditional control systems
- D) VLA models only work in simulation environments

**Answer**: B) VLA models integrate multimodal AI (vision + language) to enable autonomous decision-making from natural instructions

**Explanation**: The key distinction is that VLA models combine **multiple AI modalities** (computer vision and NLP) to understand high-level human instructions and autonomously determine actions, whereas traditional robot control relies on pre-programmed logic or low-level commands. Both systems use sensors (A is incorrect), VLA models aren't necessarily faster due to AI inference overhead (C is incorrect), and VLA models work in both simulation and real-world environments (D is incorrect).

---

### Question 4: In the object detection exercise, why do we convert images from BGR to HSV color space?

- A) HSV reduces image file size
- B) HSV is required by ROS 2
- C) HSV separates color (Hue) from brightness, making color-based detection more robust to lighting changes
- D) HSV increases image resolution

**Answer**: C) HSV separates color (Hue) from brightness, making color-based detection more robust to lighting changes

**Explanation**: The HSV (Hue, Saturation, Value) color space separates **chromatic information (Hue)** from **brightness (Value)**, making it easier to detect objects based on color regardless of lighting conditions. In BGR/RGB color spaces, a green object under bright light and dim light would have very different pixel values, but in HSV, the Hue component remains relatively stable. This is why color segmentation tasks often use HSV. Options A, B, and D are incorrect as HSV doesn't affect file size, isn't a ROS 2 requirement, and doesn't change resolution.

---

### Question 5: What is a practical application of VLA models in humanoid robotics?

- A) Compiling source code faster
- B) Enabling robots to understand commands like "bring me the red cup from the kitchen" and autonomously execute the task
- C) Storing large datasets in databases
- D) Rendering 3D graphics in video games

**Answer**: B) Enabling robots to understand commands like "bring me the red cup from the kitchen" and autonomously execute the task

**Explanation**: VLA models enable **embodied AI** applications where robots must understand natural language instructions, perceive their environment visually, and execute complex multi-step tasks autonomously. The example "bring me the red cup from the kitchen" requires: (1) **Language understanding** to parse the instruction, (2) **Vision** to identify the red cup and navigate the kitchen, and (3) **Action** to plan grasping, navigation, and delivery motions. Options A, C, and D are unrelated to robotics and physical AI systems.

---

## Glossary

- **VLA Models:** AI models integrating vision, language, and action  
- **Perception Module:** Vision processing component  
- **Language Module:** NLP component  
- **Action Module:** Motion control and planning  
- **ROS 2 Topic:** Data communication channel  
- **Simulation Environment:** Gazebo / Unity / Isaac Sim  

---

## Further Reading

### Official Documentation
1. **ROS 2 Humble Documentation** - [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
   Comprehensive ROS 2 reference for nodes, topics, and sensor integration.

2. **OpenCV Python Documentation** - [https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
   Official OpenCV tutorials for computer vision, image processing, and object detection.

3. **spaCy NLP Library** - [https://spacy.io/usage](https://spacy.io/usage)
   Industrial-strength NLP library for command parsing and language understanding.

### Research Papers
4. **Brohan, A., et al. (2023).** "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control." *arXiv preprint arXiv:2307.15818*.
   Google DeepMind's RT-2 model demonstrating VLA integration for robotic manipulation.
   [https://arxiv.org/abs/2307.15818](https://arxiv.org/abs/2307.15818)

5. **Driess, D., et al. (2023).** "PaLM-E: An Embodied Multimodal Language Model." *Proceedings of the 40th International Conference on Machine Learning*.
   Large-scale vision-language model for embodied AI and robotic reasoning.
   [https://arxiv.org/abs/2303.03378](https://arxiv.org/abs/2303.03378)

6. **Shridhar, M., et al. (2022).** "CLIPort: What and Where Pathways for Robotic Manipulation." *Conference on Robot Learning (CoRL)*.
   Vision-language approach for pick-and-place tasks using semantic and spatial reasoning.
   [https://arxiv.org/abs/2109.12098](https://arxiv.org/abs/2109.12098)

### Tutorials and Practical Guides
7. **ROS 2 Vision Integration Tutorial** - [https://github.com/ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv)
   Official ROS perception repository with cv_bridge examples for camera integration.

8. **PyTorch Vision Models** - [https://pytorch.org/vision/stable/models.html](https://pytorch.org/vision/stable/models.html)
   Pre-trained models (ResNet, Vision Transformers) for perception modules.

9. **Hugging Face Transformers for Robotics** - [https://huggingface.co/docs/transformers/main/en/tasks/visual_question_answering](https://huggingface.co/docs/transformers/main/en/tasks/visual_question_answering)
   Vision-language models (BLIP, ViLT, LLaVA) for multimodal robot understanding.

### Case Studies and Applications
10. **NVIDIA Isaac Sim VLA Examples** - [https://docs.omniverse.nvidia.com/isaacsim/latest/](https://docs.omniverse.nvidia.com/isaacsim/latest/)
    Simulation platform with VLA model integration examples for humanoid robots.

11. **Open-X Embodiment Dataset** - [https://robotics-transformer-x.github.io/](https://robotics-transformer-x.github.io/)
    Large-scale robotic dataset with vision-language demonstrations for training VLA models.

12. **Meta's Habitat AI Platform** - [https://aihabitat.org/](https://aihabitat.org/)
    Embodied AI research platform for vision-language navigation and interaction tasks.

---

## Lesson Summary

This lesson introduced **Vision-Language-Action (VLA) models**, their architecture, ROS 2 integration, and application in humanoid and autonomous robotics. Students learned how **vision, natural language, and action modules work together** to enable robots to understand instructions, perceive environments, and perform complex tasks intelligently.

---

📌 *This lesson prepares students for advanced AI-driven humanoid robotics and autonomous systems using ROS 2 and VLA models.*

---

**Version**: ROS 2 Humble  
**License**: CC BY-SA 4.0
