---
id: week-10-conversational-ai
title: ' Conversational AI'
sidebar_label: 'week 10 : Conversational AI'
---


# Conversational AI

Conversational AI enables robots and systems to **understand, process, and respond to natural language**. By combining **speech recognition, natural language understanding, and dialogue management**, robots can interact with humans **intelligently and naturally**.

This lesson introduces **the fundamentals, architecture, and ROS 2 integration** of Conversational AI in robotics.

---

## Learning Objectives

By the end of this lesson, students will be able to:

- Understand what **Conversational AI** is  
- Explain **speech-to-text and text-to-speech systems**  
- Understand **Natural Language Understanding (NLU) and Dialogue Management**  
- Integrate Conversational AI with **ROS 2 Humble**  
- Enable humanoid robots to **receive and respond to verbal instructions**  
- Implement simple voice-controlled tasks in **simulation or real robots**  
- Understand the use of **AI and ML models** for human–robot interaction  

---

## Prerequisites

- Completed Week 1-8: ROS 2, simulation, and sensor fundamentals
- Completed Week 9: Vision-Language-Action Models
- Python 3.8+ with speech recognition libraries
- Basic understanding of Natural Language Processing (NLP)
- Familiarity with audio processing and speech APIs
- ROS 2 Humble workspace configured
- Microphone and audio output device for testing

---

## 1. What is Conversational AI?

Conversational AI is a technology that allows robots to:

- Listen and understand human speech  
- Process instructions or queries  
- Respond intelligently  
- Engage in multi-turn conversations  

✅ Example:
- Human: “Bring me the red cube.”  
- Robot: Detects red cube → Grasps → Moves to human → Confirms: “Here is the red cube.”

---

## 2. Importance of Conversational AI in Robotics

Conversational AI enables:

- Hands-free robot control  
- Human–robot interaction (HRI)  
- Smart assistants and service robots  
- Accessibility for disabled users  
- Multi-modal AI integration (vision + language + action)  

✅ Humanoid and service robots rely heavily on conversational AI for **effective interaction**.

---

## 3. Architecture of Conversational AI

### 1. Speech Recognition
- Converts spoken language into text  
- Examples: Google Speech-to-Text, Whisper  

### 2. Natural Language Understanding (NLU)
- Parses text into structured data  
- Identifies **intents** and **entities**  

### 3. Dialogue Management
- Determines **appropriate responses or actions**  
- Can be rule-based or AI-based  

### 4. Text-to-Speech (TTS)
- Converts robot response text into speech  
- Examples: Google TTS, Amazon Polly  

✅ End-to-end workflow:  
**Human speech → ASR → NLU → Dialogue Manager → TTS → Robot speech/action**

---

## 4. ROS 2 Integration

Conversational AI can be integrated with ROS 2 using:

- ROS 2 topics:
  - `/voice_input` → Publishes user speech
  - `/voice_output` → Robot’s speech
  - `/robot_cmd` → Commands for motion  
- ROS 2 nodes:
  - ASR Node (speech recognition)
  - NLU Node (intent parsing)
  - Action Node (robot motion execution)
  - TTS Node (robot response)

✅ Enables **real-time human–robot interaction**.

---

## 5. Practical Examples

### Example 1: Voice-Controlled Robot
- Human: “Move forward 1 meter”
- ASR converts speech to text
- NLU identifies intent: move
- Action node executes `/cmd_vel` command
- Robot moves forward

### Example 2: Query-Based Assistance
- Human: “Where is the red cube?”  
- Robot uses camera vision + NLP to answer  
- Robot responds: “The red cube is on the table”

### Example 3: Multi-Turn Conversation
- Human: “Pick up the cube” → Robot: “Which cube?”  
- Human: “The red one” → Robot executes action  

---

## 6. Tools & Technologies Used

- ROS 2 Humble  
- Python / C++  
- Speech-to-Text APIs (Google, Whisper)  
- Text-to-Speech APIs (Google TTS, Amazon Polly)  
- NLP Libraries (Rasa, Hugging Face Transformers)  
- OpenCV (optional, for vision)  
- Unity / Gazebo / Isaac Sim (for simulation)  

---

## 🧪 7. Hands-On Exercises

### Exercise 1: Setup Speech Recognition Node with Google Speech API (Beginner)
**Objective**: Create a ROS 2 node that listens to microphone input and publishes recognized text

**Steps**:
1. Install required dependencies:
```bash
pip install SpeechRecognition pyaudio google-cloud-speech
sudo apt-get install portaudio19-dev python3-pyaudio
```
2. Create ROS 2 package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python speech_recognition_node
cd speech_recognition_node/speech_recognition_node
```
3. Create `asr_node.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        self.publisher = self.create_publisher(String, '/voice_input', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.get_logger().info('ASR Node started. Listening...')
        self.listen_loop()

    def listen_loop(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            while rclpy.ok():
                try:
                    audio = self.recognizer.listen(source, timeout=5)
                    text = self.recognizer.recognize_google(audio)
                    msg = String()
                    msg.data = text
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Recognized: {text}')
                except sr.WaitTimeoutError:
                    pass
                except sr.UnknownValueError:
                    self.get_logger().warn('Could not understand audio')

def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
4. Build and test:
```bash
cd ~/ros2_ws
colcon build --packages-select speech_recognition_node
source install/setup.bash
ros2 run speech_recognition_node asr_node
```
5. Monitor output: `ros2 topic echo /voice_input`

---

### Exercise 2: Create Intent Parser with Spacy NLP (Intermediate)
**Objective**: Parse user speech into intents and entities for robot commands

**Tasks**:
1. Install NLP libraries:
```bash
pip install spacy
python -m spacy download en_core_web_sm
```
2. Create `nlu_node.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import spacy
import re

class NLUNode(Node):
    def __init__(self):
        super().__init__('nlu_node')
        self.subscription = self.create_subscription(String, '/voice_input', self.parse_intent, 10)
        self.publisher = self.create_publisher(String, '/robot_intent', 10)
        self.nlp = spacy.load('en_core_web_sm')

        # Define intent patterns
        self.patterns = {
            'move': r'(move|go|travel|navigate)\s+(forward|backward|left|right)',
            'pick': r'(pick|grab|grasp)\s+(up\s+)?(the\s+)?(\w+)',
            'place': r'(place|put|drop)\s+(\w+)',
            'query': r'(where|what|find)\s+(is|are)\s+(the\s+)?(\w+)'
        }

    def parse_intent(self, msg):
        text = msg.data.lower()
        doc = self.nlp(text)

        intent_msg = String()
        for intent, pattern in self.patterns.items():
            match = re.search(pattern, text)
            if match:
                entities = [ent.text for ent in doc.ents]
                intent_msg.data = f"intent:{intent}|entities:{','.join(entities)}|text:{text}"
                self.publisher.publish(intent_msg)
                self.get_logger().info(f'Parsed intent: {intent}')
                return

        intent_msg.data = f"intent:unknown|text:{text}"
        self.publisher.publish(intent_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NLUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
3. Test with sample commands: "Move forward", "Pick up the red cube", "Where is the ball?"

---

### Exercise 3: Implement Robot Action Executor (Advanced)
**Objective**: Execute robot actions based on parsed intents

**Tasks**:
1. Create `action_node.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')
        self.subscription = self.create_subscription(String, '/robot_intent', self.execute_action, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.response_pub = self.create_publisher(String, '/voice_output', 10)

    def execute_action(self, msg):
        parts = msg.data.split('|')
        intent_data = {}
        for part in parts:
            key, value = part.split(':')
            intent_data[key] = value

        intent = intent_data.get('intent')
        text = intent_data.get('text', '')

        if intent == 'move':
            self.handle_movement(text)
        elif intent == 'pick':
            self.handle_pick(intent_data.get('entities', ''))
        elif intent == 'query':
            self.handle_query(text)
        else:
            self.respond("I didn't understand that command")

    def handle_movement(self, text):
        twist = Twist()
        if 'forward' in text:
            twist.linear.x = 0.5
        elif 'backward' in text:
            twist.linear.x = -0.5
        elif 'left' in text:
            twist.angular.z = 0.5
        elif 'right' in text:
            twist.angular.z = -0.5

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Executing movement: {text}')
        self.respond("Moving as requested")

    def handle_pick(self, entities):
        self.get_logger().info(f'Executing pick action for: {entities}')
        self.respond(f"Picking up {entities}")

    def handle_query(self, text):
        self.respond("Processing your query")

    def respond(self, text):
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)
        self.get_logger().info(f'Response: {text}')

def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
2. Launch all nodes together:
```bash
ros2 run speech_recognition_node asr_node &
ros2 run speech_recognition_node nlu_node &
ros2 run speech_recognition_node action_node &
```

---

### Exercise 4: Add Text-to-Speech Response System (Intermediate)
**Objective**: Enable robot to speak responses using TTS

**Tasks**:
1. Install TTS library:
```bash
pip install pyttsx3 gTTS playsound
```
2. Create `tts_node.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(String, '/voice_output', self.speak, 10)
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 0.9)

    def speak(self, msg):
        text = msg.data
        self.get_logger().info(f'Speaking: {text}')
        self.engine.say(text)
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
3. Test end-to-end conversation: Speak → ASR → NLU → Action → TTS
4. Monitor all topics:
```bash
ros2 topic list
ros2 topic echo /voice_input
ros2 topic echo /robot_intent
ros2 topic echo /voice_output
```

---

### Exercise 5: Implement Multi-Turn Dialogue with Context (Advanced)
**Objective**: Enable robot to maintain conversation context across multiple turns

**Tasks**:
1. Create `dialogue_manager.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DialogueManager(Node):
    def __init__(self):
        super().__init__('dialogue_manager')
        self.sub = self.create_subscription(String, '/robot_intent', self.process_dialogue, 10)
        self.pub = self.create_publisher(String, '/voice_output', 10)

        # Conversation context
        self.context = {
            'last_intent': None,
            'pending_clarification': None,
            'objects_mentioned': []
        }

    def process_dialogue(self, msg):
        parts = msg.data.split('|')
        intent_data = {}
        for part in parts:
            key, value = part.split(':')
            intent_data[key] = value

        intent = intent_data.get('intent')
        entities = intent_data.get('entities', '').split(',')

        # Multi-turn logic
        if self.context['pending_clarification']:
            self.handle_clarification(intent, entities)
        elif intent == 'pick' and not entities:
            self.request_clarification("Which object should I pick up?")
        else:
            self.context['last_intent'] = intent
            self.context['objects_mentioned'].extend(entities)
            self.respond(f"Executing {intent} action")

    def request_clarification(self, question):
        self.context['pending_clarification'] = self.context['last_intent']
        msg = String()
        msg.data = question
        self.pub.publish(msg)

    def handle_clarification(self, intent, entities):
        self.get_logger().info(f'Clarification received: {entities}')
        self.context['pending_clarification'] = None
        self.respond(f"Got it, working on {entities[0]}")

    def respond(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DialogueManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
2. Test conversation flow:
   - User: "Pick up the cube"
   - Robot: "Which cube?" (clarification)
   - User: "The red one"
   - Robot: "Got it, working on red cube"

---

## 8. Knowledge Check Quiz

**Question 1**: What is the correct order of components in a Conversational AI pipeline for robotics?

- A) TTS → NLU → ASR → Dialogue Manager
- B) ASR → NLU → Dialogue Manager → TTS ✓
- C) NLU → ASR → TTS → Dialogue Manager
- D) Dialogue Manager → ASR → NLU → TTS

**Answer**: B. The correct flow is: Speech Recognition (ASR) converts speech to text, Natural Language Understanding (NLU) extracts intent and entities, Dialogue Manager determines the response/action, and Text-to-Speech (TTS) converts the robot's response back to audio.

---

**Question 2**: What is the primary function of Natural Language Understanding (NLU) in Conversational AI?

- A) Convert speech to text
- B) Generate synthetic speech
- C) Extract intents and entities from text ✓
- D) Control robot motors

**Answer**: C. NLU parses structured information (intents and entities) from text. For example, from "Pick up the red cube," NLU extracts intent: "pick" and entities: "red cube".

---

**Question 3**: Which ROS 2 topic pattern is typically used for voice-controlled robot systems?

- A) `/camera/image` for vision input
- B) `/voice_input` for ASR output, `/robot_cmd` for actions ✓
- C) `/odom` for robot position
- D) `/joint_states` for motor control

**Answer**: B. Voice-controlled systems typically use `/voice_input` to publish recognized speech, `/robot_intent` for parsed commands, `/robot_cmd` for execution, and `/voice_output` for TTS responses.

---

**Question 4**: What is a key challenge when implementing multi-turn conversations in robotics?

- A) Installing microphones
- B) Maintaining conversation context and state ✓
- C) Publishing ROS 2 topics
- D) Training neural networks

**Answer**: B. Multi-turn conversations require the system to maintain context (what was previously discussed, pending clarifications, referenced objects) across multiple interactions, which requires stateful dialogue management.

---

**Question 5**: Which combination of technologies would be most suitable for implementing a voice-controlled humanoid robot?

- A) OpenCV + Gazebo + GPS
- B) Whisper ASR + Spacy NLU + ROS 2 + pyttsx3 TTS ✓
- C) TensorFlow + Arduino + MQTT
- D) Unity ML-Agents + Bluetooth + SQL

**Answer**: B. This combination provides speech recognition (Whisper), natural language understanding (Spacy), robotic middleware (ROS 2), and text-to-speech (pyttsx3) - all essential components for voice-controlled robotics.

---

## 9. Glossary

- **ASR:** Automatic Speech Recognition  
- **NLU:** Natural Language Understanding  
- **TTS:** Text-to-Speech  
- **ROS 2 Topic:** Data communication channel  
- **Dialogue Manager:** Determines robot response/action  
- **Intent:** Meaning extracted from user speech  

---

## 10. Further Reading

### Official Documentation
1. **Google Cloud Speech-to-Text API** - [https://cloud.google.com/speech-to-text/docs](https://cloud.google.com/speech-to-text/docs)
   - Comprehensive guide to implementing ASR in production systems

2. **OpenAI Whisper Documentation** - [https://github.com/openai/whisper](https://github.com/openai/whisper)
   - Open-source speech recognition model with multilingual support

3. **ROS 2 Audio Common** - [https://github.com/ros-drivers/audio_common](https://github.com/ros-drivers/audio_common)
   - ROS 2 packages for audio input/output and processing

4. **Rasa Open Source Conversational AI** - [https://rasa.com/docs/rasa/](https://rasa.com/docs/rasa/)
   - Framework for building contextual AI assistants and chatbots

### Research Papers
5. **Radford, A., et al. (2023).** "Robust Speech Recognition via Large-Scale Weak Supervision." *arXiv:2212.04356*
   - Technical paper on Whisper ASR architecture and training

6. **Thomason, J., et al. (2020).** "Improving Grounded Language Understanding in a Collaborative Environment by Interacting with Agents Through Dialogue." *IJCAI 2020*
   - Research on language grounding for human-robot interaction

7. **Tellex, S., et al. (2011).** "Understanding Natural Language Commands for Robotic Navigation and Mobile Manipulation." *AAAI 2011*
   - Foundational work on mapping language to robot actions

### Tutorials and Courses
8. **SpeechRecognition Python Library** - [https://pypi.org/project/SpeechRecognition/](https://pypi.org/project/SpeechRecognition/)
   - Easy-to-use library for speech recognition with multiple engine backends

9. **Spacy NLP Documentation** - [https://spacy.io/usage](https://spacy.io/usage)
   - Industrial-strength NLP library for intent recognition and entity extraction

10. **Natural Language Processing with Python (NLTK Book)** - [https://www.nltk.org/book/](https://www.nltk.org/book/)
    - Comprehensive introduction to NLP concepts and implementation

### Human-Robot Interaction
11. **Mavridis, N. (2015).** "A review of verbal and non-verbal human-robot interactive communication." *Robotics and Autonomous Systems*
    - Survey of communication modalities in HRI

12. **Huang, C. M., & Mutlu, B. (2016).** "Anticipatory Robot Control for Efficient Human-Robot Collaboration." *HRI 2016*
    - Research on improving natural interaction through prediction

---

## Lesson Summary

This lesson introduced **Conversational AI** and its integration with ROS 2 Humble. Students learned how **speech recognition, natural language understanding, and dialogue management** enable humanoid and autonomous robots to interact intelligently with humans. Practical applications include **voice-controlled movement, query-based assistance, and multi-turn conversations**.

---

📌 *This lesson prepares students for advanced human–robot interaction, voice-controlled robotics, and AI-integrated autonomous systems.*

---

**Version**: ROS 2 Humble  
**License**: CC BY-SA 4.0
