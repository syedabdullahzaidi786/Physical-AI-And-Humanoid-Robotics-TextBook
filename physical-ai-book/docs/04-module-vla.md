---
id: module-vla
title: "Module 4: Vision-Language-Action (VLA)"
sidebar_position: 4
---

# Module 4: Vision-Language-Action (VLA) & Conversational AI

## Introduction to VLA

**Vision-Language-Action (VLA)** models integrate visual perception and natural language understanding to produce robot actions. This enables conversational control and multimodal reasoning:

```
Visual Input (Camera)
        ↓
    [Vision Encoder] → Visual Embedding
        ↓
Natural Language Input (Speech/Text)
        ↓
    [Language Encoder] → Text Embedding
        ↓
    [Transformer Fusion] → Joint Representation
        ↓
    [Policy Head] → Robot Actions
```

## Pipeline Architecture

A complete VLA pipeline for classroom robots:

```
┌──────────────────┐
│ Speech Input     │
│ (Microphone)     │
└────────┬─────────┘
         ↓
    [Whisper]
    (Speech → Text)
         ↓
┌──────────────────┐
│ Text: "Help me   │
│ solve this math  │
│ problem"         │
└────────┬─────────┘
         ↓
    [GPT-3.5/4]
    (Intent + Plan)
         ↓
┌──────────────────┐
│ {"action":       │
│  "teach_math",   │
│  "step": 1,      │
│  "difficulty": 5}│
└────────┬─────────┘
         ↓
    [Action Mapper]
    (LLM Output → ROS)
         ↓
┌──────────────────┐
│ ROS 2 Commands   │
│ /joint_commands  │
│ /tablet_display  │
│ /audio_output    │
└──────────────────┘
```

## Component 1: Speech-to-Text (Whisper)

OpenAI's Whisper model enables on-device speech recognition:

```python
import whisper

class SpeechRecognizer:
    def __init__(self):
        self.model = whisper.load_model("base")  # ~140 MB
    
    def transcribe(self, audio_file):
        result = self.model.transcribe(audio_file, language="en")
        return result["text"]

# Usage
recognizer = SpeechRecognizer()
text = recognizer.transcribe("/tmp/classroom_audio.wav")
print(f"Recognized: {text}")
```

**Deployment**: Whisper runs on-device (privacy-first; no cloud dependency).

## Component 2: Language Understanding (LLM)

Use a fine-tuned LLM to interpret intent and generate robot actions:

```python
import openai

class IntentParser:
    def __init__(self, api_key):
        openai.api_key = api_key
    
    def parse_intent(self, text):
        prompt = f"""
        You are a classroom robot assistant. Parse the student request and output JSON.
        
        Student: "{text}"
        
        Respond with JSON: {{"action": str, "params": dict}}
        """
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}]
        )
        return json.loads(response.choices[0].message.content)

# Usage
parser = IntentParser(api_key="sk-...")
intent = parser.parse_intent("Can you help me understand the quadratic formula?")
print(intent)
# Output: {"action": "explain_math", "params": {"topic": "quadratic_formula", "level": "intermediate"}}
```

## Component 3: Action Mapping (ROS 2 Bridge)

Map LLM-generated actions to ROS 2 commands:

```python
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class ActionMapper(rclpy.Node):
    def __init__(self):
        super().__init__('action_mapper')
        self.intent_sub = self.create_subscription(
            String, '/intent', self.intent_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tablet_pub = self.create_publisher(String, '/tablet/display', 10)
    
    def intent_callback(self, msg):
        intent = json.loads(msg.data)
        action = intent["action"]
        params = intent.get("params", {})
        
        if action == "move_forward":
            cmd = Twist()
            cmd.linear.x = params.get("speed", 0.5)
            self.cmd_vel_pub.publish(cmd)
        
        elif action == "display_content":
            display_msg = String()
            display_msg.data = params.get("content", "")
            self.tablet_pub.publish(display_msg)

rclpy.init()
mapper = ActionMapper()
rclpy.spin(mapper)
```

## Multimodal Interaction: Combining Vision & Language

```python
from PIL import Image
import base64

class MultimodalAgent:
    def __init__(self):
        self.llm = openai.ChatCompletion
    
    def understand_scene(self, image_path, text_query):
        # Encode image to base64
        with open(image_path, "rb") as f:
            image_b64 = base64.b64encode(f.read()).decode()
        
        # Send to GPT-4V (Vision)
        response = self.llm.create(
            model="gpt-4-vision-preview",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": text_query},
                        {"type": "image_url", "image_url": {
                            "url": f"data:image/jpeg;base64,{image_b64}"
                        }}
                    ]
                }
            ]
        )
        return response.choices[0].message.content

# Usage
agent = MultimodalAgent()
result = agent.understand_scene(
    "/tmp/classroom.jpg",
    "What materials do students need for this activity?"
)
```

## Safety & Guardrails

Implement safety constraints to prevent harmful actions:

```python
class SafeActionValidator:
    ALLOWED_ACTIONS = {
        "teach", "explain", "display", "listen",
        "move_forward", "move_backward", "rotate"
    }
    
    MAX_SPEED = 0.5  # m/s
    UNSAFE_KEYWORDS = ["delete", "attack", "hurt", "destroy"]
    
    def validate(self, intent):
        action = intent.get("action")
        params = intent.get("params", {})
        
        # Check action whitelist
        if action not in self.ALLOWED_ACTIONS:
            raise ValueError(f"Action '{action}' not allowed")
        
        # Check for unsafe keywords
        if any(kw in str(intent).lower() for kw in self.UNSAFE_KEYWORDS):
            raise ValueError("Intent contains unsafe keywords")
        
        # Constrain speed
        if "speed" in params:
            params["speed"] = min(params["speed"], self.MAX_SPEED)
        
        return True

validator = SafeActionValidator()
try:
    validator.validate(intent)
except ValueError as e:
    print(f"Safety violation: {e}")
```

## Privacy-First Voice Processing

Keep voice data local and secure:

```python
import os
os.environ["WHISPER_CPP_THREADS"] = "4"  # Use CPU for edge devices

class PrivateVoiceProcessor:
    def __init__(self):
        # Load Whisper.cpp (C++ implementation for efficiency)
        self.recognizer = whisper.load_model("tiny")  # Smaller, faster model
    
    def process_locally(self, audio_bytes):
        # All processing happens on-device
        # No audio is sent to cloud
        result = self.recognizer.transcribe(audio_bytes)
        return result["text"]

processor = PrivateVoiceProcessor()
text = processor.process_locally(audio_input)
```

## Conversational State Management

Maintain context across multiple turns:

```python
class ConversationalContext:
    def __init__(self):
        self.history = []
        self.state = {}
    
    def add_message(self, role, content):
        self.history.append({"role": role, "content": content})
    
    def get_response(self, user_input):
        self.add_message("user", user_input)
        
        # Include conversation history for context
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=self.history,
            temperature=0.7
        )
        
        assistant_message = response.choices[0].message.content
        self.add_message("assistant", assistant_message)
        
        return assistant_message

# Usage
context = ConversationalContext()
r1 = context.get_response("What is photosynthesis?")
r2 = context.get_response("Can you explain it more simply?")  # Uses context
```

## End-to-End Example: Classroom Tutoring Agent

```python
class ClassroomTutorAgent:
    def __init__(self):
        self.speech = SpeechRecognizer()
        self.parser = IntentParser(api_key="sk-...")
        self.mapper = ActionMapper()
        self.context = ConversationalContext()
        self.validator = SafeActionValidator()
    
    def handle_student_input(self, audio_input):
        # Step 1: Transcribe
        text = self.speech.transcribe(audio_input)
        print(f"Student: {text}")
        
        # Step 2: Parse intent
        intent = self.parser.parse_intent(text)
        
        # Step 3: Validate safety
        self.validator.validate(intent)
        
        # Step 4: Maintain context
        self.context.add_message("user", text)
        response = self.context.get_response(text)
        
        # Step 5: Map to ROS actions
        self.mapper.intent_callback(String(data=json.dumps(intent)))
        
        print(f"Robot: {response}")

agent = ClassroomTutorAgent()
agent.handle_student_input("audio_input.wav")
```

## Performance & Latency

| Component | Latency | Hardware |
|-----------|---------|----------|
| Whisper (base) | 3–5s | CPU / Jetson |
| GPT-3.5 API | 0.5–2s | Cloud |
| Action Mapping | ~100ms | Local |
| **Total** | **4–8s** | Mixed |

Optimize for classroom use:
- Use smaller Whisper models for faster transcription
- Cache common intents to avoid API calls
- Precompute action mappings offline

## Next Steps

- Deploy on [Jetson Edge Hardware](../simulation/)
- Explore the case studies (Tutoring example)
- Test with the reproducibility framework

---

**References**: See the References section for VLA papers and LLM deployment guides.

**Governance**: All voice processing must comply with FERPA (student privacy) per Physical AI Constitution v1.0.0.
