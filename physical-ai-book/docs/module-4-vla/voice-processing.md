---
sidebar_position: 2
---

# Voice Processing and Command Interpretation

## Introduction

Voice processing is a critical component of the Vision-Language-Action (VLA) system, enabling natural interaction with humanoid robots. This section covers how to implement voice-to-action processing using modern speech recognition technologies.

## Speech Recognition with Whisper

OpenAI's Whisper model provides state-of-the-art speech recognition capabilities. Here's how to integrate it into your robot system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import whisper
import torch
import numpy as np
import pyaudio
import wave
import threading

class VoiceProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Publisher for recognized text
        self.text_pub = self.create_publisher(String, '/voice/text', 10)

        # Publisher for commands
        self.command_pub = self.create_publisher(String, '/robot/command', 10)

        # Initialize Whisper model
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("base")  # Use "small" or "medium" for better accuracy
        self.get_logger().info('Whisper model loaded successfully')

        # Audio recording parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.record_seconds = 5

        # Start audio recording thread
        self.audio_thread = threading.Thread(target=self.record_audio, daemon=True)
        self.audio_thread.start()

        self.get_logger().info('Voice processor initialized')

    def record_audio(self):
        """Record audio from microphone and process with Whisper"""
        p = pyaudio.PyAudio()

        while rclpy.ok():
            # Record audio
            stream = p.open(format=self.format,
                           channels=self.channels,
                           rate=self.rate,
                           input=True,
                           frames_per_buffer=self.chunk)

            self.get_logger().info('Listening...')
            frames = []

            for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
                data = stream.read(self.chunk)
                frames.append(data)

            # Stop recording
            stream.stop_stream()
            stream.close()

            # Save recorded data as WAV file for processing
            wf = wave.open('/tmp/recording.wav', 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(p.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            wf.close()

            # Process with Whisper
            result = self.model.transcribe('/tmp/recording.wav')
            text = result['text'].strip()

            if text:  # Only publish if there's text
                self.get_logger().info(f'Recognized: {text}')

                # Publish recognized text
                text_msg = String()
                text_msg.data = text
                self.text_pub.publish(text_msg)

                # Process command
                self.process_command(text)

    def process_command(self, text):
        """Process recognized text and generate robot commands"""
        # Convert to lowercase for easier processing
        text_lower = text.lower()

        # Simple command recognition (in practice, use more sophisticated NLP)
        if 'move' in text_lower or 'go' in text_lower:
            if 'forward' in text_lower:
                command = 'MOVE_FORWARD'
            elif 'backward' in text_lower or 'back' in text_lower:
                command = 'MOVE_BACKWARD'
            elif 'left' in text_lower:
                command = 'TURN_LEFT'
            elif 'right' in text_lower:
                command = 'TURN_RIGHT'
            else:
                command = 'MOVE_FORWARD'  # default
        elif 'stop' in text_lower:
            command = 'STOP'
        elif 'stand' in text_lower:
            command = 'STAND_UP'
        elif 'sit' in text_lower:
            command = 'SIT_DOWN'
        elif 'hello' in text_lower or 'hi' in text_lower:
            command = 'WAVE'
        else:
            command = f'UNKNOWN_COMMAND: {text}'

        # Publish command
        cmd_msg = String()
        cmd_msg.data = command
        self.command_pub.publish(cmd_msg)
        self.get_logger().info(f'Command generated: {command}')

def main(args=None):
    rclpy.init(args=args)
    voice_processor = VoiceProcessor()

    try:
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Alternative: Using Vosk for Offline Speech Recognition

For more privacy-focused applications, Vosk provides good offline speech recognition:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
from vosk import Model, KaldiRecognizer
import json

class VoskVoiceProcessor(Node):
    def __init__(self):
        super().__init__('vosk_voice_processor')

        # Publisher for recognized text
        self.text_pub = self.create_publisher(String, '/voice/text', 10)

        # Initialize Vosk model
        self.get_logger().info('Loading Vosk model...')
        self.model = Model(lang="en-us")  # Download model from https://alphacephei.com/vosk/models
        self.rec = KaldiRecognizer(self.model, 16000)

        # Audio recording setup
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=8000
        )

        self.get_logger().info('Vosk voice processor initialized')

        # Start recognition loop
        self.recognition_timer = self.create_timer(0.1, self.recognize_audio)

    def recognize_audio(self):
        """Continuously recognize audio from microphone"""
        data = self.stream.read(4000, exception_on_overflow=False)

        if len(data) == 0:
            return

        if self.rec.AcceptWaveform(data):
            result = self.rec.Result()
            result_json = json.loads(result)

            if 'text' in result_json and result_json['text']:
                text = result_json['text']
                self.get_logger().info(f'Vosk recognized: {text}')

                # Publish recognized text
                text_msg = String()
                text_msg.data = text
                self.text_pub.publish(text_msg)

def main(args=None):
    rclpy.init(args=args)
    vosk_processor = VoskVoiceProcessor()

    try:
        rclpy.spin(vosk_processor)
    except KeyboardInterrupt:
        pass
    finally:
        vosk_processor.stream.stop_stream()
        vosk_processor.stream.close()
        vosk_processor.p.terminate()
        vosk_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Command Processing Pipeline

Here's a more sophisticated command processing pipeline:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Subscribe to recognized text
        self.text_sub = self.create_subscription(
            String, '/voice/text', self.text_callback, 10)

        # Publisher for robot movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for higher-level commands
        self.high_level_cmd_pub = self.create_publisher(String, '/high_level_command', 10)

        self.get_logger().info('Voice command processor initialized')

    def text_callback(self, msg):
        """Process incoming text and generate appropriate commands"""
        text = msg.data.lower().strip()
        self.get_logger().info(f'Processing command: {text}')

        # Parse the command
        command = self.parse_command(text)

        if command:
            # Publish high-level command
            cmd_msg = String()
            cmd_msg.data = command
            self.high_level_cmd_pub.publish(cmd_msg)

            # Execute the command
            self.execute_command(command)

    def parse_command(self, text):
        """Parse natural language command into robot commands"""
        # Movement commands
        if any(word in text for word in ['move forward', 'go forward', 'forward', 'ahead']):
            return 'MOVE_FORWARD'
        elif any(word in text for word in ['move backward', 'go backward', 'backward', 'back']):
            return 'MOVE_BACKWARD'
        elif any(word in text for word in ['turn left', 'left', 'rotate left']):
            return 'TURN_LEFT'
        elif any(word in text for word in ['turn right', 'right', 'rotate right']):
            return 'TURN_RIGHT'
        elif any(word in text for word in ['stop', 'halt', 'pause']):
            return 'STOP'
        elif any(word in text for word in ['stand up', 'stand']):
            return 'STAND_UP'
        elif any(word in text for word in ['sit down', 'sit']):
            return 'SIT_DOWN'
        elif any(word in text for word in ['wave', 'hello', 'hi']):
            return 'WAVE'
        elif any(word in text for word in ['dance', 'dancing']):
            return 'DANCE'
        elif 'pick up' in text or 'grab' in text:
            # Extract object if mentioned
            obj_match = re.search(r'pick up (.+)|grab (.+)', text)
            if obj_match:
                obj = obj_match.group(1) or obj_match.group(2)
                return f'PICK_UP_{obj.upper().replace(" ", "_")}'
            return 'PICK_UP_OBJECT'
        elif 'go to' in text or 'move to' in text:
            # Extract destination if mentioned
            dest_match = re.search(r'go to (.+)|move to (.+)', text)
            if dest_match:
                dest = dest_match.group(1) or dest_match.group(2)
                return f'GO_TO_{dest.upper().replace(" ", "_")}'
            return 'GO_TO_LOCATION'

        return None

    def execute_command(self, command):
        """Execute the parsed command"""
        twist = Twist()

        if command == 'MOVE_FORWARD':
            twist.linear.x = 0.5  # Adjust speed as needed
        elif command == 'MOVE_BACKWARD':
            twist.linear.x = -0.5
        elif command == 'TURN_LEFT':
            twist.angular.z = 0.5
        elif command == 'TURN_RIGHT':
            twist.angular.z = -0.5
        elif command == 'STOP':
            # Twist is already zero, so robot stops
            pass
        else:
            self.get_logger().info(f'Command not implemented in basic controller: {command}')
            return

        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Executed command: {command}')

def main(args=None):
    rclpy.init(args=args)
    command_processor = VoiceCommandProcessor()

    try:
        rclpy.spin(command_processor)
    except KeyboardInterrupt:
        pass
    finally:
        command_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Noise Reduction and Audio Preprocessing

For better voice recognition in noisy environments:

```python
import numpy as np
import scipy.signal as signal
from scipy.io import wavfile

def preprocess_audio(audio_path):
    """Apply noise reduction and audio preprocessing"""
    # Read audio file
    sample_rate, audio_data = wavfile.read(audio_path)

    # If stereo, convert to mono
    if len(audio_data.shape) > 1:
        audio_data = audio_data.mean(axis=1)

    # Normalize audio
    audio_data = audio_data / np.max(np.abs(audio_data))

    # Apply noise reduction (simple spectral gating)
    # This is a simplified example - use proper noise reduction libraries in production
    fft_data = np.fft.fft(audio_data)
    magnitude = np.abs(fft_data)

    # Simple noise floor estimation and reduction
    noise_floor = np.mean(magnitude) * 0.1  # Adjust threshold as needed
    magnitude_reduced = np.maximum(magnitude - noise_floor, 0)

    # Reconstruct signal (phase remains unchanged)
    phase = np.angle(fft_data)
    fft_reduced = magnitude_reduced * np.exp(1j * phase)
    audio_clean = np.real(np.fft.ifft(fft_reduced))

    # Save processed audio
    output_path = audio_path.replace('.wav', '_clean.wav')
    wavfile.write(output_path, sample_rate, (audio_clean * 32767).astype(np.int16))

    return output_path
```

## Voice Activity Detection (VAD)

To reduce processing when no one is speaking:

```python
import webrtcvad
import collections

class VoiceActivityDetector:
    def __init__(self, sample_rate=16000, frame_duration=30):
        self.vad = webrtcvad.Vad(2)  # Aggressiveness level 0-3
        self.sample_rate = sample_rate
        self.frame_duration = frame_duration  # in ms
        self.frame_size = int(sample_rate * frame_duration / 1000) * 2  # 2 bytes per sample

        # Buffer to store audio frames
        self.ring_buffer = collections.deque(maxlen=30)
        self.triggered = False

    def is_speech(self, audio_frame):
        """Detect if the audio frame contains speech"""
        try:
            return self.vad.is_speech(audio_frame, self.sample_rate)
        except:
            return False
```

## Conclusion

Voice processing enables natural interaction with humanoid robots. Proper implementation of speech recognition, command parsing, and audio preprocessing is crucial for a responsive and accurate VLA system. In the next section, we'll explore how to integrate large language models for cognitive planning.