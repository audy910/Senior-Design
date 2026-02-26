#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json
import numpy as np
from scipy.signal import resample_poly
import sys
import os
import time

MAC_ADDR = "70:D5:EA:A4:84:1A"
DEVICE_ID = 0
RATE = 16000
CHANNELS = 2

class VoskNode(Node):
    def __init__(self):
        super().__init__('vosk_node')
        sd.default.device = 'pulse'
        
        # Audio Throttling to prevent Bluetooth crashes
        self.last_speech_time = 0.0
        self.speech_cooldown = 1.5  # Seconds between allowed audio clips

        # ROS 2 Communication
        self.voice_sub = self.create_subscription(String, 'robot_voice_trigger', self.voice_trigger_callback, 10)
        self.cmd_pub = self.create_publisher(String, 'robot_commands', 10)
        self.text_pub = self.create_publisher(String, 'speech_text', 10)

        self.command_map = {
            "forward": "MOVE_FORWARD",
            "back": "MOVE_BACKWARD",
            "stop": "EMERGENCY_STOP",
            "left": "TURN_LEFT",
            "right": "TURN_RIGHT"
        }
        
        self.speech_map = {
            "MOVE_FORWARD": "Move",
            "MOVE_BACKWARD": "Back",
            "EMERGENCY_STOP": "Stop",
            "TURN_LEFT": "left",
            "TURN_RIGHT": "right"
        }

        # Vosk Setup
        self.audio_queue = queue.Queue()
        vocab_list = list(self.command_map.keys()) + ["[unk]"]
        vocab_json = json.dumps(vocab_list)
        
        try:
            model = Model("/home/marina/Senior-Design/NavQPlus/models/vosk-model")
            self.rec = KaldiRecognizer(model, RATE, vocab_json)
        except Exception as e:
            self.get_logger().error(f"Model load failed: {e}")
            raise

        # Audio Input Stream
        self.stream = sd.InputStream(
            samplerate=48000, blocksize=8000, device=DEVICE_ID,
            dtype='int16', channels=CHANNELS, callback=self.audio_callback
        )
        self.stream.start()
        self.timer = self.create_timer(0.05, self.process_audio)

    def voice_trigger_callback(self, msg):
        """Plays audio when requested by the Autonomous Drive node (e.g., 'Stop')"""
        # We don't log the command here, just play it
        self.play_audio_out(msg.data)

    def audio_callback(self, indata, frames, time, status):
        if status:
            print(status, file=sys.stderr)
        audio = np.frombuffer(indata, dtype=np.int16).reshape(-1, CHANNELS)
        mono = audio.mean(axis=1)
        downsampled = resample_poly(mono, up=1, down=3)
        self.audio_queue.put(downsampled.astype(np.int16).tobytes())

    def process_audio(self):
        while not self.audio_queue.empty():
            data = self.audio_queue.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                text = result.get("text", "").lower()
                if text:
                    self.get_logger().info(f"Heard Voice Command: '{text}'")
                    for word in text.split():
                        if word in self.command_map:
                            self.execute_command(self.command_map[word])

    def execute_command(self, cmd_string):
        self.get_logger().warn(f"EXECUTING: {cmd_string}")
        msg = String()
        msg.data = cmd_string
        self.cmd_pub.publish(msg)
        
        response = self.speech_map.get(cmd_string, "Command received")
        self.play_audio_out(response)

    def play_audio_out(self, text):
        """The single source of truth for making the robot speak with cooldown."""
        now = time.time()
        if (now - self.last_speech_time) < self.speech_cooldown:
            return # Skip if we just spoke recently

        self.get_logger().info(f"Speaking: {text}")
        os.system(f"espeak -s 130 -p 45 -a 150 '{text}' -w /tmp/speech.wav")
        aplay_cmd = (
            f"aplay -D bluealsa:DEV={MAC_ADDR},PROFILE=a2dp "
            f"-f cd --period-time=100000 /tmp/speech.wav > /dev/null 2>&1 &"
        )
        os.system(aplay_cmd)
        self.last_speech_time = now

def main(args=None):
    rclpy.init(args=args)
    node = VoskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()