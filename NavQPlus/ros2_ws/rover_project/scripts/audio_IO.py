#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import soundfile as sf
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json
import numpy as np
from scipy.signal import resample_poly
import sys
import os  # Added to run system commands

MAC_ADDR = "70:D5:EA:A4:84:1A"

DEVICE_ID = 0
RATE = 16000
CHANNELS = 2


class VoskNode(Node):

    def __init__(self):
        super().__init__('vosk_node')
        sd.default.device = 'pulse'
        self.command_map = {
            "forward": "MOVE_FORWARD",
            "back": "MOVE_BACKWARD",
            "stop": "EMERGENCY_STOP",
            "left": "TURN_LEFT",
            "right": "TURN_RIGHT"
        }
        
        # Added mapping for what the robot should say back
        self.speech_map = {
            "MOVE_FORWARD": "Move",
            "MOVE_BACKWARD": "Back",
            "EMERGENCY_STOP": "Stop",
            "TURN_LEFT": "left",
            "TURN_RIGHT": "right"
        }

        vocab_list = list(self.command_map.keys()) + ["[unk]"]
        vocab_json = json.dumps(vocab_list)
        
        self.get_logger().info(f"Listening ONLY for: {vocab_json}")

        self.text_pub = self.create_publisher(String, 'speech_text', 10)
        self.cmd_pub = self.create_publisher(String, 'robot_commands', 10)

        self.audio_queue = queue.Queue()

        try:
            # Note: Ensure this path is correct for your user (e.g., /home/marina/...)
            model = Model("/home/marina/Senior-Design/NavQPlus/models/vosk-model")
            self.rec = KaldiRecognizer(model, RATE, vocab_json)
        except Exception as e:
            self.get_logger().error(f"Model load failed: {e}")
            raise

        self.stream = sd.InputStream(
            samplerate=48000,
            blocksize=8000,
            device=DEVICE_ID,
            dtype='int16',
            channels=CHANNELS,
            callback=self.callback
        )
        self.stream.start()

        self.timer = self.create_timer(0.05, self.process_audio)


    def callback(self, indata, frames, time, status):
        if status:
            print(status, file=sys.stderr)

        audio = np.frombuffer(indata, dtype=np.int16)
        audio = audio.reshape(-1, CHANNELS)
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
                    self.get_logger().info(f"Heard: '{text}'")
                    
                    # Check if any part of the heard text is in our command map
                    words = text.split()
                    for word in words:
                        if word in self.command_map:
                            cmd_string = self.command_map[word]
                            self.execute_command(cmd_string)

    def execute_command(self, cmd_string):
        self.get_logger().warn(f"PUBLISHING COMMAND: {cmd_string}")

        msg = String()
        msg.data = cmd_string
        self.cmd_pub.publish(msg)

        response = self.speech_map.get(cmd_string, "Command received")
        os.system(f"espeak -s 130 -p 45 -a 150 '{response}' -w /tmp/speech.wav")
        aplay_cmd = (
            f"aplay -D bluealsa:DEV={MAC_ADDR},PROFILE=a2dp "
            f"-f cd --period-time=100000 /tmp/speech.wav > /dev/null 2>&1 &"
        )
        os.system(aplay_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VoskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()