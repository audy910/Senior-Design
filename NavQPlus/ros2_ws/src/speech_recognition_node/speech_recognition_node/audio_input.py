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


DEVICE_ID = 0
RATE = 16000
CHANNELS = 2


class VoskNode(Node):

    def __init__(self):
        super().__init__('vosk_node')

        self.publisher_ = self.create_publisher(String, 'speech_text', 10)

        self.audio_queue = queue.Queue()

        try:
            model = Model("/home/user/Senior-Design/NavQPlus/models/vosk-model")
            self.rec = KaldiRecognizer(model, RATE)
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
                sentence = result.get("text", "")

                if sentence:
                    msg = String()
                    msg.data = sentence
                    self.publisher_.publish(msg)

                    self.get_logger().info(f"Final: {sentence}")

            else:
                partial = json.loads(self.rec.PartialResult())
                word = partial.get("partial", "")

                if word:
                    self.get_logger().info(f"Partial: {word}")


def main(args=None):
    rclpy.init(args=args)
    node = VoskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
