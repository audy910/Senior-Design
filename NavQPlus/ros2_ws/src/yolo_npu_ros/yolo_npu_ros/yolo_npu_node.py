#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np
import tflite_runtime.interpreter as tflite

def sigmoid(x):
    return 1 / (1 + np.exp(-x))


class YoloNPUNavQNode(Node):
    def __init__(self):
        super().__init__('yolo_npu_node')

        # ---------- ROS ----------
        self.image_pub = self.create_publisher(
            CompressedImage, '/camera/detections', 10
        )

        self.image_raw_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw', 10
        )
        self.bridge = CvBridge()

        # ---------- CAMERA ----------
        self.cap = cv2.VideoCapture(3, cv2.CAP_V4L2)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open /dev/video3')
            return

        self.get_logger().info('Camera /dev/video3 opened')

        # ---------- NPU ----------
        delegate = tflite.load_delegate('/usr/lib/libvx_delegate.so')
        self.interpreter = tflite.Interpreter(
            model_path='/home/user/Senior-Design/NavQPlus/models/model_final_npu.tflite',
            experimental_delegates=[delegate]
        )
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Quantization params for output
        

        self.get_logger().info('YOLO NPU model loaded')

        # ---------- TIMER ----------
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.resize(frame, (640, 640))  # <<< ADD THIS

          # Publish Raw image
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        self.image_raw_pub.publish(msg)

    
        # ---------- PREPROCESS ----------
        img = cv2.resize(frame, (640, 640))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        input_data = np.expand_dims(img, axis=0)
        # ---------- INFERENCE ----------
        self.interpreter.set_tensor(
            self.input_details[0]['index'], input_data
        )
        self.interpreter.invoke()

        raw_output = self.interpreter.get_tensor(
            self.output_details[0]['index']
        )
        out_info = self.output_details[0]
        if out_info['dtype'] != np.float32:
            scale, zero_point = out_info['quantization']
            raw_output = (raw_output.astype(np.float32) - zero_point) * scale
        # ---------- POST PROCESS + DRAW ----------
        self.process_and_draw(frame, raw_output)

        # ---------- PUBLISH ----------
        # frame is a numpy array from OpenCV

        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        self.image_pub.publish(msg)


    def process_and_draw(self, frame, output):
        """
        output shape: [1, 84, 8400]  (YOLOv8)
        """

        output = output[0].transpose()  # [8400, 84]
        h, w = frame.shape[:2]

        for row in output:
            row = sigmoid(row)

            cx, cy, bw, bh = row[:4]
            obj_conf = row[4]
            class_scores = row[5:]

            class_id = np.argmax(class_scores)
            conf = obj_conf * class_scores[class_id]

            # CONFIDENCE FILTER
            if conf < 0.4:
                continue

        # YOLOv8 outputs are NORMALIZED (0–1)
            x1 = int((cx - bw / 2) * w)
            y1 = int((cy - bh / 2) * h)
            x2 = int((cx + bw / 2) * w)
            y2 = int((cy + bh / 2) * h)

        # Clamp
            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w - 1, x2))
            y2 = max(0, min(h - 1, y2))

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f'ID:{class_id} {conf:.2f}',
                (x1, max(y1 - 5, 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

def main(args=None):
    rclpy.init(args=args)
    node = YoloNPUNavQNode()
    rclpy.spin(node)

    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

