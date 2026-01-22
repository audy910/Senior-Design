#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from cv_bridge import CvBridge
import cv2
import numpy as np
import time

import tflite_runtime.interpreter as tflite


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Camera setup
        self.cap = cv2.VideoCapture(3)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.bridge = CvBridge()

        # For visualizing detections
        self.overlay_pub = self.create_publisher(
            CompressedImage, 'camera_with_boxes', 10
        )

        self.bbox_pub = self.create_publisher(
            Int32MultiArray, 'ai_bboxes', 10
        )

        # Run detection
        self.timer = self.create_timer(0.05, self.get_detections)

        # Load TFLite model with NPU delegate
        self.interpreter = tflite.Interpreter(
            model_path="/home/user/ros2_ws/model/correct",
            experimental_delegates=[tflite.load_delegate('libvx_delegate.so')]
        )
        self.get_logger().info('Loaded Model: Using NPU/VX delegate')

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.input_shape = self.input_details[0]['shape']
        self.input_height = self.input_shape[1]
        self.input_width = self.input_shape[2]

        self.get_logger().info(f'Model loaded successfully')
        self.get_logger().info(f'Input shape: {self.input_shape}')
        self.get_logger().info(f'Number of outputs: {len(self.output_details)}')

        self.frame_count = 0

        # COCO class names
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
            'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
            'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
            'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
            'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def get_detections(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read from camera")
            return

        inference_start = time.time()

        input_img = self.preprocess_image(frame)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_img)
        self.interpreter.invoke()

        outputs = [self.interpreter.get_tensor(detail['index'])
                   for detail in self.output_details]

        inference_time = (time.time() - inference_start) * 1000.0
        self.frame_count += 1

        frame_height, frame_width = frame.shape[:2]

        detections, best_bbox = self.postprocess(outputs, frame_height, frame_width)

        # Draw detections for visualization
        overlay_frame = frame.copy()
        for det in detections:
            x, y, w, h = det['bbox']
            class_name = det['class']
            confidence = det['confidence']

            x = max(0, min(x, frame_width - 1))
            y = max(0, min(y, frame_height - 1))
            x2 = max(0, min(x + w, frame_width - 1))
            y2 = max(0, min(y + h, frame_height - 1))
            if x2 <= x or y2 <= y:
                continue

            color = (0, 255, 0)
            cv2.rectangle(overlay_frame, (x, y), (x2, y2), color, 3)
            label = f"{class_name}: {confidence:.2f}"
            label_size, baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            label_y = max(y - 10, label_size[1] + 10)
            cv2.rectangle(
                overlay_frame,
                (x, label_y - label_size[1] - baseline),
                (x + label_size[0], label_y + baseline),
                color, -1
            )
            cv2.putText(
                overlay_frame, label, (x, label_y - baseline),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2
            )

        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Inference time: {inference_time:.1f} ms')

        # Publish overlay image
        _, buffer = cv2.imencode('.jpg', overlay_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        overlay_msg = CompressedImage()
        overlay_msg.header.stamp = self.get_clock().now().to_msg()
        overlay_msg.format = "jpeg"
        overlay_msg.data = buffer.tobytes()
        self.overlay_pub.publish(overlay_msg)

        if best_bbox is not None:
            x, y, w, h = best_bbox
            bbox_msg = Int32MultiArray()
            bbox_msg.data = [int(x), int(y), int(w), int(h)]
            self.bbox_pub.publish(bbox_msg)
            # Debug
            self.get_logger().info(f"Published BBOX: {bbox_msg.data}")

    def preprocess_image(self, frame):
        input_img = cv2.resize(frame, (self.input_width, self.input_height))
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        input_img = np.expand_dims(input_img, axis=0)
        input_img = input_img.astype(np.uint8)
        return input_img

    def postprocess(self, outputs, frame_height, frame_width):
        # Detections by four corners    (1, 10, 4)
        # Labels                        (1, 10)
        # Confidence Scores             (1, 10)
        # Number of Detections          (1)
        boxes = outputs[0][0]
        classes = outputs[1][0]
        scores = outputs[2][0]
        num_detections = int(outputs[3][0])

        detections = []

        max_box_size = 0
        best_bbox = None

        for i in range(num_detections):
            confidence = float(scores[i])
            if confidence <= 0.5:
                continue

            ymin, xmin, ymax, xmax = boxes[i]

            ymin = max(0, min(1, ymin))
            xmin = max(0, min(1, xmin))
            ymax = max(0, min(1, ymax))
            xmax = max(0, min(1, xmax))

            x1 = int(xmin * frame_width)
            y1 = int(ymin * frame_height)
            x2 = int(xmax * frame_width)
            y2 = int(ymax * frame_height)

            width = x2 - x1
            height = y2 - y1
            if width <= 0 or height <= 0:
                continue

            area = width * height
            if area > max_box_size:
                max_box_size = area
                best_bbox = (x1, y1, width, height)

            class_id = int(classes[i])
            if 0 <= class_id < len(self.class_names):
                class_name = self.class_names[class_id]
            else:
                class_name = f"class_{class_id}"

            detections.append({
                'bbox': [x1, y1, width, height],
                'class': class_name,
                'confidence': confidence
            })

        return detections, best_bbox


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
