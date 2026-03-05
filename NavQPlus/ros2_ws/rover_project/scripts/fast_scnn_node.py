#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from std_msgs.msg import Float32
import time
import os
import sys

CAM_WIDTH = 640
CAM_HEIGHT = 480
MODEL_SIZE = 512

NAV_COLORS = {
    0: (0, 0, 0),
    1: (128, 64, 128),    # safe path
    2: (244, 35, 232),    # soft terrain
    3: (0, 255, 0),       # vegetation
    4: (0, 0, 255),       # static obstacle
    5: (0, 255, 255),     # dynamic obstacle
    6: (135, 206, 235),   # sky
}

# Hardcoded TFLite path
MODEL_PATH = "/home/audy/Senior-Design/NavQPlus/models/fast_scnn_opt_int8.tflite"

class FastSCNNNode(Node):
    def __init__(self):
        super().__init__('fast_scnn_node')
        self.bridge = CvBridge()

        # --- Declare ROS 2 Parameter ---
        # Defaults to False to save CPU. Can be toggled at launch.
        self.declare_parameter('enable_viz', False)

        model_path = MODEL_PATH
        camera_index = 3

        # Publishers
        self.overlay_pub = self.create_publisher(Image, '/segmentation/overlay', 1)
        self.error_pub = self.create_publisher(Float32, 'nav/vision_error', 1)

        # Lookup Table for visualization
        self.lut = np.zeros((256, 3), dtype=np.uint8)
        for cid, color in NAV_COLORS.items():
            self.lut[cid] = color

        # --- Hard Failures ---
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at: {model_path}")
            sys.exit(1)

        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera on /dev/video{camera_index}")
            sys.exit(1)

        # TFLite Interpreter
        try:
            self.interpreter = tflite.Interpreter(
                model_path=model_path,
                experimental_delegates=[tflite.load_delegate("/usr/lib/libvx_delegate.so")]
            )
            self.get_logger().info("SUCCESS: NPU VX Delegate Active")
        except Exception as e:
            self.get_logger().error(f"FALLBACK: NPU Delegate failed. Error: {e}")
            self.interpreter = tflite.Interpreter(model_path=model_path)

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Timer: 0.1 = 10 FPS
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("Fast-SCNN Optimized Node Started")

    def loop(self):
        # Clear V4L2 buffer
        for _ in range(2): 
            self.cap.grab()
            
        ret, frame = self.cap.retrieve()
        if not ret:
            return
         # Add a check to ensure the frame actually contains data
        if not ret or frame is None or frame.size == 0:
            self.get_logger().warn("Empty frame received. Skipping.")
            return

        # Preprocess
        img = cv2.resize(frame, (MODEL_SIZE, MODEL_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        input_img = np.expand_dims(img.astype(np.uint8), axis=0)

        # Inference
        self.interpreter.set_tensor(self.input_details[0]['index'], input_img)
        self.interpreter.invoke()

        # Postprocess
        output = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        if output.shape[-1] > 1:
            small_output = cv2.resize(output, (256, 256))
            mask = np.argmax(small_output, axis=-1).astype(np.uint8)
        else:
            mask = np.squeeze(output).astype(np.uint8)

       # --- PATH FINDING (Largest Continuous Chunk) ---
        is_habitable = (mask == 1) | (mask == 2) | (mask == 3)
        mask_h, mask_w = mask.shape
        search_row_idx = int(mask_h * 0.75)
        
        path_pixels = np.where(is_habitable[search_row_idx, :])[0]

        if len(path_pixels) > 0:
            # ALLOWABLE ERROR / GAP TOLERANCE 
            # a tolerance of 10 ignores obstacles smaller than ~4% of the screen width)
            GAP_TOLERANCE = 10 

            # Find gaps in the array that are LARGER than our tolerance
            step = np.diff(path_pixels)
            splits = np.where(step > GAP_TOLERANCE)[0] + 1
            chunks = np.split(path_pixels, splits)
            
            # Target the center of the largest contiguous safe chunk
            largest_chunk = max(chunks, key=len)
            path_center_x_small = int(np.median(largest_chunk))
            
            # Scale result back to 640 width accurately
            path_center_x = int(path_center_x_small * (CAM_WIDTH / mask_w))
            error = float(path_center_x - (CAM_WIDTH // 2))
        else:
            error = 9999.0
            # Warning message for when the rover is blind
            self.get_logger().warn("No safe path found on search row! Emitting error 9999.0")


        # Publish Error
        err_msg = Float32()
        err_msg.data = error
        self.error_pub.publish(err_msg)

        # --- VISUALIZATION (ROS 2 Parameter) ---
        enable_viz = self.get_parameter('enable_viz').value
        
        if enable_viz:
            
            mask_large = cv2.resize(mask, (CAM_WIDTH, CAM_HEIGHT), interpolation=cv2.INTER_NEAREST)
            color_mask = self.lut[mask_large]
            overlay = cv2.addWeighted(color_mask, 0.5, frame, 0.5, 0)
            
            if error != 9999.0:
                search_row_vis = int(CAM_HEIGHT * 0.75)
                cv2.circle(overlay, (path_center_x, search_row_vis), 10, (0, 255, 0), -1)
                cv2.line(overlay, (CAM_WIDTH // 2, search_row_vis - 20), 
                         (CAM_WIDTH // 2, search_row_vis + 20), (255, 255, 255), 2)

            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            self.overlay_pub.publish(overlay_msg)

def main():
    rclpy.init()
    node = FastSCNNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()