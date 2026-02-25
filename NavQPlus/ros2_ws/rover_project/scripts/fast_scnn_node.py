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

MODEL_PATH = "/home/marina/Senior-Design/NavQPlus/models/fast_scnn_opt_int8.tflite"

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

class FastSCNNNode(Node):
    def __init__(self):
        super().__init__('fast_scnn_node')
        self.bridge = CvBridge()

        # Publishers
        self.overlay_pub = self.create_publisher(Image, '/segmentation/overlay', 1)
        self.error_pub = self.create_publisher(Float32, '/vision/error', 1)

        # Speed Optimization: Pre-create Lookup Table
        self.lut = np.zeros((256, 3), dtype=np.uint8)
        for cid, color in NAV_COLORS.items():
            self.lut[cid] = color

        # Camera Setup
        self.cap = cv2.VideoCapture(3, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return

        # NPU Initialization
        try:
            self.interpreter = tflite.Interpreter(
                model_path=MODEL_PATH,
                experimental_delegates=[tflite.load_delegate("/usr/lib/libvx_delegate.so")]
            )
            self.get_logger().info("SUCCESS: NPU VX Delegate Active")
        except Exception as e:
            self.get_logger().error(f"FALLBACK: NPU Delegate failed. Error: {e}")
            self.interpreter = tflite.Interpreter(model_path=MODEL_PATH)

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # Timer: 0.1 = 10 FPS (Much better for sharing CPU with teammate)
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("Fast-SCNN Optimized Node Started")

    def loop(self):
        for _ in range(2): 
            self.cap.grab()
            
        ret, frame = self.cap.retrieve()
        if not ret:
            return

        # --- Preprocess (UINT8 for NPU) ---
        img = cv2.resize(frame, (MODEL_SIZE, MODEL_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        input_img = np.expand_dims(img.astype(np.uint8), axis=0)
        '''
        # --- Inference (The NPU ) ---
        self.interpreter.set_tensor(self.input_details[0]['index'], input_img)
        self.interpreter.invoke()
        '''
        start = time.perf_counter()
        
        self.interpreter.set_tensor(self.input_details[0]['index'], input_img)
        self.interpreter.invoke()
        
        end = time.perf_counter()
        # If this is < 0.05, the NPU is doing the heavy lifting!
        print(f"Inference time: {end - start:.4f}s")

        # --- Postprocess (Optimized at 256x256 to save CPU) ---
        output = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        if output.shape[-1] > 1:
            # Resizing small reduces argmax load by 4x
            small_output = cv2.resize(output, (256, 256))
            mask = np.argmax(small_output, axis=-1).astype(np.uint8)
        else:
            mask = np.squeeze(output).astype(np.uint8)

        # --- PATH FINDING (Uses the small mask to stay fast) ---
        is_habitable = (mask == 1) | (mask == 2) | (mask == 3)
        mask_h, mask_w = mask.shape
        search_row_idx = int(mask_h * 0.75) # Fixed: Dynamically picks row 192 if height is 256
        
        path_pixels = np.where(is_habitable[search_row_idx, :])[0]

        if len(path_pixels) > 0:
            path_center_x_small = int(np.median(path_pixels))
            # Scale result back to 640 width accurately
            path_center_x = int(path_center_x_small * (CAM_WIDTH / mask_w))
            error = float(path_center_x - (CAM_WIDTH // 2))
        else:
            error = 9999.0

        # --- PUBLISH ERROR (Always do this for the teammate) ---
        err_msg = Float32()
        err_msg.data = error
        self.error_pub.publish(err_msg)

        # --- VISUALIZATION  ---
        # UNCOMMENT the block below if you need to see the image in Foxglove.
        # KEEP COMMENTED to make htop look great for your teammate.
        
        #mask_large = cv2.resize(mask, (CAM_WIDTH, CAM_HEIGHT), interpolation=cv2.INTER_NEAREST)
        #color_mask = self.lut[mask_large]
        #overlay = cv2.addWeighted(color_mask, 0.5, frame, 0.5, 0)
        '''
        if error != 9999.0:
            search_row_vis = int(CAM_HEIGHT * 0.75)
            cv2.circle(overlay, (path_center_x, search_row_vis), 10, (0, 255, 0), -1)
            cv2.line(overlay, (CAM_WIDTH // 2, search_row_vis - 20), 
                     (CAM_WIDTH // 2, search_row_vis + 20), (255, 255, 255), 2)
'''
        if error != 9999.0:
            search_row_vis = int(CAM_HEIGHT * 0.75)
            # Draw the Green Dot (Path Center)
            cv2.circle(frame, (path_center_x, search_row_vis), 10, (0, 255, 0), -1)
            # Draw the White Centerline (Robot Heading)
            cv2.line(frame, (CAM_WIDTH // 2, search_row_vis - 20), 
                     (CAM_WIDTH // 2, search_row_vis + 20), (255, 255, 255), 2)

        # Publish the raw frame with just the guidance markers
        overlay_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
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