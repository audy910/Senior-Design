import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.lite.python.interpreter import load_delegate


MODEL_PATH = "/home/user/Senior-Design/NavQPlus/models/fast_scnn_opt_int8.tflite"


CAM_WIDTH = 640
CAM_HEIGHT = 480
MODEL_SIZE = 512


CLASS_NAMES = {
    0: "void",
    1: "safe_path",
    2: "soft_terrain",
    3: "vegetation",
    4: "static_obstacle",
    5: "dynamic_obstacle",
    6: "sky",
}


NAV_COLORS = {
    0: (0, 0, 0),
    1: (128, 64, 128),    # safe path
    2: (244, 35, 232),    # soft terrain
    3: (0, 255, 0),       # vegetation
    4: (0, 0, 255),       # static obstacle
    5: (0, 255, 255),     # dynamic obstacle
    6: (135, 206, 235),   # sky
}


BOX_CLASSES = {
    4,  # static
    5,  # dynamic
}


BOX_COLORS = {
    4: (0, 0, 255),      # red = static obstacle
    5: (0, 255, 255),    # yellow = dynamic obstacle
}




class FastSCNNNode(Node):
    def __init__(self):
        super().__init__('fast_scnn_node')


        self.bridge = CvBridge()


        # Do NOT publish to /image_raw
        self.image_pub = self.create_publisher(Image, '/camera/image', 10)
        self.seg_pub = self.create_publisher(Image, '/segmentation/color', 10)
        self.box_pub = self.create_publisher(Image, '/segmentation/boxes', 10)
        self.overlay_pub = self.create_publisher(Image, '/segmentation/overlay', 10)


        self.cap = cv2.VideoCapture("/dev/video3", cv2.CAP_V4L2)
    


        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)


        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return


        self.interpreter = tf.lite.Interpreter(
            model_path=MODEL_PATH,
            experimental_delegates=[
                load_delegate("/usr/lib/libvx_delegate.so")
            ]
        )
        self.interpreter.allocate_tensors()


        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()


        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("Fast-SCNN camera node started")


    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame failed")
            return


        # --- Preprocess for model ---
        img = cv2.resize(frame, (MODEL_SIZE, MODEL_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #img = img.astype(np.uint8)
        input_img = np.expand_dims(img, axis=0)


        # --- Inference ---
        self.interpreter.set_tensor(
            self.input_details[0]['index'],
            input_img
        )
        self.interpreter.invoke()


        # --- Postprocess ---
        output = self.interpreter.get_tensor(
        self.output_details[0]['index'])[0]


        # Dequantize output
        scale, zero_point = self.output_details[0]['quantization']
        output = scale * (output.astype(np.float32) - zero_point)


        mask = np.argmax(output, axis=-1).astype(np.uint8)

        mask = cv2.resize(mask,(CAM_WIDTH, CAM_HEIGHT),interpolation=cv2.INTER_NEAREST)


        # --- Bounding boxes ---
        boxed = frame.copy()
        present_classes = np.unique(mask)
       
        for class_id in present_classes:
            if class_id not in BOX_CLASSES:
                continue


            binary = (mask == class_id).astype(np.uint8) * 255
            kernel = np.ones((5, 5), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                binary,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )


            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                if (w * h) < 1200:
                    continue
               
                aspect_ratio = w / float(h)


                if aspect_ratio < 0.3 or aspect_ratio > 3.0:
                    continue


                extent = cv2.contourArea(c) / float(w * h)
                if extent < 0.3:
                    continue


                color = BOX_COLORS.get(class_id, (255, 255, 255))
                cv2.rectangle(boxed, (x, y), (x + w, y + h), color, 2)




                label = CLASS_NAMES.get(class_id, str(class_id))
                cv2.putText(boxed, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


        # --- Color segmentation image ---
        color_mask = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)


        for cid, color in NAV_COLORS.items():
            color_mask[mask == cid] = color


        seg_msg = self.bridge.cv2_to_imgmsg(color_mask, encoding='bgr8')
        self.seg_pub.publish(seg_msg)
        # --- Publish boxed image ONCE ---
        box_msg = self.bridge.cv2_to_imgmsg(boxed, encoding='bgr8')
        self.box_pub.publish(box_msg)
        # --- Overlay segmentation + boxes ---
        overlay = cv2.addWeighted(color_mask, 0.6, boxed, 0.4, 0)
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        self.overlay_pub.publish(overlay_msg)








def main():
    rclpy.init()
    node = FastSCNNNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



