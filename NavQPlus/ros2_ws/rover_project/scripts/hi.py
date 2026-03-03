import cv2

for i in range(4):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"Camera {i} OK, frame size: {frame.shape}")
        else:
            print(f"Camera {i} opened but no frame received")
        cap.release()
    else:
        print(f"Camera {i} not detected")