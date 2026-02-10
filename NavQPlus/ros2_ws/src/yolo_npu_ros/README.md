# 🛰️ NavQ+ NPU Accelerated Semantic Segmentation

---
## 🚀 Quick Start

If you are on a NavQ+ board with the workspace already set up:

1. **Source the workspace:**
   ```bash
   cd ~/Senior-Design/NavQPlus/ros2_ws
   source install/setup.bash
   ```
2. **Run the NPU Node:**
    ```bash
    ros2 run yolo_npu_ros fast_scnn_node
    ```

## 🛠️ Hardware Acceleration Verification

To ensure the NPU is actually doing the work (and not the CPU), run this command in a separate terminal while the node is active:
```bash
sudo watch -n 1 cat /sys/kernel/debug/gc/load
```
Success: You should see core 1 showing a load between 15% - 35%.
Failure: If you see 0%, the model is falling back to the CPU (usually due to a non-quantized model).

## 🦊 Visualizing with Foxglove
To see the live segmentation output from the NavQ+ on your laptop:

Launch the Bridge on NavQ+:

```Bash
ros2 run foxglove_bridge foxglove_bridge
```
Connect in Foxglove Studio:
* Open a connection to ws://<your_navq_ip>:8765.
* Add an Image Panel to view the segmentation topic like /segmentation/color or /segmentation/boxes.

## 📂 Project Structure & NPU Requirements
* Package Name: yolo_npu_ros
* Target Hardware: NXP i.MX8M Plus (NavQ+)
* NPU Delegate: libvx_delegate.so (Located in /usr/lib/)
* Model Format: The NPU strictly requires INT8 Quantized .tflite models.
* Model Path: models/fast_scnn_int8.tflite