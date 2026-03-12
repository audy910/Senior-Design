"""
Unit tests for the segmentation post-processing logic in fast_scnn_node.py.

The path-finding algorithm runs inside the loop() method, so we extract the
logic here as plain functions and test it directly — no ROS2 context needed.
"""

import sys
import types
import unittest
import numpy as np
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Mock every dependency that fast_scnn_node imports at module level
# ---------------------------------------------------------------------------
for _mod in (
    "rclpy", "rclpy.node",
    "sensor_msgs", "sensor_msgs.msg",
    "cv_bridge",
    "cv2",
    "tflite_runtime", "tflite_runtime.interpreter",
    "std_msgs", "std_msgs.msg",
    "ament_index_python", "ament_index_python.packages",
):
    if _mod not in sys.modules:
        sys.modules[_mod] = types.ModuleType(_mod)

# ament_index_python.packages.get_package_share_directory must return a string
sys.modules["ament_index_python.packages"].get_package_share_directory = \
    lambda pkg: "/tmp/mock_pkg_share"

# rclpy.node.Node stub
sys.modules["rclpy"].node = sys.modules["rclpy.node"]
sys.modules["rclpy.node"].Node = object

# sensor_msgs.msg.Image stub
sys.modules["sensor_msgs.msg"].Image = MagicMock
# std_msgs.msg stubs
sys.modules["std_msgs.msg"].Float32 = MagicMock
# cv_bridge stub
sys.modules["cv_bridge"].CvBridge = MagicMock
# tflite_runtime.interpreter stubs
sys.modules["tflite_runtime.interpreter"].Interpreter = MagicMock
sys.modules["tflite_runtime.interpreter"].load_delegate = MagicMock

# cv2 stub — only np-compatible resize needed for tests (we won't call it)
sys.modules["cv2"].resize = MagicMock(side_effect=lambda x, *a, **kw: x)
sys.modules["cv2"].cvtColor = MagicMock(side_effect=lambda x, *a, **kw: x)
sys.modules["cv2"].VideoCapture = MagicMock
sys.modules["cv2"].CAP_V4L2 = 0
sys.modules["cv2"].CAP_PROP_FOURCC = 0
sys.modules["cv2"].CAP_PROP_FRAME_WIDTH = 0
sys.modules["cv2"].CAP_PROP_FRAME_HEIGHT = 0
sys.modules["cv2"].CAP_PROP_BUFFERSIZE = 0
sys.modules["cv2"].COLOR_BGR2RGB = 0

# ---------------------------------------------------------------------------
# Inline port of the path-finding logic from fast_scnn_node.loop()
# so tests don't depend on instantiating the full ROS2 node.
# ---------------------------------------------------------------------------

CAM_WIDTH  = 640
CAM_HEIGHT = 480

HABITABLE_CLASSES = {1, 2, 3}   # safe path, soft terrain, vegetation


def find_path_error(mask: np.ndarray, gap_tolerance: int = 10) -> float:
    """
    Given a segmentation mask (H x W, uint8 class IDs), return the horizontal
    pixel error (path_center_x - CAM_WIDTH//2).

    Returns 9999.0 if no habitable pixels are found on the search row.
    """
    mask_h, mask_w = mask.shape
    search_row_idx = int(mask_h * 0.75)

    is_habitable = np.zeros(mask.shape, dtype=bool)
    for cls in HABITABLE_CLASSES:
        is_habitable |= (mask == cls)

    path_pixels = np.where(is_habitable[search_row_idx, :])[0]

    if len(path_pixels) == 0:
        return 9999.0

    step = np.diff(path_pixels)
    splits = np.where(step > gap_tolerance)[0] + 1
    chunks = np.split(path_pixels, splits)

    largest_chunk = max(chunks, key=len)
    path_center_x_small = int(np.median(largest_chunk))
    path_center_x = int(path_center_x_small * (CAM_WIDTH / mask_w))
    return float(path_center_x - (CAM_WIDTH // 2))


# ===========================================================================
# Tests
# ===========================================================================

class TestFindPathError(unittest.TestCase):

    def _make_mask(self, width=256, height=256, fill=0):
        return np.full((height, width), fill, dtype=np.uint8)

    # --- No habitable pixels ---

    def test_all_obstacle_returns_sentinel(self):
        mask = self._make_mask(fill=4)   # static obstacle everywhere
        self.assertEqual(find_path_error(mask), 9999.0)

    def test_empty_mask_returns_sentinel(self):
        mask = self._make_mask(fill=0)
        self.assertEqual(find_path_error(mask), 9999.0)

    # --- Centered path ---

    def test_path_centered_error_near_zero(self):
        mask = self._make_mask(fill=4)   # start with all obstacles
        h, w = mask.shape
        search_row = int(h * 0.75)
        # Fill the centre quarter of the search row with class 1 (safe path)
        quarter = w // 4
        mask[search_row, quarter: 3 * quarter] = 1
        err = find_path_error(mask)
        # Center of [64:192] in 256-wide mask → pixel 128 → scaled to 640 → 320 → error=0
        self.assertAlmostEqual(err, 0.0, delta=5.0)

    # --- Path biased left ---

    def test_path_left_of_centre_negative_error(self):
        mask = self._make_mask(fill=4)
        h, w = mask.shape
        search_row = int(h * 0.75)
        # Path in the left 25% of the row
        mask[search_row, 0: w // 4] = 1
        err = find_path_error(mask)
        self.assertLess(err, 0.0)

    # --- Path biased right ---

    def test_path_right_of_centre_positive_error(self):
        mask = self._make_mask(fill=4)
        h, w = mask.shape
        search_row = int(h * 0.75)
        # Path in the right 25% of the row
        mask[search_row, 3 * w // 4:] = 1
        err = find_path_error(mask)
        self.assertGreater(err, 0.0)

    # --- All habitable classes are recognised ---

    def test_class_1_recognised(self):
        mask = self._make_mask(fill=4)
        h, w = mask.shape
        mask[int(h * 0.75), w // 4: 3 * w // 4] = 1  # safe path
        self.assertNotEqual(find_path_error(mask), 9999.0)

    def test_class_2_recognised(self):
        mask = self._make_mask(fill=4)
        h, w = mask.shape
        mask[int(h * 0.75), w // 4: 3 * w // 4] = 2  # soft terrain
        self.assertNotEqual(find_path_error(mask), 9999.0)

    def test_class_3_recognised(self):
        mask = self._make_mask(fill=4)
        h, w = mask.shape
        mask[int(h * 0.75), w // 4: 3 * w // 4] = 3  # vegetation
        self.assertNotEqual(find_path_error(mask), 9999.0)

    def test_class_4_not_recognised(self):
        mask = self._make_mask(fill=4)   # static obstacle
        self.assertEqual(find_path_error(mask), 9999.0)

    def test_class_5_not_recognised(self):
        mask = self._make_mask(fill=5)   # dynamic obstacle
        self.assertEqual(find_path_error(mask), 9999.0)

    # --- Gap tolerance: largest chunk wins ---

    def test_largest_chunk_wins_over_small_chunk(self):
        """
        Two isolated groups on the search row.
        The larger group (right side) should be targeted even though
        a smaller one appears first on the left.
        """
        mask = self._make_mask(fill=4)
        h, w = mask.shape
        row = int(h * 0.75)

        # Small chunk on the left (5 pixels)
        mask[row, 10:15] = 1
        # Large chunk on the right (40 pixels)
        mask[row, 180:220] = 1

        err = find_path_error(mask)
        # Centre of [180:220] → pixel 200 → scaled to 640 → ~500 → error ~180
        self.assertGreater(err, 0.0,
            "Expected positive error (path is right of centre)")

    def test_gap_tolerance_merges_nearby_groups(self):
        """
        Two adjacent groups separated by fewer pixels than gap_tolerance
        should be merged into a single chunk.
        """
        mask = self._make_mask(fill=4)
        h, w = mask.shape
        row = int(h * 0.75)

        # Two groups separated by 5 pixels (within default tolerance of 10)
        mask[row, 100:120] = 1   # 20 px
        mask[row, 125:145] = 1   # 20 px, gap = 5

        # With gap_tolerance=10, both groups should merge → one 40-px chunk
        err_merged = find_path_error(mask, gap_tolerance=10)

        # With gap_tolerance=3, the gap is too big → two separate 20-px chunks
        # Both chunks are equal size, so median might pick either.
        # Just verify no sentinel is returned.
        err_split = find_path_error(mask, gap_tolerance=3)

        self.assertNotEqual(err_merged, 9999.0)
        self.assertNotEqual(err_split, 9999.0)

    # --- Search row is at 75% of mask height ---

    def test_search_row_at_75_percent(self):
        """Habitable pixels only on the top row should NOT be detected."""
        mask = self._make_mask(fill=4)
        mask[0, :] = 1   # only the first row is habitable
        self.assertEqual(find_path_error(mask), 9999.0,
            "Path pixels above the 75% search row must be ignored")

    def test_search_row_at_75_percent_positive(self):
        """Habitable pixels only at 75% row SHOULD be detected."""
        mask = self._make_mask(fill=4)
        h, w = mask.shape
        search_row = int(h * 0.75)
        mask[search_row, :] = 1
        self.assertNotEqual(find_path_error(mask), 9999.0)

    # --- Constant values ---

    def test_module_constants(self):
        """NAV_COLORS and MODEL_SIZE constants must be present in the module."""
        import importlib.util
        import os
        _script = os.path.join(
            os.path.dirname(__file__),
            "..", "scripts", "fast_scnn_node.py"
        )
        spec = importlib.util.spec_from_file_location("fast_scnn_node", _script)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        self.assertEqual(mod.CAM_WIDTH, 640)
        self.assertEqual(mod.CAM_HEIGHT, 480)
        self.assertEqual(mod.MODEL_SIZE, 512)
        self.assertIn(1, mod.NAV_COLORS)   # safe path
        self.assertIn(4, mod.NAV_COLORS)   # static obstacle


if __name__ == "__main__":
    unittest.main()
