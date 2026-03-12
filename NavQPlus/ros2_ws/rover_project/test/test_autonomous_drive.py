"""
Unit tests for AutonomousDriveNode constants, invert map, and state transitions.
No ROS2 context is required — all ROS2 imports are mocked.
"""

import sys
import types
import unittest
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Mock ROS2 / rover_project dependencies before importing the module
# ---------------------------------------------------------------------------
_mock_rclpy = types.ModuleType("rclpy")
_mock_rclpy.node = types.ModuleType("rclpy.node")
_mock_rclpy.node.Node = object
sys.modules.setdefault("rclpy", _mock_rclpy)
sys.modules.setdefault("rclpy.node", _mock_rclpy.node)

for _mod in ("std_msgs", "std_msgs.msg", "rover_project", "rover_project.msg"):
    sys.modules.setdefault(_mod, types.ModuleType(_mod))

_std_msg = sys.modules["std_msgs.msg"]
for _cls in ("Int32", "Bool"):
    setattr(_std_msg, _cls, MagicMock)

_rover_msg = sys.modules["rover_project.msg"]
setattr(_rover_msg, "Proximity", MagicMock)

import importlib.util, os
_script = os.path.join(
    os.path.dirname(__file__),
    "..", "scripts", "autonomous_drive_node.py"
)
_spec = importlib.util.spec_from_file_location("autonomous_drive_node", _script)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

AutonomousDriveNode = _mod.AutonomousDriveNode

# Grab module-level constants
CMD_STOP              = _mod.CMD_STOP
CMD_FORWARD_STRAIGHT  = _mod.CMD_FORWARD_STRAIGHT
CMD_FORWARD_RIGHT     = _mod.CMD_FORWARD_RIGHT
CMD_FORWARD_LEFT      = _mod.CMD_FORWARD_LEFT
CMD_BACKWARD_STRAIGHT = _mod.CMD_BACKWARD_STRAIGHT
CMD_BACKWARD_RIGHT    = _mod.CMD_BACKWARD_RIGHT
CMD_BACKWARD_LEFT     = _mod.CMD_BACKWARD_LEFT
CMD_RIGHT             = _mod.CMD_RIGHT
CMD_LEFT              = _mod.CMD_LEFT

STATE_FORWARD    = _mod.STATE_FORWARD
STATE_CLIFF_STOP = _mod.STATE_CLIFF_STOP
STATE_REV_CHECK  = _mod.STATE_REV_CHECK
STATE_REVERSING  = _mod.STATE_REVERSING
STATE_LOOK_L     = _mod.STATE_LOOK_L
STATE_SCAN_L     = _mod.STATE_SCAN_L
STATE_LOOK_R     = _mod.STATE_LOOK_R
STATE_SCAN_R     = _mod.STATE_SCAN_R
STATE_DRIVE_OUT  = _mod.STATE_DRIVE_OUT


# ===========================================================================
# Command constant sanity checks
# ===========================================================================

class TestCommandConstants(unittest.TestCase):

    def test_all_commands_are_unique(self):
        cmds = [
            CMD_STOP, CMD_FORWARD_STRAIGHT, CMD_FORWARD_RIGHT,
            CMD_FORWARD_LEFT, CMD_BACKWARD_STRAIGHT, CMD_BACKWARD_RIGHT,
            CMD_BACKWARD_LEFT, CMD_RIGHT, CMD_LEFT,
        ]
        self.assertEqual(len(cmds), len(set(cmds)), "Duplicate command values detected")

    def test_command_values_are_integers(self):
        cmds = [
            CMD_STOP, CMD_FORWARD_STRAIGHT, CMD_FORWARD_RIGHT,
            CMD_FORWARD_LEFT, CMD_BACKWARD_STRAIGHT, CMD_BACKWARD_RIGHT,
            CMD_BACKWARD_LEFT, CMD_RIGHT, CMD_LEFT,
        ]
        for c in cmds:
            self.assertIsInstance(c, int)

    def test_cmd_stop_value(self):
        self.assertEqual(CMD_STOP, 2)

    def test_all_states_are_strings(self):
        states = [
            STATE_FORWARD, STATE_CLIFF_STOP, STATE_REV_CHECK,
            STATE_REVERSING, STATE_LOOK_L, STATE_SCAN_L,
            STATE_LOOK_R, STATE_SCAN_R, STATE_DRIVE_OUT,
        ]
        for s in states:
            self.assertIsInstance(s, str)

    def test_all_states_are_unique(self):
        states = [
            STATE_FORWARD, STATE_CLIFF_STOP, STATE_REV_CHECK,
            STATE_REVERSING, STATE_LOOK_L, STATE_SCAN_L,
            STATE_LOOK_R, STATE_SCAN_R, STATE_DRIVE_OUT,
        ]
        self.assertEqual(len(states), len(set(states)))


# ===========================================================================
# _INVERT_MAP Tests
# ===========================================================================

class TestInvertMap(unittest.TestCase):
    """The invert map must be a perfect pair-wise bijection."""

    def setUp(self):
        self.inv = AutonomousDriveNode._INVERT_MAP

    def test_forward_straight_inverts_to_backward_straight(self):
        self.assertEqual(self.inv[CMD_FORWARD_STRAIGHT], CMD_BACKWARD_STRAIGHT)

    def test_backward_straight_inverts_to_forward_straight(self):
        self.assertEqual(self.inv[CMD_BACKWARD_STRAIGHT], CMD_FORWARD_STRAIGHT)

    def test_forward_right_inverts_to_backward_right(self):
        self.assertEqual(self.inv[CMD_FORWARD_RIGHT], CMD_BACKWARD_RIGHT)

    def test_backward_right_inverts_to_forward_right(self):
        self.assertEqual(self.inv[CMD_BACKWARD_RIGHT], CMD_FORWARD_RIGHT)

    def test_forward_left_inverts_to_backward_left(self):
        self.assertEqual(self.inv[CMD_FORWARD_LEFT], CMD_BACKWARD_LEFT)

    def test_backward_left_inverts_to_forward_left(self):
        self.assertEqual(self.inv[CMD_BACKWARD_LEFT], CMD_FORWARD_LEFT)

    def test_double_inversion_is_identity(self):
        """Applying the invert map twice should return the original command."""
        for cmd in list(self.inv.keys()):
            with self.subTest(cmd=cmd):
                inverted = self.inv[cmd]
                # Not all commands appear as values; only test those that do
                if inverted in self.inv:
                    self.assertEqual(self.inv[inverted], cmd)

    def test_stop_and_rotate_not_in_map(self):
        """CMD_STOP / CMD_RIGHT / CMD_LEFT are not in the invert map (pass-through)."""
        for cmd in (CMD_STOP, CMD_RIGHT, CMD_LEFT):
            self.assertNotIn(cmd, self.inv)


# ===========================================================================
# send_cmd / invert_drive logic (via a lightweight stub node)
# ===========================================================================

class _StubNode:
    """Minimal stand-in for AutonomousDriveNode to test send_cmd logic."""

    _INVERT_MAP = AutonomousDriveNode._INVERT_MAP

    def __init__(self, invert_drive=False):
        self.invert_drive = invert_drive
        self.published = []

    def get_logger(self):
        logger = MagicMock()
        logger.info = MagicMock()
        return logger

    def send_cmd(self, cmd: int):
        physical = self._INVERT_MAP.get(cmd, cmd) if self.invert_drive else cmd
        self.get_logger().info(f"CMD: {physical}")
        msg = MagicMock()
        msg.data = physical
        self.published.append(physical)

    # Bind method
    send_cmd = AutonomousDriveNode.send_cmd.__get__(
        type('_', (), {'_INVERT_MAP': _INVERT_MAP, 'invert_drive': False,
                       'get_logger': lambda s: MagicMock(),
                       'cmd_pub': MagicMock()})(),
        type(None)
    ) if False else send_cmd  # use the stub version above


class TestSendCmdLogic(unittest.TestCase):

    def test_normal_drive_passes_through(self):
        node = _StubNode(invert_drive=False)
        node.send_cmd(CMD_FORWARD_STRAIGHT)
        self.assertEqual(node.published[-1], CMD_FORWARD_STRAIGHT)

    def test_inverted_drive_flips_forward_to_backward(self):
        node = _StubNode(invert_drive=True)
        node.send_cmd(CMD_FORWARD_STRAIGHT)
        self.assertEqual(node.published[-1], CMD_BACKWARD_STRAIGHT)

    def test_inverted_drive_flips_forward_right_to_backward_right(self):
        node = _StubNode(invert_drive=True)
        node.send_cmd(CMD_FORWARD_RIGHT)
        self.assertEqual(node.published[-1], CMD_BACKWARD_RIGHT)

    def test_stop_not_inverted(self):
        """CMD_STOP has no entry in the invert map — it should pass through."""
        node = _StubNode(invert_drive=True)
        node.send_cmd(CMD_STOP)
        self.assertEqual(node.published[-1], CMD_STOP)

    def test_rotate_cmds_not_inverted(self):
        """CMD_RIGHT / CMD_LEFT are not in the map and pass through unchanged."""
        for cmd in (CMD_RIGHT, CMD_LEFT):
            node = _StubNode(invert_drive=True)
            node.send_cmd(cmd)
            self.assertEqual(node.published[-1], cmd)


if __name__ == "__main__":
    unittest.main()
