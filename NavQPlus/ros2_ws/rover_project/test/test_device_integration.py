"""
Device-level integration tests.

Tests the communication contracts between physical devices:
  - ESP32 → CAN Bus → NavQPlus  (CanBridgeNode decoding)
  - NavQPlus → UART  → Arduino  (UartNode encoding + mode switching)

No live hardware, CAN socket, or UART port is needed — all hardware
interfaces are mocked at the module boundary.
"""

import sys
import types
import time
import unittest
from unittest.mock import MagicMock

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 base mocks  (must be set before any script module is loaded)
# ─────────────────────────────────────────────────────────────────────────────
_mock_rclpy       = types.ModuleType("rclpy")
_mock_rclpy.node  = types.ModuleType("rclpy.node")
sys.modules.setdefault("rclpy",      _mock_rclpy)
sys.modules.setdefault("rclpy.node", _mock_rclpy.node)


# ─────────────────────────────────────────────────────────────────────────────
# Message module mocks — use MagicMock() as class factory so that msg.attr
# assignments and reads work transparently in node code
# ─────────────────────────────────────────────────────────────────────────────
def _msg_module(mod_name, *cls_names):
    mod = types.ModuleType(mod_name)
    for n in cls_names:
        setattr(mod, n, MagicMock)   # MagicMock()() → MagicMock; attrs r/w freely
    return mod

_rover_msg  = _msg_module("rover_project.msg",  "GpsFix", "ImuOrientation", "Proximity",
                                                 "NavGoal", "WaypointList")
_std_msg    = _msg_module("std_msgs.msg",        "Int32", "Bool", "Float32", "String",
                                                 "Float64MultiArray")
_sensor_msg = _msg_module("sensor_msgs.msg",     "NavSatFix", "NavSatStatus", "Image")

_sensor_msg.NavSatStatus = MagicMock(
    STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_GBAS_FIX=2, SERVICE_GPS=1)
_sensor_msg.NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN = 2

for _pkg, _msg_mod in [
    ("rover_project",  _rover_msg),
    ("std_msgs",       _std_msg),
    ("sensor_msgs",    _sensor_msg),
]:
    _p = types.ModuleType(_pkg)
    sys.modules.setdefault(_pkg, _p)
    sys.modules[f"{_pkg}.msg"] = _msg_mod

# ─── ament_index_python ───────────────────────────────────────────────────────
_ament_pkgs = types.ModuleType("ament_index_python.packages")
_ament_pkgs.get_package_share_directory = lambda _: "/tmp/mock_pkg_share"
sys.modules.setdefault("ament_index_python",          types.ModuleType("ament_index_python"))
sys.modules["ament_index_python.packages"] = _ament_pkgs

# ─── cantools ────────────────────────────────────────────────────────────────
_cantools = types.ModuleType("cantools")
_cantools.database = types.ModuleType("cantools.database")
_cantools.database.load_file = MagicMock()
sys.modules.setdefault("cantools",          _cantools)
sys.modules.setdefault("cantools.database", _cantools.database)

# ─── python-can ──────────────────────────────────────────────────────────────
_can = types.ModuleType("can")
_can.interface = types.ModuleType("can.interface")
_can.interface.Bus = MagicMock()
sys.modules.setdefault("can",           _can)
sys.modules.setdefault("can.interface", _can.interface)

# ─── pyserial ────────────────────────────────────────────────────────────────
_serial = types.ModuleType("serial")
_serial.Serial      = MagicMock()
_serial.EIGHTBITS   = 8
_serial.PARITY_NONE = "N"
_serial.STOPBITS_ONE = 1
sys.modules.setdefault("serial", _serial)


# ─────────────────────────────────────────────────────────────────────────────
# MockNode infrastructure
# ─────────────────────────────────────────────────────────────────────────────
class _MockPublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    @property
    def last(self):
        return self.msgs[-1] if self.msgs else None


class _MockClock:
    def __init__(self):
        self._t = 1000.0          # start at an arbitrary non-zero epoch

    def set(self, t):
        self._t = t

    def advance(self, dt):
        self._t += dt

    def now(self):
        t = self._t
        class _NS:
            nanoseconds = int(t * 1e9)
            def to_msg(self_inner):
                return MagicMock()
        return _NS()


class MockNode:
    """Minimal ROS2 Node replacement.  Provides just enough API surface for
    the rover nodes to initialise and run their callbacks under test."""

    def __init__(self, *args, **kwargs):
        self._params = {}
        self._pubs   = {}
        self.clock   = _MockClock()

    # ── Parameter API ─────────────────────────────────────────────────────────
    def declare_parameter(self, name, default):
        self._params[name] = default

    def get_parameter(self, name):
        v = self._params.get(name)
        class _P:
            value = v
        return _P()

    # ── Publisher / Subscriber / Timer ────────────────────────────────────────
    def create_publisher(self, msg_type, topic, qos=10):
        pub = _MockPublisher()
        self._pubs[topic] = pub
        return pub

    def create_subscription(self, *args, **kwargs):
        return None

    def create_timer(self, *args, **kwargs):
        return None

    # ── Logging / Clock ───────────────────────────────────────────────────────
    def get_logger(self):
        class _L:
            def info(self, *a, **kw):  pass
            def warn(self, *a, **kw):  pass
            def error(self, *a, **kw): pass
        return _L()

    def get_clock(self):
        return self.clock


# Install MockNode as the ROS2 Node base BEFORE loading any script module
_mock_rclpy.node.Node = MockNode


# ─────────────────────────────────────────────────────────────────────────────
# Module loader helper
# ─────────────────────────────────────────────────────────────────────────────
import importlib.util, os as _os

def _load(script_name):
    """Load a scripts/<script_name>.py as an isolated module."""
    path = _os.path.join(_os.path.dirname(__file__), "..", "scripts", f"{script_name}.py")
    spec = importlib.util.spec_from_file_location(script_name, path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ─────────────────────────────────────────────────────────────────────────────
# CAN frame IDs matching the rover.dbc definitions
# ─────────────────────────────────────────────────────────────────────────────
_CAN_IDS = {
    "GPS_Position":     0x100,
    "GPS_Velocity":     0x101,
    "GPS_Accuracy":     0x102,
    "IMU_Orientation":  0x105,
    "Proximity_Sensors":0x107,
}

def _make_can_bridge():
    """Instantiate CanBridgeNode with all hardware replaced by mocks."""
    mod = _load("can_bridge_node")

    # Mock cantools DB
    mock_db = MagicMock()
    mock_db.messages = []           # no generic CAN messages needed
    def _by_name(name):
        m = MagicMock()
        m.frame_id = _CAN_IDS[name]
        return m
    mock_db.get_message_by_name.side_effect = _by_name
    _cantools.database.load_file.return_value = mock_db

    # Mock CAN bus (idle by default)
    mock_bus = MagicMock()
    mock_bus.recv.return_value = None
    _can.interface.Bus.return_value = mock_bus

    node = mod.CanBridgeNode()
    return node, mock_db, mock_bus


def _inject_can(node, mock_db, mock_bus, arb_id, decoded_dict):
    """Simulate one CAN frame arriving on the bus."""
    fake_frame              = MagicMock()
    fake_frame.arbitration_id = arb_id
    fake_frame.data         = bytes(8)                 # content unused; decoded via mock
    mock_db.decode_message.return_value = decoded_dict
    mock_bus.recv.side_effect = [fake_frame, None]     # one message, then bus idle
    node.can_callback()


# ═════════════════════════════════════════════════════════════════════════════
# ESP32 → CAN → NavQPlus : GPS decoding
# ═════════════════════════════════════════════════════════════════════════════
class TestCanBridgeGpsDecoding(unittest.TestCase):
    """Verify that GPS CAN frames are decoded and accumulated into GpsFix."""

    def setUp(self):
        self.node, self.db, self.bus = _make_can_bridge()

    # ── Individual frame decoding ─────────────────────────────────────────────

    def test_position_frame_sets_latitude_and_longitude(self):
        """0x100 GPS_Position → GpsFix.latitude / longitude correct."""
        _inject_can(self.node, self.db, self.bus, 0x100,
                    {"GPS_Latitude": 33.9737, "GPS_Longitude": -117.3281})
        msg = self.node.gps_pub.last
        self.assertIsNotNone(msg)
        self.assertAlmostEqual(msg.latitude,  33.9737,  places=4)
        self.assertAlmostEqual(msg.longitude, -117.3281, places=4)

    def test_velocity_frame_sets_fix_type_sats_and_speed(self):
        """0x101 GPS_Velocity → GpsFix.fix_type, num_sats, speed correct."""
        _inject_can(self.node, self.db, self.bus, 0x101, {
            "GPS_Speed": 1.5, "GPS_Course": 90.0,
            "GPS_FixType": 3, "GPS_NumSats": 9, "GPS_HDOP": 1.1,
        })
        msg = self.node.gps_pub.last
        self.assertEqual(msg.fix_type,  3)
        self.assertEqual(msg.num_sats,  9)
        self.assertAlmostEqual(msg.speed, 1.5, places=2)

    def test_accuracy_frame_sets_h_acc_and_v_acc(self):
        """0x102 GPS_Accuracy → GpsFix.h_acc / v_acc correct."""
        _inject_can(self.node, self.db, self.bus, 0x102,
                    {"GPS_hAcc": 2.5, "GPS_vAcc": 4.0})
        msg = self.node.gps_pub.last
        self.assertAlmostEqual(msg.h_acc, 2.5, places=2)
        self.assertAlmostEqual(msg.v_acc, 4.0, places=2)

    # ── Accumulation across frames ────────────────────────────────────────────

    def test_gps_state_survives_subsequent_frames(self):
        """Lat/lon from 0x100 must still be present after 0x101 arrives."""
        _inject_can(self.node, self.db, self.bus, 0x100,
                    {"GPS_Latitude": 33.9737, "GPS_Longitude": -117.3281})
        _inject_can(self.node, self.db, self.bus, 0x101, {
            "GPS_Speed": 0.5, "GPS_Course": 45.0,
            "GPS_FixType": 3, "GPS_NumSats": 7, "GPS_HDOP": 1.2,
        })
        last = self.node.gps_pub.last
        self.assertAlmostEqual(last.latitude, 33.9737, places=4)
        self.assertEqual(last.fix_type, 3)

    def test_each_gps_frame_triggers_one_publish(self):
        """Every GPS CAN frame (0x100, 0x101, 0x102) must call publish once."""
        _inject_can(self.node, self.db, self.bus, 0x100,
                    {"GPS_Latitude": 1.0, "GPS_Longitude": 2.0})
        c1 = len(self.node.gps_pub.msgs)

        _inject_can(self.node, self.db, self.bus, 0x101, {
            "GPS_Speed": 0.0, "GPS_Course": 0.0,
            "GPS_FixType": 2, "GPS_NumSats": 5, "GPS_HDOP": 1.5,
        })
        c2 = len(self.node.gps_pub.msgs)

        _inject_can(self.node, self.db, self.bus, 0x102,
                    {"GPS_hAcc": 1.0, "GPS_vAcc": 2.0})
        c3 = len(self.node.gps_pub.msgs)

        self.assertEqual(c1, 1)
        self.assertEqual(c2, 2)
        self.assertEqual(c3, 3)

    def test_unknown_can_id_does_not_publish(self):
        """An unrecognised arbitration ID must not trigger any GPS publish."""
        _inject_can(self.node, self.db, self.bus, 0x999, {})
        self.assertEqual(len(self.node.gps_pub.msgs), 0)


# ═════════════════════════════════════════════════════════════════════════════
# ESP32 → CAN → NavQPlus : IMU decoding
# ═════════════════════════════════════════════════════════════════════════════
class TestCanBridgeImuDecoding(unittest.TestCase):
    """Verify that IMU_Orientation CAN frames produce correct ImuOrientation msgs."""

    def setUp(self):
        self.node, self.db, self.bus = _make_can_bridge()

    def _inject_imu(self, decoded):
        _inject_can(self.node, self.db, self.bus, 0x105, decoded)

    def test_heading_pitch_roll_decoded_correctly(self):
        self._inject_imu({
            "IMU_Heading": 270.5, "IMU_Pitch": -5.0, "IMU_Roll": 2.0,
            "IMU_CalSys": 3, "IMU_CalMag": 2,
        })
        msg = self.node.imu_ori_pub.last
        self.assertAlmostEqual(msg.heading, 270.5, places=2)
        self.assertAlmostEqual(msg.pitch,   -5.0,  places=2)
        self.assertAlmostEqual(msg.roll,     2.0,  places=2)

    def test_calibration_status_decoded_correctly(self):
        self._inject_imu({
            "IMU_Heading": 0.0, "IMU_Pitch": 0.0, "IMU_Roll": 0.0,
            "IMU_CalSys": 3, "IMU_CalMag": 3,
        })
        msg = self.node.imu_ori_pub.last
        self.assertEqual(msg.cal_sys, 3)
        self.assertEqual(msg.cal_mag, 3)

    def test_imu_frame_does_not_affect_gps_publisher(self):
        """IMU frame must not cause a GPS publish."""
        self._inject_imu({
            "IMU_Heading": 90.0, "IMU_Pitch": 0.0, "IMU_Roll": 0.0,
            "IMU_CalSys": 0, "IMU_CalMag": 0,
        })
        self.assertEqual(len(self.node.gps_pub.msgs), 0)


# ═════════════════════════════════════════════════════════════════════════════
# ESP32 → CAN → NavQPlus : Proximity decoding
# ═════════════════════════════════════════════════════════════════════════════
class TestCanBridgeProximityDecoding(unittest.TestCase):
    """
    Verify Proximity_Sensors CAN frames are decoded into Proximity messages.

    IMPORTANT — front/rear field swap:
      can_bridge_node.py lines 137-138 assign the CAN field 'Proximity_Rear'
      to the ROS message field proximity_front, and 'Proximity_Front' to
      proximity_rear.  The _Valid flags are swapped the same way (lines 141-142).
      These tests document the *current* behaviour so that any correction to
      the swap is immediately caught by a failing test.
    """

    def setUp(self):
        self.node, self.db, self.bus = _make_can_bridge()

    def _inject_prox(self, decoded):
        _inject_can(self.node, self.db, self.bus, 0x107, decoded)

    def test_cliff_detected_flag_published(self):
        self._inject_prox({
            "Proximity_Front": 800, "Proximity_Rear": 600, "Proximity_Cliff": 10,
            "Cliff_Detected": True, "Front_Valid": True, "Rear_Valid": True,
            "Cliff_Valid": True,
        })
        msg = self.node.proximity_pub.last
        self.assertTrue(msg.cliff_detected)
        self.assertTrue(msg.cliff_valid)

    def test_cliff_not_detected_when_clear(self):
        self._inject_prox({
            "Proximity_Front": 900, "Proximity_Rear": 900, "Proximity_Cliff": 800,
            "Cliff_Detected": False, "Front_Valid": True, "Rear_Valid": True,
            "Cliff_Valid": True,
        })
        self.assertFalse(self.node.proximity_pub.last.cliff_detected)

    def test_proximity_cliff_distance_published(self):
        self._inject_prox({
            "Proximity_Front": 900, "Proximity_Rear": 400, "Proximity_Cliff": 250,
            "Cliff_Detected": False, "Front_Valid": True, "Rear_Valid": True,
            "Cliff_Valid": True,
        })
        self.assertEqual(self.node.proximity_pub.last.proximity_cliff, 250)

    def test_front_rear_field_swap_documented(self):
        """
        The CAN field 'Proximity_Rear' (value=400) is written to the ROS
        proximity_front field, and 'Proximity_Front' (value=900) is written
        to proximity_rear.  If this test fails it means the swap was fixed —
        update the assertion and remove this note.
        """
        self._inject_prox({
            "Proximity_Front": 900, "Proximity_Rear": 400, "Proximity_Cliff": 0,
            "Cliff_Detected": False, "Front_Valid": True, "Rear_Valid": False,
            "Cliff_Valid": False,
        })
        msg = self.node.proximity_pub.last
        # Swapped: CAN Rear → ROS front
        self.assertEqual(msg.proximity_front, 400)
        self.assertEqual(msg.proximity_rear,  900)
        # Valid flags also swapped: CAN Rear_Valid (False) → ros front_valid
        self.assertFalse(msg.front_valid)
        self.assertTrue(msg.rear_valid)


# ═════════════════════════════════════════════════════════════════════════════
# NavQPlus → UART → Arduino : UartNode mode switching + serial write
# ═════════════════════════════════════════════════════════════════════════════
class TestUartNodeDeviceInterface(unittest.TestCase):
    """
    Verify that UartNode correctly:
      - Forwards AI drive commands as single bytes to the Arduino
      - Switches between AI and MANUAL modes on Bluetooth input
      - Sends a STOP byte when the failsafe timeout fires
    """

    def setUp(self):
        self.mod = _load("uart_node")

        # Fresh mock serial port for each test
        self.mock_serial = MagicMock()
        _serial.Serial.return_value = self.mock_serial

        self.node = self.mod.UartNode()
        # Start in a known state
        self.node.state = self.mod.AI

    def _int32(self, v):
        msg = MagicMock()
        msg.data = v
        return msg

    # ── AI-mode forwarding ───────────────────────────────────────────────────

    def test_ai_drive_cmd_forwarded_to_uart_in_ai_mode(self):
        """nav/drive_cmd value is written verbatim as a single byte."""
        self.node.ai_command(self._int32(self.mod.CMD_FORWARD_STRAIGHT))
        self.mock_serial.write.assert_called_once_with(
            bytes([self.mod.CMD_FORWARD_STRAIGHT]))

    def test_ai_drive_cmd_ignored_in_manual_mode(self):
        """nav/drive_cmd must NOT reach UART when in MANUAL mode."""
        self.node.state = self.mod.MANUAL
        self.node.ai_command(self._int32(self.mod.CMD_FORWARD_STRAIGHT))
        self.mock_serial.write.assert_not_called()

    def test_all_ai_commands_forwarded_as_correct_bytes(self):
        """Each command value is written as the matching single byte."""
        cmds = [
            self.mod.CMD_STOP, self.mod.CMD_FORWARD_STRAIGHT,
            self.mod.CMD_FORWARD_RIGHT, self.mod.CMD_FORWARD_LEFT,
            self.mod.CMD_LEFT, self.mod.CMD_RIGHT,
        ]
        for cmd in cmds:
            self.mock_serial.reset_mock()
            self.node.ai_command(self._int32(cmd))
            self.mock_serial.write.assert_called_once_with(bytes([cmd]))

    # ── Mode switching (Bluetooth) ────────────────────────────────────────────

    def test_bluetooth_manual_cmd_switches_to_manual_mode(self):
        """Any non-AI Bluetooth command must set mode to MANUAL."""
        self.node.manual_command(self._int32(self.mod.CMD_FORWARD_STRAIGHT))
        self.assertEqual(self.node.state, self.mod.MANUAL)
        self.mock_serial.write.assert_called_once_with(
            bytes([self.mod.CMD_FORWARD_STRAIGHT]))

    def test_bluetooth_cmd_ai_switches_to_ai_mode(self):
        """CMD_AI (=1) from Bluetooth must switch from MANUAL to AI."""
        self.node.state = self.mod.MANUAL
        self.node.manual_command(self._int32(self.mod.CMD_AI))
        self.assertEqual(self.node.state, self.mod.AI)
        self.mock_serial.write.assert_called_once_with(bytes([self.mod.CMD_AI]))

    def test_bluetooth_cmd_ai_when_already_ai_is_idempotent(self):
        """CMD_AI when already in AI mode must not send a redundant UART byte."""
        self.node.state = self.mod.AI
        self.node.manual_command(self._int32(self.mod.CMD_AI))
        self.assertEqual(self.node.state, self.mod.AI)
        self.mock_serial.write.assert_not_called()

    def test_manual_then_ai_then_manual_mode_cycling(self):
        """Repeated mode switching must track state correctly."""
        self.node.manual_command(self._int32(self.mod.CMD_FORWARD_STRAIGHT))
        self.assertEqual(self.node.state, self.mod.MANUAL)

        self.node.manual_command(self._int32(self.mod.CMD_AI))
        self.assertEqual(self.node.state, self.mod.AI)

        self.node.manual_command(self._int32(self.mod.CMD_STOP))
        self.assertEqual(self.node.state, self.mod.MANUAL)

    def test_ai_command_ignored_after_bluetooth_manual_switch(self):
        """After switching to MANUAL, AI commands must be silently dropped."""
        self.node.manual_command(self._int32(self.mod.CMD_FORWARD_STRAIGHT))
        self.mock_serial.reset_mock()

        self.node.ai_command(self._int32(self.mod.CMD_FORWARD_STRAIGHT))
        self.mock_serial.write.assert_not_called()

    # ── Failsafe ─────────────────────────────────────────────────────────────

    def test_failsafe_sends_stop_after_timeout_exceeded(self):
        """failsafe_check must write CMD_STOP when last command is stale."""
        self.node.last_cmd_time = time.time() - self.node.timeout_seconds - 1.0
        self.node.failsafe_check()
        self.mock_serial.write.assert_called_with(bytes([self.mod.CMD_STOP]))

    def test_failsafe_does_not_fire_within_timeout(self):
        """failsafe_check must do nothing when a recent command was received."""
        self.node.last_cmd_time = time.time()
        self.node.failsafe_check()
        self.mock_serial.write.assert_not_called()

    def test_failsafe_resets_timer_after_firing(self):
        """After firing, failsafe must update last_cmd_time to prevent thrashing."""
        self.node.last_cmd_time = time.time() - self.node.timeout_seconds - 1.0
        t_before = time.time()
        self.node.failsafe_check()
        self.assertGreaterEqual(self.node.last_cmd_time, t_before)


if __name__ == "__main__":
    unittest.main()
