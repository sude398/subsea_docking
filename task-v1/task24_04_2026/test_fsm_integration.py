"""
Integration test for the docking FSM.

Tests the actual loop() logic by:
  - Mocking rclpy / ROS message types / pymavlink / cv_bridge (not available
    in this sandbox)
  - Aliasing updated_* modules to their un-prefixed names as the docking node
    imports them
  - Subclassing DockingNode to skip hardware __init__ while keeping the real
    loop(), Kalman, PID, and config constants
  - Driving loop() frame-by-frame with controllable vision.process outputs

The critical scenario is Test 5: marker disappears at close range during
descent, system must continue to DOCKED and NEVER return to SEARCHING.
"""
import sys
import time
import types
import unittest.mock as um

import numpy as np

# ─────────────────────────────────────────────────────────────────
# 1. Stub modules that aren't available in the test environment
# ─────────────────────────────────────────────────────────────────

# rclpy / ROS
class _FakeNode:
    def __init__(self, *a, **kw): pass
    def declare_parameter(self, name, default):
        p = um.MagicMock()
        p.value = default
        return p
    def get_parameter(self, name):
        p = um.MagicMock()
        p.value = 0
        return p
    def create_publisher(self, *a, **kw): return um.MagicMock()
    def create_timer(self, *a, **kw): return um.MagicMock()
    def get_logger(self):
        lg = um.MagicMock()
        return lg

fake_rclpy = types.ModuleType("rclpy")
fake_rclpy.init = lambda *a, **kw: None
fake_rclpy.shutdown = lambda *a, **kw: None
fake_rclpy.spin = lambda *a, **kw: None
fake_rclpy_node = types.ModuleType("rclpy.node")
fake_rclpy_node.Node = _FakeNode
fake_rclpy.node = fake_rclpy_node
sys.modules["rclpy"] = fake_rclpy
sys.modules["rclpy.node"] = fake_rclpy_node

fake_sensor_msgs = types.ModuleType("sensor_msgs")
fake_sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
fake_sensor_msgs_msg.Image = um.MagicMock
fake_sensor_msgs.msg = fake_sensor_msgs_msg
sys.modules["sensor_msgs"] = fake_sensor_msgs
sys.modules["sensor_msgs.msg"] = fake_sensor_msgs_msg

fake_cv_bridge = types.ModuleType("cv_bridge")
fake_cv_bridge.CvBridge = um.MagicMock
sys.modules["cv_bridge"] = fake_cv_bridge

# pymavlink
fake_pymavlink = types.ModuleType("pymavlink")
fake_mavutil = types.ModuleType("pymavlink.mavutil")
class _FakeMavConn:
    def __init__(self):
        self.target_system = 1
        self.mav = um.MagicMock()
    def wait_heartbeat(self, timeout=5): return True
    def mode_mapping(self): return {"MANUAL": 0}
fake_mavutil.mavlink_connection = lambda *a, **kw: _FakeMavConn()
fake_mavutil.mavlink = um.MagicMock()
fake_mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
fake_pymavlink.mavutil = fake_mavutil
sys.modules["pymavlink"] = fake_pymavlink
sys.modules["pymavlink.mavutil"] = fake_mavutil

# ─────────────────────────────────────────────────────────────────
# 2. Alias updated_* modules to the names used in imports across files.
#    Order matters: updated_vision and updated_docking_node both import
#    `from config import ...`, so config must be aliased first, then we can
#    safely import the rest.
# ─────────────────────────────────────────────────────────────────
import updated_config        # noqa: E402
sys.modules["config"] = updated_config

import updated_controller    # noqa: E402
sys.modules["controller"] = updated_controller

import updated_vision        # noqa: E402
sys.modules["vision"] = updated_vision

# ─────────────────────────────────────────────────────────────────
# 3. Now import the actual docking node
# ─────────────────────────────────────────────────────────────────
import updated_docking_node  # noqa: E402
from updated_docking_node import DockingNode  # noqa: E402
from updated_config import (  # noqa: E402
    MAX_LATERAL, MAX_YAW,
    CAMERA_OFFSET_Y, CONFIDENCE_THRESHOLD,
    FINAL_ALIGN_DIST, DIST_DOCKED,
    BLIND_DESCENT_DIST, BLIND_DESCENT_TIMEOUT,
)
from updated_controller import PID, Kalman2D  # noqa: E402
from updated_config import PITCH_ANGLE_RAD, UNDERWATER_REFRACTION  # noqa: E402


class VisionStub:
    """
    Minimal stand-in for Vision. Re-implements compute_world() byte-for-byte
    from updated_vision.py so loop()'s blended-pose math is exercised with
    real arithmetic. process() is patched per-frame by the test harness.
    """
    def compute_world(self, tvec):
        x = tvec[0] / 1.2
        y = (tvec[2] * np.cos(PITCH_ANGLE_RAD) - tvec[1] * np.sin(PITCH_ANGLE_RAD)) / UNDERWATER_REFRACTION
        return x, y

    def process(self, frame):
        # Will be replaced per-frame by the harness. Default: nothing visible.
        return (frame, False, False, 0, None, 0.0, 0.0)


# ─────────────────────────────────────────────────────────────────
# 4. Harness: DockingNode with hardware __init__ bypassed
# ─────────────────────────────────────────────────────────────────
class HarnessNode(DockingNode):
    def __init__(self):
        # Skip Node/mavlink/camera init entirely.
        # Populate only the attributes loop() reads/writes.
        self.cap = um.MagicMock()
        self.bridge = um.MagicMock()
        self.pub_img = um.MagicMock()
        self.master = _FakeMavConn()

        # Real algorithmic components. VisionStub reimplements compute_world()
        # exactly as updated_vision.py does, bypassing the ArUco detector which
        # isn't exercised here.
        self.vision = VisionStub()
        self.kalman = Kalman2D()
        self.pid_x = PID(0.7, 0.05, 0.4, MAX_LATERAL)
        self.pid_y = PID(0.7, 0.05, 0.4, MAX_LATERAL)
        self.pid_yaw = PID(0.5, 0.02, 0.3, MAX_YAW)

        # FSM state
        self.state = "SEARCHING"
        self.hover_start = None
        self.blind_start_t = None
        self.is_settled = False
        self.confirmed_4_markers = False
        self.approach_confidence = 0
        self.search_start_t = None
        self.hold_start_t = None
        self.last_seen_t = time.time()
        self.armed = False

        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vyaw = 0.0
        self.last_x_w = 0.0
        self.last_y_w = 0.0
        self.last_d_h = 999.0
        self.last_d_v = 999.0
        self.desc_stable_cnt = 0
        self.final_stable_cnt = 0
        self.last_vz_cmd = 0.0
        self.vision_confidence = 0.0

        # Test bookkeeping
        self.sent_commands = []
        self.transition_log = []     # list of (prev_state, new_state, note)
        self._prev_state = self.state

    def get_logger(self):
        class _L:
            def info(self, *a, **kw): pass
            def warn(self, *a, **kw): pass
            def error(self, *a, **kw): pass
        return _L()

    def send(self, vx, vy, vz, yaw):
        self.sent_commands.append((round(vx, 3), round(vy, 3), round(vz, 3), round(yaw, 3)))

    def _tick(self, visible, n_valid, target_tvec, target_yaw, d_v):
        """Drive one loop() iteration with controlled vision outputs."""
        fake_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.cap.read = lambda: (True, fake_frame)
        # visible, locked, n_valid, target_tvec, target_yaw, d_v
        self.vision.process = lambda frame: (
            frame, visible, visible, n_valid, target_tvec, target_yaw, d_v
        )
        before = self.state
        self.loop()
        after = self.state
        if before != after:
            self.transition_log.append((before, after))
        self._prev_state = after

    def run_frames(self, n, **kw):
        for _ in range(n):
            self._tick(**kw)


# ─────────────────────────────────────────────────────────────────
# 5. Scenario helpers
# ─────────────────────────────────────────────────────────────────
def tvec_for(x_cam, y_cam, z_cam):
    """Build a 3-element tvec as the real Vision would return (camera-frame)."""
    return np.array([x_cam, y_cam, z_cam], dtype=np.float32)


def world_to_tvec(x_w_target, y_w_target, d_v_target):
    """
    Inverse of Vision.compute_world + d_v formula (approximately).
    Vision.compute_world:
        x_cam = x_w * 1.2
        y_cam·sin(θ) - z_cam·cos(θ) = -y_w * refraction   (note the sign convention)
        → z_cam·cos(θ) - y_cam·sin(θ) = y_w * refraction
    And:
        d_v = |z_cam·cos(θ) - y_cam·sin(θ)| / refraction

    We pick y_cam = 0 for simplicity; then:
        z_cam·cos(θ) = y_w * refraction  → z_cam = y_w * refraction / cos(θ)
        d_v = |z_cam·cos(θ)| / refraction = |y_w|
    So to get a specific d_v independent of y_w, we pick z_cam directly:
        z_cam = d_v_target * refraction / cos(θ) ... but then y_w is also fixed.

    For our FSM-oriented tests we don't need perfect geometry; we just need the
    blended (x_w, y_w, d_v) values that feed the state logic. The simplest
    approach is to bypass compute_world/d_v entirely by patching Vision.process
    to return whatever x_w/y_w we want downstream — but DockingNode feeds
    target_tvec through compute_world itself.

    We provide a simple mapping that hits the important invariants:
        - x_w sign tracks x_cam sign
        - y_w sign tracks z_cam sign
        - d_v tracks z_cam magnitude
    """
    from updated_config import PITCH_ANGLE_RAD, UNDERWATER_REFRACTION
    cos_p = np.cos(PITCH_ANGLE_RAD)
    sin_p = np.sin(PITCH_ANGLE_RAD)
    # We want: d_v = |z_cam·cos - y_cam·sin|/refraction ≈ d_v_target  → pick y_cam=0
    # y_w_from_vision = (z_cam·cos - y_cam·sin)/refraction  → with y_cam=0: y_w = z_cam·cos/refraction
    # So z_cam = y_w_target * refraction / cos (this forces y_w and d_v to be related)
    # Choose z_cam based on d_v_target so d_v lines up, accept whatever y_w comes out.
    z_cam = d_v_target * UNDERWATER_REFRACTION / cos_p
    y_cam = 0.0
    x_cam = x_w_target * 1.2
    # Sign flip for y: if we want NEGATIVE y_w (marker below), we need negative z·cos
    # But z_cam must be positive (depth). So we can't directly produce negative y_w via
    # z_cam alone. Use y_cam to pull y_w negative:
    # y_w = (z·cos - y·sin)/refraction  → to hit y_w_target, solve for y_cam:
    # y_cam = (z_cam·cos - y_w_target·refraction) / sin
    y_cam = (z_cam * cos_p - y_w_target * UNDERWATER_REFRACTION) / sin_p
    return np.array([x_cam, y_cam, z_cam], dtype=np.float32), abs((z_cam * cos_p - y_cam * sin_p) / UNDERWATER_REFRACTION)


# ─────────────────────────────────────────────────────────────────
# 6. Tests
# ─────────────────────────────────────────────────────────────────
RESULTS = []

def record(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    tag = "PASS" if ok else "FAIL"
    print(f"[{tag}] {name} — {detail}")


def test_1_searching_no_markers():
    """No markers for 30 frames; must remain in SEARCHING and never jump state."""
    n = HarnessNode()
    for _ in range(30):
        n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)
    record(
        "T1 SEARCHING holds under no-detection",
        n.state == "SEARCHING" and len(n.transition_log) == 0,
        f"state={n.state}, transitions={n.transition_log}"
    )


def test_2_searching_to_homing_conf_gate():
    """
    Confidence-gated entry: a single-frame detection must NOT trigger HOMING
    (conf only rises to 0.15), but ≥4 consecutive detections must (conf>0.5).
    """
    n = HarnessNode()
    tvec, _ = world_to_tvec(x_w_target=0.2, y_w_target=0.5, d_v_target=3.5)

    # Single-frame flicker: 1 visible, then 2 not visible → conf = 0.15 then decay
    n._tick(visible=True, n_valid=1, target_tvec=tvec, target_yaw=0.0, d_v=3.5)
    after_1 = n.state
    record(
        "T2a Single-frame flicker does NOT exit SEARCHING",
        after_1 == "SEARCHING",
        f"state={after_1}, conf={n.vision_confidence:.2f}"
    )

    # Now drive sustained detections: conf should cross 0.5 after 4 frames
    frames_to_lock = 0
    for i in range(10):
        n._tick(visible=True, n_valid=1, target_tvec=tvec, target_yaw=0.0, d_v=3.5)
        if n.state == "HOMING":
            frames_to_lock = i + 1
            break

    record(
        "T2b SEARCHING→HOMING after sustained detection",
        n.state == "HOMING" and 3 <= frames_to_lock <= 5,
        f"state={n.state}, frames_to_lock={frames_to_lock}, conf={n.vision_confidence:.2f}"
    )


def test_3_homing_accumulates_confirmation():
    """
    In HOMING with 4 markers visible for > CONFIDENCE_THRESHOLD frames,
    confirmed_4_markers must become True.
    """
    n = HarnessNode()
    tvec, _ = world_to_tvec(x_w_target=0.1, y_w_target=0.4, d_v_target=2.5)

    # Enter HOMING
    for _ in range(5):
        n._tick(visible=True, n_valid=4, target_tvec=tvec, target_yaw=0.0, d_v=2.5)

    # Stay in HOMING with 4 markers for CONFIDENCE_THRESHOLD frames
    for _ in range(CONFIDENCE_THRESHOLD + 2):
        n._tick(visible=True, n_valid=4, target_tvec=tvec, target_yaw=0.0, d_v=2.5)

    record(
        "T3 HOMING accumulates 4-marker lock",
        n.confirmed_4_markers and n.state == "HOMING",
        f"state={n.state}, confirmed={n.confirmed_4_markers}, apr_conf={n.approach_confidence}"
    )


def test_4_homing_to_descending_commit_safe():
    """
    HOMING→DESCENDING only fires when:
      (a) confirmed_4_markers, AND
      (b) y_w < CAMERA_OFFSET_Y (vehicle over marker), AND
      (c) commit_safe = (conf > 0.6) OR blind_active.
    """
    n = HarnessNode()
    # y_cam chosen so y_w < CAMERA_OFFSET_Y (-0.35). From world_to_tvec
    # we control y_w_target directly.
    tvec, d_v_real = world_to_tvec(x_w_target=0.0, y_w_target=-0.5, d_v_target=1.0)

    # Build up lock
    for _ in range(CONFIDENCE_THRESHOLD + 10):
        n._tick(visible=True, n_valid=4, target_tvec=tvec, target_yaw=0.0, d_v=1.0)

    record(
        "T4 HOMING→DESCENDING on confirmed+y_w<OFFSET+conf>0.6",
        n.state == "DESCENDING" and n.confirmed_4_markers,
        f"state={n.state}, confirmed={n.confirmed_4_markers}, conf={n.vision_confidence:.2f}, y_w={n.last_y_w:.2f}"
    )


def test_5_critical_marker_lost_at_close_range():
    """
    CRITICAL scenario: Marker disappears at close range during descent.

    This test is split into three phases:
      (C1) short blind window (~1 s sim) — must stay in DESCENDING,
           and MUST NOT transition to SEARCHING at any point.
      (C2) advance time past BLIND_DESCENT_TIMEOUT — the blind-timeout
           fallback must fire DESCENDING→FINAL_ALIGN.
      (C3) advance time again — FINAL_ALIGN→DOCKED blind-timeout fallback
           must fire, completing the mission.

    Throughout, we record every state transition and assert SEARCHING is
    never among them.
    """
    n = HarnessNode()

    # Phase A: acquire lock at medium range
    tvec_far, _ = world_to_tvec(x_w_target=0.0, y_w_target=0.5, d_v_target=3.0)
    for _ in range(10):
        n._tick(visible=True, n_valid=4, target_tvec=tvec_far, target_yaw=0.0, d_v=3.0)

    # Phase B: close in so confirmed_4_markers accumulates and y_w < OFFSET
    tvec_close, _ = world_to_tvec(x_w_target=0.0, y_w_target=-0.5, d_v_target=1.0)
    for _ in range(CONFIDENCE_THRESHOLD + 15):
        n._tick(visible=True, n_valid=4, target_tvec=tvec_close, target_yaw=0.0, d_v=1.0)

    assert n.state in ("DESCENDING", "FINAL_ALIGN"), \
        f"Pre-condition failed: expected DESCENDING/FINAL_ALIGN, got {n.state}"

    all_transitions = []

    # C1: short blind period — ~25 frames (1 s sim, well under the 8 s blind timeout)
    for _ in range(25):
        before = n.state
        n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)
        if before != n.state:
            all_transitions.append((before, n.state, "C1"))

    state_after_c1 = n.state

    # C2: advance time past BLIND_DESCENT_TIMEOUT; the DESCENDING blind-timeout
    # fallback must trigger FINAL_ALIGN on the very next tick.
    n.last_seen_t = time.time() - (BLIND_DESCENT_TIMEOUT + 1.0)
    before = n.state
    n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)
    if before != n.state:
        all_transitions.append((before, n.state, "C2"))

    state_after_c2 = n.state

    # C3: still invisible, time still aged → FINAL_ALIGN blind-timeout fallback
    n.last_seen_t = time.time() - (BLIND_DESCENT_TIMEOUT + 1.0)
    before = n.state
    n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)
    if before != n.state:
        all_transitions.append((before, n.state, "C3"))

    returned_to_searching = any(t[1] == "SEARCHING" for t in all_transitions)

    record(
        "T5a CRITICAL: NEVER returns to SEARCHING during close-range loss",
        not returned_to_searching,
        f"transitions={all_transitions}"
    )
    record(
        "T5b C1 stable during short loss (<blind timeout)",
        state_after_c1 == "DESCENDING",
        f"state_after_c1={state_after_c1}"
    )
    record(
        "T5c C2 DESCENDING→FINAL_ALIGN via blind timeout fallback",
        state_after_c2 == "FINAL_ALIGN",
        f"state_after_c2={state_after_c2}"
    )
    record(
        "T5d C3 FINAL_ALIGN→DOCKED via blind timeout fallback",
        n.state == "DOCKED",
        f"final_state={n.state}"
    )


def test_6_descending_final_align_docked_hysteresis():
    """Normal descent with vision intact; hysteresis must fire (3 stable frames)."""
    n = HarnessNode()

    # Acquire + commit
    tvec_approach, _ = world_to_tvec(x_w_target=0.0, y_w_target=-0.5, d_v_target=1.0)
    for _ in range(CONFIDENCE_THRESHOLD + 10):
        n._tick(visible=True, n_valid=4, target_tvec=tvec_approach, target_yaw=0.0, d_v=1.0)

    assert n.state == "DESCENDING", f"Pre-condition failed: {n.state}"

    # Feed d_v just below FINAL_ALIGN threshold for several frames
    d_v_below_final = FINAL_ALIGN_DIST - 0.05
    tvec_mid, _ = world_to_tvec(x_w_target=0.0, y_w_target=-0.5, d_v_target=d_v_below_final)
    for _ in range(10):
        n._tick(visible=True, n_valid=4, target_tvec=tvec_mid, target_yaw=0.0, d_v=d_v_below_final)

    at_final_align = n.state == "FINAL_ALIGN"

    # Feed d_v just below DOCKED threshold
    d_v_below_dock = DIST_DOCKED - 0.03
    tvec_dock, _ = world_to_tvec(x_w_target=0.0, y_w_target=-0.5, d_v_target=d_v_below_dock)
    for _ in range(10):
        n._tick(visible=True, n_valid=4, target_tvec=tvec_dock, target_yaw=0.0, d_v=d_v_below_dock)

    record(
        "T6a DESCENDING→FINAL_ALIGN via hysteresis",
        at_final_align,
        f"state_after_desc={n.state}"
    )
    record(
        "T6b FINAL_ALIGN→DOCKED via hysteresis",
        n.state == "DOCKED",
        f"final_state={n.state}"
    )


def test_7_homing_to_holding_to_homing():
    """Temporary marker drop in HOMING: conf<0.3 → HOLDING, then reacquire → HOMING."""
    n = HarnessNode()
    tvec, _ = world_to_tvec(x_w_target=0.1, y_w_target=0.5, d_v_target=2.5)  # mid-range
    # d_v=2.5 > BLIND_DESCENT_DIST (1.5): outside close-range window
    # Build up HOMING (but NOT reach DESCENDING: keep y_w > OFFSET_Y)

    for _ in range(20):
        n._tick(visible=True, n_valid=3, target_tvec=tvec, target_yaw=0.0, d_v=2.5)

    assert n.state == "HOMING", f"Pre-condition failed: {n.state}"

    # Drop markers for ~12 frames → conf decays 1.0 → 0.04
    saw_holding = False
    for _ in range(12):
        n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)
        if n.state == "HOLDING":
            saw_holding = True

    # Reacquire: sustained detection → conf rises > 0.6 → HOLDING→HOMING
    returned_home = False
    for _ in range(10):
        n._tick(visible=True, n_valid=3, target_tvec=tvec, target_yaw=0.0, d_v=2.5)
        if n.state == "HOMING":
            returned_home = True
            break

    record(
        "T7a HOMING→HOLDING on confidence drop (mid-range)",
        saw_holding,
        f"transitions={n.transition_log}"
    )
    record(
        "T7b HOLDING→HOMING on reacquire",
        returned_home,
        f"state={n.state}"
    )


def test_7c_holding_to_searching_timeout():
    """HOLDING with prolonged loss AND not blind_active → SEARCHING after 3 s."""
    n = HarnessNode()
    # Need last_d_h > BLIND_DESCENT_DIST (1.5 m). d_h = sqrt(x_w² + y_w²).
    # Pick y_w = 2.0 so d_h ≈ 2.0 > 1.5.
    tvec, _ = world_to_tvec(x_w_target=0.2, y_w_target=2.0, d_v_target=3.0)
    for _ in range(20):
        n._tick(visible=True, n_valid=3, target_tvec=tvec, target_yaw=0.0, d_v=3.0)
    assert n.state == "HOMING", f"Pre-condition failed: {n.state}"
    assert n.last_d_h > BLIND_DESCENT_DIST, f"Expected mid-range, d_h={n.last_d_h}"

    # Drop markers → go to HOLDING, then wait for timeout
    for _ in range(12):
        n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)
    assert n.state == "HOLDING", f"Expected HOLDING, got {n.state}"

    # The HOLDING timeout is 3 s wall-clock. Artificially age hold_start_t.
    n.hold_start_t = time.time() - 3.5
    # Age last_seen_t so blind_active is False (already true for mid-range d_h)
    n.last_seen_t = time.time() - (BLIND_DESCENT_TIMEOUT + 1.0)

    n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)

    record(
        "T7c HOLDING→SEARCHING after 3 s timeout (mid-range, no blind)",
        n.state == "SEARCHING",
        f"state={n.state}"
    )


def test_8_holding_to_descending_close_range_commit():
    """
    HOLDING entered at close range with confirmed + aligned → commit to
    DESCENDING instead of retreating.
    """
    n = HarnessNode()

    # Build confirmed_4_markers + close pose with y_w < OFFSET, but keep d_h
    # under BLIND_DESCENT_DIST (1.5 m) so blind_active can be true after we
    # drop markers.
    tvec_close, _ = world_to_tvec(x_w_target=0.0, y_w_target=-0.4, d_v_target=0.8)
    for _ in range(CONFIDENCE_THRESHOLD + 5):
        n._tick(visible=True, n_valid=4, target_tvec=tvec_close, target_yaw=0.0, d_v=0.8)
    # If we already transitioned to DESCENDING, this test is already proven;
    # but we want to verify the HOLDING→DESCENDING path specifically. Force
    # HOMING back to check the HOLDING branch:
    # The natural flow already commits to DESCENDING at close range from HOMING,
    # so to test the HOLDING→DESCENDING edge we construct it directly.
    n.state = "HOMING"
    n.confirmed_4_markers = True
    n.last_d_h = 1.0     # close
    n.last_y_w = -0.5    # aligned
    n.last_seen_t = time.time()
    n.vision_confidence = 0.4  # above HOMING→HOLDING threshold, but below commit_safe

    # Now force conf below 0.3 by dropping markers. Meanwhile close range so
    # blind_active stays true.
    n.vision_confidence = 0.29  # just below HOMING→HOLDING threshold
    n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)
    went_holding = n.state == "HOLDING"

    # Now in HOLDING at close range → HOLDING→DESCENDING should fire immediately
    n._tick(visible=False, n_valid=0, target_tvec=None, target_yaw=0.0, d_v=0.0)

    record(
        "T8 HOLDING→DESCENDING (close-range blind commit)",
        went_holding and n.state == "DESCENDING",
        f"went_holding={went_holding}, final={n.state}"
    )


# ─────────────────────────────────────────────────────────────────
# 7. Run
# ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 72)
    print("FSM INTEGRATION TESTS")
    print("=" * 72)
    try:
        test_1_searching_no_markers()
        test_2_searching_to_homing_conf_gate()
        test_3_homing_accumulates_confirmation()
        test_4_homing_to_descending_commit_safe()
        test_5_critical_marker_lost_at_close_range()
        test_6_descending_final_align_docked_hysteresis()
        test_7_homing_to_holding_to_homing()
        test_7c_holding_to_searching_timeout()
        test_8_holding_to_descending_close_range_commit()
    except AssertionError as e:
        print(f"[ABORT] Pre-condition assertion failed: {e}")

    print("=" * 72)
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    total = len(RESULTS)
    print(f"RESULT: {passed}/{total} tests PASSED")
    print("=" * 72)
    sys.exit(0 if passed == total else 1)
