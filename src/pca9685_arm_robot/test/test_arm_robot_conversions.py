"""
test_arm_robot_conversions.py
─────────────────────────────
Unit tests for JointConfig angle→pulse conversion with arm_robot_pkg joints.

All joints use SYMMETRIC limits about 0 rad:
  0 rad  → 1500 µs (servo centre)
  +limit → max_pulse_us
  -limit → min_pulse_us

Run without ROS or hardware:
  pytest test/test_arm_robot_conversions.py -v
"""

import math
import sys
import os
import types

# ── Stub out ROS / smbus2 so the module loads without hardware ────────────────
for mod_name in ["smbus2"]:
    stub = types.ModuleType(mod_name)
    stub.SMBus = type("SMBus", (), {"__init__": lambda s, *a: None})
    sys.modules[mod_name] = stub

for mod_name in ["rclpy", "rclpy.logging", "hardware_interface",
                 "hardware_interface.hardware_interface"]:
    sys.modules[mod_name] = types.ModuleType(mod_name)

import rclpy.logging as _rl
_rl.get_logger = lambda n: type("L", (), {
    "info": lambda s, m: None,
    "error": lambda s, m: None,
    "warn": lambda s, m: None,
})()

import hardware_interface as _hi
_hi.SystemInterface = object
_hi.HardwareInfo    = object
_hi.return_type     = type("rt", (), {"OK": 0, "ERROR": 1})()

_hih = sys.modules["hardware_interface.hardware_interface"]
_hih.StateInterface   = lambda *a, **kw: None
_hih.CommandInterface = lambda *a, **kw: None

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pca9685_arm_robot.pca9685_arm_robot import JointConfig, _JOINT_DEFAULTS

# ── Helpers ───────────────────────────────────────────────────────────────────

def make_joint(name="base_joint", min_us=500, max_us=2500,
               limit_rad=1.046, home_deg=0.0, channel=0):
    params = {
        "channel":        str(channel),
        "min_pulse_us":   str(min_us),
        "max_pulse_us":   str(max_us),
        "urdf_limit_rad": str(limit_rad),
        "home_angle_deg": str(home_deg),
    }
    return JointConfig(name, params)


MID   = 1500.0
RANGE = 1000.0   # half of 2000 µs total span


# ── Symmetric mapping tests ────────────────────────────────────────────────────

def test_zero_radians_gives_mid_pulse():
    """0 rad (URDF neutral) → 1500 µs"""
    j = make_joint()
    assert j.radians_to_pulse_us(0.0) == MID

def test_positive_limit_gives_max_pulse():
    """+1.046 rad → 2500 µs"""
    j = make_joint()
    assert abs(j.radians_to_pulse_us(1.046) - 2500.0) < 0.5

def test_negative_limit_gives_min_pulse():
    """-1.046 rad → 500 µs"""
    j = make_joint()
    assert abs(j.radians_to_pulse_us(-1.046) - 500.0) < 0.5

def test_half_positive_limit():
    """+0.523 rad (half of ±1.046) → 2000 µs"""
    j = make_joint()
    result = j.radians_to_pulse_us(0.523)
    assert abs(result - 2000.0) < 2.0

def test_half_negative_limit():
    """-0.523 rad → 1000 µs"""
    j = make_joint()
    result = j.radians_to_pulse_us(-0.523)
    assert abs(result - 1000.0) < 2.0

# ── Clamping ──────────────────────────────────────────────────────────────────

def test_clamping_above_positive_limit():
    """Values > +limit clamp to max_pulse"""
    j = make_joint()
    assert j.radians_to_pulse_us(2.0) == j.radians_to_pulse_us(1.046)

def test_clamping_below_negative_limit():
    """Values < -limit clamp to min_pulse"""
    j = make_joint()
    assert j.radians_to_pulse_us(-2.0) == j.radians_to_pulse_us(-1.046)

# ── Wrist and gripper (±1.57 rad limit) ──────────────────────────────────────

def test_wrist_zero_is_mid():
    j = make_joint(name="wrist_joint", limit_rad=1.570)
    assert j.radians_to_pulse_us(0.0) == MID

def test_wrist_positive_limit():
    j = make_joint(name="wrist_joint", limit_rad=1.570)
    assert abs(j.radians_to_pulse_us(1.570) - 2500.0) < 0.5

def test_wrist_negative_limit():
    j = make_joint(name="wrist_joint", limit_rad=1.570)
    assert abs(j.radians_to_pulse_us(-1.570) - 500.0) < 0.5

def test_gripper_zero_is_mid():
    j = make_joint(name="gripper_joint", limit_rad=1.570)
    assert j.radians_to_pulse_us(0.0) == MID

# ── Home position ─────────────────────────────────────────────────────────────

def test_home_at_zero_deg_gives_mid_pulse():
    """All arm_robot_pkg joints home at 0° → 1500 µs"""
    j = make_joint(home_deg=0.0)
    assert j.home_pulse_us() == MID

def test_home_radians_is_zero():
    j = make_joint(home_deg=0.0)
    assert abs(j.home_radians) < 1e-9

# ── Default values from _JOINT_DEFAULTS ───────────────────────────────────────

def test_all_five_joints_have_defaults():
    expected = {"base_joint", "shoulder_joint", "elbow_joint",
                "wrist_joint", "gripper_joint"}
    assert set(_JOINT_DEFAULTS.keys()) == expected

def test_arm_joints_limit_is_1046():
    for name in ["base_joint", "shoulder_joint", "elbow_joint"]:
        assert _JOINT_DEFAULTS[name]["urdf_limit_rad"] == 1.046

def test_wrist_gripper_limit_is_1570():
    for name in ["wrist_joint", "gripper_joint"]:
        assert _JOINT_DEFAULTS[name]["urdf_limit_rad"] == 1.570

def test_channels_are_0_to_4():
    for expected_ch, name in enumerate(
        ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint", "gripper_joint"]
    ):
        assert _JOINT_DEFAULTS[name]["channel"] == expected_ch

# ── PCA9685 prescale formula ──────────────────────────────────────────────────

def test_prescale_50hz():
    assert round(25_000_000 / (4096 * 50)) - 1 == 121

# ── OFF_COUNT spot checks ─────────────────────────────────────────────────────

def test_off_count_mid_pulse():
    """1500 µs at 50 Hz → OFF = 307"""
    assert round(1500 / 20000 * 4096) == 307

def test_off_count_min_pulse():
    """500 µs at 50 Hz → OFF = 102"""
    assert round(500 / 20000 * 4096) == 102

def test_off_count_max_pulse():
    """2500 µs at 50 Hz → OFF = 512"""
    assert round(2500 / 20000 * 4096) == 512