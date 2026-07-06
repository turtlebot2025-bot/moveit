#!/usr/bin/env python3
"""
test_right_gear.py
────────────────────────────────────────────────────────────────────────────
Targeted diagnostic for right_gear_joint (finger open/close servo).
No ROS needed. Talks directly to PCA9685 over I2C.

Tests channels 4, 5, AND nearby channels (3, 6) in case the servo
is wired to a different channel than expected.

Run:
  python3 test_right_gear.py

Safety: fingers will move. Keep hands clear of gripper.
"""

import time
import sys

try:
    import smbus2
except ImportError:
    print("ERROR: pip3 install smbus2 --break-system-packages")
    sys.exit(1)

I2C_BUS  = 7
I2C_ADDR = 0x40
FREQ     = 50.0

_MODE1    = 0x00
_MODE2    = 0x01
_PRE_SCALE= 0xFE
_LED0_ON_L= 0x06
_ALL_OFF_H= 0xFD
_SLEEP    = 0x10
_AI       = 0x20
_RESTART  = 0x80


def init(bus):
    try:
        bus.write_byte_data(0x00, 0x06, 0x00)
        time.sleep(0.02)
    except Exception:
        pass
    pre = int(round(25_000_000 / (4096 * FREQ))) - 1
    bus.write_byte_data(I2C_ADDR, _MODE1, _AI)
    bus.write_byte_data(I2C_ADDR, _MODE2, 0x04)
    time.sleep(0.005)
    old = bus.read_byte_data(I2C_ADDR, _MODE1)
    bus.write_byte_data(I2C_ADDR, _MODE1, (old & 0x7F) | _SLEEP)
    bus.write_byte_data(I2C_ADDR, _PRE_SCALE, pre)
    bus.write_byte_data(I2C_ADDR, _MODE1, old)
    time.sleep(0.005)
    bus.write_byte_data(I2C_ADDR, _MODE1, old | _RESTART | _AI)
    time.sleep(0.01)
    actual = 25_000_000 / (4096 * (pre + 1))
    print(f"  PCA9685 ready  prescale={pre}  freq={actual:.2f} Hz")
    print(f"  MODE1 = 0x{bus.read_byte_data(I2C_ADDR, _MODE1):02X}")


def pulse(bus, ch, us):
    off = int(round(us / 20000.0 * 4096))
    off = max(0, min(4095, off))
    base = _LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(I2C_ADDR, base,
        [0x00, 0x00, off & 0xFF, (off >> 8) & 0x0F])
    return off


def stop_all(bus):
    bus.write_byte_data(I2C_ADDR, _ALL_OFF_H, 0x10)


def read_channel(bus, ch):
    """Read back ON_L ON_H OFF_L OFF_H for a channel."""
    base = _LED0_ON_L + 4 * ch
    regs = []
    for r in range(4):
        regs.append(bus.read_byte_data(I2C_ADDR, base + r))
    off_count = regs[2] | ((regs[3] & 0x0F) << 8)
    on_count  = regs[0] | ((regs[1] & 0x0F) << 8)
    return on_count, off_count


# ── Test 1: Read all channel register states ──────────────────────────────────

def test_read_registers(bus):
    print("\n── Test 1: PCA9685 register state for channels 3-7 ─────────")
    print("  This shows what pulse is currently programmed per channel.")
    print("  A channel set to 0 µs or 20000 µs means it is OFF or fully ON.")
    print()
    print(f"  {'Ch':<4} {'ON count':<12} {'OFF count':<12} {'Pulse µs':>10}")
    print("  " + "─" * 42)
    for ch in range(3, 8):
        on, off = read_channel(bus, ch)
        pulse_us = off / 4096.0 * 20000.0
        flag = ""
        if off == 0:   flag = "  ← ZERO (servo off)"
        if off > 4090: flag = "  ← FULL ON (bad)"
        print(f"  {ch:<4} {on:<12} {off:<12} {pulse_us:>10.0f} µs{flag}")


# ── Test 2: Channel scan — pulse each channel 4, 5, 3, 6 ─────────────────────

def test_channel_scan(bus):
    print("\n── Test 2: Channel scan — which channel moves the finger? ──")
    print("  Sends 620 µs (open) then 2760 µs (close) to each channel.")
    print("  WATCH THE GRIPPER and note which channel number makes it move.")
    print()

    for ch in [4, 5, 3, 6, 7]:
        print(f"\n  Testing channel {ch}...")
        print(f"    Sending OPEN pulse (620 µs)...")
        pulse(bus, ch, 620)
        time.sleep(1.5)

        print(f"    Sending CLOSE pulse (2760 µs)...")
        pulse(bus, ch, 2760)
        time.sleep(1.5)

        moved = input(f"    Did the finger move on channel {ch}? (y/n): ")
        pulse(bus, ch, 1500)   # return to neutral
        time.sleep(0.5)

        if moved.strip().lower() == 'y':
            print(f"\n  ✓ RIGHT GEAR JOINT IS ON CHANNEL {ch}")
            print(f"    Update _JOINT_CONFIG: 'right_gear_joint' channel → {ch}")
            return ch

    print("\n  No channel moved the finger.")
    print("  Check physical wiring — servo signal wire may be disconnected.")
    return None


# ── Test 3: Pulse range test on confirmed channel ─────────────────────────────

def test_pulse_range(bus, ch):
    print(f"\n── Test 3: Pulse range test on channel {ch} ─────────────────")
    print("  MG90S typical range: 500–2400 µs")
    print("  Testing key positions...")
    print()

    positions = [
        (500,  "minimum (full open attempt)"),
        (1000, "25% closed"),
        (1500, "centre / neutral"),
        (2000, "75% closed"),
        (2400, "maximum (full close attempt)"),
        (2760, "extended max (current config)"),
    ]

    for us, label in positions:
        off = pulse(bus, ch, us)
        print(f"  {us:4d} µs  OFF={off:3d}  → {label}")
        time.sleep(2.0)

    pulse(bus, ch, 1500)
    time.sleep(0.5)
    print("\n  Record where finger is fully open and fully closed.")
    print("  Use those values as min_pulse_us and max_pulse_us.")


# ── Test 4: Verify /joint_states is reaching the servo ───────────────────────

def test_direct_vs_ros():
    print("\n── Test 4: ROS topic verification (run in another terminal) ─")
    print()
    print("  While robot bringup is running, check if joint_state_broadcaster")
    print("  is publishing right_gear_joint:")
    print()
    print("    ros2 topic echo /joint_states | grep right_gear")
    print()
    print("  Expected output (position changes when you send a goal):")
    print("    - name: ['right_gear_joint']")
    print("      position: [0.5]   ← changes with each trajectory goal")
    print()
    print("  If right_gear_joint appears but position never changes:")
    print("    → gripper_controller is not sending goals to the joint")
    print("    → check controllers.yaml: right_gear_joint must be listed")
    print("       under gripper_controller joints")
    print()
    print("  If right_gear_joint does NOT appear in /joint_states:")
    print("    → the URDF ros2_control block is missing the joint declaration")
    print("    → check arm_robot_ros2_control_block.urdf has <joint name='right_gear_joint'>")
    print()
    print("  Verify gripper_controller is active:")
    print("    ros2 control list_controllers")
    print("    Expected: gripper_controller  active")
    print()
    print("  Send a test goal directly to gripper_controller:")
    print("    ros2 topic pub --once /gripper_controller/joint_trajectory \\")
    print("      trajectory_msgs/msg/JointTrajectory '{")
    print("        joint_names: [right_gear_joint],")
    print("        points: [{positions: [1.2], velocities: [0.0],")
    print("                  time_from_start: {sec: 1}}]}'")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print()
    print("═" * 58)
    print("  right_gear_joint diagnostic")
    print(f"  I2C bus {I2C_BUS}  addr 0x{I2C_ADDR:02X}")
    print("═" * 58)
    print()
    print("  Servo should be connected to PCA9685 channel 5 (right_gear_joint).")
    print("  This script will confirm the correct channel.")
    print()
    input("  Press ENTER to start (or Ctrl-C to cancel)... ")

    bus = smbus2.SMBus(I2C_BUS)
    try:
        val = bus.read_byte_data(I2C_ADDR, _MODE1)
        print(f"  PCA9685 found  MODE1=0x{val:02X}")
    except Exception as e:
        print(f"\n  ERROR: cannot reach PCA9685 — {e}")
        print("  Check I2C wiring and run: sudo i2cdetect -y -r 7")
        sys.exit(1)

    init(bus)

    try:
        # Test 1 — read register state
        test_read_registers(bus)

        # Test 2 — find which channel actually moves the finger
        confirmed_ch = test_channel_scan(bus)

        # Test 3 — pulse range on confirmed channel
        if confirmed_ch is not None:
            ans = input(f"\n  Run pulse range test on ch {confirmed_ch}? (y/n): ")
            if ans.strip().lower() == 'y':
                test_pulse_range(bus, confirmed_ch)

        # Test 4 — ROS verification instructions
        test_direct_vs_ros()

    except KeyboardInterrupt:
        print("\n  Interrupted")
    finally:
        stop_all(bus)
        bus.close()
        print("\n  Done. All PWM outputs stopped.")
        print()
        print("  NEXT STEPS based on results:")
        print("  ┌─────────────────────────────────────────────────────┐")
        print("  │ Finger moved on ch 5 → channel is correct.          │")
        print("  │   Issue is in ROS layer — check gripper_controller  │")
        print("  │   and deadband filter in pca9685_i2c_node.py        │")
        print("  │                                                      │")
        print("  │ Finger moved on ch X (not 5) → wrong channel.       │")
        print("  │   Update right_gear_joint channel to X in           │")
        print("  │   pca9685_i2c_node.py _JOINT_CONFIG                 │")
        print("  │                                                      │")
        print("  │ No channel moved finger → wiring problem.            │")
        print("  │   Check signal wire from PCA9685 to servo.          │")
        print("  │   Check servo power (6V, not 5V) and GND.           │")
        print("  │   Try swapping the servo with a working one.         │")
        print("  └─────────────────────────────────────────────────────┘")


if __name__ == "__main__":
    main()