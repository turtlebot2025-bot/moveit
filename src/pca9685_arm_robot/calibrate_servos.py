#!/usr/bin/env python3
"""
calibrate_servos.py
────────────────────────────────────────────────────────────────────────────
Interactive servo calibration tool for arm_robot_pkg (5-DOF arm).

Run this BEFORE building the ROS 2 package. For each joint (channel 0-4),
find the real min and max pulse widths for your specific servos, then paste
the printed values into urdf/arm_robot_ros2_control_block.urdf.

Usage (on the Jetson, no ROS required):
  python3 calibrate_servos.py --channel 0    # base_joint
  python3 calibrate_servos.py --channel 1    # shoulder_joint
  python3 calibrate_servos.py --channel 2    # elbow_joint
  python3 calibrate_servos.py --channel 3    # wrist_joint
  python3 calibrate_servos.py --channel 4    # gripper_joint

  python3 calibrate_servos.py --channel 0 --bus 1 --addr 0x40 --freq 50

Controls (keyboard):
  Arrow UP    — increase pulse by current step
  Arrow DOWN  — decrease pulse by current step
  + or =      — double the step size  (max 200 µs)
  -           — halve the step size   (min 10 µs)
  h           — jump to 1500 µs (safe centre / home)
  n           — move to next channel  (saves nothing, just switches)
  s           — save current value as min (< 1500) or max (>= 1500)
  p           — print calibration table so far
  q           — quit and print final URDF <param> snippets

Requirements:
  pip3 install smbus2 --break-system-packages

Joint → channel map (matches arm_robot_ros2_control_block.urdf):
  base_joint     → channel 0
  shoulder_joint → channel 1
  elbow_joint    → channel 2
  wrist_joint    → channel 3
  gripper_joint  → channel 4
"""

import argparse
import sys
import time

try:
    import smbus2
except ImportError:
    print("ERROR: smbus2 not found.")
    print("Fix:   pip3 install smbus2 --break-system-packages")
    sys.exit(1)

try:
    import tty
    import termios
    _TERM_OK = True
except ImportError:
    _TERM_OK = False   # Windows fallback — use typed input

# ── PCA9685 register constants ────────────────────────────────────────────────
_MODE1         = 0x00
_MODE2         = 0x01
_PRE_SCALE     = 0xFE
_LED0_ON_L     = 0x06
_ALL_LED_OFF_H = 0xFD
_MODE1_SLEEP   = 0x10
_MODE1_AI      = 0x20
_MODE1_RESTART = 0x80
_INTERNAL_OSC  = 25_000_000
_PWM_STEPS     = 4096

# ── Joint name lookup ──────────────────────────────────────────────────────────
_JOINT_NAMES = {
    2: "rotating_base_joint",
    0: "shoulder_link_joint",
    1: "Elbow_link_joint",
    3: "wrist_link_joint",
    4: "gripper_joint",
    5: "right_gear_joint"
}


# ── PCA9685 helpers ───────────────────────────────────────────────────────────

def pca_init(bus, addr, freq):
    """Reset and configure PCA9685 at the given PWM frequency."""
    prescale = int(round(_INTERNAL_OSC / (_PWM_STEPS * freq))) - 1
    prescale = max(3, min(255, prescale))
    bus.write_byte_data(addr, _MODE1, _MODE1_AI)
    time.sleep(0.005)
    old = bus.read_byte_data(addr, _MODE1)
    bus.write_byte_data(addr, _MODE1, (old & 0x7F) | _MODE1_SLEEP)
    bus.write_byte_data(addr, _PRE_SCALE, prescale)
    bus.write_byte_data(addr, _MODE1, old)
    time.sleep(0.005)
    bus.write_byte_data(addr, _MODE1, old | _MODE1_RESTART | _MODE1_AI)
    print(f"PCA9685 ready  I2C bus {bus._fd if hasattr(bus,'_fd') else '?'}"
          f"  addr 0x{addr:02X}  freq {freq} Hz  prescale {prescale}")


def set_pulse(bus, addr, channel, pulse_us, freq):
    """Write a pulse width (µs) to one PCA9685 channel."""
    period_us = 1_000_000.0 / freq
    off = int(round(pulse_us / period_us * _PWM_STEPS))
    off = max(0, min(_PWM_STEPS - 1, off))
    base = _LED0_ON_L + 4 * channel
    bus.write_i2c_block_data(addr, base, [
        0x00, 0x00,
        off & 0xFF, (off >> 8) & 0x0F,
    ])
    return off


def all_off(bus, addr):
    bus.write_byte_data(addr, _ALL_LED_OFF_H, 0x10)


# ── Keyboard input ────────────────────────────────────────────────────────────

def getch():
    """Read one keypress (or arrow-key escape sequence) without echo."""
    if not _TERM_OK:
        # Fallback: typed commands
        return input("cmd (u/d/+/-/h/s/p/q)? ").strip()[:1] or 'x'
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            seq = sys.stdin.read(2)
            return ch + seq      # e.g. '\x1b[A' for UP
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Calibration session ───────────────────────────────────────────────────────

def calibrate(args):
    bus  = smbus2.SMBus(args.bus)
    addr = args.addr
    freq = args.freq

    pca_init(bus, addr, freq)

    channel  = args.channel
    pulse_us = 1500.0
    step_us  = 50.0

    # saved[channel] = {"min": float, "max": float}
    saved = {}

    def status_line():
        joint = _JOINT_NAMES.get(channel, f"ch{channel}")
        off   = set_pulse(bus, addr, channel, pulse_us, freq)
        saved_str = ""
        if channel in saved:
            lo = saved[channel].get("min", "?")
            hi = saved[channel].get("max", "?")
            saved_str = f"  saved=[{lo}, {hi}]"
        print(
            f"\r  ch {channel} ({joint})  "
            f"pulse={pulse_us:6.0f} µs  "
            f"OFF={off:4d}  "
            f"step={step_us:.0f} µs"
            f"{saved_str}      ",
            end="", flush=True
        )

    print()
    print("  Controls:  UP/DOWN adjust  |  +/- change step  |"
          "  h centre  |  s save  |  p print  |  n next ch  |  q quit")
    print()
    status_line()

    while True:
        k = getch()

        if k == '\x1b[A' or k == 'u':        # UP / u
            pulse_us = min(pulse_us + step_us, 3000.0)

        elif k == '\x1b[B' or k == 'd':       # DOWN / d
            pulse_us = max(pulse_us - step_us, 100.0)

        elif k in ('+', '='):
            step_us = min(step_us * 2, 200.0)

        elif k == '-':
            step_us = max(step_us / 2, 10.0)

        elif k == 'h':
            pulse_us = 1500.0

        elif k == 's':
            label = "min" if pulse_us < 1500.0 else "max"
            if channel not in saved:
                saved[channel] = {}
            saved[channel][label] = round(pulse_us)
            print(f"\n  Saved  ch {channel} ({_JOINT_NAMES.get(channel,'?')}) "
                  f"{label} = {pulse_us:.0f} µs")

        elif k == 'p':
            print()
            print_results(saved)

        elif k == 'n':
            channel = (channel + 1) % 5
            pulse_us = 1500.0
            print(f"\n  Switched to channel {channel} "
                  f"({_JOINT_NAMES.get(channel,'?')})")

        elif k in ('q', '\x03', '\x04'):      # q / Ctrl-C / Ctrl-D
            break

        else:
            continue

        status_line()

    print("\n")
    all_off(bus, addr)
    bus.close()
    print_results(saved)


def print_results(saved):
    """Print the calibration table and copy-paste URDF snippets."""
    joints_order = [0, 1, 2, 3, 4,5]

    print("─" * 62)
    print(f"  {'Channel':<8} {'Joint':<18} {'min_pulse_us':>13} {'max_pulse_us':>13}")
    print("─" * 62)
    for ch in joints_order:
        name = _JOINT_NAMES.get(ch, f"ch{ch}")
        lo   = saved.get(ch, {}).get("min", "NOT SET")
        hi   = saved.get(ch, {}).get("max", "NOT SET")
        print(f"  {ch:<8} {name:<18} {str(lo):>13} {str(hi):>13}")
    print("─" * 62)

    print()
    print("Copy these <param> blocks into urdf/arm_robot_ros2_control_block.urdf:\n")
    for ch in joints_order:
        name = _JOINT_NAMES.get(ch, f"ch{ch}")
        lo   = saved.get(ch, {}).get("min")
        hi   = saved.get(ch, {}).get("max")
        if lo is None and hi is None:
            continue
        print(f"  <!-- {name} (channel {ch}) -->")
        if lo is not None:
            print(f"  <param name=\"min_pulse_us\">{lo}</param>")
        if hi is not None:
            print(f"  <param name=\"max_pulse_us\">{hi}</param>")
        print()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Interactive servo calibration for arm_robot_pkg PCA9685 arm."
    )
    parser.add_argument(
        "--channel", "-c",
        type=int, default=0, choices=[0, 1, 2, 3, 4,5],
        help="PCA9685 channel to start on (0=base … 4=gripper). Default: 0"
    )
    parser.add_argument(
        "--bus", "-b",
        type=int, default=1,
        help="I2C bus number. Jetson AGX Orin = 1. Default: 1"
    )
    parser.add_argument(
        "--addr", "-a",
        type=lambda x: int(x, 16), default=0x40,
        help="PCA9685 I2C address in hex. Default: 0x40"
    )
    parser.add_argument(
        "--freq", "-f",
        type=float, default=50.0,
        help="PWM frequency in Hz. Default: 50"
    )
    args = parser.parse_args()

    print()
    print("  arm_robot_pkg servo calibration")
    print(f"  Starting on channel {args.channel} "
          f"({_JOINT_NAMES.get(args.channel, '?')})")
    print(f"  I2C bus {args.bus}  addr 0x{args.addr:02X}  freq {args.freq} Hz")
    print()
    print("  SAFETY: servos will move immediately.")
    print("  Make sure the arm is clear of obstacles before proceeding.")
    input("  Press ENTER to start, or Ctrl-C to cancel... ")
    print()

    try:
        calibrate(args)
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    except Exception as e:
        print(f"\n  ERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()