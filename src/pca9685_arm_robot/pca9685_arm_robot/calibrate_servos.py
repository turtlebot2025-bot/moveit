#!/usr/bin/env python3
"""
calibrate_servos.py
────────────────────────────────────────────────────────────────────────────
Interactive servo calibration tool for complete_assembly (5-DOF arm +
parallel gripper actuated by right_gear_joint on ch 5).

Run this BEFORE the ROS 2 bringup. For each actuated joint,
find the real min and max pulse widths for your specific servos, then paste
the printed values into urdf/complete_assembly.urdf (pca9685_i2c_node reads
these from _JOINT_CONFIG inside its source file).

Actuated joint → PCA9685 channel map:
  rotating_base_joint  → channel 2
  shoulder_joint       → channel 0
  elbow_joint          → channel 1
  wrist_joint          → channel 3
  gripper_joint        → channel 4  (gripper BODY rotation servo)
  right_gear_joint     → channel 5  (finger OPEN/CLOSE servo — PRIMARY actuator)

Mimic joints (right_link, left_link, left_gear, right_finger, left_finger)
have NO servo and are NOT calibrated here. They follow right_gear_joint.

Usage (on the Jetson, no ROS required):
  python3 calibrate_servos.py --channel 2    # rotating_base_joint (default)
  python3 calibrate_servos.py --channel 0    # shoulder_joint
  python3 calibrate_servos.py --channel 1    # elbow_joint
  python3 calibrate_servos.py --channel 3    # wrist_joint
  python3 calibrate_servos.py --channel 4    # gripper_joint (body rotation)
  python3 calibrate_servos.py --channel 5    # right_gear_joint (fingers)

  python3 calibrate_servos.py --bus 7 --addr 0x40 --channel 5

Controls (keyboard):
  Arrow UP    — increase pulse by current step
  Arrow DOWN  — decrease pulse by current step
  + or =      — double the step size  (max 200 µs)
  -           — halve the step size   (min 10 µs)
  h           — jump to 1500 µs (safe centre / home)
  n           — move to next channel in calibration order
  s           — save current value as min (< 1500) or max (>= 1500)
  p           — print calibration table so far
  q           — quit and print final URDF <param> snippets

Requirements:
  pip3 install smbus2 --break-system-packages
  Also ensure ina3221 is unbound from bus 7:
    echo '7-0040' | sudo tee /sys/bus/i2c/devices/7-0040/driver/unbind
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
    _TERM_OK = False

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

# ── Actuated joint → channel map ─────────────────────────────────────────────
_CHANNEL_TO_JOINT = {
    0: "shoulder_joint",
    1: "elbow_joint",
    2: "rotating_base_joint",
    3: "wrist_joint",
    4: "gripper_joint",
    5: "right_gear_joint",
}

# Physical calibration order (base first, fingers last)
_CALIBRATION_ORDER = [2, 0, 1, 3, 4, 5]


# ── PCA9685 helpers ───────────────────────────────────────────────────────────

def pca_init(bus, addr, freq, bus_num):
    """
    Reset and configure PCA9685 at the given PWM frequency.
    bus_num is passed explicitly (no global state).
    """
    prescale = int(round(_INTERNAL_OSC / (_PWM_STEPS * freq))) - 1
    prescale = max(3, min(255, prescale))

    # Software reset via General Call — clears stuck EXTCLK/SLEEP state
    try:
        bus.write_byte_data(0x00, 0x06, 0x00)
        time.sleep(0.01)
    except Exception:
        pass   # NACK on general call is normal on some hosts

    bus.write_byte_data(addr, _MODE1, _MODE1_AI)
    bus.write_byte_data(addr, _MODE2, 0x04)
    time.sleep(0.005)
    old = bus.read_byte_data(addr, _MODE1)
    bus.write_byte_data(addr, _MODE1, (old & 0x7F) | _MODE1_SLEEP)
    bus.write_byte_data(addr, _PRE_SCALE, prescale)
    bus.write_byte_data(addr, _MODE1, old)
    time.sleep(0.005)
    bus.write_byte_data(addr, _MODE1, old | _MODE1_RESTART | _MODE1_AI)
    time.sleep(0.01)
    print(f"PCA9685 ready  I2C bus {bus_num}  "
          f"addr 0x{addr:02X}  freq {freq} Hz  prescale {prescale}")


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
    """Read one keypress (or arrow escape sequence) without echo."""
    if not _TERM_OK:
        return input("cmd (u/d/+/-/h/s/p/n/q)? ").strip()[:1] or 'x'
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            seq = sys.stdin.read(2)
            return ch + seq
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Calibration session ───────────────────────────────────────────────────────

def calibrate(args):
    bus_obj  = smbus2.SMBus(args.bus)
    addr     = args.addr
    freq     = args.freq

    # Pass bus number explicitly — no global state
    pca_init(bus_obj, addr, freq, args.bus)

    channel  = args.channel
    pulse_us = 1500.0
    step_us  = 50.0
    saved    = {}   # {channel: {"min": float, "max": float}}

    def status_line():
        joint = _CHANNEL_TO_JOINT.get(channel, f"<unknown ch{channel}>")
        off   = set_pulse(bus_obj, addr, channel, pulse_us, freq)
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
            end="", flush=True,
        )

    print()
    print("  Controls:  ↑/↓ adjust  |  +/- step size  |  h centre  |"
          "  s save  |  p print  |  n next  |  q quit")
    print()
    print("  Actuated channels: 0(shoulder) 1(elbow) 2(base) 3(wrist)"
          " 4(gripper_body) 5(right_gear/fingers)")
    print("  Mimic joints have no servo — skip them.")
    print()
    status_line()

    while True:
        k = getch()

        if k in ('\x1b[A', 'u'):               # UP
            pulse_us = min(pulse_us + step_us, 3000.0)

        elif k in ('\x1b[B', 'd'):             # DOWN
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
            print(f"\n  Saved  ch {channel} "
                  f"({_CHANNEL_TO_JOINT.get(channel, '?')}) "
                  f"{label} = {pulse_us:.0f} µs")

        elif k == 'p':
            print()
            print_results(saved)

        elif k == 'n':
            try:
                idx = _CALIBRATION_ORDER.index(channel)
                channel = _CALIBRATION_ORDER[(idx + 1) % len(_CALIBRATION_ORDER)]
            except ValueError:
                channel = _CALIBRATION_ORDER[0]
            pulse_us = 1500.0
            print(f"\n  → ch {channel} ({_CHANNEL_TO_JOINT.get(channel, '?')})")

        elif k in ('q', '\x03', '\x04'):
            break

        else:
            continue

        status_line()

    print("\n")
    all_off(bus_obj, addr)
    bus_obj.close()
    print_results(saved)


def print_results(saved):
    print("─" * 70)
    print(f"  {'Ch':<4} {'Joint':<26} {'min_pulse_us':>13} {'max_pulse_us':>13}")
    print("─" * 70)
    for ch in _CALIBRATION_ORDER:
        name = _CHANNEL_TO_JOINT.get(ch, f"ch{ch}")
        lo   = saved.get(ch, {}).get("min", "NOT SET")
        hi   = saved.get(ch, {}).get("max", "NOT SET")
        flag = "  ← PRIMARY finger actuator" if ch == 5 else ""
        print(f"  {ch:<4} {name:<26} {str(lo):>13} {str(hi):>13}{flag}")
    print("─" * 70)
    print()
    print("  Mimic joints (right_link, left_link, left_gear, right_finger,")
    print("  left_finger) have no servo — no calibration needed.\n")
    print("  Update _JOINT_CONFIG in pca9685_i2c_node.py with these values:\n")
    for ch in _CALIBRATION_ORDER:
        name = _CHANNEL_TO_JOINT.get(ch, f"ch{ch}")
        lo   = saved.get(ch, {}).get("min")
        hi   = saved.get(ch, {}).get("max")
        if lo is None and hi is None:
            continue
        note = "  # PRIMARY finger actuator" if ch == 5 else ""
        print(f'    "{name}": {{  # ch {ch}{note}')
        if lo is not None:
            print(f'        "min_pulse_us": {lo},')
        if hi is not None:
            print(f'        "max_pulse_us": {hi},')
        print(f'    }}')
        print()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Interactive servo calibration for complete_assembly PCA9685 arm.\n"
            "Calibrates actuated joints (ch 0–5) only.\n"
            "Mimic joints have no servo and are skipped."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--channel", "-c",
        type=int, default=2,
        choices=sorted(_CHANNEL_TO_JOINT.keys()),
        help=(
            "PCA9685 channel to start on:\n"
            "  0=shoulder  1=elbow  2=rotating_base(default)\n"
            "  3=wrist  4=gripper_body  5=right_gear(fingers)"
        ),
    )
    parser.add_argument(
        "--bus", "-b",
        type=int, default=7,
        help="I2C bus number. Jetson AGX Orin 40-pin header = 7. Default: 7",
    )
    parser.add_argument(
        "--addr", "-a",
        type=lambda x: int(x, 16), default=0x40,
        help="PCA9685 I2C address in hex. Default: 0x40",
    )
    parser.add_argument(
        "--freq", "-f",
        type=float, default=50.0,
        help="PWM frequency in Hz. Default: 50",
    )
    args = parser.parse_args()

    print()
    print("  complete_assembly servo calibration")
    print(f"  Starting on channel {args.channel} "
          f"({_CHANNEL_TO_JOINT.get(args.channel, '?')})")
    print(f"  I2C bus {args.bus}  addr 0x{args.addr:02X}  freq {args.freq} Hz")
    print()
    print("  If you get 'Device or resource busy', run:")
    print("    echo '7-0040' | sudo tee /sys/bus/i2c/devices/7-0040/driver/unbind")
    print()
    print("  SAFETY: servos will move immediately.")
    print("  Ensure the arm is clear of obstacles before proceeding.")
    input("  Press ENTER to start, or Ctrl-C to cancel... ")
    print()

    try:
        calibrate(args)
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    except Exception as e:
        print(f"\n  ERROR: {e}")
        import traceback; traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()