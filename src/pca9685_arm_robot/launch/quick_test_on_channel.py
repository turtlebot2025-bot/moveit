import board
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # 50 Hz for standard servos

# Helper: angle (0-180) to 12-bit duty cycle
def angle_to_duty(angle):
    min_pulse = 500   # µs — calibrate for your specific servos
    max_pulse = 2500  # µs
    pulse_us = min_pulse + (max_pulse - min_pulse) * angle / 180.0
    return int(pulse_us / 20000.0 * 0xFFFF)

# Move channel 0 to 90 degrees
pca.channels[0].duty_cycle = angle_to_duty(90)