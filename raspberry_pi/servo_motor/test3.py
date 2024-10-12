# test3.py
# 세 개의 서보모터를 제어합니다:
# - 서보모터 1: RPi.GPIO를 통해 GPIO18에 연결
# - 서보모터 2와 3: Adafruit ServoKit의 채널 0과 4에 연결
# 모든 서보모터는 0도와 90도를 번갈아가며 이동
import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit

# --- GPIO Mode Configuration ---
DESIRED_MODE = GPIO.BCM  # Set to BCM to match current mode

# Check the current GPIO mode
current_mode = GPIO.getmode()

if current_mode is None:
    GPIO.setmode(DESIRED_MODE)
    print(f"GPIO mode set to {DESIRED_MODE} (BCM).")
elif current_mode != DESIRED_MODE:
    raise ValueError(f"GPIO mode already set to {current_mode}. Cannot set to {DESIRED_MODE}.")
else:
    print(f"GPIO mode is already set to {current_mode} (BCM).")

# --- Servo Motor 1 Configuration (GPIO18) ---
SERVO_PIN = 18  # BCM pin number 18 (BOARD pin number 12)
SERVO_MIN_DUTY = 2  # Duty cycle for 0 degrees
SERVO_MAX_DUTY = 11  # Duty cycle for 90 degrees

# Setup GPIO for Servo Motor 1
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialize PWM for Servo Motor 1 at 50Hz
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)  # Start PWM with 0% duty cycle


def set_gpio_servo_angle(angle):
    """
    Sets the angle of Servo Motor 1 connected to GPIO18.

    Parameters:
    angle (int): Desired angle between 0 and 90 degrees.
    """
    # Constrain the angle to the 0-90 degree range
    angle = max(0, min(180, angle))

    # Calculate the corresponding duty cycle
    duty = SERVO_MIN_DUTY + (angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)

    # Update the duty cycle to move the servo
    servo_pwm.ChangeDutyCycle(duty)
    print(f"Servo Motor 1 set to {angle} degrees (Duty Cycle: {duty:.2f}%)")


# --- Servo Motors 2 & 3 Configuration (ServoKit) ---
kit = ServoKit(channels=16, address=0x40)  # Adjust 'address' if necessary


def set_serokit_servos(angle):
    """
    Sets the angles of Servo Motors 2 and 3 connected to channels 0 and 4.

    Parameters:
    angle (int): Desired angle between 0 and 90 degrees.
    """
    # Constrain the angle to the 0-90 degree range
    angle = max(0, min(180, angle))

    # Set the angle for Servo Motor 2 (channel 0)
    kit.servo[0].angle = angle
    print(f"Servo Motor 2 set to {angle} degrees")

    # Set the angle for Servo Motor 3 (channel 4)
    kit.servo[4].angle = angle
    print(f"Servo Motor 3 set to {angle} degrees")


try:
    while True:
        # Move all servos to 90 degrees
        print("\nSetting all servos to 90 degrees...")
        set_gpio_servo_angle(90)
        set_serokit_servos(90)
        time.sleep(1)  # Wait for 1 second

        # Move all servos to 0 degrees
        print("\nSetting all servos to 0 degrees...")
        set_gpio_servo_angle(0)
        set_serokit_servos(0)
        time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
    print("\nProgram interrupted by user. Exiting...")

finally:
    # Stop PWM and clean up GPIO settings for Servo Motor 1
    servo_pwm.stop()
    GPIO.cleanup()
    print("PWM stopped and GPIO cleanup completed.")