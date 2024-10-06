# Raspberry Pi : Control Servo Motor by Centroid Coordinates
# cx : PWM0 (horizontal movement), cy : PWM1 (vertical movement)
import RPi.GPIO as GPIO
import time

# GPIO pin numbers where the servo motors are connected
SERVO_PIN_0 = 12  # PWM0 - GPIO18 (pin number 12)
SERVO_PIN_1 = 33  # PWM1 - GPIO13 (pin number 33)

# Servo motor duty cycle constants
SERVO_MAX_DUTY = 12  # Duty cycle corresponding to 180 degrees
SERVO_MIN_DUTY = 3   # Duty cycle corresponding to 0 degrees

# Camera and frame settings
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
HFOV = 108.92  # Calculated horizontal field of view in degrees
VFOV = 92.0    # Calculated vertical field of view in degrees

FRAME_CENTER_X = FRAME_WIDTH / 2
FRAME_CENTER_Y = FRAME_HEIGHT / 2

# Initialize current servo angles
current_servo0_angle = 90.0
current_servo1_angle = 90.0

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN_0, GPIO.OUT)
GPIO.setup(SERVO_PIN_1, GPIO.OUT)

# Create PWM instances (frequency 50Hz)
servo0 = GPIO.PWM(SERVO_PIN_0, 50)
servo1 = GPIO.PWM(SERVO_PIN_1, 50)
servo0.start(0)  # Start PWM signal
servo1.start(0)

def setServoPos(target_servo0_angle, target_servo1_angle):
    global current_servo0_angle, current_servo1_angle

    # Limit the target angles between 0 and 180 degrees
    target_servo0_angle = max(0, min(180, target_servo0_angle))
    target_servo1_angle = max(0, min(180, target_servo1_angle))

    # Define the maximum change in angle per update to prevent sudden movements
    max_delta = 5.0  # degrees per update

    # Calculate the angle differences
    delta0 = target_servo0_angle - current_servo0_angle
    delta1 = target_servo1_angle - current_servo1_angle

    # Limit the angle changes to max_delta
    if abs(delta0) > max_delta:
        delta0 = max_delta if delta0 > 0 else -max_delta
    if abs(delta1) > max_delta:
        delta1 = max_delta if delta1 > 0 else -max_delta

    # Update current angles
    current_servo0_angle += delta0
    current_servo1_angle += delta1

    # Calculate the duty cycles
    duty0 = SERVO_MIN_DUTY + (current_servo0_angle * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)
    duty1 = SERVO_MIN_DUTY + (current_servo1_angle * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)

    time.sleep(0.5)
    # Move the servo motors
    servo0.ChangeDutyCycle(duty0)
    servo1.ChangeDutyCycle(duty1)

try:
    while True:
        # Receive centroid coordinates from the host PC
        # In an actual implementation, data would be received via socket or serial communication.
        # Here we take input for demonstration purposes.
        centroid_coordinate = input("Enter centroid (cx,cy): ")
        if centroid_coordinate.lower() == 'exit':
            break
        try:
            # Parse the input string into cx and cy
            cx = int(centroid_coordinate.split(',')[0])
            cy = int(centroid_coordinate.split(',')[1])
        except (ValueError, IndexError) as e:
            print("Invalid input. Please enter data in the correct format (e.g., 320,240).")
            continue

        # Calculate target angles for servos
        target_servo0_angle = ((cx - FRAME_CENTER_X) / FRAME_WIDTH) * HFOV + 90
        target_servo1_angle = ((cy - FRAME_CENTER_Y) / FRAME_HEIGHT) * VFOV + 90

        # Set the servo motor positions
        setServoPos(target_servo0_angle, target_servo1_angle)

        # Print the current angles
        print(f"Servo0 (PWM0) is now at {current_servo0_angle:.2f} degrees.")
        print(f"Servo1 (PWM1) is now at {current_servo1_angle:.2f} degrees.")

except KeyboardInterrupt:
    pass
finally:
    # Clean up GPIO settings
    servo0.stop()
    servo1.stop()
    GPIO.cleanup()