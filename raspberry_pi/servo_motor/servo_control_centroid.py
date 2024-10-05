# Raspberry Pi : Control Servo Motor by Centroid Coordinates
# cx : PWM0, cy : PWM1
import RPi.GPIO as GPIO
import time

# GPIO pin numbers where the servo motors are connected
SERVO_PIN_0 = 12  # PWM0 - GPIO18 (pin number 12)
SERVO_PIN_1 = 33  # PWM1 - GPIO13 (pin number 33)

# Servo motor duty cycle constants
SERVO_MAX_DUTY = 12  # Duty cycle corresponding to 180 degrees
SERVO_MIN_DUTY = 3  # Duty cycle corresponding to 0 degrees

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN_0, GPIO.OUT)
GPIO.setup(SERVO_PIN_1, GPIO.OUT)

# Create PWM instances (frequency 50Hz)
servo0 = GPIO.PWM(SERVO_PIN_0, 50)
servo1 = GPIO.PWM(SERVO_PIN_1, 50)
servo0.start(0)  # Start PWM signal
servo1.start(0)


def setServoPos(servo, degree):
    """
    Moves the servo motor to the specified angle.

    Parameters:
    servo (PWM): The PWM instance of the servo motor to control
    degree (float): The target angle between 0 and 180 degrees
    """
    # Limit the angle between 0 and 180 degrees
    if degree < 0:
        degree = 0
    elif degree > 180:
        degree = 180

    # Calculate the duty cycle
    duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)

    # Print for debugging
    print(f"Set {servo} Degree: {degree} to {duty} (Duty Cycle)")

    # Move the servo motor
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Wait for the servo motor to reach the position
    servo.ChangeDutyCycle(0)  # Stop PWM signal to prevent jitter


try:
    while True:
        # Receive centroid coordinates from the host PC
        # In an actual implementation, data would be received via socket or serial communication.
        # Here we take input for demonstration purposes.
        centroid_coordinate = input("Enter centroid : ")
        if centroid_coordinate.lower() == 'exit':
            break
        try:
            cx = int(centroid_coordinate.split(',')[0])
            cy = int(centroid_coordinate.split(',')[1])
        except (ValueError, KeyError, IndexError) as e:
            print("Invalid input. Please enter data in the correct format.")
            continue

        # Calculate angle for servo 0 (X-axis)
        servo0_angle = (-60 / 320) * (cx - 320) + 90  # Range between 30 and 150 degrees
        # Calculate angle for servo 1 (Y-axis)
        servo1_angle = (-60 / 240) * (cy - 240) + 90  # Range between 30 and 150 degrees

        # Limit the angles between 0 and 180 degrees
        servo0_angle = max(0, min(180, servo0_angle))
        servo1_angle = max(0, min(180, servo1_angle))

        # Set the servo motor positions
        setServoPos(servo0, servo0_angle)
        setServoPos(servo1, servo1_angle)

        # Print the current angle
        print(f"Servo0 (PWM0) is now at {servo0_angle} degrees.")
        print(f"Servo1 (PWM1) is now at {servo1_angle} degrees.")

except KeyboardInterrupt:
    pass
finally:
    # Clean up GPIO settings
    servo0.stop()
    servo1.stop()
    GPIO.cleanup()
