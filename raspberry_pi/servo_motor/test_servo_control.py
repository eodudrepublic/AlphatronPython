# Raspberry Pi : Control Servo Motor by input angle
import RPi.GPIO as GPIO
import time

# GPIO pin number where the servo is connected
SERVO_PIN = 12  # Using Physical pin 12 (GPIO18 when using BCM numbering)

# Constants for servo duty cycle
SERVO_MAX_DUTY = 12  # Duty cycle for 180 degrees
SERVO_MIN_DUTY = 3  # Duty cycle for 0 degrees

# Set GPIO mode to BOARD numbering
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Create PWM instance with 50Hz frequency
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)  # Start PWM with 0% duty cycle (servo off)


def setServoPos(degree):
    """
    Sets the servo motor to the specified angle.

    Parameters:
    degree (float): The target angle between 0 and 180 degrees.
    """
    # Ensure the angle is within 0 to 180 degrees
    if degree < 0:
        degree = 0
    elif degree > 180:
        degree = 180

    # Calculate duty cycle from the angle
    duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)

    # Print the duty cycle for debugging
    print(f"Degree: {degree} to {duty} (Duty Cycle)")

    # Change the duty cycle to move the servo
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Wait for the servo to reach the position
    servo.ChangeDutyCycle(0)  # Stop sending PWM signal to prevent jitter


try:
    while True:
        # Get user input for the angle
        angle_input = input("Enter the angle to rotate (0-180 degrees): ")

        # Validate angle input
        try:
            if angle_input == "exit":
                servo.stop()
                GPIO.cleanup()
                break
            angle = float(angle_input)
            if not 0 <= angle <= 180:
                print("Please enter an angle between 0 and 180.")
                continue
        except ValueError:
            print("Invalid input. Please enter a numeric value.")
            continue

        # Set the servo position
        setServoPos(angle)

        # Print the current angle
        print(f"Servo is now at {angle} degrees.")

except KeyboardInterrupt:
    # Clean up GPIO settings on Ctrl+C exit
    servo.stop()
    GPIO.cleanup()
