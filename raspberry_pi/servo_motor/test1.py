# GPIO12에 연결된 서보모터 1 : 0도 <-> 90도 테스트
import RPi.GPIO as GPIO
import time

# Set the GPIO pin for the servo motor
SERVO_PIN = 12  # GPIO12 (PWM0 - GPIO18, pin number 12)

# Servo motor duty cycle constants
SERVO_MIN_DUTY = 2  # Duty cycle corresponding to 0 degrees
SERVO_MAX_DUTY = 11  # Duty cycle corresponding to 90 degrees

# GPIO configuration
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Create a PWM instance (frequency 50Hz)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)  # Start with an initial duty cycle of 0


def set_gpio_servo_angle(angle):
    """
    Function to set the angle of the GPIO servo motor
    angle: 0 to 180 degrees
    """
    # Limit the angle to 0-180 degrees
    angle = max(0, min(180, angle))

    # Calculate the duty cycle
    duty = SERVO_MIN_DUTY + (angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)

    # Move the servo motor
    servo_pwm.ChangeDutyCycle(duty)


try:
    while True:
        # Set the servo motor to 90 degrees
        set_gpio_servo_angle(90)
        print("GPIO servo motor set to 90 degrees")
        time.sleep(1)

        # Set the servo motor to 0 degrees
        set_gpio_servo_angle(0)
        print("GPIO servo motor set to 0 degrees")
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting program...")
finally:
    # Cleanup PWM and GPIO
    servo_pwm.stop()
    GPIO.cleanup()
    print("PWM cleanup and GPIO exit complete")