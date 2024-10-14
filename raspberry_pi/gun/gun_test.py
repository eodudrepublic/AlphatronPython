import RPi.GPIO as GPIO
import time

# GPIO pin setup
relay_pin = 17  # Using GPIO17 pin
n = 0.2

GPIO.setmode(GPIO.BCM)  # Set pin numbering to BCM mode
GPIO.setup(relay_pin, GPIO.OUT)  # Set relay pin to output mode

# Turn ON the relay
GPIO.output(relay_pin, GPIO.HIGH)  # Output High signal (relay activated)

time.sleep(n)  # Wait for n seconds

# Turn OFF the relay
GPIO.output(relay_pin, GPIO.LOW)  # Output Low signal (relay deactivated)

GPIO.cleanup()  # Reset GPIO settings