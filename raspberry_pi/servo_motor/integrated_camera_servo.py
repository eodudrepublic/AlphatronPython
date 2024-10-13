# Raspberry Pi : integrated_camera_servo.py
from picamera2 import Picamera2
import cv2
import socket
import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit

# ================================
# GPIO and Servo Motor Setup
# ================================

# Camera field of view
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
HFOV = 109.3  # Raspberry Pi Camera Module 3 Wide Horizontal FOV in degrees
VFOV = 48.8  # Raspberry Pi Camera Module 3 Wide Vertical FOV in degrees

FRAME_CENTER_X = FRAME_WIDTH / 2
FRAME_CENTER_Y = FRAME_HEIGHT / 2

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
SERVO_MAX_DUTY = 11  # Duty cycle for 180 degrees

# Setup GPIO for Servo Motor 1
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialize PWM for Servo Motor 1 at 50Hz
servo0 = GPIO.PWM(SERVO_PIN, 50)
servo0.start(0)  # Start PWM with 0% duty cycle

# --- Servo Motor 2 Configuration (ServoKit) ---
kit = ServoKit(channels=16, address=0x40)  # Adjust 'address' if necessary


def setServoPosGPIO(angle):
    """
    Moves the servo motor connected to GPIO18 to the specified angle.

    Parameters:
    angle (float): The target angle between 0 and 180 degrees
    """
    # Reverse the angle (180 - angle)
    reversed_angle = 180 - angle

    # Limit the reversed angle between 0 and 180 degrees
    reversed_angle = max(0, min(180, reversed_angle))

    # Calculate the duty cycle
    duty = SERVO_MIN_DUTY + (reversed_angle * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)

    # Move the servo motor
    servo0.ChangeDutyCycle(duty)
    time.sleep(0.03)
    servo0.ChangeDutyCycle(0)  # Reset duty cycle to prevent jitter


def setServoPosServoKit(angle):
    """
    Moves the servo motor connected to ServoKit channel 0 to the specified angle.

    Parameters:
    angle (float): The target angle between 0 and 180 degrees
    """
    # Reverse the angle (180 - angle)
    reversed_angle = 180 - angle

    # Limit the reversed angle between 0 and 180 degrees
    reversed_angle = max(0, min(180, reversed_angle))

    # Set the angle for Servo Motor 2 (channel 0)
    kit.servo[0].angle = reversed_angle


# ================================
# Camera and Networking Setup
# ================================

# Camera setup
picam2 = Picamera2()

main_config = {"format": "RGB888", "size": (640, 480)}
camera_config = picam2.create_preview_configuration(main=main_config)
picam2.configure(camera_config)
picam2.set_controls({"FrameRate": 30.0})  # Set initial FPS to 30

picam2.start()

# Socket setup
HOST = '192.168.1.1'  # Change to the server's IP address
PORT = 5000  # Match the port number


def send_frame_via_socket(s, frame):
    try:
        frame = cv2.resize(frame, (640, 480))  # Resize the frame to 640x480
        encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])[1].tobytes()

        # Send the image size information
        frame_size = len(encoded_frame).to_bytes(4, byteorder='big')
        s.sendall(frame_size + encoded_frame)  # Send both size and image data
        print(f"Sent frame of size: {len(encoded_frame)} bytes")
    except Exception as e:
        print(f"Error sending frame: {e}")


def receive_centroids(s):
    try:
        data = s.recv(4096)  # Receive the centroid data from the server
        if not data:
            return None
        # Parse the received centroid data
        tokens = data.decode().split(',')
        if len(tokens) == 3 and tokens[0] == 'None' and tokens[1] == 'None' and tokens[2] == 'None':
            return None  # No detection case
        centroids = []
        for i in range(0, len(tokens), 3):
            track_id = int(tokens[i])
            cx = int(tokens[i + 1])
            cy = int(tokens[i + 2])
            centroids.append({'id': track_id, 'cx': cx, 'cy': cy})
        return centroids
    except Exception as e:
        print(f"Error receiving centroids: {e}")
        return None


def main():
    servo0_moved = False

    # Establish socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"Connected to {HOST}:{PORT}")

        prev_servo0_angle = None
        prev_servo1_angle = None

        while True:
            try:
                # Capture frame
                frame = picam2.capture_array()
                # Send the frame to the server
                send_frame_via_socket(s, frame)
                # Receive the object centroids from the server
                centroids = receive_centroids(s)

                # Check if centroids are received
                if centroids:
                    # Use the first centroid for servo control
                    first_centroid = centroids[0]
                    cx = first_centroid['cx']
                    cy = first_centroid['cy']
                    track_id = first_centroid['id']
                    # Display the centroid on the frame as a yellow dot
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                    # Display the ID near the centroid
                    cv2.putText(frame, f'ID: {track_id}', (cx + 5, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 255), 1)
                    print(f"Received centroids:\nID: {track_id}, Centroid: ({cx}, {cy})")

                    # Calculate target angles for servos
                    target_servo0_angle = ((cx - FRAME_CENTER_X) / FRAME_WIDTH) * HFOV + 90
                    target_servo1_angle = ((cy - FRAME_CENTER_Y) / FRAME_HEIGHT) * VFOV + 90
                else:
                    print("No objects detected.")
                    # Set target angles to center
                    target_servo0_angle = 90.0
                    target_servo1_angle = 90.0

                # Only update servo positions if there is a significant change
                if prev_servo0_angle is None or abs(target_servo0_angle - prev_servo0_angle) > 1:
                    setServoPosGPIO(target_servo0_angle)
                    prev_servo0_angle = target_servo0_angle
                    servo0_moved = True

                if prev_servo1_angle is None or abs(target_servo1_angle - prev_servo1_angle) > 1:
                    setServoPosServoKit(target_servo1_angle)
                    prev_servo1_angle = target_servo1_angle

                # Display the captured frame with centroids
                cv2.imshow('Camera Feed', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                if servo0_moved:
                    servo0_moved = False
                else:
                    # Optional: Add a small delay to prevent rapid updates
                    time.sleep(0.03)
                    servo0_moved = False

            except Exception as e:
                print(f"Error: {e}")
                break

    # Cleanup operations
    picam2.stop()
    cv2.destroyAllWindows()
    # Clean up GPIO settings
    servo0.stop()
    GPIO.cleanup()
    print("Connection closed")


if __name__ == '__main__':
    main()
