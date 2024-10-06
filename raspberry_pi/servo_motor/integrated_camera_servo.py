# Raspberry Pi : integrated_camera_servo.py
from picamera2 import Picamera2
import cv2
import socket
import RPi.GPIO as GPIO
import time

# ================================
# Servo Motor Setup
# ================================

# GPIO pin numbers where the servo motors are connected
SERVO_PIN_0 = 12  # PWM0 - GPIO18 (pin number 12)
SERVO_PIN_1 = 33  # PWM1 - GPIO13 (pin number 33)

# Servo motor duty cycle constants
SERVO_MAX_DUTY = 11  # Duty cycle corresponding to 180 degrees
SERVO_MIN_DUTY = 2  # Duty cycle corresponding to 0 degrees

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN_0, GPIO.OUT)
GPIO.setup(SERVO_PIN_1, GPIO.OUT)

# Create PWM instances (frequency 50Hz)
servo0 = GPIO.PWM(SERVO_PIN_0, 50)
servo1 = GPIO.PWM(SERVO_PIN_1, 50)
servo0.start(0)  # Start PWM signal with 0% duty cycle
servo1.start(0)

# Camera field of view
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
HFOV = 109.3  # Raspberry Pi Camera Module 3 Wide Horizontal FOV in degrees
VFOV = 48.8  # Raspberry Pi Camera Module 3 Wide Vertical FOV in degrees

FRAME_CENTER_X = FRAME_WIDTH / 2
FRAME_CENTER_Y = FRAME_HEIGHT / 2


def setServoPos(servo, angle):
    """
    Moves the servo motor to the specified angle.

    Parameters:
    servo (PWM): The PWM instance of the servo motor to control
    angle (float): The target angle between 0 and 180 degrees
    """
    # Limit the angle between 0 and 180 degrees
    angle = 180 - (max(0, min(180, angle)))

    # Calculate the duty cycle
    duty = SERVO_MIN_DUTY + (angle * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)

    # Move the servo motor
    servo.ChangeDutyCycle(duty)
    time.sleep(0.03)  # Wait for a short time
    servo.ChangeDutyCycle(0)  # Reset duty cycle to prevent jitter


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
                if (prev_servo0_angle is None or abs(target_servo0_angle - prev_servo0_angle) > 1):
                    setServoPos(servo0, target_servo0_angle)
                    prev_servo0_angle = target_servo0_angle

                if (prev_servo1_angle is None or abs(target_servo1_angle - prev_servo1_angle) > 1):
                    setServoPos(servo1, target_servo1_angle)
                    prev_servo1_angle = target_servo1_angle

                # Display the captured frame with centroids
                cv2.imshow('Camera Feed', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Optional: Add a small delay to prevent rapid updates
                time.sleep(0.05)

            except Exception as e:
                print(f"Error: {e}")
                break

    # Cleanup operations
    picam2.stop()
    cv2.destroyAllWindows()
    # Clean up GPIO settings
    servo0.stop()
    servo1.stop()
    GPIO.cleanup()
    print("Connection closed")


if __name__ == '__main__':
    main()
