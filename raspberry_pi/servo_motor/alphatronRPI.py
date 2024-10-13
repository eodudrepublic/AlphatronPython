# Raspberry Pi : alphatronRPI.py
import threading
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

# Shared variables for target positions
# Initialize with default values (center of the frame)
target_positions = {'cx': FRAME_CENTER_X, 'cy': FRAME_CENTER_Y}
position_lock = threading.Lock()  # Lock for synchronizing access

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


def servo_control_thread():
    """
    Thread function to control the servo motors asynchronously.
    """
    prev_servo0_angle = None
    prev_servo1_angle = None

    while True:
        with position_lock:
            cx = target_positions['cx']
            cy = target_positions['cy']

        if cx is not None and cy is not None:
            # Calculate target angles for servos
            target_servo0_angle = ((cx - FRAME_CENTER_X) / FRAME_WIDTH) * HFOV + 90
            target_servo1_angle = ((cy - FRAME_CENTER_Y) / FRAME_HEIGHT) * VFOV + 90

            # Only update servo positions if there is a significant change
            if prev_servo0_angle is None or abs(target_servo0_angle - prev_servo0_angle) > 1:
                setServoPosGPIO(target_servo0_angle)
                prev_servo0_angle = target_servo0_angle

            if prev_servo1_angle is None or abs(target_servo1_angle - prev_servo1_angle) > 1:
                setServoPosServoKit(target_servo1_angle)
                prev_servo1_angle = target_servo1_angle

        time.sleep(0.1)  # Small delay to prevent CPU overutilization
        servo0.ChangeDutyCycle(0)  # Reset duty cycle to prevent jitter


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
        data = s.recv(4096)
        if not data:
            return None
        data_str = data.decode().strip()
        if data_str == "None,None,None":
            print("Received acknowledgment from server")
            return "ACK"  # Indicates acknowledgment received from the server
        tokens = data_str.split(',')
        if len(tokens) != 3:
            print(f"Unexpected data format: {data_str}")
            return None
        try:
            track_id = int(tokens[0])
            cx = int(tokens[1])
            cy = int(tokens[2])
            return {'id': track_id, 'cx': cx, 'cy': cy}
        except ValueError:
            print(f"Invalid data format: {data_str}")
            return None
    except socket.timeout:
        print("Socket timeout while waiting for centroids")
        return "TIMEOUT"
    except Exception as e:
        print(f"Error receiving centroids: {e}")
        return None

def main():
    # Start the servo control thread
    servo_thread = threading.Thread(target=servo_control_thread, daemon=True)
    servo_thread.start()

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

    frame_transmission_paused = False  # Flag to control frame transmission

    # Establish socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.settimeout(5)  # Set a timeout of 5 seconds
        print(f"Connected to {HOST}:{PORT}")

        # Capture frame
        frame = picam2.capture_array()
        # Send the frame to the server
        send_frame_via_socket(s, frame)
        # Handle initial connection (ack)
        initial_data = receive_centroids(s)
        if initial_data == "ACK":
            print("Received initial ack from server")
        else:
            print("Unexpected initial data from server")

        while True:
            try:
                if frame_transmission_paused:
                    # Pause frame transmission
                    print("Frame transmission is paused, waiting before resuming...")
                    time.sleep(1)  # Wait for 1 second before trying again
                    frame_transmission_paused = False
                    continue

                # Capture frame
                frame = picam2.capture_array()
                # Send the frame to the server
                send_frame_via_socket(s, frame)

                # Wait until a response is received from the server
                received_data = receive_centroids(s)

                if received_data == "TIMEOUT":
                    # When a timeout occurs, pause frame transmission
                    print("Timeout occurred, pausing frame transmission")
                    frame_transmission_paused = True
                    continue  # Skip to the next iteration to pause sending frames

                if received_data == "ACK":
                    # When "None,None,None" is received from the server
                    print("Received ACK, proceeding to next frame")
                    continue  # Skip to the next iteration to send the next frame

                elif received_data:
                    # When centroid data is received from the server
                    centroids = received_data
                    cx = centroids['cx']
                    cy = centroids['cy']
                    track_id = centroids['id']
                    # Display the centroid on the frame as a yellow dot
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                    # Display the ID near the centroid
                    cv2.putText(frame, f'ID: {track_id}', (cx + 5, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 255), 1)
                    print(f"Processing centroid:\nID: {track_id}, Centroid: ({cx}, {cy})")
                    receive_centroids(s)
                    # Update the target positions for the servo control thread
                    with position_lock:
                        target_positions['cx'] = cx
                        target_positions['cy'] = cy

                else:
                    # When no valid data is received from the server
                    print("No valid data received from server")
                    # Do not update target_positions to keep the last known values

                # Display the captured frame with centroids
                cv2.imshow('Camera Feed', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except Exception as e:
                print(f"Error: {e}")
                break
            finally:
                s.close()

    # Cleanup operations
    picam2.stop()
    cv2.destroyAllWindows()
    # Clean up GPIO settings
    servo0.stop()
    GPIO.cleanup()
    print("Connection closed")


if __name__ == '__main__':
    main()
