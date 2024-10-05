# Raspberry Pi : camera_final.py
from picamera2 import Picamera2
import cv2
import socket

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
        encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUAL>

        # Send the image size information
        frame_size = len(encoded_frame).to_bytes(4, byteorder='big')
        s.sendall(frame_size + encoded_frame)  # Send both size and image data
        print(f"Sent frame of size: {len(encoded_frame)} bytes")
    except Exception as e:
        print(f"Error sending frame: {e}")

def receive_centroids(s):
    try:
        data = s.recv(1024)  # Receive the centroid data from the server
        if not data:
            return None
        # Parse the received centroid data
        tokens = data.decode().split(',')
        if len(tokens) == 3 and tokens[0] == 'None' and tokens[1] == 'None' an>
            return None  # No detection case
        centroids = []
        for i in range(0, len(tokens), 3):
            track_id = int(tokens[i])
            cx = int(tokens[i+1])
            cy = int(tokens[i+2])
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

        # Send first frame and wait for response
        frame = picam2.capture_array()
        send_frame_via_socket(s, frame)
        centroids = receive_centroids(s)
        if centroids:
            print(f"Received centroids: {centroids}")
        else:
            print("No objects detected.")

        # After first frame, change frame rate to 30 fps
        picam2.set_controls({"FrameRate": 30.0})

        while True:
            try:
                # Capture frame
                frame = picam2.capture_array()
                # Send the frame to the server
                send_frame_via_socket(s, frame)
                # Receive the object centroids from the server
                centroids = receive_centroids(s)
                if centroids:
                    for item in centroids:
                        cx = item['cx']
                        cy = item['cy']
                        track_id = item['id']
                        # Display the centroid on the frame as a yellow dot
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                        # Display the ID near the centroid
                        cv2.putText(frame, f'ID: {track_id}', (cx + 5, cy + 5)>
                    print(f"Received centroids: {centroids}")
                else:
                    print("No objects detected.")

                # Display the captured frame with centroids
                cv2.imshow('Camera Feed', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception as e:
                print(f"Error: {e}")
                break

    # Cleanup operations
    picam2.stop()
    cv2.destroyAllWindows()
    print("Connection closed")

if __name__ == '__main__':
    main()