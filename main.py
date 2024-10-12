# 객체 추적 및 추적한 것들의 중점 좌표들을 그려서 해당 프레임과 객체 ID에 따른 중점 좌료들을 웹소캣 서버로 전송
import socket
import cv2
import numpy as np
from ultralytics import YOLO
from boxmot import DeepOCSORT
from pathlib import Path
import websocket
import json
import struct

# 서버 설정
HOST = '0.0.0.0'
PORT = 5000

# YOLOv8n 모델 로드
model = YOLO('yolov8n.pt')

# DeepOCSORT 트래커 초기화
weights_path = Path(r"C:\Work\alphatronPython\osnet_x0_25_msmt17.pt")  # 모델 가중치 경로 설정
tracking_config = 'DeepOCSORT/config.yaml'  # 트래킹 설정 파일 경로 설정

tracking_method = DeepOCSORT(
    model_weights=weights_path,
    device='cpu',
    fp16=False,
    config_file=tracking_config
)

def receive_all(conn, length):
    """주어진 길이만큼 데이터를 모두 수신하는 함수"""
    data = b''
    while len(data) < length:
        to_read = length - len(data)
        packet = conn.recv(4096 if to_read > 4096 else to_read)
        if not packet:
            return None
        data += packet
    return data

def send_centroids(conn, centroids):
    """클라이언트에게 중점 좌표 전송"""
    try:
        if centroids:
            # 중점 좌표 리스트를 문자열로 변환하여 전송 (ID, x, y 형태)
            message = ','.join([f'{item["id"]},{item["cx"]},{item["cy"]}' for item in centroids])
            conn.sendall(message.encode())
            print(f"Sent centroids: {message}")
        else:
            message = "None,None,None"
            conn.sendall(message.encode())
    except Exception as e:
        print(f"Error sending centroids: {e}")

def process_frame(frame):
    """
    프레임을 처리하여 객체 탐지 및 추적을 수행하고, 중점 좌표를 반환합니다.
    """
    # YOLOv8n 모델로 객체 탐지 수행
    results = model(frame, classes=[0], verbose=False)  # 클래스 0(person)만 탐지
    detections = results[0].boxes.data.cpu().numpy()

    # DeepOCSORT로 객체 추적
    tracks = tracking_method.update(detections, frame)

    centroids = []  # 중점 좌표 리스트

    # 추적된 객체 그리기 및 중점 계산
    for track in tracks:
        x1, y1, x2, y2, track_id, class_id, conf = map(int, track[:7])
        cx = (x1 + x2) // 2  # 중점 x 좌표
        cy = (y1 + y2) // 2  # 중점 y 좌표
        centroids.append({'id': track_id, 'cx': cx, 'cy': cy})  # ID 포함

        # 바운딩 박스 및 ID 표시
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'ID: {track_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0), 2)
    return frame, centroids

def process_and_send_frame(frame, ws):
    """
    프레임을 처리하고, 결과를 WebSocket으로 전송합니다.
    """
    # 프레임 처리
    processed_frame, centroids = process_frame(frame)

    # WebSocket으로 데이터 전송
    try:
        centroids_json = json.dumps(centroids)
        centroids_bytes = centroids_json.encode('utf-8')
        centroid_length = len(centroids_bytes)

        # 프레임을 JPEG로 인코딩
        _, buffer = cv2.imencode('.jpg', processed_frame)
        frame_data = buffer.tobytes()
        frame_length = len(frame_data)

        # 데이터 패킹
        message = struct.pack('>I', centroid_length) + centroids_bytes + struct.pack('>I', frame_length) + frame_data

        # WebSocket으로 데이터 전송
        ws.send(message, opcode=websocket.ABNF.OPCODE_BINARY)
        print("Sent frame and centroids over websocket")

    except Exception as e:
        print(f"Error sending data over websocket: {e}")

    return processed_frame, centroids

def receive_frame(conn):
    """
    소켓으로부터 프레임 데이터를 수신합니다.
    """
    # 이미지 데이터 크기 수신
    length_data = receive_all(conn, 4)
    if length_data is None:
        print("No length data received. Exiting.")
        return None
    data_length = int.from_bytes(length_data, byteorder='big')
    print(f"Expected data length: {data_length} bytes")

    # 이미지 데이터 수신
    img_data = receive_all(conn, data_length)
    if img_data is None:
        print("No image data received. Exiting.")
        return None
    print(f"Received image data of length: {len(img_data)} bytes")

    # 데이터를 NumPy 배열로 변환 및 이미지 디코딩
    img_array = np.frombuffer(img_data, dtype=np.uint8)
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    if frame is None:
        print("Failed to decode image.")
        return None

    return frame

def handle_connection(conn, ws):
    """클라이언트 연결을 처리하는 함수"""
    with conn:
        print(f"Connected by {conn.getpeername()}")

        while True:
            try:
                frame = receive_frame(conn)
                if frame is None:
                    break

                # 프레임 처리 및 WebSocket으로 전송
                processed_frame, centroids = process_and_send_frame(frame, ws)

                # 클라이언트로 중점 좌표 전송
                send_centroids(conn, centroids)

                # 이미지 출력
                cv2.imshow('YOLO + DeepOCSORT Tracking', processed_frame)

                # 'q' 키로 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception as e:
                print(f"Error processing frame: {e}")
                break

    # 소켓 연결 종료
    conn.close()

def main():
    # TCP/IP 소켓 생성
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"Listening on {HOST}:{PORT}...")

        # WebSocket 연결 설정 (프로그램 시작 시 한 번만 연결)
        try:
            ws = websocket.create_connection("ws://localhost:7777/test", ping_interval=None)
            print("WebSocket client connected to ws://localhost:7777/test")
        except Exception as e:
            print(f"Error connecting to WebSocket server: {e}")
            return

        try:
            while True:
                conn, addr = s.accept()
                print(f"Accepted connection from {addr}")
                handle_connection(conn, ws)
        except KeyboardInterrupt:
            print("Server shutting down...")
        finally:
            # 프로그램 종료 시 WebSocket 연결 종료
            ws.close()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
