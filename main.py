# Host PC Python Code
import socket
import cv2
import numpy as np
from ultralytics import YOLO

# 서버 설정
HOST = '0.0.0.0'  # 모든 IP에서 수신 가능하게 설정
PORT = 5000  # 포트 번호 맞추기

# YOLOv8n 모델 로드
model = YOLO('yolov8n.pt')  # YOLOv8n 모델 로드 (경량화 버전)


def receive_all(conn, length):
    """주어진 길이만큼 데이터를 모두 수신하는 함수"""
    data = b''
    while len(data) < length:
        to_read = length - len(data)
        packet = conn.recv(4096 if to_read > 4096 else to_read)  # 가변적으로 수신할 크기 조정
        if not packet:
            return None
        data += packet
    return data


def send_centroids(conn, centroids):
    """클라이언트에게 중점 좌표 전송"""
    try:
        # 중점 좌표 리스트를 문자열로 변환하여 전송
        if centroids:
            message = ','.join([f'{cx},{cy}' for cx, cy in centroids])  # 쉼표로 구분된 중점 좌표 문자열
            conn.sendall(message.encode())  # 문자열을 바이트로 인코딩하여 전송
            print(f"Sent centroids: {message}")
        else:
            message = "None,None"
            conn.sendall(message.encode())
    except Exception as e:
        print(f"Error sending centroids: {e}")

def main():
    # TCP/IP 소켓 생성
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"Listening on {HOST}:{PORT}...")

        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")

            while True:
                try:
                    # 이미지 데이터 크기 수신
                    length_data = receive_all(conn, 4)
                    if length_data is None:
                        print("No length data received. Exiting.")
                        break

                    data_length = int.from_bytes(length_data, byteorder='big')
                    print(f"Expected data length: {data_length} bytes")

                    # 이미지 데이터 수신
                    img_data = receive_all(conn, data_length)
                    if img_data is None:
                        print("No image data received. Exiting.")
                        break

                    print(f"Received image data of length: {len(img_data)} bytes")

                    # 데이터를 NumPy 배열로 변환 및 이미지 디코딩
                    img_array = np.frombuffer(img_data, dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                    if frame is not None:
                        # YOLOv8n 모델로 객체 탐지 수행
                        results = model(frame, classes=[0], verbose=False)  # 클래스 0(person)만 탐지
                        detections = results[0].boxes.data.cpu().numpy()

                        centroids = []  # 중점 좌표 리스트

                        # 탐지된 객체 그리기 및 중점 계산
                        for detection in detections:
                            x1, y1, x2, y2, conf, class_id = map(int, detection[:6])
                            cx = (x1 + x2) // 2  # 중점 x 좌표
                            cy = (y1 + y2) // 2  # 중점 y 좌표
                            centroids.append((cx, cy))  # 중점 좌표 추가
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 바운딩 박스 그리기
                            cv2.putText(frame, f'Conf: {conf}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                                        (0, 255, 0), 2)

                        # 클라이언트로 중점 좌표 전송
                        send_centroids(conn, centroids)

                        # 이미지 출력
                        cv2.imshow('YOLO Object Detection', frame)

                        # 'q' 키로 종료
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                    else:
                        print("Failed to decode image.")
                except Exception as e:
                    print(f"Error receiving frame: {e}")
                    break

    # 정리 작업
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
