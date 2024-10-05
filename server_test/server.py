# 스프링 서버와 연결하기 전 테스트 용 임시 웹소캣 서버 코드

import asyncio
import websockets
import json
import cv2
import numpy as np


async def receive_data(websocket):
    """
    웹소켓으로부터 데이터를 수신하고, 파싱하여 반환합니다.
    """
    data = await websocket.recv()  # 데이터 수신
    if isinstance(data, bytes):
        offset = 0

        # 중점 좌표 길이 추출
        if len(data) < 4:
            print("Data too short to contain centroid length")
            return None, None
        centroid_length = int.from_bytes(data[offset:offset + 4], byteorder='big')
        offset += 4

        # 중점 좌표 데이터 추출
        if len(data) < offset + centroid_length:
            print("Data too short to contain centroid data")
            return None, None
        centroid_data = data[offset:offset + centroid_length]
        offset += centroid_length

        # 프레임 길이 추출
        if len(data) < offset + 4:
            print("Data too short to contain frame length")
            return None, None
        frame_length = int.from_bytes(data[offset:offset + 4], byteorder='big')
        offset += 4

        # 프레임 데이터 추출
        if len(data) < offset + frame_length:
            print("Data too short to contain frame data")
            return None, None
        frame_data = data[offset:offset + frame_length]

        # 중점 좌표 디코딩
        print()
        print(centroid_data)
        print()
        centroids_json = centroid_data.decode('utf-8')
        centroids = json.loads(centroids_json)
        print("Received centroids:")
        for item in centroids:
            print(f"ID: {item['id']}, Centroid: ({item['cx']}, {item['cy']})")

        # 프레임 디코딩
        img_array = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if frame is None:
            print("Failed to decode frame")
            return None, None

        return centroids, frame
    else:
        print("Received non-binary data")
        return None, None


async def handle_connection(websocket, path):
    if path == "/test":
        try:
            print("Connection established at /test")

            while True:
                centroids, frame = await receive_data(websocket)
                if centroids is None or frame is None:
                    break

                # 프레임 출력
                cv2.imshow('Received Frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except websockets.exceptions.ConnectionClosed as e:
            print(f"Connection closed: {e}")
        finally:
            cv2.destroyAllWindows()


# WebSocket 서버 설정
start_server = websockets.serve(handle_connection, "localhost", 7777, ping_interval=None)

# asyncio를 이용한 서버 실행
asyncio.get_event_loop().run_until_complete(start_server)
print("WebSocket server started")
asyncio.get_event_loop().run_forever()
