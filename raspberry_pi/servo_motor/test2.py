# Servo Driver HAT의 0, 4번 채널에 연결된 서보모터 2, 3 : 0도 <-> 90도 테스트
import time
from adafruit_servokit import ServoKit

# Create a ServoKit instance using 16 channels of the PCA9685 board
kit = ServoKit(channels=16, address=0x40)

while True :
  kit.servo[0].angle = 90
  kit.servo[4].angle = 90

  time.sleep(1)

  kit.servo[0].angle = 0
  kit.servo[4].angle = 0

  time.sleep(1)