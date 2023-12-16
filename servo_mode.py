import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

servo_pin_left = 16  # 왼쪽 서보 핀
servo_pin_right = 18  # 오른쪽 서보 핀
SERVO_MAX_DUTY = 12  # 서보의 최대(180도) 위치의 주기
SERVO_MIN_DUTY = 3   # 서보의 최소(0도) 위치의 주기

class EmotionControlNode(Node):
    def __init__(self):
        super().__init__('emotion_control_node')

        self.emotion_subscriber = self.create_subscription(
            String,
            '/heendy_emotion',
            self.emotion_callback,
            10
        )

        GPIO.setmode(GPIO.BOARD)  # GPIO 설정
        GPIO.setup(servo_pin_left, GPIO.OUT)  # 서보핀 출력으로 설정
        GPIO.setup(servo_pin_right, GPIO.OUT)  # 서보핀 출력으로 설정

        self.servo_left = GPIO.PWM(servo_pin_left, 50)  # 50Hz로 PWM 설정
        self.servo_right = GPIO.PWM(servo_pin_right, 50)  # 50Hz로 PWM 설정

        self.servo_left.start(0)  # 서보 PWM 시작, duty = 0이면 서보 동작하지 않음
        self.servo_right.start(0)

    def set_servo_pos(self, servo, degree):
        if degree > 180:
            degree = 180

        # duty 계산
        duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)
        print(f"Degree: {degree} to {duty}(Duty)")

        # duty 값을 사용하여 서보 모터 제어
        servo.ChangeDutyCycle(duty)
        
    def emotion_callback(self, msg):
        emotion = msg.data
        print(emotion)
        type(emotion)

        if emotion == '기쁨':
            # 왼쪽 서보 모터와 오른쪽 서보 모터를 왔다갔다 움직임
            for _ in range(3):
                self.set_servo_pos(self.servo_left, 60)
                self.set_servo_pos(self.servo_right, 100)
                time.sleep(0.5)

                self.set_servo_pos(self.servo_left, 100)
                self.set_servo_pos(self.servo_right, 60)
                time.sleep(0.5)
                
        elif emotion == '2':
            # 쫑긋 상태
            self.set_servo_pos(self.servo_left, 90)
            self.set_servo_pos(self.servo_right, 90)
            time.sleep(0.3)

        elif emotion == '무기분':
            # 기본 상태
            self.set_servo_pos(self.servo_left, 70)
            self.set_servo_pos(self.servo_right, 110)
            time.sleep(0.3)

        elif emotion == '슬픔':
            # 질문 상태
            self.set_servo_pos(self.servo_left, 90)
            self.set_servo_pos(self.servo_right, 120)
            time.sleep(0.3)

        self.servo_left.ChangeDutyCycle(0)  # PWM 신호를 중지하여 서보 모터의 움직임을 멈춤
        self.servo_right.ChangeDutyCycle(0)  # PWM 신호를 중지하여 서보 모터의 움직임을 멈춤


def main(args=None):
    rclpy.init(args=args)
    node = EmotionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
