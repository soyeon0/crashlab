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
            '/topic',
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

    def move_servo_back_and_forth(self, servo, start_degree, end_degree, num_cycles):
        # 각도를 서보가 왔다갔다 하는 함수
        delay = 10  # 각도를 유지하는 시간 (milliseconds)

        for _ in range(num_cycles):
            self.set_servo_pos(servo, start_degree)
            self.spin_for_duration(delay)

            self.set_servo_pos(servo, end_degree)
            self.spin_for_duration(delay)

    def spin_for_duration(self, duration_ms):
        start_time = time.time()

        while (time.time() - start_time) * 1000 < duration_ms:
            rclpy.spin_once(self, timeout_sec=0.01)  # 적절한 timeout 값 사용

    def emotion_callback(self, msg):
        emotion = msg.data

        if emotion == "1":
            # 왼쪽 서보 모터를 왔다갔다 움직임
            print("1번 모드")
            self.move_servo_back_and_forth(self.servo_left, start_degree = 60, end_degree=100, num_cycles=3)
            
        elif emotion == "2":
            # 쫑긋 상태
            print("2번 모드")
            self.set_servo_pos(self.servo_left, 90)
        else:
            # TODO: 다른 감정에 대한 서보 모터 및 GIF 제어 코드 추가
            pass

def main(args=None):
    rclpy.init(args=args)
    node = EmotionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
