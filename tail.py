import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

servo_pin_tail = 22  # 꼬리 핀
SERVO_MAX_DUTY = 12  # 서보의 최대(180도) 위치의 주기
SERVO_MIN_DUTY = 3   # 서보의 최소(0도) 위치의 주기
mode = 0
motorState = 0

class EmotionControlNode(Node):
    def __init__(self):
        super().__init__('tail_control_node')

        self.emotion_subscriber = self.create_subscription(
            String,
            '/user_command',
            self.walking_callback,
            10
        )

        self.place_subscriber = self.create_subscription(
            String,
            '/heendy_place',
            self.place_callback,
            10
        )

        self.place_subscriber = self.create_subscription(
            String,
            '/chatter',
            self.stop_callback,
            10
        )

        self.place_subscriber = self.create_subscription(
            String,
            '/heendy_home',
            self.place_callback,
            10
        )

        GPIO.setmode(GPIO.BOARD)  # GPIO 설정
        GPIO.setup(servo_pin_tail, GPIO.OUT)  # 서보핀 출력으로 설정

        self.servo_tail = GPIO.PWM(servo_pin_tail, 50)  # 50Hz로 PWM 설정

        self.servo_tail.start(0)  # 서보 PWM 시작, duty = 0이면 서보 동작하지 않음
        
        # 주기적으로 실행될 함수 등록
        self.timer = self.create_timer(1.0, self.periodic_callback)  # 1초 주기로 설정

    def set_servo_pos(self, servo, degree):
        if degree > 180:
            degree = 180

        # duty 계산
        duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)
        print(f"Degree: {degree} to {duty}(Duty)")

        # duty 값을 사용하여 서보 모터 제어
        servo.ChangeDutyCycle(duty)
        
    def walking_callback(self, msg):
        walking = msg.data
        print(walking)

        if walking == 'start':
            mode = 1

        elif walking == 'stop':
            # 꼬리 멈추기
            mode = 0

    def stop_callback(self, msg):
        place_value = msg.data
        print(f"chatter: {place_value}")
        
        mode = 0

    def place_callback(self, msg):
        place_value = msg.data
        print(f"Heendy's place: {place_value}")
        
        mode = 1

    def periodic_callback(self):
        # 주기적으로 실행될 코드를 작성
        print("Periodic callback")
        if mode == 1 :
            if motorState == 0 :
                self.set_servo_pos(self.servo_tail, 60)
                time.sleep(0.2)
                motorState == 1
            else :
                self.set_servo_pos(self.servo_tail, 60)
                time.sleep(0.2)
                motorState == 0
        
        self.servo_tail.ChangeDutyCycle(0)  # PWM 신호를 중지하여 서보 모터의 움직임을 멈춤



def main(args=None):
    rclpy.init(args=args)
    node = EmotionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()