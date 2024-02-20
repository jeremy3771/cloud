import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from .ddsm115_operator_def import MOTOR_COMMAND
import time

class StopWheelMotorNode(Node):
    def __init__(self):
        super().__init__('stop_wheel_motor')
        self.MOTOR1 = MOTOR_COMMAND(0)
        
        # "/direction" 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/direction',
            self.direction_callback,
            10)
        
        self.last_direction_time = self.get_clock().now()
        
        # 5초마다 실행될 타이머 설정
        self.timer = self.create_timer(5.0, self.check_direction_timeout)

    def direction_callback(self, msg):
        # 마지막으로 데이터를 받은 시간을 갱신
        self.last_direction_time = self.get_clock().now()

    def check_direction_timeout(self):
        # 현재 시간과 마지막으로 데이터를 받은 시간의 차이 계산
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.last_direction_time
        if elapsed_time.nanoseconds > 1e9:  # 5초를 초과했는지 확인
            # 모터 정지 로직
            self.MOTOR1.SET_VELOCITY(ID=1, SPEED=0)
            time.sleep(0.005)
            self.MOTOR1.SET_VELOCITY(ID=2, SPEED=0)
            self.get_logger().info('No direction data received for 1 seconds. Stopping motors.')

def main(args=None):
    rclpy.init(args=args)
    stop_wheel_motor_node = StopWheelMotorNode()
    try:
        rclpy.spin(stop_wheel_motor_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
