import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from .ddsm115_operator_def import MOTOR_COMMAND
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.MOTOR1 = MOTOR_COMMAND(0)
        
        # "/direction" 토픽 구독 설정
        self.subscription = self.create_subscription(
            String,
            '/direction',
            self.direction_callback,
            10)
        
    def direction_callback(self, msg):
        direction = msg.data
        self.get_logger().info(f'Received direction: {direction}')
        
        # 방향에 따른 모터 속도 조절
        if direction == 'Turn Left':
            # 모터 속도 조절 로직 구현
            self.MOTOR1.SET_VELOCITY(ID=1, SPEED=50)
            time.sleep(0.005)
            self.MOTOR1.SET_VELOCITY(ID=2, SPEED=100)
            time.sleep(0.005)
        elif direction == 'Go Straight':
            # 모터 속도 조절 로직 구현
            self.MOTOR1.SET_VELOCITY(ID=1, SPEED=100)
            time.sleep(0.005)
            self.MOTOR1.SET_VELOCITY(ID=2, SPEED=100)
            time.sleep(0.005)
        elif direction == 'Turn Right':
            # 모터 속도 조절 로직 구현
            self.MOTOR1.SET_VELOCITY(ID=1, SPEED=100)
            time.sleep(0.005)
            self.MOTOR1.SET_VELOCITY(ID=2, SPEED=50)
            time.sleep(0.005)
        elif direction == 'Go Back':
            # 모터 속도 조절 로직 구현
            self.MOTOR1.SET_VELOCITY(ID=1, SPEED=-100)
            time.sleep(0.005)
            self.MOTOR1.SET_VELOCITY(ID=2, SPEED=-100)
            time.sleep(0.005)
        elif direction == 'Stop':
            # 모터 속도 조절 로직 구현
            self.MOTOR1.SET_VELOCITY(ID=1, SPEED=0)
            time.sleep(0.005)
            self.MOTOR1.SET_VELOCITY(ID=2, SPEED=0)
            time.sleep(0.005)
        else:
            self.get_logger().info('Unknown direction received.')

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
