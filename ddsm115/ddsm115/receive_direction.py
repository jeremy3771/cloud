import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReceiveDirectionNode(Node):
    def __init__(self):
        super().__init__('receive_direction')
        # 로그 레벨을 INFO로 설정
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.subscription = self.create_subscription(
            String,
            'direction',
            self.direction_callback,
            10)
        self.subscription  # prevent unused variable warning

    def direction_callback(self, msg):
        direction_message = {
            'Turn Right': '우회전 하세요',
            'Turn Left': '좌회전하세요',
            'Go Straight': '직진하세요'
        }
        cleaned_data = msg.data.strip()  # 양쪽 공백 제거
        self.get_logger().info(f'Received direction data: "{cleaned_data}"')
        response = direction_message.get(cleaned_data, '알 수 없는 방향입니다')
        self.get_logger().info('Direction: "%s"' % response)
        

def main(args=None):
    rclpy.init(args=args)
    receive_direction_node = ReceiveDirectionNode()
    try:
        rclpy.spin(receive_direction_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
