import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(String, '/direction', 10)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        direction_msg = String()
        try:
            if key == keyboard.Key.up:
                direction_msg.data = 'Go Straight'
            elif key == keyboard.Key.down:
                direction_msg.data = 'Go Back'
            elif key == keyboard.Key.left:
                direction_msg.data = 'Turn Left'
            elif key == keyboard.Key.right:
                direction_msg.data = 'Turn Right'
            elif key == keyboard.KeyCode(char='/'):
                direction_msg.data = 'Stop'
            else:
                return
            self.publisher_.publish(direction_msg)
        except AttributeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()
    try:
        rclpy.spin(keyboard_node)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
