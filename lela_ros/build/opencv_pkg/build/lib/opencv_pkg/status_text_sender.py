import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Mavlink
import struct

class StatusTextSender(Node):
    def __init__(self):
        super().__init__('status_text_sender')
        self.pub = self.create_publisher(Mavlink, '/mavros/mavlink/to', 10)
        # Send every 2 seconds
        self.timer = self.create_timer(2.0, self.send_text)
        self.get_logger().info("StatusTextSender ready, sending 'hello'...")

    def send_text(self):
        text = "hello"
        mavlink_msg = Mavlink()
        mavlink_msg.msgid = 253  # STATUSTEXT
        mavlink_msg.payload64 = self.pack_statustext(text)
        self.pub.publish(mavlink_msg)
        self.get_logger().info(f"Sent STATUSTEXT: {text}")

    def pack_statustext(self, text: str):
        severity = 0
        txt = text.encode('utf-8')[:50].ljust(50, b'\0')
        payload = struct.pack('<B50s', severity, txt)
        # Pad to 8-byte multiples for payload64
        while len(payload) % 8 != 0:
            payload += b'\0'
        return list(struct.unpack('<' + 'Q' * (len(payload)//8), payload))


def main(args=None):
    rclpy.init(args=args)
    node = StatusTextSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
