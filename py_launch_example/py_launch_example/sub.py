import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubNode(Node):
    def __init__(self):
        super().__init__('sub_node')  # 노드 이름 설정
        # hello_topic 구독, 수신 시 callback 호출
        self.sub = self.create_subscription(
            String,
            'hello_topic',
            self.callback,
            10                        # QoS depth
        )

    def callback(self, msg):
        # 수신한 메시지를 로그로 출력
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    rclpy.spin(node)                  # 메시지 수신 대기
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
