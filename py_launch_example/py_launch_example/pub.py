import rclpy                          # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node           # Node 기본 클래스
from std_msgs.msg import String       # 문자열 메시지 타입

class PubNode(Node):
    def __init__(self):
        super().__init__('pub_node')  # 노드 이름 설정
        # QoS 설정: 최근 10개 메시지 유지
        self.pub = self.create_publisher(String, 'hello_topic', 10)
        # 1초(1.0Hz) 주기 타이머 생성 → timer_callback 호출
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello world!'     # 발행할 메시지 내용
        self.pub.publish(msg)         # 토픽 발행
        self.get_logger().info(f'Publish: {msg.data}')  # 로그 출력

def main(args=None):
    rclpy.init(args=args)             # ROS2 초기화
    node = PubNode()
    rclpy.spin(node)                  # 콜백 대기 루프
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
