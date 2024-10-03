import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
import subprocess


class TrafficJudgmenter(Node):
    def __init__(self):
        super().__init__('traffic_judgmenter')

        # 
        self.current_waypoint_sub = self.create_subscription(
            Int32,  # 1つ目のトピックのメッセージ型
            'waypoint_manager2/current_waypoint',  # 1つ目のトピック名
            self.current_waypoint_callback,
            1)

        self.traffic_yolov8_sub = self.create_subscription(
            String,  # 2つ目のトピックのメッセージ型
            'shell_output',  # 2つ目のトピック名
            self.shell_output_callback,
            1)

        # 各トピックからのデータを保持する変数
        self.current_waypoint_msg = None
        self.traffic_msg = None
        

    def current_waypoint_callback(self, msg):
        self.get_logger().info(f'current waypoint received: {msg.data}')
        self.current_waypoint_msg = msg.data

        # 条件を満たしたらアクションを実行
        self.traffic_judgment()

    def shell_output_callback(self, msg):
        #self.get_logger().info(f'shell output received: {msg.data}')
        self.traffic_msg = msg.data

        # 条件を満たしたらアクションを実行
        self.traffic_judgment()

    def traffic_judgment(self):
        # 両方のデータが取得されていて、特定の条件を満たしているかチェック
        if self.current_waypoint_msg is not None and self.traffic_msg is not None:
            if self.current_waypoint_msg == 5 and "chair" in self.traffic_msg:  # 条件を定義
                self.next_waypoint_service()

    def next_waypoint_service(self):
        # next waypoint
        subprocess.Popen(['ros2', 'service', 'call', '/waypoint_manager2/next_wp', 'std_srvs/srv/Trigger'])
        self.get_logger().info('success!! next waypoint!!')
        
def main(args=None):
    rclpy.init(args=args)

    traffic_judgmenter = TrafficJudgmenter()

    rclpy.spin(traffic_judgmenter)

    # 終了時のクリーンアップ
    multi_topic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
