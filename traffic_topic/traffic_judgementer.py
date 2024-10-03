import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
import subprocess
from std_srvs.srv import Trigger


class TrafficJudgmenter(Node):
    def __init__(self):
        super().__init__('traffic_judgmenter')

        # subscriver 
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

        # client
        self.client = self.create_client(Trigger, 'waypoint_manager2/next_wp')

        # timer
        self.timer = self.create_timer(2.0, self.traffic_judgment)

        # 各トピックからのデータを保持する変数
        self.current_waypoint_msg = None
        self.traffic_msg = None
        

    def current_waypoint_callback(self, msg):
        #self.get_logger().info(f'current waypoint received: {msg.data}')
        self.current_waypoint_msg = msg.data

        # 条件を満たしたらアクションを実行
        #self.traffic_judgment()

    def shell_output_callback(self, msg):
        #self.get_logger().info(f'shell output received: {msg.data}')
        self.traffic_msg = msg.data

        # 条件を満たしたらアクションを実行
        #self.traffic_judgment()

    def traffic_judgment(self):
        # 両方のデータが取得されていて、特定の条件を満たしているかチェック
        if self.current_waypoint_msg is not None and self.traffic_msg is not None:
            if self.current_waypoint_msg == 5 and "blue" in self.traffic_msg:  # 条件を定義
                #self.next_waypoint_service()

                # サービスリクエストの送信
                self.send_request()
            else:
                self.get_logger().info('wait the crossing chance...')
        else:
            self.get_logger().info('wait srart...')

    def send_request(self):
        # リクエストメッセージを作成
        request = Trigger.Request()

        # 非同期でサービスコールを実行
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: success={response.success}, message="{response.message}"')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


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
