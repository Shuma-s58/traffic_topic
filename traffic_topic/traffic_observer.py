import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading

class TrafficObserver(Node):

    def __init__(self):
        super().__init__('traffic_observer')
        self.publisher_ = self.create_publisher(String, 'current_traffic_output', 1)
        
        # スクリプトを非同期に実行し、出力を逐次読み取る
        self.process = subprocess.Popen(
            ['exec /root/shell/connect_yolov8.sh'],  # 実行したいスクリプトを指定
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            shell=True,
            bufsize=1  # 出力を1行ごとにバッファリング
        )

        # 別スレッドでシェルスクリプトの出力を監視
        self.thread = threading.Thread(target=self.read_output)
        self.thread.start()

    def read_output(self):
        # シェルスクリプトの出力を逐次読み取り、ROS 2にパブリッシュ
        for line in iter(self.process.stdout.readline, ''):
            msg = String()
            msg.data = line.strip()  # 改行を取り除く
            self.publisher_.publish(msg)
            #self.get_logger().info('Publishing: "%s"' % msg.data)

        # プロセスが終了した場合の処理
        if self.process.stdout:
            self.process.stdout.close()
            self.process.wait()

        #self.process.terminate()
        self.process.kill()
        self.process.wait()

    def stop(self):
        if self.thread:
            self.thread.join()  # スレッドを終了

def main(args=None):
    rclpy.init(args=args)

    traffic_observer = TrafficObserver()

    rclpy.spin(traffic_observer)

    # 停止時にスレッドの終了を待機
    traffic_observer.stop()

    traffic_observer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
