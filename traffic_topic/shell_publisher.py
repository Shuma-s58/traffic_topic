# SPDX-FileCopyrightText: 2024 Shuma Suzuki
# SPDX-LIcense-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading

class ShellPublisher(Node):

    def __init__(self):
        super().__init__('shell_publisher')
        self.publisher_ = self.create_publisher(String, 'shell_output', 1)
        
        # スクリプトを非同期に実行し、出力を逐次読み取る
        self.process = subprocess.Popen(
            ['/root/shell/connect_yolov8.sh'],  # 実行したいスクリプトを指定
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
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
        self.process.stdout.close()
        self.process.wait()

def main(args=None):
    rclpy.init(args=args)

    shell_publisher = ShellPublisher()

    try:
        rclpy.spin(shell_publisher)
    except KeyboardInterrupt:
        shell_publisher.process.terminate()  # プロセスを終了
        shell_publisher.thread.join()  # スレッドを終了

    shell_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
