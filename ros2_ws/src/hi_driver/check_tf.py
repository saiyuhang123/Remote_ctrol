import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
import time

class OdomTfChecker(Node):
    def __init__(self):
        super().__init__('odom_tf_checker')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.last_tf_time_sec = 0.0
        self.last_real_time = time.time()
        self.count = 0
        self.freq = 0.0
        
        # 监听 /tf，寻找 base_link
        self.create_subscription(TFMessage, '/tf', self.tf_cb, qos_profile)
        self.create_timer(0.5, self.print_status)

    def tf_cb(self, msg):
        for t in msg.transforms:
            # 我们只关心 odom -> base_link (或者 base_link -> odom)
            if t.child_frame_id == 'base_link' and t.header.frame_id == 'odom':
                current_sec = t.header.stamp.sec + t.header.stamp.nanosec / 1e9
                
                # 计算真实的 TF 发布频率
                now = time.time()
                time_diff = now - self.last_real_time
                if time_diff > 0:
                    self.freq = 1.0 / time_diff
                
                self.last_real_time = now
                self.last_tf_time_sec = current_sec
                self.count += 1

    def print_status(self):
        if self.count == 0:
            print("🔴 致命错误：完全没有收到 odom -> base_link 的 TF 数据！FAST-LIO 挂了吗？")
        else:
            # 计算时间戳的跳跃（判断是否断档）
            print(f"🟢 TF帧数: {self.count} | 实时频率: {self.freq:.1f} Hz | 最新时间戳: {self.last_tf_time_sec:.3f}", end='\r')
            
            # 如果频率低于 5Hz，说明 FAST-LIO 被卡住了
            if self.freq < 5.0 and self.count > 3:
                print(f"\n🚨🚨🚨 警告！FAST-LIO 发布频率过低 ({self.freq:.1f} Hz)，Nav2 必定丢包！🚨🚨🚨")

rclpy.init()
node = OdomTfChecker()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass