#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import psutil
from std_msgs.msg import Float32


class SystemStatsNode(Node):
    def __init__(self):
        super().__init__("system_stats")

        self.cpu_publisher_ = self.create_publisher(Float32, 'system_stats/cpu_usage', 10)
        self.mem_publisher_ = self.create_publisher(Float32, 'system_stats/virtual_mem_usage', 10)
        self.swap_publisher_ = self.create_publisher(Float32, 'system_stats/swap_mem_usage', 10)
        self.disk_publisher_ = self.create_publisher(Float32, 'system_stats/disk_usage', 10)
        self.timer_ = self.create_timer(5, self.publish_stats)

        self.get_logger().info("System Stats has been started")

    # This method publishes system stats, reusing the msg object as all messages are Float32
    def publish_stats(self):
        # Publish CPU usage
        msg = Float32()
        msg.data = psutil.cpu_percent()
        self.cpu_publisher_.publish(msg)

        # Publish virtual memory usage
        msg.data = psutil.virtual_memory().percent
        self.mem_publisher_.publish(msg)

        # Publish swap memory usage
        msg.data = psutil.swap_memory().percent
        self.swap_publisher_.publish(msg)

        # Publish disk usage
        msg.data = psutil.disk_usage('/').percent
        self.disk_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemStatsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
