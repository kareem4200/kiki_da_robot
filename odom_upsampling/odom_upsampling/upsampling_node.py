import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from copy import deepcopy
import math

class OdomInterpolator(Node):
    def __init__(self):
        super().__init__('odom_upsampler')

        self.get_logger().info("Odometry upsampling initialized ......")

        # Parameters
        self.declare_parameter('slow_topic', '/scan_odom')
        self.slow_topic = self.get_parameter('slow_topic').get_parameter_value().string_value

        self.declare_parameter('fast_topic', '/odom_interpolated')
        self.fast_topic = self.get_parameter('fast_topic').get_parameter_value().string_value

        self.declare_parameter('source_frequency', 4.0)
        self.source_freq = self.get_parameter('source_frequency').get_parameter_value().double_value

        self.declare_parameter('target_frequency', 50.0)
        self.target_freq = self.get_parameter('target_frequency').get_parameter_value().double_value

        # self.get_logger().info(f"target_freq: {self.target_freq}, source_freq: {self.source_freq}")

        self.interpolation_count = math.ceil((1 / self.source_freq) / (1 / self.target_freq))

        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(Odometry, self.slow_topic, self.odom_callback, qos)
        self.pub = self.create_publisher(Odometry, self.fast_topic, qos)

        self.timer = self.create_timer(1.0 / self.target_freq, self.publish_next_interpolation)

        self.prev_odom = None
        self.curr_odom = None
        self.interpolated_msgs = []  # buffer of pre-fit messages
        self.current_index = 0

    def odom_callback(self, msg):
        self.prev_odom = self.curr_odom
        self.curr_odom = msg
        # self.get_logger().info(f"interpolation_count: {self.interpolation_count}")

        if self.prev_odom:
            self.interpolated_msgs = self.fit_odometry(self.prev_odom, self.curr_odom, self.interpolation_count)
            self.current_index = 0

    def fit_odometry(self, odom1, odom2, num_points):
        """ Linearly interpolate num_points between odom1 and odom2 """
        points = []

        for i in range(1, num_points + 1):  # from 1 to num_points (exclusive of endpoints)
            alpha = i / (num_points + 1)
            interp = Odometry()
            interp.header.frame_id = odom1.header.frame_id
            interp.header.stamp = self.get_clock().now().to_msg()

            # Position
            p1 = odom1.pose.pose.position
            p2 = odom2.pose.pose.position
            interp.pose.pose.position.x = p1.x + alpha * (p2.x - p1.x)
            interp.pose.pose.position.y = p1.y + alpha * (p2.y - p1.y)
            interp.pose.pose.position.z = p1.z + alpha * (p2.z - p1.z)

            # Linear velocity
            v1 = odom1.twist.twist.linear
            v2 = odom2.twist.twist.linear
            interp.twist.twist.linear.x = v1.x + alpha * (v2.x - v1.x)
            interp.twist.twist.linear.y = v1.y + alpha * (v2.y - v1.y)
            interp.twist.twist.linear.z = v1.z + alpha * (v2.z - v1.z)

            # Orientation and angular: copy latest (or later add slerp)
            interp.pose.pose.orientation = deepcopy(odom2.pose.pose.orientation)
            interp.twist.twist.angular = deepcopy(odom2.twist.twist.angular)

            points.append(interp)

        return points

    def publish_next_interpolation(self):
        if self.current_index < len(self.interpolated_msgs):
            msg = self.interpolated_msgs[self.current_index]
            self.pub.publish(msg)
            self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = OdomInterpolator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
