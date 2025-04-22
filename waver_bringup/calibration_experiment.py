import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class CalibrationExperiment(Node):
    def __init__(self):
        super().__init__('calibration_experiment_node')

        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('linear_velocity', 0.3)

        self.target_distance = self.get_parameter('target_distance').value
        self.linear_velocity = self.get_parameter('linear_velocity').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.reset_client = self.create_client(Empty, '/reset_odometry')

        # Global state
        self.initial_position = None
        self.distance_moved = 0.0
        self.odometry_reset_done = False

        # Timers
        self.reset_timer = self.create_timer(0.5, self.reset_odometry_timer)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def reset_odometry_timer(self):
        if not self.odometry_reset_done and self.reset_client.service_is_ready():
            self.get_logger().info('Calling /reset_odometry...')
            req = Empty.Request()
            result = self.reset_client.call(req)
            if result is not None:
                self.get_logger().info('Odometry reset complete.')
                self.odometry_reset_done = True
                self.initial_position = None  # Reset initial position
                self.reset_timer.cancel()
            else:
                self.get_logger().warn('Failed to call /reset_odometry')

    def send_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        self.get_logger().info('Robot stopped.')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.initial_position is None:
            self.initial_position = (x, y)
            return

        dx = x - self.initial_position[0]
        dy = y - self.initial_position[1]
        self.distance_moved = math.sqrt(dx * dx + dy * dy)

    def control_loop(self):
        if self.odometry_reset_done:
            self.send_velocity()
            if self.distance_moved >= self.target_distance:
                self.get_logger().info(f'Target distance reached: {self.distance_moved:.2f} m')
                self.stop_robot()
                self.control_timer.cancel()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationExperiment()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
