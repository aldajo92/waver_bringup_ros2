import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ParticleCloudBridge(Node):
    def __init__(self):
        super().__init__('particlecloud_bridge')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.callback,
            qos_profile=sensor_qos
        )

        self.publisher = self.create_publisher(
            PoseArray,
            '/particlecloud',
            10
        )

    def callback(self, msg):
        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.poses = [p.pose for p in msg.particles]
        self.publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleCloudBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
