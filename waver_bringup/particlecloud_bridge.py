import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray, PoseStamped

class ParticleCloudBridge(Node):
    def __init__(self):
        super().__init__('particlecloud_bridge')
        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.callback,
            10)
        self.publisher = self.create_publisher(
            PoseArray,
            '/particlecloud',
            10)

    def callback(self, msg):
        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.poses = [pose.pose for pose in msg.particles]
        self.publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleCloudBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
