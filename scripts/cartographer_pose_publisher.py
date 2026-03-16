# Save as: cartographer_pose_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

class PosePublisher(Node):
    def __init__(self):
        super().__init__('cartographer_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/cartographer_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.publish_pose)
        self.get_logger().info("Publishing /cartographer_pose from TF...")

    def publish_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = trans.transform.translation.x
            msg.pose.pose.position.y = trans.transform.translation.y
            msg.pose.pose.orientation = trans.transform.rotation
            self.publisher.publish(msg)
        except Exception:
            pass

def main():
    rclpy.init()
    rclpy.spin(PosePublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
