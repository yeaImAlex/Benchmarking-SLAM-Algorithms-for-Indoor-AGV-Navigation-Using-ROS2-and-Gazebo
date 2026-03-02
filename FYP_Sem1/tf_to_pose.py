import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, TransformException
from geometry_msgs.msg import PoseStamped

class TfToPose(Node):
    def __init__(self):
        super().__init__('tf_to_pose_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(PoseStamped, '/pose', 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info('TF to Pose relay started')

    def on_timer(self):
        try:
            #look up the transform from map to base link
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation

            self.publisher_.publish(msg)
        except TransformException as ex:
            pass

    def main(args=None):
        rclpy.init(args=args)
        node = TfToPose()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

