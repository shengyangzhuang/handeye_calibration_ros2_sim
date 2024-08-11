import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import tf_transformations

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # Look up the transformation from lbr/camera_link_optical to lbr/link_ee
            transform = self.tf_buffer.lookup_transform(
                'lbr/link_ee',  # target frame
                'lbr/camera_link_optical',  # source frame
                rclpy.time.Time())  # get the latest available transform

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Print the transformation (translation + rotation quaternion)
            self.get_logger().info('Translation: x={}, y={}, z={}'.format(translation.x, translation.y, translation.z))
            self.get_logger().info('Rotation (quaternion): x={}, y={}, z={}, w={}'.format(rotation.x, rotation.y, rotation.z, rotation.w))

        except Exception as e:
            self.get_logger().warn('Could not transform: {}'.format(str(e)))


def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
