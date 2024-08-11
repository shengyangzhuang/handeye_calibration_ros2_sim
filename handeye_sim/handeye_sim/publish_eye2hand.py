import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
import tf_transformations

import numpy as np
import yaml

# Create a QoS profile for subscribing to /tf_static
qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class TransformPublisher(Node):

    def __init__(self):
        super().__init__('transform_publisher')

        with open('src/handeye_sim/config.yaml', 'r') as file:
            config = yaml.safe_load(file)
        self.handeye_result_file_name = config["handeye_result_file_name"]
        self.base_link = config["base_link"]
        self.ee_link = config["ee_link"]
        self.world_frame = config["world_frame"]
        self.calculated_camera_optical_frame_name = config["calculated_camera_optical_frame_name"]
        self.link_order_publish_eye2hand = config["link_order_publish_eye2hand"]

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Load transformation from YAML file
        with open(self.handeye_result_file_name, 'r') as file_2:
            hand_eye_data = yaml.safe_load(file_2)
            if isinstance(hand_eye_data, list) and len(hand_eye_data) > 1:
                hand_eye_data = hand_eye_data[-1]
            elif isinstance(hand_eye_data, list):
                hand_eye_data = hand_eye_data[0]

        T = np.eye(4)
        T[:3, :3] = np.array(hand_eye_data['rotation']).reshape((3, 3))
        T[:3, 3] = np.array(hand_eye_data['translation']).reshape((3,))

        self.translation = np.array(hand_eye_data['translation']).reshape((3, 1))
        self.rotation = np.array(hand_eye_data['rotation']).reshape((3, 3))
        self.handeye_quaternion = tf_transformations.quaternion_from_matrix(T)

        print(f'rotation: {self.rotation}')
        print(f'translation: {self.translation}')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_handeye_transform)

        self.subscription_tf = self.create_subscription(TFMessage, '/tf', self.listener_callback_tf, 10)
        self.subscription_tf_static = self.create_subscription(TFMessage,'/tf_static', self.listener_callback_tf_static, qos_profile)

        self.transformations = {}

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """ Convert a quaternion into a full three-dimensional rotation matrix. """
        return R.from_quat([x, y, z, w]).as_matrix()

    def listener_callback_tf(self, msg):
        """ Handle incoming transform messages. """
        for transform in msg.transforms:
            if transform.child_frame_id and transform.header.frame_id:
                self.transformations[(transform.header.frame_id, transform.child_frame_id)] = transform

    def listener_callback_tf_static(self, msg):
        """ Handle incoming transform messages. """
        for transform in msg.transforms:
            if transform.child_frame_id and transform.header.frame_id:
                self.transformations[(transform.header.frame_id, transform.child_frame_id)] = transform
        print("Subcribed to /tf_static successfully")


    def get_full_transformation_matrix(self):
        T = np.eye(4)  # Start with the identity matrix
        for (frame_id, child_frame_id) in self.link_order_publish_eye2hand:
            if (frame_id, child_frame_id) in self.transformations:
                trans = self.transformations[(frame_id, child_frame_id)].transform
                translation = [trans.translation.x, trans.translation.y, trans.translation.z]
                rotation = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
                T_local = np.eye(4)
                T_local[:3, :3] = self.quaternion_to_rotation_matrix(*rotation)
                T_local[:3, 3] = translation
                T = np.dot(T, T_local)

        T_inv = np.linalg.inv(T)
        # T is ee points to link_0
        return T


    def publish_transform(self, translation_vector, rotation_matrix, frame_id, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        # Set the translation
        t.transform.translation.x = translation_vector[0]
        t.transform.translation.y = translation_vector[1]
        t.transform.translation.z = translation_vector[2]

        # Convert the rotation matrix to a quaternion
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation_vector
        quaternion = tf_transformations.quaternion_from_matrix(transformation_matrix)

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_handeye_transform(self):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = self.ee_link
        transform_msg.child_frame_id = self.calculated_camera_optical_frame_name
        transform_msg.transform.translation.x = self.translation[0, 0]
        transform_msg.transform.translation.y = self.translation[1, 0]
        transform_msg.transform.translation.z = self.translation[2, 0]
        transform_msg.transform.rotation.x = self.handeye_quaternion[0]
        transform_msg.transform.rotation.y = self.handeye_quaternion[1]
        transform_msg.transform.rotation.z = self.handeye_quaternion[2]
        transform_msg.transform.rotation.w = self.handeye_quaternion[3]

        self.tf_broadcaster.sendTransform(transform_msg)

        # Send the transform from camera_sim to world
        Tc = self.get_full_transformation_matrix()
        rc = Tc[:3, :3]
        tc = Tc[:3, 3]
        self.publish_transform(tc, rc, self.world_frame, self.ee_link)


def main(args=None):
    rclpy.init(args=args)

    transform_publisher = TransformPublisher()

    rclpy.spin(transform_publisher)

    transform_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
