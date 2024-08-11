import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import pandas as pd
from pathlib import Path
import yaml


# Create a QoS profile for subscribing to /tf_static
qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# ArUco dictionary lookup
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class PublishMarkerNode(Node):
    def __init__(self):
        super().__init__('publish_marker_node')

        with open('src/handeye_sim/config.yaml', 'r') as file:
            config = yaml.safe_load(file)

        aruco_dictionary_name = config["aruco_dictionary_name"]
        self.aruco_marker_name = config["aruco_marker_name"]
        self.aruco_marker_side_length = config["aruco_marker_side_length"]
        self.camera_calibration_parameters_filename = config["camera_calibration_parameters_filename"]
        self.image_topic = config["image_topic"]
        self.calculated_camera_optical_frame_name = config["calculated_camera_optical_frame_name"]
        self.base_link = config["base_link"]
        self.world_frame = config["world_frame"]
        self.link_order_publish_marker = config["link_order_publish_marker"]


        if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
            self.get_logger().error(f"ArUCo tag of '{aruco_dictionary_name}' is not supported")
            return

        cv_file = cv2.FileStorage(self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        cv_file.release()

        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()

        self.subscription = self.create_subscription(Image, self.image_topic, self.listener_callback, 10)

        self.pose_publisher = self.create_publisher(Pose, 'aruco_marker_pose', 10)
        self.marker_pub = self.create_publisher(Marker,'marker',10)

        self.bridge = CvBridge()

        self.subscription_tf = self.create_subscription(TFMessage, '/tf', self.listener_callback_tf, 10)
        self.subscription_tf_static = self.create_subscription(TFMessage,'/tf_static', self.listener_callback_tf_static, qos_profile)
        self.transformations = {}

    def listener_callback_tf(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id and transform.header.frame_id:
                self.transformations[(transform.header.frame_id, transform.child_frame_id)] = transform

    def listener_callback_tf_static(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id and transform.header.frame_id:
                self.transformations[(transform.header.frame_id, transform.child_frame_id)] = transform
        print("Subcribed to /tf_static successfully")
    
    def quaternion_to_rotation_matrix(self, x, y, z, w):
        return R.from_quat([x, y, z, w]).as_matrix()

    def get_full_transformation_matrix(self):
        T = np.eye(4)  # Start with the identity matrix
        for (frame_id, child_frame_id) in self.link_order_publish_marker:
            if (frame_id, child_frame_id) in self.transformations:
                trans = self.transformations[(frame_id, child_frame_id)].transform
                translation = [trans.translation.x, trans.translation.y, trans.translation.z]
                rotation = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
                T_local = np.eye(4)
                T_local[:3, :3] = self.quaternion_to_rotation_matrix(*rotation)
                T_local[:3, 3] = translation
                T = np.dot(T, T_local)

        T_inv = np.linalg.inv(T)
        # T is lbr/camera_link_optical points to world
        return T

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data)
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_side_length, self.mtx, self.dst)
            data = []
            
            for i in range(len(marker_ids)):
                R_marker2camera = cv2.Rodrigues(rvecs[i][0])[0]
                t_marker2camera = tvecs[i][0]
                T_marker2camera = np.zeros((4,4))
                T_marker2camera[:3, :3] = R_marker2camera
                T_marker2camera[:3, 3] = t_marker2camera
                T_marker2camera[3, 3] = 1
                T_camera2world = self.get_full_transformation_matrix()
                T_marker2world = np.dot(T_camera2world, T_marker2camera)
                R_marker2world = T_marker2world[:3, :3]
                t_marker2world = T_marker2world[:3, 3]
                r = R.from_matrix(R_marker2world)
                quat = r.as_quat()

                # Create Pose message
                pose_msg = Pose()
                pose_msg.position.x = t_marker2world[0]
                pose_msg.position.y = t_marker2world[1]
                pose_msg.position.z = t_marker2world[2]
                pose_msg.orientation.x = quat[0]
                pose_msg.orientation.y = quat[1]
                pose_msg.orientation.z = quat[2]
                pose_msg.orientation.w = quat[3]

                # Publish the Pose
                self.pose_publisher.publish(pose_msg)


                # Log the Pose message as a string
                pose_info = f"Position: ({pose_msg.position.x}, {pose_msg.position.y}, {pose_msg.position.z}), " \
                            f"Orientation: ({pose_msg.orientation.x}, {pose_msg.orientation.y}, {pose_msg.orientation.z}, {pose_msg.orientation.w})"
                #self.get_logger().info(pose_info)

                # Publish visualization marker
                marker = Marker()
                marker.header.frame_id = self.world_frame  # or your world frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "aruco_marker_visualization"
                marker.id = int(marker_ids[i])
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = t_marker2world[0]
                marker.pose.position.y = t_marker2world[1]
                marker.pose.position.z = t_marker2world[2]
                marker.pose.orientation.x = quat[0]
                marker.pose.orientation.y = quat[1]
                marker.pose.orientation.z = quat[2]
                marker.pose.orientation.w = quat[3]
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.01
                marker.color.a = 1.0 
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                self.marker_pub.publish(marker)

                data.append([
                    pose_msg.position.x,
                    pose_msg.position.y,
                    pose_msg.position.z,
                    pose_msg.orientation.x,
                    pose_msg.orientation.y,
                    pose_msg.orientation.z,
                    pose_msg.orientation.w

                ])

def main(args=None):
    rclpy.init(args=args)
    publish_marker_node = PublishMarkerNode()
    try:
        rclpy.spin(publish_marker_node)
    finally:
        publish_marker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()