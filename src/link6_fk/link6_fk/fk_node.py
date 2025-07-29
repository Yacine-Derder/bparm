import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import os
from ament_index_python.packages import get_package_share_directory

class FKNode(Node):
    def __init__(self):
        super().__init__('link6_fk_node')

        self.urdf_file = os.path.join(
            get_package_share_directory('link6_fk'),
            'urdf',
            '5.1.urdf'
        )

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joints = self.parse_urdf()

        self.subscription = self.create_subscription(
            JointState,
            '/bparm_joints',
            self.joint_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            '/link6_pose',
            10
        )

        self.get_logger().info("Forward kinematics node ready.")

    def parse_origin(self, origin_elem):
        if origin_elem is None:
            return np.eye(4)
        xyz = [float(x) for x in origin_elem.attrib.get('xyz', '0 0 0').split()]
        rpy = [float(r) for r in origin_elem.attrib.get('rpy', '0 0 0').split()]
        rot = R.from_euler('xyz', rpy).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = xyz
        return T

    def rot_matrix(self, axis, angle):
        axis = np.array(axis)
        axis = axis / np.linalg.norm(axis)
        return R.from_rotvec(axis * angle).as_matrix()

    def parse_urdf(self):
        tree = ET.parse(self.urdf_file)
        root = tree.getroot()
        joints = {}
        for joint in root.findall('joint'):
            name = joint.attrib['name']
            if name not in self.joint_names:
                continue
            joint_type = joint.attrib.get('type', 'fixed')
            axis_elem = joint.find('axis')
            axis = [1, 0, 0]
            if axis_elem is not None:
                axis = [float(x) for x in axis_elem.attrib['xyz'].split()]
            origin = self.parse_origin(joint.find('origin'))
            child_link = joint.find('child').attrib['link']
            parent_link = joint.find('parent').attrib['link']
            joints[name] = {
                'type': joint_type,
                'axis': axis,
                'origin': origin,
                'child_link': child_link,
                'parent_link': parent_link,
            }
        return joints

    def compute_fk(self, joint_positions_dict):
        parent_to_joint = {
            j['parent_link']: (name, j)
            for name, j in self.joints.items()
        }

        current_link = 'base_link'
        target_link = 'link6'
        T = np.eye(4)

        while current_link != target_link:
            if current_link not in parent_to_joint:
                self.get_logger().error(f"No joint from {current_link} → ?")
                return None
            jname, jdata = parent_to_joint[current_link]
            T = T @ jdata['origin']
            q = joint_positions_dict.get(jname, 0.0)
            if jdata['type'] in ('revolute', 'continuous'):
                R_joint = np.eye(4)
                R_joint[:3, :3] = self.rot_matrix(jdata['axis'], q)
                T = T @ R_joint
            current_link = jdata['child_link']

        return T

    def joint_callback(self, msg: JointState):
        joint_positions = dict(zip(msg.name, msg.position))
        T = self.compute_fk(joint_positions)
        if T is None:
            return

        pos = T[:3, 3]
        rot = R.from_matrix(T[:3, :3])
        quat = rot.as_quat()  # [x, y, z, w]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'  # or your robot's base frame

        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.publisher.publish(pose_msg)
        self.get_logger().info(f"\nPublished pose of link6: {pose_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
