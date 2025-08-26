#!/usr/bin/env python
import rospy
import numpy as np
from cgn_ros.srv import GetGrasps
from cgn_ros.msg import Grasps
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import struct

def depth_to_point_cloud(depth, K):
    """ Convert a depth image and intrinsics to a 3D point cloud (N x 3) """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    height, width = depth.shape
    x, y = np.meshgrid(np.arange(width), np.arange(height))

    z = depth
    x3 = (x - cx) * z / fx
    y3 = (y - cy) * z / fy
    xyz = np.stack((x3, y3, z), axis=-1)

    return xyz.reshape(-1, 3)

def flatten_segmentation(seg):
    """ Flatten the segmentation map to match point cloud layout """
    return seg.flatten().astype(np.uint32)

def load_npy_file(npy_path):
    data = np.load(npy_path, allow_pickle=True).item()
    depth = data['depth']      # (480, 640)
    K = data['K']              # (3, 3)
    seg = data['seg']          # (480, 640)
    return depth, K, seg

def call_grasp_service(npy_path):
    rospy.wait_for_service('/get_grasps')
    try:
        grasp_service = rospy.ServiceProxy('/get_grasps', GetGrasps)

        # Load npy
        depth, K, seg = load_npy_file(npy_path)

        # Convert to 3D points
        points = depth_to_point_cloud(depth, K)

        # Mask out invalid points
        mask_valid = np.isfinite(points).all(axis=1)
        points = points[mask_valid]
        seg = flatten_segmentation(seg)[mask_valid]

        # Convert to ROS Point array
        # points_ros = [Point(x, y, z) for x, y, z in points.tolist()]
        points_ros = points.astype(np.float32).flatten().tolist()


        # Call service
        resp = grasp_service(points_ros, seg.tolist())

        print(f"Received {len(resp.grasps.poses)} grasps")
        for i, pose in enumerate(resp.grasps.poses[:5]):
            print(f"[{i}] Pose: {pose}, Score: {resp.grasps.scores[i]:.3f}")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('client_grasp_request')
    npy_path = '../test_data/0.npy'  # <-- CHANGE THIS PATH
    call_grasp_service(npy_path)

