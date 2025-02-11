#!/usr/bin/env python
import sys
import copy
import random

import numpy as np

import tensorflow.compat.v1 as tf

tf.disable_eager_execution()
physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)

sys.path.append(
    '/home/user/cgn_ws/src/cgn_ros/contact_graspnet/contact_graspnet'
)
import config_utils
from contact_grasp_estimator import GraspEstimator
from visualization_utils import visualize_grasps

import rospy
from rospkg import RosPack
import transformations as tfs
from std_msgs.msg import Int64
from geometry_msgs.msg import Point, Pose, PoseStamped

from cgn_ros.msg import Grasps
from cgn_ros.srv import GetGrasps, GetGraspsResponse

# from grasp_plotter import GraspPlotter


def list_to_pose(pose_list):
    pose_msg = Pose()
    pose_msg.position.x = pose_list[0]
    pose_msg.position.y = pose_list[1]
    pose_msg.position.z = pose_list[2]
    pose_msg.orientation.x = pose_list[3]
    pose_msg.orientation.y = pose_list[4]
    pose_msg.orientation.z = pose_list[5]
    pose_msg.orientation.w = pose_list[6]
    return pose_msg


def matrix_to_pose(matrix):
    translation = list(tfs.translation_from_matrix(matrix))
    quaternion = list(tfs.quaternion_from_matrix(matrix))
    pose_list = translation + quaternion[1:] + quaternion[:1]
    # pose_list = translation + quaternion
    return list_to_pose(pose_list)


def pose_to_list(pose_msg):
    pose = []
    pose.append(pose_msg.position.x)
    pose.append(pose_msg.position.y)
    pose.append(pose_msg.position.z)
    pose.append(pose_msg.orientation.x)
    pose.append(pose_msg.orientation.y)
    pose.append(pose_msg.orientation.z)
    pose.append(pose_msg.orientation.w)
    return pose


def pose_to_matrix(pose_msg):
    pose_list = pose_to_list(pose_msg)
    translation = pose_list[:3]
    quaternion = pose_list[6:] + pose_list[3:6]
    transform = tfs.quaternion_matrix(quaternion)
    transform[:3, 3] = translation
    return transform


class GraspPlanner():

    def __init__(self):
        rp = RosPack()
        pkg_path = rp.get_path('cgn_ros')
        ckpt_dir = pkg_path + '/checkpoints/scene_test_2048_bs3_hor_sigma_001/'

        global_config = config_utils.load_config(
            ckpt_dir,
            batch_size=5,
            # arg_configs=['TEST.second_thres:0.0', 'TEST.first_thres:0.0']
        )

        # Build the model
        grasp_estimator = GraspEstimator(global_config)
        grasp_estimator.build_network()

        # Add ops to save and restore all the variables.
        saver = tf.train.Saver(save_relative_paths=True)

        # Create a session
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        sess = tf.Session(config=config)

        # Load weights
        grasp_estimator.load_weights(sess, saver, ckpt_dir, mode='test')

        # Set Class variables
        self.sess = sess
        self.grasp_estimator = grasp_estimator
        self.rotate_z = tfs.rotation_matrix(np.pi, [0, 0, 1])

        # init visualizer
        # self.grasp_plotter = GraspPlotter()

    def get_grasp_poses(
        self,
        points,  # std_msgs/Float32MultiArray
        target_points,  # std_msgs/Float32MultiArray
    ):
        points = np.array(points.data, dtype=np.float32).reshape(-1, 3)
        target_points = np.array(target_points.data, dtype=np.float32)
        target_points = target_points.reshape(-1, 3)

        # transform to trained coordinate frame
        frame_rotate = np.array(
            [
                [0, -1., 0., 0.],
                [0., 0, -1., 0.],
                [1., 0., 0., 0.],
                [0., 0., 0., 1.],
            ]
        )
        points = (frame_rotate[:3, :3] @ points.T).T
        target_points = (frame_rotate[:3, :3] @ target_points.T).T

        pc_segments = {0: target_points}
        grasps, scores, samples, _ = self.grasp_estimator.predict_scene_grasps(
            self.sess,
            points,
            pc_segments=pc_segments,
            local_regions=True,
            filter_grasps=True,
            forward_passes=5
        )

        print(grasps.keys())
        print(scores.keys())
        print(samples.keys())

        pose_list = []
        score_list = []
        sample_list = []
        for pose, score, sample in zip(grasps[0], scores[0], samples[0]):

            # transform back to input frame
            pose = frame_rotate.T @ pose

            # swap x and y axes
            pose = pose @ np.array(
                [
                    [0., 1., 0., 0.],
                    [-1, 0., 0., 0.],
                    [0., 0., 1., 0.],
                    [0., 0., 0., 1.],
                ]
            )

            # rotate around symmetric axis for another possible grasp
            pose_rot = pose @ self.rotate_z

            for pose_i in [pose, pose_rot]:
                pose_list.append(matrix_to_pose(pose_i))
                score_list.append(score)
                sample_list.append(Point(sample[0], sample[1], sample[2]))

        return pose_list, score_list, sample_list

    def handle_grasp_request(self, req):
        grasps, scores, samples = self.get_grasp_poses(
            req.points,
            req.target_points,
        )
        grasps_msg = Grasps()
        grasps_msg.poses = grasps
        grasps_msg.scores = scores
        grasps_msg.samples = samples

        # self.grasp_plotter.draw_grasps(grasps, scores, frame='world')

        return GetGraspsResponse(grasps_msg)


if __name__ == "__main__":
    rospy.init_node('cgn_ros_grasp_server')
    grasp_planner = GraspPlanner()
    s0 = rospy.Service(
        'get_grasps', GetGrasps, grasp_planner.handle_grasp_request
    )
    print("Ready to generate grasps...")
    rospy.spin()
