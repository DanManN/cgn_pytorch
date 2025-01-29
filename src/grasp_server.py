#!/usr/bin/env python
import copy
import random

import torch
import numpy as np
from cgn_pytorch import CGN
from torch_geometric.nn import fps
import cgn_pytorch.util.config_utils as config_utils

import rospy
from rospkg import RosPack
import transformations as tf
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, PoseStamped

from cgn_ros.msg import Grasps
from cgn_ros.srv import GetGrasps, GetGraspsResponse


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
    translation = list(tf.translation_from_matrix(matrix))
    quaternion = list(tf.quaternion_from_matrix(matrix))
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
    transform = tf.quaternion_matrix(quaternion)
    transform[:3, 3] = translation
    return transform


class GraspPlanner():

    def __init__(self):
        rp = RosPack()
        config_path = rp.get_path('cgn_ros') + '/cgn_pytorch/checkpoints'
        load_path = rp.get_path(
            'cgn_ros'
        ) + '/cgn_pytorch/checkpoints/current.pth'

        ### Initialize net
        cnet, optim, config = self.initialize_net(config_path, True, load_path)
        self.model = cnet

    def initialize_net(self, config_file, load_model, save_path):
        print('initializing net')
        torch.cuda.empty_cache()
        config_dict = config_utils.load_config(config_file)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        model = CGN(config_dict, device).to(device)
        optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
        if load_model:
            print('loading model')
            checkpoint = torch.load(save_path, map_location=device)
            model.load_state_dict(checkpoint['state_dict'])
            optimizer.load_state_dict(checkpoint['optimizer'])

        return model, optimizer, config_dict

    def cgn_infer(self, cgn, pcd, obj_mask=None, threshold=0.5):
        cgn.eval()
        # if pcd.shape[0] > 20000:
        #     downsample = np.array(random.sample(range(pcd.shape[0] - 1), 20000))
        # else:
        #     downsample = np.arange(20000)
        # pcd = pcd[downsample, :]

        pcd = torch.Tensor(pcd).to(dtype=torch.float32).to(cgn.device)
        batch = torch.zeros(pcd.shape[0]).to(dtype=torch.int64).to(cgn.device)
        idx = fps(pcd, batch, 2048 / pcd.shape[0]).to(cgn.device)
        #idx = torch.linspace(0, pcd.shape[0]-1, 2048).to(dtype=torch.int64).to(cgn.device)

        if obj_mask is not None:
            # obj_mask = torch.Tensor(obj_mask[downsample]).to(cgn.device)
            obj_mask = torch.Tensor(obj_mask).to(cgn.device)
            obj_mask = obj_mask[idx]
        else:
            obj_mask = torch.ones(idx.shape[0]).to(cgn.device)

        result = cgn(pcd[:, 3:], pos=pcd[:, :3], batch=batch, idx=idx)
        points, pred_grasps, confidence, pred_widths, _, pred_collide = result
        sig = torch.nn.Sigmoid()
        confidence = sig(confidence)
        confidence = confidence.reshape(-1, 1)
        pred_grasps = torch.flatten(pred_grasps, start_dim=0, end_dim=1)
        pred_grasps = pred_grasps.detach().cpu().numpy()

        confidence = confidence.detach().cpu().numpy()
        confidence = (obj_mask.detach().cpu().numpy() * confidence).reshape(-1)
        pred_widths = torch.flatten(pred_widths, start_dim=0, end_dim=1)
        pred_widths = pred_widths.detach().cpu().numpy()
        points = torch.flatten(points, start_dim=0, end_dim=1)
        points = points.detach().cpu().numpy()

        success_mask = (confidence > threshold).nonzero()[0]
        if len(success_mask) == 0:
            print('failed to find successful grasps')
            raise Exception

        return (
            pred_grasps[success_mask],
            confidence[success_mask],
            points[success_mask],
        )

    def get_grasp_poses(
        self,
        points,  # std_msgs/Float32MultiArray
        target_mask,  # bool[]
        threshold,  # float32
    ):
        result = self.cgn_infer(self.model, points, target_mask, threshold)
        grasps, scores, points = result

        pose_list = []
        score_list = []
        sample_list = []
        for pose, score, sample in zip(grasps, scores, points):

            #swap x and z axes
            # pose = np.matmul(
            #     pose,
            #     [
            #         [0., 0., 1., 0.],
            #         [0, -1., 0., 0.],
            #         [1., 0., 0., 0.],
            #         [0., 0., 0., 1.],
            #     ],
            # )

            pose_list.append(matrix_to_pose(pose))
            score_list.append(score)
            sample_list.append(sample)
        return pose_list, score_list, sample_list

    def handle_grasp_request(self, req):
        grasps, scores, samples = self.get_grasp_poses(
            req.points,
            req.cloud,
            req.camera_position,
        )
        grasps_msg = Grasps()
        grasps_msg.poses = grasps
        grasps_msg.scores = scores
        grasps_msg.samples = samples

        return GetGraspsResponse(grasps_msg)


if __name__ == "__main__":
    rospy.init_node('cgn_ros_grasp_server')
    grasp_planner = GraspPlanner()
    s0 = rospy.Service(
        'get_grasps', GetGrasps, grasp_planner.handle_grasp_request
    )
    print("Ready to generate grasps...")
    rospy.spin()
