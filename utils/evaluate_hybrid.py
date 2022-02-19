# Compute error in estimated hybrid fisheye/stereo camera poses.

# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Gideon Billings, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This script computes the mean translation and rotation error from the ground truth
hybrid image trajectories and the estimated hybrid image trajectories.
"""

from tabulate import tabulate
import numpy as np
from math import acos, sqrt, isnan
from os import path as osp
import csv
from transforms3d.quaternions import mat2quat, qconjugate, qmult, quat2mat
from transforms3d.axangles import mat2axangle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.metrics import auc
import sys
import argparse
import associate
from plot_coordinate_frame import plotCoordinateFrame 


if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated SMIRC trajectory. 
    ''')
    parser.add_argument('stereo_file', help='estimated stereo trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('fish_file', help='estimated fisheye trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('gt_file', help='ground truth trajectory (format: timestamp fish_tx fish_ty fish_tz fish_qw fish_qx fish_qy fish_qz stereo_tx stereo_ty stereo_tz stereo_qw stereo_qx stereo_qy stereo_qz)')
    args = parser.parse_args()

    stereo_list = associate.read_file_list(args.stereo_file, False)
    fish_list = associate.read_file_list(args.fish_file, False)
    gt_list = associate.read_file_list(args.gt_file, False, time_scale=1.0/3.0)

    matches_fs = associate.associate(fish_list, stereo_list, 0, 0.1)    
    if len(matches_fs)<2:
        sys.exit("Couldn't find matching timestamp pairs between fisheye and stereo trajectory! Did you choose the correct sequence?")

    matches_fgt = associate.associate(fish_list, gt_list, 0, 0.1)    
    if len(matches_fgt)<2:
        sys.exit("Couldn't find matching timestamp pairs between fisheye and gt trajectory! Did you choose the correct sequence?")

    fish_pose = np.matrix([[float(value) for value in fish_list[a][0:]] for a,b in matches_fs])
    stereo_pose = np.matrix([[float(value) for value in stereo_list[b][0:]] for a,b in matches_fs])
    gt_pose = np.matrix([[float(value) for value in gt_list[b][0:]] for a,b in matches_fgt])

    t_diffs = []
    q_diffs = []

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    for i in range(fish_pose.shape[0]):
        # gt pose diff
        gt_t_fish = gt_pose[i,0:3].tolist()[0]
        gt_q_fish = gt_pose[i,3:7].tolist()[0] # wxyz
        gt_T_fish = np.eye(4)
        gt_T_fish[:3,:3] = quat2mat(gt_q_fish)
        gt_T_fish[:3,3] = gt_t_fish
        # invert fisheye pose to get world to fisheye tf
        gt_T_fish = np.linalg.inv(gt_T_fish)
        t_left = gt_pose[i,7:10].tolist()[0]
        q_left = gt_pose[i,10:14].tolist()[0]
        gt_T_left = np.eye(4)
        gt_T_left[:3,:3] = quat2mat(q_left)
        # this is stereo to world tf
        gt_T_left[:3,3] = t_left
        gt_T_left = np.linalg.inv(gt_T_left)

        # estimated pose diff
        t_fish = fish_pose[i,0:3].tolist()[0]
        q_fish = fish_pose[i,3:7].tolist()[0]
        q_fish[0], q_fish[1], q_fish[2], q_fish[3] = q_fish[3], q_fish[0], q_fish[1], q_fish[2] #wxyz
        T_fish = np.eye(4)
        T_fish[:3,:3] = quat2mat(q_fish)
        # this is world to fish tf
        T_fish[:3,3] = t_fish
        t_left = stereo_pose[i,0:3].tolist()[0]
        q_left = stereo_pose[i,3:7].tolist()[0]
        q_left[0], q_left[1], q_left[2], q_left[3] = q_left[3], q_left[0], q_left[1], q_left[2] #wxyz
        T_left = np.eye(4)
        T_left[:3,:3] = quat2mat(q_left)
        T_left[:3,3] = t_left

        # align gt left camera frame and SLAM left frame with origin
        gt_T_fish = np.linalg.inv(gt_T_left).dot(gt_T_fish)
        gt_T_left = np.eye(4)
        T_fish = np.linalg.inv(T_left).dot(T_fish)
        T_left = np.eye(4)

        # error is difference in fisheye frames
        t_diff = gt_T_fish[:3,3] - T_fish[:3,3]
        gt_q_fish = mat2quat(gt_T_fish[:3,:3])
        q_fish = mat2quat(T_fish[:3,:3])

        td = np.linalg.norm(t_diff)
        qd = np.degrees(2*acos(abs(np.dot(gt_q_fish,q_fish))))
        if ~np.isnan(td) and ~np.isnan(qd):
            t_diffs.append(td)
            q_diffs.append(qd)

        # ax.clear()
        # plotCoordinateFrame(ax, gt_T_left)
        # plotCoordinateFrame(ax, gt_T_fish)
        # plotCoordinateFrame(ax, T_left, linewidth=2)
        # plotCoordinateFrame(ax, T_fish, linewidth=2)
        # ax.set_xlim3d([-3,3])
        # ax.set_ylim3d([-3,3])
        # ax.set_zlim3d([-3,3])
        # ax.set_aspect('equal')
        # plt.ion()
        # plt.show()
        # plt.pause(0.001)
        # plt.waitforbuttonpress()

    rmse_t = np.sqrt(np.mean(np.square(t_diffs)))
    rmse_q = np.sqrt(np.mean(np.square(q_diffs)))

    print("RMSE trans: {}, RMSE rot: {}".format(rmse_t, rmse_q))
    print("Mean trans: {}, Mean rot: {}".format(np.mean(t_diffs), np.mean(q_diffs)))
    print("Median trans: {}, Median rot: {}".format(np.median(t_diffs), np.median(q_diffs)))
    print("{} images matched out of {}".format(len(t_diffs), len(gt_list)))