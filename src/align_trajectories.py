#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#
# script to align odometries vs each other, and compute ATE
#

import numpy as np
import argparse
import rosbag
import rospy
import math

import tf_conversions.posemath as pm
import numpy.linalg
from collections import defaultdict
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def align_umeyama(model, data):
    """Implementation of the paper: S. Umeyama, Least-Squares Estimation
    of Transformation Parameters Between Two Point Patterns,
    IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, 1991.
    model = s * R * data + t
    Input:
    model -- first trajectory (nx3), numpy array type
    data -- second trajectory (nx3), numpy array type
    Output:
    s -- scale factor (scalar)
    R -- rotation matrix (3x3)
    t -- translation vector (3x1)
    t_error -- translational error per point (1xn)
    """
    # originally from https://github.com/uzh-rpg/rpg_trajectory_evaluation/

    # substract mean
    mu_M = model.mean(0)
    mu_D = data.mean(0)
    model_zerocentered = model - mu_M
    data_zerocentered = data - mu_D
    n = np.shape(model)[0]

    # correlation
    C = 1.0/n*np.dot(model_zerocentered.transpose(), data_zerocentered)
    sigma2 = 1.0/n*np.multiply(data_zerocentered, data_zerocentered).sum()
    U_svd, D_svd, V_svd = np.linalg.linalg.svd(C)
    D_svd = np.diag(D_svd)
    V_svd = np.transpose(V_svd)

    S = np.eye(3)
    if(np.linalg.det(U_svd)*np.linalg.det(V_svd) < 0):
        S[2, 2] = -1

    R = np.dot(U_svd, np.dot(S, np.transpose(V_svd)))
    s = 1
    t = mu_M-s*np.dot(R, mu_D)
    return s, R, t


def read_odom(fname, topic, start_time, end_time):
    bag = rosbag.Bag(fname, 'r')
    if not bag:
        raise 'cannot open bag' + fname
    iterator = bag.read_messages(topics=[topic], start_time=start_time, end_time=end_time)
    t = []
    p = {}
    for (topic, msg, time) in iterator:
        if msg._type == 'nav_msgs/Odometry':
            T = pm.fromMsg(msg.pose.pose)
            t_sec = msg.header.stamp.to_sec()
            t.append(t_sec)
            p[t_sec] = T
    T = np.unique(np.array(t))  # discard duplicates
    output = np.zeros((T.shape[0], 16))
    for i, t in enumerate(T):
        output[i, :] = pm.toMatrix(p[t]).reshape(-1)
    print('read %d transforms from %s topic %s' % (output.shape[0], fname, topic))

    return T, output


def interpolate_3d(t_target, t, d):
    """ interpolate the data given by t, d to grid t_target """
    d_target = np.zeros((t_target.shape[0], d.shape[1]))
    for i in range(d_target.shape[1]):
        d_target[:, i] = np.interp(t_target, t, d[:, i])
    return d_target

def position_only(d):
    return d[:, (3, 7, 11)]

def plot_position(all_time_series):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for ts in all_time_series:
        t = ts[0]
        d = ts[1]
        ax.plot(d[:, 0], d[:, 1], d[:, 2])
    plt.show()

def compute_ate(t1, p1, p2):
    ate = 0
    T = 0
    for i in range(1, len(t1)):
        dt = t1[i] - t1[i-1]
        d = p2[i, :] - p1[i, :]
        l = np.sum(d * d)
        print i, dt, math.sqrt(l), d[0], d[1], d[2]
        ate += dt * l
        T += dt
    return math.sqrt(ate/ T)

def filter_position(t, p, max_z):
    idx = p[:,2] < max_z
    return t[idx], p[idx, :]

def align_odom(t1, p1, t2, p2, max_z):
    # ignore rotations for now and just align trajectories
    p1 = position_only(p1)
    t1, p1 = filter_position(t1, p1, max_z)
    p2_i = interpolate_3d(t1, t2, position_only(p2))

    # compute transform: p2_i = s * R * p1 + t
    s, R, t = align_umeyama(p2_i, p1)
    p2_aligned = np.matmul(R.T, (p2_i - t).T).T
    
    #plot_position(((t1, p1), (t1, p2_interp)))
    plot_position(((t1, p1), (t1, p2_aligned)))
    disp_1 = p1[-1, :] - p1[0, :]
    disp_2 = p2_aligned[-1, :] - p2_aligned[0,:]
    print(disp_1)
    print(disp_2)
    print(disp_1-disp_2)
    print 'dist vicon: ', np.linalg.norm(disp_1)
    print 'dist tagslam: ', np.linalg.norm(disp_2)
    #rot2s = [R.from_matrix(p2[t]) for t in t2]
    #slerp = Slerp(t2, rot12)
    #rot1s = slerp(t1)
    p2p = position_only(p2)
    disp_2_orig = p2p[-1, :] - p2p[0, :]
    print 'orig dist tagslam: ', np.linalg.norm(disp_2_orig)
    ate = compute_ate(t1, p1, p2_aligned)
    print 'ATE: ', ate

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='aligns odometry data from two different sources.')
    parser.add_argument('--verbose',  type=bool, action='store', default=False,
                        help='verbose printout')
    parser.add_argument('--topic_1', action='store', default=None, required=True,
                        help='odom topic 1 (ground truth!)')
    parser.add_argument('--topic_2', action='store', default=None, required=True,
                        help='odom topic 2')
    parser.add_argument('--start_time', action='store', type=float,
                        default=0, required=False,
                        help='start_time (in seconds from beginning of first bag)')
    parser.add_argument('--max_z', action='store', type=float,
                        default=1e10, required=False,
                        help='maximum z height for track comparison (in m)')
    parser.add_argument('--duration', action='store', type=float,
                        default=1e6, required=False,
                        help='duration (seconds)')

    parser.add_argument('bagfile1')
    parser.add_argument('bagfile2')
    args = parser.parse_args()
    
    bag = rosbag.Bag(args.bagfile1, 'r')
    if not bag:
        raise 'cannot open bag' + fname

    start_time = rospy.Time(bag.get_start_time() + args.start_time)
    end_time = rospy.Time(bag.get_start_time() + args.start_time + args.duration)
    t1, p1 = read_odom(args.bagfile1, args.topic_1, start_time, end_time)
    t2, p2 = read_odom(args.bagfile2, args.topic_2, start_time, end_time)

    align_odom(t1, p1, t2, p2, args.max_z)
