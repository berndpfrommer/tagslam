#!/usr/bin/env python
#------------------------------------------------------------------------------
# convert camera_poses.yaml to kalibr format
#
# 2019 Bernd Pfrommer

import rospy
import tf
import argparse
import yaml
import numpy as np

def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)

def rvec_tvec_to_mat(rvec, tvec):
    l = np.linalg.norm(rvec)
    n = rvec/l if l > 1e-8 else np.array([1.0, 0.0, 0.0])
    T = tf.transformations.rotation_matrix(l, n)
    T[0:3, 3] = tvec
    return T

def print_tf(T):
    for i in range(0,4):
        x = T[i,:]
        print '  - [%15.12f, %15.12f, %15.12f, %15.12f]' % \
            (x[0], x[1], x[2], x[3])
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='convert camera_poses.yaml to kalibr transform')
    parser.add_argument(
        '--camera_poses', '-c',  action='store', default=None, required=True,
        help='name of camera_poses.yaml file')

    args = parser.parse_args()

    y = read_yaml(args.camera_poses)
    T_w_cnm1 = np.eye(4)
    for cam in sorted(y.keys()):
        p = y[cam]['pose']['position']
        pos = np.asarray([p['x'], p['y'], p['z']])
        r = y[cam]['pose']['rotation']
        rvec = np.asarray([r['x'], r['y'], r['z']])
        T_w_cn = rvec_tvec_to_mat(rvec, pos)
        print cam
        print '  T_cn_cnm1:'
        print_tf(np.matmul(np.linalg.inv(T_w_cn),T_w_cnm1))
        T_w_cnm1 = T_w_cn
