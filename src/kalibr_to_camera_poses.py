#!/usr/bin/env python
#------------------------------------------------------------------------------
# convert kalibr format to camera_poses.yaml format
#
# 2019 Bernd Pfrommer

import rospy
import argparse
import copy
import yaml
import re
import numpy as np
import geometry_msgs
import sensor_msgs
import tf2_msgs
import time
import math

import read_calib


R = np.asarray(
    [ 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
      0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 
      0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 
      0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 
      0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 
      0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0])


def quaternion_to_axis_angle(q):
    a = 2.0 * math.acos(q.w)
    sqinv = 1.0 / math.sqrt(1.0 - q.w * q.w) if q.w * q.w < 1.0 - 1e-8 else 0
    aa = a * np.asarray((q.x, q.y, q.z)) * sqinv
    return aa

def rvec_tvec_to_mat(rvec, tvec):
    l = np.linalg.norm(rvec)
    n = rvec/l if l > 1e-8 else np.array([1.0, 0.0, 0.0])
    T = tf.transformations.rotation_matrix(l, n)
    T[0:3, 3] = tvec
    return T

def print_item(name, tf):
    aa = quaternion_to_axis_angle(tf.rotation)
    print "%s:" % name
    print "  pose:"
    print "    position:"
    print "      x: ", tf.translation.x
    print "      y: ", tf.translation.y
    print "      z: ", tf.translation.z
    print "    rotation:"
    print "      x: ", aa[0]
    print "      y: ", aa[1]
    print "      z: ", aa[2]
    print "    R:"
    print "      [", ('{:.8f}, '*6).format(*R[0:6])
    for i in range(0,4):
        print "       ", ('{:.8f}, '*6).format(*R[(i*6 + 6):(i*6 + 12)])
    print "       ", ('{:.8f}, '*5).format(*R[30:35]), "%.8f]" % R[35]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='convert kalibr to camera_poses.yaml')
    parser.add_argument(
        '--use_imu_tf', '-i', action='store', default=False, type=bool,
        help='use imu transform.')
    parser.add_argument(
        '--calib',  action='store', default=None, required=True,
        help='name of calibration file')

    args = parser.parse_args()

    tfs = read_calib.read_calib(args.calib, args.use_imu_tf)
    for name in sorted(tfs.keys()):
        print_item(name, tfs[name])
