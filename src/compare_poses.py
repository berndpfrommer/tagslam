#!/usr/bin/python
#
# script to compare poses from odom and transforms from two different bags
#
# The first bag is the reference bag, the second one the one to be tested
# against the first one.
#
# example run:
#
# ./compare_poses.py odom/body_rig ~/.ros/t1.bag ~/.ros/t2.bag  -f2 body_
#

import rosbag, rospy, numpy as np
import tf
import argparse
import tf_conversions.posemath as pm

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def compare_transforms(t1, p1, f1, t2, p2, f2):
    i2 = 0
    for i1 in range(0, t1.shape[0]):
        t = t1[i1]
        while t2[i2] < t and i2 < t2.shape[0]:
            i2 = i2 + 1
        if i2 == t2.shape[0]:
            print 'reached end searching for ', t
            break
        if t2[i2] != t:
            print '%s --' % (str(t))
            print 'no matching time stamp for ', t
        else:
#            print 'found matching time stamp ', t2[i2], ' ', t
            for tf1 in p1[i1]:
                foundMatch = False
                for tf2 in p2[i2]:
#                    print ' trying %s -> %s ' % (f2+tf2[0], tf2[1])
                    if f1+tf1[0] == f2+tf2[0] and tf1[1] == tf2[1]:
                        d = np.matmul(np.linalg.inv(tf1[2]),tf2[2])
                        angle, n, pt = tf.transformations.rotation_from_matrix(d)
                        print '%s %-12s - %-12s ang: %8.5f  disp: %8.5f' % (str(t), tf1[0], tf1[1], angle, np.linalg.norm(d[3,0:3]))
                        foundMatch = True
                if not foundMatch:
                    pass
#                    print ' no match found for transform %s -> %s ' % (tf1[0], tf1[1])
            
def compare_poses(t1, p1, t2, p2):
    i2 = 0
    for i1 in range(0, t1.shape[0]):
        t = t1[i1]
        while t2[i2] < t and i2 < t2.shape[0]:
            i2 = i2 + 1
        if i2 == t2.shape[0]:
            print 'reached end searching for ', t
            break
        if t2[i2] != t:
            print '%s --' % (str(t))
            print 'no matching time stamp for ', t
        else:
            p1inv = p1[i1].Inverse()
            delta = p1inv * p2[i2]
            print '%s rot angle: %8.5f  disp: %8.5f' % (str(t), delta.M.GetRotAngle()[0], delta.p.Norm())

def read_poses(fname, topic):
    bag = rosbag.Bag(fname, 'r')
    if not bag:
        raise 'cannot open bag' + fname
    iterator = bag.read_messages(topics=topic)
    t = []
    p = []
    for (topic, msg, time) in iterator:
        if msg._type == 'nav_msgs/Odometry':
            T = pm.fromMsg(msg.pose.pose)
            t.append(msg.header.stamp)
            p.append(T)
    print 'read %d poses from %s' % (len(t), fname)
    return np.array(t), np.array(p)

def read_tf(fname):
    bag = rosbag.Bag(fname, 'r')
    if not bag:
        raise 'cannot open bag' + fname
    iterator = bag.read_messages(topics='/tf')
    t = []
    p = []
    for (topic, msg, time) in iterator:
        if msg._type == 'tf/tfMessage':
            t.append(time)
            tfs = []
            for i in msg.transforms:
                tr = i.transform.translation
                q  = i.transform.rotation
                m  = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
                m[0:3,3] = [tr.x, tr.y, tr.z]
                tfs.append([i.header.frame_id, i.child_frame_id, m])
            p.append(tfs)
    print 'read %d transforms from %s' % (len(t), fname)
    return np.array(t), np.array(p)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='compare poses from two different bags.')
    parser.add_argument('--frame1_prefix', '-f1', action='store', default="",
                        help='prefix for bag1 transform')
    parser.add_argument('--frame2_prefix', '-f2', action='store', default="",
                        help='prefix for bag2 transform')

    parser.add_argument('topic')
    parser.add_argument('bagfile1')
    parser.add_argument('bagfile2')
    args = parser.parse_args()

    t1, p1 = read_poses(args.bagfile1, args.topic)
    t2, p2 = read_poses(args.bagfile2, args.topic)
    compare_poses(t1, p1, t2, p2)
    t1, p1 = read_tf(args.bagfile1)
    t2, p2 = read_tf(args.bagfile2)
    compare_transforms(t1, p1, args.frame1_prefix, t2, p2, args.frame2_prefix)
