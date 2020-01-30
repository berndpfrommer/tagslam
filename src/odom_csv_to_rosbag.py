#!/usr/bin/env python
#
# read csv file with odom and convert it to rosbag for playing it in rviz
#
# The format of the cvs file is:
#
# ros_time_stamp px py pz qw qx qy qz
#

import rosbag, rospy
import argparse
import sensor_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import tf2_msgs.msg


def make_odom_msg(seq, parent_frame_id, child_frame_id, t, q, p):
    m = nav_msgs.msg.Odometry()
    m.header.seq = seq
    m.header.stamp = t
    m.header.frame_id = parent_frame_id
    m.pose.pose.position.x = float(p[0])
    m.pose.pose.position.y = float(p[1])
    m.pose.pose.position.z = float(p[2])
    m.pose.pose.orientation.w = float(q[0])
    m.pose.pose.orientation.x = float(q[1])
    m.pose.pose.orientation.y = float(q[2])
    m.pose.pose.orientation.z = float(q[3])
    return m

def make_tf_msg(seq, parent_frame_id, child_frame_id, t, q, p):
    m = tf2_msgs.msg.TFMessage()
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.seq = seq
    tf.header.stamp = t
    tf.header.frame_id = parent_frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = float(p[0])
    tf.transform.translation.y = float(p[1])
    tf.transform.translation.z = float(p[2])
    tf.transform.rotation.w = float(q[0])
    tf.transform.rotation.x = float(q[1])
    tf.transform.rotation.y = float(q[2])
    tf.transform.rotation.z = float(q[3])
    m.transforms.append(tf)
    return m

def main(args):
    print 'opening bag: ', args.out_bag, ' for writing'
    out_bag = rosbag.Bag(args.out_bag, mode = 'w')

    for fname in args.csv_files:
        label = fname.split('.')[0].split('_')[1]
        print 'writing data for label: ', label
        parent_frame = args.parent_frame_id
        child_frame = args.child_frame_id + '_' + label
        odom_topic = args.odom_topic + '_' + label
        with open(fname) as csv_file:
            seq = 0
            for line in csv_file:
                line = line.strip().split()
                ts = line[0]
                p  = line[1:4]
                q  = line[4:8]
                t = rospy.Time(float(ts))
                odom_msg = make_odom_msg(seq, parent_frame,
                                         child_frame, t, q, p)
                out_bag.write(odom_topic, odom_msg, t)
                tf_msg = make_tf_msg(seq, parent_frame,
                                     child_frame, t, q, p)
                out_bag.write('/tf', tf_msg, t)
                seq = seq + 1
    out_bag.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description = 'convert odom from csv to rosbag',
        formatter_class = argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--out_bag', '-o', action='store', default="csv_out.bag",
                        help='name of the sliced output bag.')
    parser.add_argument('--odom_topic', '-t', action='store', default='/csv/odom',
                        required=False, help='odometry topic.')
    parser.add_argument('--child_frame_id', '-c', action='store', default='child',
                        required=False,  help='child frame id.')
    parser.add_argument('--parent_frame_id', '-p', action='store', default='parent',
                        required=False,  help='parent frame id.')
    parser.add_argument('csv_files', nargs='*')

    args = parser.parse_args(rospy.myargv()[1:])
    main(args)
