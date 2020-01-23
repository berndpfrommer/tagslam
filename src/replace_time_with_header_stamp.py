#!/usr/bin/env python
#
# replace ros time stamp with header time stamp
#
# lifted from the ROS cook book: http://wiki.ros.org/rosbag/Cookbook
#
import rosbag
import argparse
import rospy

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'replace ros time with header stamp',
                                     formatter_class =
                                     argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--out_bag', '-o', action='store', default="out.bag",
                        help='name of the output bag.')
    parser.add_argument('--in_bag', '-i', action='store', default=None,
                        required=True, help='name of the input bag.')

    args    = parser.parse_args(rospy.myargv()[1:])

    with rosbag.Bag(args.out_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(args.in_bag).read_messages():
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
