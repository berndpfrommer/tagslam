#!/usr/bin/env python
#
# add camerainfo to a data set
#
#

import rosbag, rospy
import sys, os, time
import argparse
import yaml
import sensor_msgs.msg



def make_caminfo_msg(c):
    m = sensor_msgs.msg.CameraInfo()
    m.height = c['image_height']
    m.width = c['image_width']
    m.distortion_model = c['distortion_model']
    m.D = c['distortion_coefficients']['data']
    m.K = c['camera_matrix']['data']
    m.R = c['rectification_matrix']['data']
    m.P = c['projection_matrix']['data']
    return m

def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'add camerainfo to bag.',
                                     formatter_class =
                                     argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--out_bag', '-o', action='store', default="out.bag",
                        help='name of the sliced output bag.')
    parser.add_argument('--start', '-s', action='store', default=0, type=float,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store',
                        default=float(sys.maxint), type=float,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--caminfo_file', '-f', action='store', default=None,
                        required=True,
                        help='name of the file with the camerainfo data.')
    parser.add_argument('--caminfo_topic', '-t', action='store', default=None,
                        required=True, help='camerainfo topic.')
    parser.add_argument('--image_topic', '-i', action='store', default=None,
                        required=True,  help='image topic.')
    parser.add_argument('--chunk_threshold', '-c', action='store',
                        default=None, type=int, help='chunk threshold in bytes.')
    parser.add_argument('bagfile')

    args    = parser.parse_args(rospy.myargv()[1:])
    print "opening bag ", args.bagfile
    in_bag  = rosbag.Bag(args.bagfile, mode = 'r')
    cthresh = args.chunk_threshold if args.chunk_threshold else in_bag.chunk_threshold
    out_bag = rosbag.Bag(args.out_bag, mode = 'w', chunk_threshold = cthresh)
    print "using chunk threshold of ", cthresh
    
    iterator = in_bag.read_messages(start_time=rospy.Time(args.start),
                                    end_time=rospy.Time(args.end))
    print "processing bag!"
    caminfo_msg = make_caminfo_msg(read_yaml(args.caminfo_file))
    t0 = time.time()
    for (topic, msg, t) in iterator:
        if (topic == args.image_topic):
            caminfo_msg.header.seq   = msg.header.seq
            caminfo_msg.header.stamp = msg.header.stamp
            caminfo_msg.header.frame_id = msg.header.frame_id
            out_bag.write(args.caminfo_topic, caminfo_msg, t)
        out_bag.write(topic, msg, t)
    out_bag.close()
    print "finished processing in ", time.time() - t0
