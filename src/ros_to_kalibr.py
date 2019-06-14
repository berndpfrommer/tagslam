#!/usr/bin/python
#
# script to convert ROS calibration format to Kalibr/TagSLAM style

import yaml
import argparse
import sys


def write_cameras_file(c, f):
    K = c['camera_matrix']['data']
    D = c["distortion_coefficients"]["data"]
    f.write("cam0:\n")
    f.write("  camera_model: pinhole\n")
    f.write("  intrinsics: [%.5f, %.5f, %.5f, %.5f]\n" % (K[0],K[4],K[2],K[5]))
    f.write("  distortion_model: %s\n" % c["distortion_model"])
    f.write("  distortion_coeffs: [%.5f, %.5f, %.5f, %.5f, %.5f]\n" % tuple(D))
    f.write("  resolution: [%d, %d]\n" % (c["image_width"], c["image_height"]))
    f.write("  tagtopic: /%s/tags\n" % (c["camera_name"]))
    f.write("  rig_body: XXX_NAME_OF_RIG_BODY\n")
    
def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='convert ROS style format to calibr.')
    parser.add_argument('--in_file',  '-i', action='store',
                        required=True, help='ROS calibration input yaml file')
    parser.add_argument('--out_file', '-o', action='store',
                        required=True, help='TagSLAM output file cameras.yaml')
    args = parser.parse_args()
    ros_calib = read_yaml(args.in_file)
    with open(args.out_file, 'w') as f:
        write_cameras_file(ros_calib, f)
