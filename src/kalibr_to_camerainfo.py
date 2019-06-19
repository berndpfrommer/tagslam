#!/usr/bin/env python

import yaml
import argparse
import numpy as np

def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)

def write_yaml(ydict, fname):
    with open(fname, 'w') as f:
        try:
            yaml.dump(ydict, f)
        except yaml.YAMLError as y:
            print("Error:", y)

def kalibr_to_camerainfo(k):
    y = {}
    intr = k['intrinsics']
    intrinsics = np.array([[intr[0], 0, intr[2]],[0, intr[1], intr[3]],[0,0,1]])

    distortion_model = k['distortion_model']
    if (distortion_model == 'radtan'):
        distortion_model = 'plumb_bob'
    
    y['image_width']  = k['resolution'][0]
    y['image_height'] = k['resolution'][1]
    y['camera_name']  = k['rostopic']
    y['camera_matrix'] = {'rows': 3,
                          'cols': 3,
                          'data': intrinsics.flatten().tolist()}
    y['distortion_model'] = distortion_model
    y['distortion_coefficients'] = {'rows': 1,
                                    'cols': 4,
                                    'data': k['distortion_coeffs']}
    y['rectification_matrix'] = {'rows': 3,
                                 'cols': 3,
                                 'data': np.eye(3).flatten().tolist()}
    y['projection_matrix'] = {'rows': 3,
                              'cols': 4,
                              'data': np.hstack([intrinsics,np.array([[0],[0],[0]])]).flatten().tolist()}
    return y


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True,
                        help="input file in Kalibr format")
    parser.add_argument("-o", "--output", required=True,
                        help="output file in opencv/camerainfo format")
    parser.add_argument("-c", "--camera", default="cam0",
                        help="name of camera to extract from Kalibr file")
    args = parser.parse_args()

    print args.input
    kalibr_dict = read_yaml(args.input)
    camerainfo_dict = kalibr_to_camerainfo(kalibr_dict[args.camera])
    write_yaml(camerainfo_dict, args.output)
    
if __name__=="__main__":
    main()    
