#!/usr/bin/env python
#
# plot error as a function of time
#

import numpy as np
import matplotlib.pyplot as plt;
import argparse
from os.path import expanduser


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='plot error as function of time')
    parser.add_argument('--file', '-f', action='store',
                        default=expanduser("~")+'/.ros/time_diagnostics.txt',
                        help='the time diagnostics file.')
    args = parser.parse_args()

    v = []
    with open(args.file) as file:
        for line in file.readlines():
            arr = line.strip().split(' ')
            v.append([float(x) for x in arr[0:2]])
    va = np.array(v)
    plt.plot(va[1:,0], va[1:,1])
    plt.show()
