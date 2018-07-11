#!/bin/bash
#
# extracts pose from rosbag
#
# usage:
#
# rosrun tagslam extract_position.bash wand_tracking.bag /tagslam/odom/body_wand 1530625491.57 > plot.txt
#
# where the timestamp is the start_time of the original bag (in seconds, what you get from rosbag info)
#
#

bag=$1
topic=$2
stime=$3
start_time=`bc <<< "1000000000.0 * ${stime}"`
rostopic echo -p -b $bag $topic | tail -n +2 | awk -v ts="$start_time" -F\, '{print ($3-ts)/1.0e9, $6, $7, $8}'
