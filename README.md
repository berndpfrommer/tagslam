# TagSLAM: SLAM with Tags

TagSLAM is a ROS-based package for Simultaneous Localization and
Mapping using AprilTag fiducial markers.

## Installation

Supported on Ubuntu 16.04 and 18.04LTS.

Install GTSAM from the ppa:

    sudo apt-add-repository ppa:bernd-pfrommer/gtsam
    sudo apt update
    sudo apt install gtsam

You may also need the ``catkin build`` command:

    sudo apt install python-catkin-tools

Now the ROS stuff (this assumes you know your way around ROS packages):

    cat catkin_ws
    mkdir src
    cd src
    git clone https://github.com/versatran01/apriltag.git
    git clone https://github.com/daniilidis-group/flex_sync.git
    git clone https://github.com/berndpfrommer/tagslam.git
    cd ..
    catkin config -DCMAKE_BUILD_TYPE=Release
    catkin build

## Example

In the ``example`` folder you can find a simple example for a
monucular camera setup. Here's how to run it.

### Running in online mode

Run these three commands, each in a separate terminal:

    roslaunch tagslam tagslam.launch run_online:=true
    roslaunch tagslam apriltag_detector_node.launch
    rviz -d `rospack find tagslam`/example/tagslam_example.rviz
    rosbag play --clock `rospack find tagslam`/example/example.bag --topics /pg_17274483/image_raw/compressed

If all goes well, your rviz window should look like this:
![alt text](https://github.com/berndpfrommer/tagslam/blob/master/example/tagslam_example.png "rviz image").

### Running from bag

The example bag file already contains the extracted tags,
so you can directly feed the bag into tagslam:

    rviz -d `rospack find tagslam`/example/tagslam_example.rviz
    roslaunch tagslam tagslam.launch bag:=`rospack find tagslam`/example/example.bag

The program will complete too quickly to observe it in rviz, but you can
ask tagslam to replay the sequence with a ROS service call (tagslam must
still be running at that point):

    rosservice call /tagslam/replay

There are a lot more example configurations in the tagslam test
repository (TODO: break out the examples directory into separate repo)

## Detecting tags in a bag

Unless you must run online, it is best to run the tag detector on the
whole bag first, and then directly feed the detections into tagslam.
The tagslam workspace contains ``sync_and_detect``, a tool to extract
tags and write them to a new bag. At the same time, it also synchronizes
the detections that come from multiple cameras, i.e it filters frames
that don't have images from all configured cameras.

Here is an example of how to run it on the example bag:

    roslaunch tagslam sync_and_detect.launch bag:=`rospack find tagslam`/example/example.bag

This will produce a bag in the ``example`` directory:

    example.bag_output.bag

Under the topic ``/detector/tags`` the bag has the tag detections, so it
can be used to directly drive tagslam. Besides that there is the topic
``/annotated_images/compressed`` which has the tags highlighted in the
original image.

## More examples

Many more examples can be found in the 
[tagslam_test](https://github.com/berndpfrommer/tagslam_test)
repository on github!


## Documentation

TODO: explain how to configure bodies etc, how to do simple camera localization,
SLAM, object tracking, extrinsic camera calibration.
