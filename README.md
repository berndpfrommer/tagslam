# TagSLAM: SLAM with Tags

TagSLAM is a ROS2-based package for Simultaneous Localization and
Mapping using Apriltag fiducial markers.

ROS2 is WORK IN PROGRESS. Documentation will follow.

## Platforms supported

At the moment TagSLAM requires ROS2 Rolling/Jazzy or newer.
Older versions of ROS2 lack some important rosbag2 features.

## How to build

The build instructions follow the standard procedure for ROS2. Set the following shell variables:

```bash
repo=tagslam
url=https://github.com/berndpfrommer/${repo}.git
```

and follow the ROS2 build instructions
[here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md).
However, you need to modify the instructions to compile the ``ros2`` branch.

Make sure to source your workspace's ``install/setup.bash`` afterwards.


## How to use


### Sync and detect

TagSLAM operates off of tags (and odometry messages, if provided). In a scenerio with multiple cameras or odometry it is important
that all sensors are synchronized, meaning that the sensor data has matching ROS header.stamp fields.
Sync_and_detect runs the apriltag detector across multiple cameras and emits synchronized messages with the decoded tags.
These messages in turn are used by TagSLAM. Note that ``sync_and_detect`` can also deal with odometry:
it drops all odometry messages except for the ones that coincide (approximately) with the camera images, and alters the header.stamp
field to match exactly the ones of the image messages.

You can run ``sync_and_detect`` either from a bag file, and write the
detected tags into another bag, or you can run it as a stand-alone (composable) node.
Here is how to run it from a bag:
```
ros2 run tagslam sync_and_detect_from_bag --ros-args -p "cameras:=./cameras.yaml" -p "tagslam_config:=./tagslam.yaml" -p "in_bag:=name_of_input_bag" -p "out_bag:=./tag_bag"
```
It will use the ``cameras.yaml`` file to determine what topics to read from the input bag, what image transport (raw vs compressed), what tag detector
(MIT vs UMich), and what output tag topics to use. The ``tagslam.yaml`` file is searched for bodies with odometry topics.

### TagSLAM

TagSLAM can run off a rosbag, or as a node. When running off a bag, TagSLAM will automatically recognize when there are only
image topics, but no tag topics in the rosbag, and will start ``sync_and_detect`` to do tag detection.

Run TagSLAM from a rosbag like this:
```
ros2 run tagslam tagslam_from_bag --ros-args -p "cameras:=./cameras.yaml" -p "tagslam_config:=./tagslam.yaml" -p "camera_poses:=./camera_poses.yaml" -p "in_bag:=./bag_with_tags_and_odom" -p "out_bag:=./out_bag"
```


## License

This software and any future contributions to it are licensed under
the [Apache License 2.0](LICENSE).
