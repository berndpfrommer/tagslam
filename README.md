# TagSLAM: SLAM with Tags

TagSLAM is a ROS-based package for Simultaneous Localization and
Mapping using AprilTag fiducial markers.


## Installation

The only supported platform is Ubuntu 16.04:

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
	git clone https://github.com/berndpfrommer/tagslam.git
	cd ..
    catkin config -DCMAKE_BUILD_TYPE=Release
	catkin build


## Running

Adjust the launch scripts to your liking and remap topics, then run the detector
and tagslam:

    roslaunch tagslam apritag_detector_node.launch
    roslaunch tagslam tagslam.launch

