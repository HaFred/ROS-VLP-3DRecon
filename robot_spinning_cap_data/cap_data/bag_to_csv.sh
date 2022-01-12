#!/bin/bash

echo "put in your timestamp folder name: $1"
# read -p "put in your timestamp folder name:" $1
cd $1
rostopic echo -b pose.bag -p bag_quat > quat.csv  # to dump csv from rosbag file
rostopic echo -b pose.bag -p bag_rotmat > rotmat.csv  # to dump csv from rosbag file
rostopic echo -b pose.bag -p bag_pose2d > pose2d.csv  # to dump csv from rosbag file
rostopic echo -b pose.bag -p bag_tran_bool > bool.csv  # to dump csv from rosbag file