#!/bin/bash

# echo "put in your timestamp folder name: $param1"
read -p "put in your timestamp folder name:" param1
cd $param1
rostopic echo -b pose.bag -p bag_quat > quat.csv  # to dump csv from rosbag file
rostopic echo -b pose.bag -p bag_rotmat > rotmat.csv  # to dump csv from rosbag file


