#!/bin/bash

echo "put in your bag file name: $1"

rostopic echo -b ${1}.bag -p bag_quat > quat_${1}.csv  # to dump csv from rosbag file
rostopic echo -b ${1}.bag -p bag_rotmat > rotmat_${1}.csv  # to dump csv from rosbag file
