#!/usr/bin/env python

import bagpy
from bagpy import bagreader
import pandas as pd

b = bagreader('/home/liphy/.ros/pose1.bag')

# replace the topic name as per your need
quat = b.message_by_topic('/bag_quat')
rot = b.message_by_topic('/bag_rotmat')
quat
rot
df_quat = pd.read_csv(quat)
df_rot = pd.read_csv(rot)
df_rot  # prints data in the form of pandas dataframe
