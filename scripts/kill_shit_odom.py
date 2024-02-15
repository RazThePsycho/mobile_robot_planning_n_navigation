#!/usr/bin/env python3

import os

try:
    os.system('rosnode kill /robot/filter_odom')
except:
    print('so bad, so bad odom')
try:
    os.system('rosnode kill /robot/filter_world')
except:
    print('so bad, so bad odom')
try:
    os.system('rosnode kill /robot/navsat_transform')
except:
    print('so bad, so bad odom')
# output = stream.read().split('\n')[]
