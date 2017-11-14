#!/bin/bash

set -e 

export PYTHONPATH=$PYTHONPATH:/usr/local/src/LinuxPUMAs/src/LeapSDK/lib/x64
roscore & 
rosrun leap_motion sender.py
