#!/bin/bash

roslaunch rslidar_sdk start.launch & sleep 5;
roslaunch fast_lio mapping_agrilira4d.launch rviz:=false & sleep 5;
wait;
