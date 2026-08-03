#!/bin/bash

sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch mavros px4.launch & sleep 10;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
roslaunch rslidar_sdk start.launch & sleep 5;
roslaunch fast_lio mapping_agrilira4d.launch rviz:=false & sleep 5;

wait;
