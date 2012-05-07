#!/bin/bash
python memdb/memdb.py 8766 2> /dev/shm/memdb.err.log &
sleep 2
bin/ardrone_comm 2> /dev/shm/ardrone.err.log &
Control/control Control/icp_slam_config.ini 2> /dev/shm/control.err.log &
#Lidar/readlidar /dev/ttyUSB0 8765 &
#Lidar/readlidar Lidar/lidar.dump 8765 &
#Lidar/readlidar /home/huipeng/Desktop/lidar1.dump 8765 &

