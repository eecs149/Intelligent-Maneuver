#!/bin/bash
python memdb/memdb.py $1 2> /dev/shm/memdb.err.log &
sleep 2
bin/ardrone_comm -p $1 2> /dev/shm/ardrone.err.log &
Control/control Control/icp_slam_config.ini $1 2> /dev/shm/control.err.log &
Ultrasonic/readultrasonic /dev/ttyUSB0 $1 &
#Ultrasonic/ultrasonic_simulator foo.dump $1 &
