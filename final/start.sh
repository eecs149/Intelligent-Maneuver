#!/bin/bash
python memdb/memdb.py 8765 2> /dev/shm/memdb.err.log &
sleep 2
bin/ardrone_comm 2> /dev/shm/ardrone.err.log &
Control/control Control/icp_slam_config.ini 2> /dev/shm/control.err.log &

