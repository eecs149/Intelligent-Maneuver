#!/bin/bash
python memdb/memdb.py 8765 &
sleep 1
bin/ardrone_comm &
Control/control Control/icp_slam_config.ini &

