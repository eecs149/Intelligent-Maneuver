#!/bin/bash
ps aux | grep -v grep | grep -e 'memdb/memdb\.py' | awk '{print $2}' | xargs kill

