#!/bin/bash

source /opt/ros/kinetic/setup.bash
source $(catkin locate)/devel/setup.bash

exec "$@"