#!/bin/bash
#===============================================================================
# RECONCELL DOCUMENTATION LOCAL DOCKER BUILD SCRIPT
#
# This script will build the current package documentation locally by running
# the ReconCell documentation build script in a local ROS Kinetic docker image. 
# This method assumes no dependencies other than docker.
#
# NOTE: The script must be run from the root directory of the current package.
#
# After the build, the documentation will be available in the 'public' folder
# in the root directory of the current package.
#
# Author: Barry Ridge, JSI
# Date: 16th April 2018
#===============================================================================

# Spoof environment variables so it looks like a regular GitLab CI build
export MOUNT_POINT=${PWD}
export CI_PROJECT_NAME=${PWD/*\//}

# Run the documentation build script in a ROS Kinetic docker image
docker run --rm -v "$MOUNT_POINT:/$CI_PROJECT_NAME" -e CI_PROJECT_NAME=$CI_PROJECT_NAME -e CI_REPOSITORY_URL=/$CI_PROJECT_NAME ros:kinetic-ros-base /$CI_PROJECT_NAME/doc/build.sh
