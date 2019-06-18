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

# Check if we're in a package root that is also a repo root,
# or we're in a package root that is not a repo root (i.e. we're
# actually in a repo stack of packages).
if [ -d $PWD/.git ] && [ -f $PWD/package.xml ]; then
    export MOUNT_POINT=`realpath $PWD`
    export PACKAGE_NAME=
elif [ -d $PWD/../.git ] && [ -f $PWD/package.xml ]; then
    export MOUNT_POINT=`realpath $PWD/..`
    export PACKAGE_NAME=${PWD/*\//}
else
    echo "ERROR: this tool must be run from a git-managed ROS package root directory!"
    exit 1
fi

# Spoof environment variables so it looks like a regular GitLab CI build
export CI_PROJECT_NAME=${MOUNT_POINT/*\//}

# Run the documentation build script in a ROS Kinetic docker image
docker run -rm -v "$MOUNT_POINT:/$CI_PROJECT_NAME" -e CI_PROJECT_NAME=$CI_PROJECT_NAME -e CI_REPOSITORY_URL=/$CI_PROJECT_NAME ros:kinetic-ros-base /$CI_PROJECT_NAME/$PACKAGE_NAME/doc/build.sh
