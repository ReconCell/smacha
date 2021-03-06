#!/bin/bash
#===============================================================================
# RECONCELL DOCUMENTATION LOCAL BUILD SCRIPT
#
# This script will build the current package documentation locally assuming
# build dependencies have been installed. It should be quite a bit faster than
# running a full docker build.
#
# This method assumes that the following dependencies have been installed:
#   sphinx
#   doxygen (for C++ APIs)
#   sphinx_rtd_theme
#   rosdoc_lite
#
# It also assumes that the current package has been built as part of a catkin
# workspace and that the catkin workspace has been appropriately sourced.
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

# Remove previous folders
# NOTE: This may require sudo if these folders were previously
# built by the docker build script.
rm -rf $MOUNT_POINT/$PACKAGE_NAME/public_build
rm -rf $MOUNT_POINT/public/$PACKAGE_NAME

# Build the documentation using rosdoc_lite
rosdoc_lite -o $MOUNT_POINT/$PACKAGE_NAME/public_build .

# Copy the html to the public folder
mkdir -p $MOUNT_POINT/public/$PACKAGE_NAME/
cp -r $MOUNT_POINT/$PACKAGE_NAME/public_build/html/* $MOUNT_POINT/public/$PACKAGE_NAME/
