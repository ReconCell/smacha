#!/bin/bash
#===============================================================================
# RECONCELL DOCUMENTATION BUILD SCRIPT
#
# This script will build the current package documentation either locally,
# or remotely in a GitLab repository, depending on how the environment variables
# are configured.
#
# The following environment variables need to be set:
# CI_PROJECT_NAME: The name of the current package (automatically set by GitLab).
# CI_REPOSITORY_URL: The repository URL for cloning the package (automatically set
#                    by GitLab).
#
# This documentation build procedure is based on that used by the moveit_tutorials
# package:
# https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/.travis.yml
#
# Author: Barry Ridge, JSI
# Date: 16th April 2018
#===============================================================================

# NOTE: if the cloned repository does not contain a package.xml file in its root
# directory, then we must assume that the
# repository is actually a stack of packages, in which case we should set
# $PACKAGE_NAME as follows for the docs build below:
export PACKAGE_DOC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
export PACKAGE_DIR=`realpath $PACKAGE_DOC_DIR/..`
if [ ! -f $CI_REPOSITORY_URL/package.xml ]; then
    export PACKAGE_NAME=${PACKAGE_DIR/*\//}
else
    export PACKAGE_NAME=
fi

# Update and install useful tools
apt-get -qq update
apt-get -qq install wget
apt-get -qq install apt-utils
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get -qq install -y python-rosdep python-wstool python-catkin-tools

# Setup rosdep
rosdep init
rosdep update
apt-get -qq update

# Update pip
# apt-get -qq install python-pip
# pip install --upgrade pip
easy_install -U pip

# Install newer version of sphinx
pip install --upgrade sphinx

# Install recommonmark
pip install --upgrade recommonmark

# Install sphinx_rtd_theme
pip install --upgrade sphinx_rtd_theme

# Install sphinxcontrib-programoutput
pip install --upgrade sphinxcontrib-programoutput

# Install newer version of ruby
# apt-get -qq install software-properties-common
# apt-add-repository -y ppa:brightbox/ruby-ng
# apt-get -qq update
# apt-get -qq install ruby2.3-dev ruby2.3

# Install htmlproofer
# apt-get -qq install zlib1g-dev
# gem update
# gem install pkg-config
# gem install mini_portile2
# gem install html-proofer

# Install rosdoc_lite
apt-get -qq install ros-kinetic-rosdoc-lite
source /opt/ros/kinetic/setup.bash

# Initialize catkin workspace
mkdir -p catkin_ws/src
cd catkin_ws
catkin build
source ./devel/setup.bash

# Clone repo
git clone $CI_REPOSITORY_URL src

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

# Catkin build
catkin build
cd ..

# Source catkin workspace
source ./catkin_ws/devel/setup.bash

# Test build with non-ROS wrapped Sphinx command to allow warnings and errors to be caught
sphinx-build -W -b html /$CI_PROJECT_NAME/$PACKAGE_NAME /$CI_PROJECT_NAME/$PACKAGE_NAME/public_native_build

# Test build with ROS-version of Sphinx command so that it is generated same as ros.org
rosdoc_lite -o /$CI_PROJECT_NAME/$PACKAGE_NAME/public_build /$CI_PROJECT_NAME/$PACKAGE_NAME

# Copy the html to the public folder
mkdir -p /$CI_PROJECT_NAME/$PACKAGE_NAME/public
cp -r /$CI_PROJECT_NAME/$PACKAGE_NAME/public_build/html/* /$CI_PROJECT_NAME/$PACKAGE_NAME/public/

# Run HTML tests on generated build output to check for 404 errors, etc
# htmlproofer /$CI_PROJECT_NAME/$PACKAGE_NAME/public --only-4xx --check-html --file-ignore /$CI_PROJECT_NAME/$PACKAGE_NAME/public/genindex.html,/$CI_PROJECT_NAME/$PACKAGE_NAME/public/search.html,/$CI_PROJECT_NAME/$PACKAGE_NAME/public/index-msg.html --alt-ignore '/.*/' --url-ignore '#'
