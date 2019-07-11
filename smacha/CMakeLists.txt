cmake_minimum_required(VERSION 2.8.3)
project(smacha)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  smach
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smacha
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/generate
  scripts/extract
  scripts/contain
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  src/smacha/templates/
  DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/templates
  FILES_MATCHING PATTERN "*.tpl.py"
)
install(DIRECTORY
  test/smacha_test_examples/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/smacha_test_examples
  FILES_MATCHING PATTERN "*.py"
)
install(DIRECTORY
  test/smacha_templates/smacha_test_examples/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/smacha_templates/smacha_test_examples
  FILES_MATCHING PATTERN "*.tpl.py"
)
install(DIRECTORY
  test/smacha_scripts/smacha_test_examples/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/smacha_scripts/smacha_test_examples
  FILES_MATCHING PATTERN "*.yml"
)
install(DIRECTORY
  test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/smacha_scripts/smacha_test_examples
  FILES_MATCHING PATTERN "*.yml"
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()