cmake_minimum_required(VERSION 3.0.2)
project(live_tracker)

find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

catkin_install_python(PROGRAMS scripts/live_tracker.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
