cmake_minimum_required(VERSION 2.8.3)
project(omav_local_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############

include_directories(SYSTEM "${LIB_DIR}/Include")


############
# BINARIES #
############

cs_add_executable(voliro_trajectories_node
  src/voliro_trajectories_node.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
