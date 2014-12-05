cmake_minimum_required(VERSION 2.8.3)
project(fs100_motoman)

## Find catkin macros and libraries
find_package(catkin REQUIRED
	roscpp
	std_msgs
	sensor_msgs
	trajectory_msgs

	message_generation
	message_runtime

	)
add_service_files(
    FILES
    setReset.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

add_library(robotState src/fs100_state.cpp)
add_library(robotCmd src/fs100_cmd.cpp)
## Declare a cpp executable
add_executable(mh5l_traj src/mh5l_traj.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mh5l_traj robotState robotCmd ${catkin_LIBRARIES} )

