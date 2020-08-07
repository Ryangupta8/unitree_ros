cmake_minimum_required(VERSION 3.0.2)
project(math_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    )
find_package(Eigen3 REQUIRED)

find_package(gazebo REQUIRED)

# generate_messages(
#    DEPENDENCIES
   #  std_msgs
   #  geometry_msgs
   #  )

catkin_package(
    CATKIN_DEPENDS 
    # unitree_legged_msgs
    # nav_msgs 
    # geometry_msgs
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )


link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(running_stats
    src/running_stats.cpp
)

add_library(orientation_utils
	src/orientation_utils.cpp
)

# target_link_libraries( math_utils
#   ${catkin_LIBRARIES} #${EXTRA_LIBS}  
#   ${Boost_LIBRARIES}
#   running_stats
#   orientation_utils
#   )


install(TARGETS orientation_utils running_stats 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
	)

install(
	DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	)
