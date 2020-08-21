cmake_minimum_required(VERSION 3.0.2)
project(unitree_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
    nav_msgs
    math_utils
    )

find_package(gazebo REQUIRED)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs
    nav_msgs 
    geometry_msgs
    math_utils
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
# add_library(${PROJECT_NAME}
#     src/orientation_utils.cpp 
# )

# add_dependencies(${PROJECT_NAME} geometry_msgs_gencpp)

# target_link_libraries(# ${PROJECT_NAME}
#   ${catkin_LIBRARIES} #${EXTRA_LIBS}
# )


add_executable(laikago_amrl_nav src/laikago_amrl_nav.cpp)
target_link_libraries(laikago_amrl_nav ${catkin_LIBRARIES})

add_executable(a1_amrl_nav src/a1_amrl_nav.cpp)
target_link_libraries(a1_amrl_nav ${catkin_LIBRARIES})
