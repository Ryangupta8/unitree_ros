cmake_minimum_required(VERSION 2.8.3)
project(laikago_real)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    roscpp
    tf
    geometry_msgs
    nav_msgs
    std_msgs
    unitree_legged_msgs
    sensor_msgs
    math_utils
)
find_package(Eigen3 REQUIRED)


find_library(RUNNING_COVARIANCE_LIBRARY running_covariance HINTS /home/amrl/workspaces/unitree_ws/)

catkin_package(

    CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    std_msgs 
    unitree_legged_msgs
    sensor_msgs
    math_utils
    )

include_directories(
    include
    /home/amrl/robots/laikago_sdk/include
    /home/amrl/workspaces/unitree_ws/src/unitree_ros/math_utils/include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)

link_directories(/home/amrl/robots/laikago_sdk/lib)
set(EXTRA_LIBS liblaikago_comm.so lcm)

add_executable(position_lcm src/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(velocity_lcm src/velocity_mode.cpp)
target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(torque_lcm src/torque_mode.cpp)
target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(walk_lcm src/walk_mode.cpp)
target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(laikago_driver src/laikago_driver.cpp)
target_link_libraries(laikago_driver 
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
    ${RUNNING_COVARIANCE_LIBRARY}
)
