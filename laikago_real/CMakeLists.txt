cmake_minimum_required(VERSION 2.8.3)
project(laikago_real)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    roscpp
    tf
    geometry_msgs
    unitree_legged_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs
)

include_directories(
    include
    /home/knapsack/robots/laikago_sdk/include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)

link_directories(/home/knapsack/robots/laikago_sdk/lib)
set(EXTRA_LIBS liblaikago_comm.so lcm)

add_executable(position_lcm src/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(velocity_lcm src/velocity_mode.cpp)
target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(torque_lcm src/torque_mode.cpp)
target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(walk_lcm src/walk_mode.cpp)
target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
