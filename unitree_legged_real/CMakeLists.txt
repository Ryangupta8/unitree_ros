cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    roscpp
    tf
    geometry_msgs
    unitree_legged_msgs
    math_utils
    std_msgs
    nav_msgs
    sensor_msgs
    amrl_msgs
    )

find_package(Eigen3 REQUIRED)

find_library(RUNNING_COVARIANCE_LIBRARY running_covariance HINTS /home/amrl/workspaces/autonomy_ws/)

catkin_package( CATKIN_DEPENDS
    math_utils
    )

include_directories(
    /home/amrl/robots/unitree_legged_sdk/include
    /home/amrl/workspaces/autonomy_ws/src/unitree_ros/math_utils/include
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # $ENV{UNITREE_LEGGED_SDK_PATH}/include
)


link_directories(/home/amrl/robots/unitree_legged_sdk/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(position_lcm src/exe/position_mode.cpp)
target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(position_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(velocity_lcm src/exe/velocity_mode.cpp)
target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(velocity_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(torque_lcm src/exe/torque_mode.cpp)
target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(torque_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(walk_lcm src/exe/walk_mode.cpp)
target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(walk_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(example_walk src/exe/walk_example.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lcm_server /home/amrl/robots/unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(a1_waypoints src/a1_waypoints.cpp)
target_link_libraries(a1_waypoints 
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
    ${RUNNING_COVARIANCE_LIBRARY}
)
add_dependencies(a1_waypoints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(a1_driver_main src/A1HighDriver_main.cc src/A1HighDriver.cc)
target_link_libraries(a1_driver_main 
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
)
add_dependencies(a1_driver_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(a1_121_initialpose_convert src/a1_121_initialpose_converter.cc)
target_link_libraries(a1_121_initialpose_convert
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
)
add_dependencies(a1_121_initialpose_convert ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(a1_950_initialpose_convert src/a1_950_initialpose_converter.cc)
target_link_libraries(a1_950_initialpose_convert
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
)
add_dependencies(a1_950_initialpose_convert ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

######################
# NOTE NOTE NOTE NOTE 
### a1_driver_recovery with the y was the original file I rm'd
### a1_driver_recover is the cleaner rewrite
add_executable(a1_driver_recover src/A1HighDriver_recover.cc src/A1HighDriver.cc)
target_link_libraries(a1_driver_recover
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
)
add_dependencies(a1_driver_recover ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(a1_driver_gather_publish src/A1HighDriver_gather_publish.cc src/A1HighDriver.cc)
target_link_libraries(a1_driver_gather_publish 
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
)
add_dependencies(a1_driver_gather_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(a1_driver_waypoint_pose src/A1HighDriver_waypoint_pose.cc src/A1HighDriver.cc)
target_link_libraries(a1_driver_waypoint_pose 
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
)
add_dependencies(a1_driver_waypoint_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


