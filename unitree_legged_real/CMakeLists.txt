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
    )

find_package(Eigen3 REQUIRED)

find_library(RUNNING_COVARIANCE_LIBRARY running_covariance HINTS /home/knapsack/workspaces/locomotion_ws/)

catkin_package( CATKIN_DEPENDS
    math_utils
    )

include_directories(
    /home/knapsack/robots/unitree_legged_sdk/include
    /home/knapsack/workspaces/locomotion_ws/src/unitree_ros/math_utils/include
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # $ENV{UNITREE_LEGGED_SDK_PATH}/include
)

#system
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

#PnC
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(PnC REQUIRED)
include_directories(${PNC_INCLUDE_DIRS})
include_directories(/use/local/lib/PnC)

link_directories(/home/knapsack/robots/unitree_legged_sdk/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(wbic src/wbic.cpp)
target_link_libraries(wbic ${EXTRA_LIBS} ${catkin_LIBRARIES} ${PNC_LIBRARIES} ${PNC_WBC_LIBRARY})
add_dependencies(wbic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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

# add_executable(test_pd src/test/test_pd.cpp)
# target_link_libraries(test_pd ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(test_pd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_pnc src/test/test_pnc.cpp)
target_link_libraries(test_pnc ${EXTRA_LIBS} ${catkin_LIBRARIES} ${PNC_LIBRARIES} ${PNC_WBC_LIBRARY})
add_dependencies(test_pnc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(example_walk src/exe/walk_example.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lcm_server /home/knapsack/robots/unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(a1_driver_main src/A1HighDriver_main.cc src/A1HighDriver.cc)
target_link_libraries(a1_driver_main 
    ${EXTRA_LIBS} 
    ${catkin_LIBRARIES} 
)
add_dependencies(a1_driver_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


