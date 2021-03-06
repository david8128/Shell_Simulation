cmake_minimum_required(VERSION 3.10.0)
project(airsim_car_ros_pkgs)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
if(DEFINED ENV{AIRSIM_ROOT})
  set(AIRSIM_ROOT $ENV{AIRSIM_ROOT})
else()
  set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../AirSim)
endif()


add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake")
LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules")
INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")
CommonSetup()

set(CMAKE_CXX_STANDARD 11)
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  swri_profiler
  swri_roscpp
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_message_files(
  FILES
  GimbalAngleEulerCmd.msg
  GimbalAngleQuatCmd.msg
  GPSYaw.msg
  VelCmd.msg
  VelCmdGroup.msg
  CarCmd.msg
  Collision.msg
)

add_service_files(
  FILES
  SetGPSPosition.srv
  Takeoff.srv
  TakeoffGroup.srv
  Land.srv
  LandGroup.srv
  Reset.srv
  SetLocalPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES airsim_car_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(odometer nodes/odometer.cpp)
target_link_libraries(odometer ${catkin_LIBRARIES})

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser ${catkin_LIBRARIES} AirLib)

add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
add_dependencies(pd_position_controller_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pd_position_controller_simple ${catkin_EXPORTED_TARGETS})

add_library(airsim_car_ros src/airsim_ros_wrapper.cpp)
add_dependencies(airsim_car_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_car_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser)

add_executable(airsim_car_node src/airsim_node.cpp)
target_link_libraries(airsim_car_node airsim_car_ros ${catkin_LIBRARIES} AirLib)

add_executable(pd_position_controller_simple_node src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node pd_position_controller_simple airsim_car_ros ${catkin_LIBRARIES} AirLib)

catkin_install_python(
  PROGRAMS
    nodes/airsim_ps_joy
    nodes/airsim_teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    airsim_car_node
    airsim_car_ros
    airsim_settings_parser
    odometer
    pd_position_controller_simple
    pd_position_controller_simple_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  #list of necessary files (xml...)
  README.md
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
