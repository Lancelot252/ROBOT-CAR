cmake_minimum_required(VERSION 2.8.3)
project(motorControl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motorControl
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS}
 ${PROJECT_SOURCE_DIR}/src/motorControl/include/motorControl
 /usr/include/jsoncpp)
#link_directories()



set(LIB_ARCHITECTURES arm)
find_library(LIBCONTROLCAN libcontrolcan.so ${PROJECT_SOURCE_DIR}/lib/libcontrolcan/${LIB_ARCHITECTURES}/64bit/)

#aux_source_directory(${PROJECT_SOURCE_DIR}/src/motorControl/src MOTORCONRROL_SRC)

# add_executable(MotorControlNode src/MotorControl.cpp src/MotorCan.cpp src/MotorControlNode.cpp)
# #add_executable(MotorControlNode src/MotorCan.cpp src/MotorControl.cpp src/MotorControlNode.cpp)
# #target_link_libraries(MotorControlNode ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libcontrolcan/x86_64/64bit/libcontrolcan.so)
# target_link_libraries(MotorControlNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(RobotArmControlNode src/RobotArmControlNode.cpp src/MotorCan.cpp src/RobotArmControl.cpp)
# #add_executable(MotorControlNode src/MotorCan.cpp src/MotorControl.cpp src/MotorControlNode.cpp)
# #target_link_libraries(RobotArmControlNode ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libcontrolcan/x86_64/64bit/libcontrolcan.so)
# target_link_libraries(RobotArmControlNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

#add_executable(test src/MotorControl.cpp src/MotorCan.cpp test/test.cpp)
#target_link_libraries(test ${catkin_LIBRARIES} /home/slam/Desktop/motorControl/src/motorControl/lib/libcontrolcan/x86_64/libcontrolcan.so)

#add_executable(RobotTrackControlNode src/RobotTrackControlNode.cpp src/MotorCan.cpp src/RobotTrackControl.cpp src/RobotControl.cpp)
#target_link_libraries(RobotTrackControlNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

#add_executable(RobotClawControlNode src/RobotClawControlNode.cpp src/MotorCan.cpp src/RobotClawControl.cpp src/RobotControl.cpp)
#target_link_libraries(RobotClawControlNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(RobotAllControlNode src/RobotAllControlNode.cpp src/MotorCan.cpp src/RobotAllControl.cpp src/RobotControl.cpp)
# target_link_libraries(RobotAllControlNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(RobotControlNode src/RobotControlNode.cpp src/MotorCan.cpp src/RobotTrackControl.cpp src/RobotControl.cpp src/MotorControl.cpp src/RobotAllControl.cpp src/RobotClawControl.cpp)
# target_link_libraries(RobotControlNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(RobotAllStepControlNode src/RobotAllStepControlNode.cpp src/MotorCan.cpp src/RobotAllControl.cpp src/RobotControl.cpp)
# target_link_libraries(RobotAllStepControlNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(RobotTrackControlNode222 src/RobotTrackControlNode222.cpp src/MotorCan.cpp src/RobotTrackControl.cpp src/RobotControl.cpp)
# target_link_libraries(RobotTrackControlNode222 ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(RobotIntegrateNode src/RobotIntegrateNode.cpp src/MotorCan.cpp src/RobotAllControl.cpp src/RobotTrackControl.cpp src/MotorControl.cpp src/RobotCameraControl.cpp src/RobotControl.cpp src/SerialTrackControl.cpp src/MotorSerial.cpp src/SwitchSerial.cpp)
# target_link_libraries(RobotIntegrateNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(RobotAutoNode src/RobotAutoNode.cpp src/MotorCan.cpp src/RobotTrackControl.cpp src/RobotControl.cpp src/RobotAllControl.cpp)
# target_link_libraries(RobotAutoNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

add_executable(CarControlNode src/CarControlNode.cpp src/MotorControl.cpp src/MotorCan.cpp src/RobotControl.cpp)
target_link_libraries(CarControlNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)
#target_link_libraries(CarContorlNode ${catkin_LIBRARIES} /home/pi/SmartCar_Dir/SmartCar_1.1_beta/motorControl/src/motorControl/lib/libcontrolcan/arm/64bit/libcontrolcan.so jsoncpp)

# add_executable(SerialMotorNode src/SerialMotorNode.cpp src/MotorSerial.cpp src/SerialTrackControl.cpp)
# target_link_libraries(SerialMotorNode ${catkin_LIBRARIES} ${LIBCONTROLCAN})

# add_executable(RobotCameraNode src/RobotCameraNode.cpp src/RobotCameraControl.cpp src/RobotControl.cpp src/MotorCan.cpp)
# target_link_libraries(RobotCameraNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(TrackFastSlowNode src/TrackFastSlowNode.cpp src/MotorSerial.cpp src/SerialTrackControl.cpp src/SwitchSerial.cpp src/MotorCan.cpp src/RobotTrackControl.cpp src/RobotControl.cpp)
# target_link_libraries(TrackFastSlowNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)

# add_executable(RobotPoseNode src/RobotPoseNode.cpp src/MotorCan.cpp src/RobotAllControl.cpp src/RobotControl.cpp)
# target_link_libraries(RobotPoseNode ${catkin_LIBRARIES} ${LIBCONTROLCAN} jsoncpp)
