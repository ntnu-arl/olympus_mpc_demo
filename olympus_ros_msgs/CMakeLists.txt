cmake_minimum_required(VERSION 3.0.2)
project(olympus_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

# # Generate messages in the 'msg' folder
add_message_files(
  FILES
  SetpointArray.msg
  MotorStatus.msg
  MotorStatusArray.msg
  PIDState.msg
  PIDStateArray.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SetBoolArray.srv
  TriggerArray.srv
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)
