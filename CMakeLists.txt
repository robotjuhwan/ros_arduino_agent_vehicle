cmake_minimum_required(VERSION 2.8.3)
project(gazebo_agent_arduino)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
message_generation
gazebo_dev
)
find_package(Eigen3 REQUIRED)

add_message_files(FILES MsgTutorial.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES hero_agent_cont
  CATKIN_DEPENDS std_msgs roscpp message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gazebo_agent_arduino src/gazebo_agent_arduino.cpp)
add_dependencies(gazebo_agent_arduino ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} msg_pkg_generate_messages_cpp)
target_link_libraries(gazebo_agent_arduino ${catkin_LIBRARIES})