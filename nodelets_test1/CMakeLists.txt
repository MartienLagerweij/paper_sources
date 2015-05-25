cmake_minimum_required(VERSION 2.8.3)
project(nodelets_test1)
find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs message_generation)

add_message_files(DIRECTORY msg FILES latency_test_message.msg)

generate_messages(DEPENDENCIES std_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
catkin_package(
  LIBRARIES nodelets_test1_pub nodelets_test1_sub
  CATKIN_DEPENDS nodelet roscpp std_msgs
)

add_library(nodelets_test1_pub src/pubnodelet.cpp)
add_dependencies(nodelets_test1_pub nodelets_test1_generate_messages_cpp)

add_library(nodelets_test1_sub src/subnodelet.cpp)
add_dependencies(nodelets_test1_sub nodelets_test1_generate_messages_cpp)

install(TARGETS nodelets_test1_pub nodelets_test1_sub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelets_test1_pub.xml nodelets_test1_sub.xml nobond.launch ## nodelets_test1.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
