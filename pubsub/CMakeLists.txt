cmake_minimum_required(VERSION 2.8.3)
project(pubsub)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

add_message_files(DIRECTORY msg FILES latency_test_message.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/pub.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker pubsub_generate_messages_cpp)

add_executable(listener src/sub.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener pubsub_generate_messages_cpp)
