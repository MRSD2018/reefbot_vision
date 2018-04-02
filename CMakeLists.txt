cmake_minimum_required(VERSION 2.8.3)
project(reefbot_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# add_service_files(
#   FILES
# )

# add_message_files(
#   FILES
# )

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_dependencies(reefbot-pymba ${catkin_EXPORTED_TARGETS})
# target_link_libraries(reefbot-pymba ${catkin_LIBRARIES} )
