cmake_minimum_required(VERSION 2.8.3)
project(publishers_arbotix)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp
rospy
std_msgs
genmsg 
message_generation)

add_service_files(
  FILES
  GetPosition.srv
  SetSpeed.srv
)



## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs )


## Declare a catkin package
catkin_package()
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(publisher_3 scripts/publisher_3.py)
add_executable(publisher_2 src/publisher_2.cpp)
add_executable(publisher src/publisher.cpp)

#target_link_libraries(publisher_3 ${catkin_LIBRARIES})
#add_dependencies(publisher_3 publishers_arbotix_genpy)

target_link_libraries(publisher ${catkin_LIBRARIES})
add_dependencies(publisher publishers_arbotix_gencpp)

target_link_libraries(publisher_2 ${catkin_LIBRARIES})
add_dependencies(publisher_2 publishers_arbotix_gencpp)

