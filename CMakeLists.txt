cmake_minimum_required(VERSION 2.8.3)
project(five_dof_robotarm)


#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation rospy roscpp std_msgs genmsg)

#add_message_files(
#   FILES
#   Num.msg
#)
#find_package(catkin REQUIRED COMPONENTS moveit_ros_planning_interface)

#find_package(Eigen3 REQUIRED) 


#add_service_files(
#  FILES
#  AddTwoInts.srv
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package()
#catkin_package(CATKIN_DEPENDS message_runtime)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
#include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
#link_directories(~/my_libs)


#add_executable(talker src/main.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker five_dof_robotarm_messages_cpp)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY images meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


