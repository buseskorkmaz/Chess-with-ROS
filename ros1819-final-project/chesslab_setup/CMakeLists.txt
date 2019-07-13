cmake_minimum_required(VERSION 2.8.3)
project(chesslab_setup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf2
  tf2_ros
  robotiq_2f_model
  ur_description
  kautham
  roslib
  ff
  ur3ik
)

add_service_files(
  FILES
  setrobconf.srv
  setobjpose.srv
  attachobs2robot.srv
  dettachobs.srv
  ffplan.srv
  ik.srv
  shareID.srv
  getobstaclepos.srv
  checkrule.srv	
  checkmate.srv	
  checkcastle.srv	
)

add_message_files(
  FILES
  chessmap.msg
  iksol.msg
)
    
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_basics_pkg
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime robotiq_2f_model ur_description kautham ff ur3ik
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(node src/node.cpp)
add_executable(attach src/attach.cpp)
add_executable(deattach src/deattach.cpp)
add_executable(movepiece src/movepiece.cpp)
add_executable(collision src/collision.cpp)
add_executable(planmotions src/planmotions.cpp)
add_executable(ik src/ik.cpp)


add_dependencies(node ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(attach ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(deattach ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(movepiece ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(collision ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(planmotions ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ik ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(node ${catkin_LIBRARIES})
target_link_libraries(attach ${catkin_LIBRARIES})
target_link_libraries(deattach ${catkin_LIBRARIES})
target_link_libraries(movepiece ${catkin_LIBRARIES})
target_link_libraries(collision ${catkin_LIBRARIES})
target_link_libraries(planmotions ${catkin_LIBRARIES})
target_link_libraries(ik ${catkin_LIBRARIES})


#############
## Install ##
#############

#############
## Testing ##
#############
