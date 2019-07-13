#     Copyright 2016 Institute of Industrial and Control Engineering (IOC)
#                  Universitat Politecnica de Catalunya
#                  BarcelonaTech
#     All Rights Reserved.
#
#     This program is free software; you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation; either version 2 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with this program; if not, write to the
#     Free Software Foundation, Inc.,
#     59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
#
#    Author:   Aliakbar Akbari, Muhayyuddin


cmake_minimum_required(VERSION 2.4.6)
project(ur3ik)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  geometry_msgs
  message_generation
)

find_package( Eigen3 REQUIRED )

#include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} include)


add_service_files(
   DIRECTORY srv
   FILES
   UR3IK.srv
)

add_message_files(
   DIRECTORY msg
   FILES
   ik.msg
)
add_definitions( "--std=gnu++11" )
generate_messages(DEPENDENCIES roscpp std_msgs geometry_msgs)


catkin_package(
  #INCLUDE_DIRS  ${CMAKE_SOURCE_DIR}/src
  INCLUDE_DIRS  ${CMAKE_SOURCE_DIR}
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs
)


add_library(urIk src/ur3.cpp)


add_executable( urik-node ${CMAKE_CURRENT_SOURCE_DIR}/src/ik_server.cpp )
target_link_libraries( urik-node ${catkin_LIBRARIES} urIk )
add_dependencies(urik-node ${catkin_EXPORTED_TARGETS} ur3ik_generate_messages_cpp)


add_executable( ik_client_node ${CMAKE_CURRENT_SOURCE_DIR}/src/ik_client.cpp )
target_link_libraries( ik_client_node ${catkin_LIBRARIES} )
add_dependencies(ik_client_node ${catkin_EXPORTED_TARGETS} ur3ik_generate_messages_cpp)






