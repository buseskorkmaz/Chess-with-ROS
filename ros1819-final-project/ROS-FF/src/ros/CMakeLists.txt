#file(GLOB SRC_FILES *.c* )
#file(GLOB HDR_FILES *.h*)

include_directories(${PROJECT_SOURCE_DIR})


add_executable( ff_node_service ${PROJECT_SOURCE_DIR}/src/ros/ff-node-service.cpp ${SRC_FILES} ${HDR_FILES} ${BISON_ScanFct_OUTPUTS} ${BISON_ScanOps_OUTPUTS})
target_link_libraries( ff_node_service fflib ${catkin_LIBRARIES})
add_dependencies(ff_node_service ${catkin_EXPORTED_TARGETS})


add_executable( ff_node ${PROJECT_SOURCE_DIR}/src/ros/ff-node.cpp ${SRC_FILES} ${HDR_FILES} ${BISON_ScanFct_OUTPUTS} ${BISON_ScanOps_OUTPUTS})
target_link_libraries( ff_node fflib ${catkin_LIBRARIES})
add_dependencies(ff_node ${catkin_EXPORTED_TARGETS})


add_executable( ff_client_node ${PROJECT_SOURCE_DIR}/src/ros/ff_client.cpp)
target_link_libraries( ff_client_node ${catkin_LIBRARIES})
add_dependencies(ff_client_node ${catkin_EXPORTED_TARGETS})

