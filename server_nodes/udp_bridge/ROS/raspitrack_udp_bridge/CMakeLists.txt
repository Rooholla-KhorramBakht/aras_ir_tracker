cmake_minimum_required(VERSION 3.0.2)
project(raspitrack_udp_bridge)

# generate the server
find_package(catkin REQUIRED COMPONENTS image_transport message_generation roscpp sensor_msgs std_msgs cv_bridge camera_info_manager)
find_package(OpenCV REQUIRED)
add_service_files(DIRECTORY srv FILES GetPolledImage.srv)

add_message_files(
   FILES
   ir_markers.msg
 )
generate_messages(DEPENDENCIES sensor_msgs geometry_msgs std_msgs)

# define the project
catkin_package(CATKIN_DEPENDS image_transport message_runtime roscpp sensor_msgs std_msgs message_generation
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)


# create some library and exe
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_library(${PROJECT_NAME} src/publication_server.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(image_publisher src/image_publisher.cpp)
add_dependencies(image_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_publisher ${PROJECT_NAME}
                             ${catkin_LIBRARIES}
)
add_executable(marker_publisher src/marker_publisher.cpp)
add_dependencies(marker_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(marker_publisher ${PROJECT_NAME}
                             ${catkin_LIBRARIES}
)

install(TARGETS image_publisher
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS marker_publisher
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
