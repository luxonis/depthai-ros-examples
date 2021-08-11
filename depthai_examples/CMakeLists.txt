cmake_minimum_required(VERSION 3.5)


set (CMAKE_POSITION_INDEPENDENT_CODE ON)
project(depthai_examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)

find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
# find_package(depthai CONFIG REQUIRED PATHS "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib/cmake/depthai") 

find_package(depthai CONFIG REQUIRED) 

set(dependencies
rclcpp
sensor_msgs
std_msgs
vision_msgs
depthai_ros_msgs
depthai_bridge
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
              lib/stereo_pipeline.cpp
              lib/nn_pipeline.cpp)

target_link_libraries(${PROJECT_NAME}
                      depthai::core 
                      opencv_imgproc
                      opencv_highgui) 


set(mobilenet_blob "${PROJECT_SOURCE_DIR}/resources/mobilenet-ssd_openvino_2021.2_6shave.blob")
file(DOWNLOAD "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_6shave.blob" 
${mobilenet_blob}
TIMEOUT 60  # seconds
EXPECTED_HASH SHA1=f0e14978b3f77a4f93b9f969cd39e58bb7aef490
TLS_VERIFY ON)



add_executable(stereo_node src/stereo_publisher.cpp)
ament_target_dependencies(stereo_node 
                          ${dependencies} 
                          )    

target_link_libraries(stereo_node 
                      depthai::core 
                      ${OpenCV_LIBRARIES} 
                      ${PROJECT_NAME})

add_executable(mobilenet_node src/mobilenet_publisher.cpp)
ament_target_dependencies(mobilenet_node 
                          ${dependencies} 
                          )    

target_link_libraries(mobilenet_node 
                      depthai::core 
                      ${OpenCV_LIBRARIES} 
                      ${PROJECT_NAME})
target_compile_definitions(mobilenet_node PRIVATE BLOB_PATH="${mobilenet_blob}")
                      
                      
ament_export_include_directories(include)
ament_export_libraries(depthai_examples)
ament_export_dependencies(${dependencies}
                          depthai::core
                          ${OpenCV_LIBRARIES})


install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
        stereo_node
        mobilenet_node
        DESTINATION lib/${PROJECT_NAME})
ament_package()

# dai_add_node(stereo_node src/stereo_publisher.cpp)
# dai_add_node(rgb_node src/rgb_publisher.cpp)
# dai_add_node(mobilenet_node src/mobilenet_publisher.cpp)
# dai_add_node(rgb_stereo_node src/rgb_stereo_node.cpp)
# dai_add_node(stereo_mimic_node src/stereo_mimic_node.cpp)
# dai_add_node(rgb_subscriber_node src/rgb_video_subscriber.cpp)
# 
# target_compile_definitions(mobilenet_node PRIVATE BLOB_PATH="${mobilenet_blob}")

# add_executable(stereo_node
#   src/stereo_publisher.cpp
# )

# add_dependencies(stereo_node
#   ${catkin_EXPORTED_TARGETS} 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${PROJECT_NAME} 
# )

# target_link_libraries(stereo_node 
#   ${catkin_LIBRARIES}
#   ${PROJECT_NAME}
#   depthai-core
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_depthai_examples.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
