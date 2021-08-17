cmake_minimum_required(VERSION 3.5)
project(dai_turtlebot3_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)

find_package(depthai CONFIG REQUIRED PATHS "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib/cmake/depthai") 
# find_package(depthai CONFIG REQUIRED) 

set(dependencies
rclcpp
sensor_msgs
std_msgs
geometry_msgs
depthai_ros_msgs
depthai_bridge
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(mobilenet_blob "${PROJECT_SOURCE_DIR}/resources/mobilenet-ssd_openvino_2021.4_6shave.blob")
file(DOWNLOAD "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_6shave.blob" 
${mobilenet_blob}
TIMEOUT 60  # seconds
EXPECTED_HASH SHA1=4f4506726e3083981064938a0faaf9af6180d2c6
TLS_VERIFY ON)

add_executable(dynamic_tracker src/dynamic_tracker.cpp)
ament_target_dependencies(dynamic_tracker 
                          ${dependencies})    

target_link_libraries(dynamic_tracker 
                      depthai::core)

target_compile_definitions(dynamic_tracker PRIVATE BLOB_PATH="${mobilenet_blob}")

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(TARGETS
        dynamic_tracker
        DESTINATION lib/${PROJECT_NAME})

ament_package()
