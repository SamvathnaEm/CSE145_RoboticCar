cmake_minimum_required(VERSION 3.5)
project(ros2_april_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(apriltag_DIR "/usr/local/lib/cmake/apriltag")
find_package(apriltag REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable for april_detection_node
add_executable(april_detection_node src/april_detection_node.cpp src/april_detection.cpp)
ament_target_dependencies(april_detection_node 
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "cv_bridge"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs")

target_link_libraries(april_detection_node
  apriltag::apriltag
  ${OpenCV_LIBRARIES}
)

# Add executable for aptil_tf_subscriber
add_executable(aptil_tf_subscriber src/aptil_tf_subsciber.cpp)
ament_target_dependencies(aptil_tf_subscriber 
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs")

install(TARGETS
  april_detection_node
  aptil_tf_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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

ament_package()
