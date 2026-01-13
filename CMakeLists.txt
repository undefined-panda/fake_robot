cmake_minimum_required(VERSION 3.20)
project(fake_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(fake_robot src/fake_robot.cpp)
add_executable(mcl_node src/mcl_node.cpp) # added by me

include_directories(include) # added by me

target_include_directories(fake_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(fake_robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(fake_robot rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2_ros)
ament_target_dependencies(mcl_node rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs) # added by me

install(TARGETS fake_robot
  DESTINATION lib/${PROJECT_NAME})

# added by me
install(TARGETS mcl_node 
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install data files
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
