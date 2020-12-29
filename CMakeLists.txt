cmake_minimum_required(VERSION 3.5)
project(control_toolbox)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(WIN32)
  # Enable Math Constants
  # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dither.cpp
  src/limited_proxy.cpp
  src/pid_ros.cpp
  src/pid.cpp
  src/sine_sweep.cpp
  src/sinusoid.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_definitions(${PROJECT_NAME} PRIVATE "CONTROL_TOOLBOX_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME}
    control_msgs
    rclcpp
    rcutils
    realtime_tools)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(pid_tests test/pid_tests.cpp)
  target_link_libraries(pid_tests ${PROJECT_NAME})

  ament_add_gtest(pid_parameters_tests test/pid_parameters_tests.cpp)
  target_link_libraries(pid_parameters_tests ${PROJECT_NAME})

  ament_add_gtest(pid_publisher_tests test/pid_publisher_tests.cpp)
  target_link_libraries(pid_publisher_tests ${PROJECT_NAME})
  ament_target_dependencies(pid_publisher_tests rclcpp_lifecycle)
endif()

# Install
install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(
  rclcpp
  rcutils
  realtime_tools)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
