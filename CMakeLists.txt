cmake_minimum_required(VERSION 3.5)
project(control_toolbox)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME}
  #src/dither.cpp
  #src/limited_proxy.cpp
  src/pid.cpp
  #src/pid_gains_setter.cpp
  #src/sine_sweep.cpp
  #src/sinusoid.cpp
  )
ament_target_dependencies(${PROJECT_NAME} "rclcpp")
ament_target_dependencies(${PROJECT_NAME} "realtime_tools")
target_include_directories(${PROJECT_NAME} PUBLIC include)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  # Tests
  ament_add_gmock(pid_tests test/pid_tests.cpp)
  target_link_libraries(pid_tests ${PROJECT_NAME})

  # add_executable(test_linear test/linear.cpp)
endif()

# Install
install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(
  "rclcpp"
  "realtime_tools")
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
