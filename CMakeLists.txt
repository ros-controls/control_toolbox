cmake_minimum_required(VERSION 3.16)
project(control_toolbox LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(WIN32)
  # Enable Math Constants
  # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  rclcpp
  rcutils
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(control_toolbox SHARED
  src/dither.cpp
  src/limited_proxy.cpp
  src/pid_ros.cpp
  src/pid.cpp
  src/sine_sweep.cpp
  src/sinusoid.cpp
)
target_compile_features(control_toolbox PUBLIC cxx_std_17)
target_include_directories(control_toolbox PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/control_toolbox>
)
ament_target_dependencies(control_toolbox PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(control_toolbox PRIVATE "CONTROL_TOOLBOX_BUILDING_LIBRARY")

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  ament_add_gmock(pid_tests test/pid_tests.cpp)
  target_link_libraries(pid_tests control_toolbox)

  ament_add_gtest(pid_parameters_tests test/pid_parameters_tests.cpp)
  target_link_libraries(pid_parameters_tests control_toolbox)

  ament_add_gtest(pid_publisher_tests test/pid_publisher_tests.cpp)
  target_link_libraries(pid_publisher_tests control_toolbox)
  ament_target_dependencies(pid_publisher_tests rclcpp_lifecycle)
endif()

install(
  DIRECTORY include/
  DESTINATION include/control_toolbox
)
install(TARGETS control_toolbox
  EXPORT export_control_toolbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_control_toolbox HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
