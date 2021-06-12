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

########################
# Build control filters
########################
find_package(filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(CONTROL_FILTERS_INCLUDE_DEPENDS
  filters
  geometry_msgs
  pluginlib
  rclcpp
  tf2_geometry_msgs
  tf2_ros
)

add_library(gravity_compensation SHARED
  src/control_filters/gravity_compensation.cpp
)
target_include_directories(gravity_compensation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gravity_compensation ${CONTROL_FILTERS_INCLUDE_DEPENDS})


##########
# Testing
##########
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  ament_add_gmock(pid_tests test/pid_tests.cpp)
  target_link_libraries(pid_tests ${PROJECT_NAME})

  ament_add_gtest(pid_parameters_tests test/pid_parameters_tests.cpp)
  target_link_libraries(pid_parameters_tests ${PROJECT_NAME})

  ament_add_gtest(pid_publisher_tests test/pid_publisher_tests.cpp)
  target_link_libraries(pid_publisher_tests ${PROJECT_NAME})
  ament_target_dependencies(pid_publisher_tests rclcpp_lifecycle)

  ## Control Filters
  ament_add_gmock(test_gravity_compensation test/control_filters/test_gravity_compensation.cpp)
  target_link_libraries(test_gravity_compensation gravity_compensation)
  ament_target_dependencies(test_gravity_compensation ${CONTROL_FILTERS_INCLUDE_DEPENDS})

endif()

# Install
install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS gravity_compensation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install pluginlib xml
pluginlib_export_plugin_description_file(filters control_filters.xml)

ament_export_dependencies(
  rclcpp
  rcutils
  realtime_tools
  ${CONTROL_FILTERS_INCLUDE_DEPENDS}
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  gravity_compensation
)

ament_package()
