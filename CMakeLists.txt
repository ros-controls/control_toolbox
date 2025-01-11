cmake_minimum_required(VERSION 3.16)
project(control_toolbox LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
  # set the same behavior for windows as it is on linux
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
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


########################
# Build control filters
########################
set(CONTROL_FILTERS_INCLUDE_DEPENDS
  filters
  rclcpp
  generate_parameter_library
  pluginlib
  geometry_msgs
)
foreach(Dependency IN ITEMS ${CONTROL_FILTERS_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

generate_parameter_library(
  low_pass_filter_parameters
  src/control_filters/low_pass_filter_parameters.yaml
)

add_library(low_pass_filter SHARED
  src/control_filters/low_pass_filter.cpp
)
target_compile_features(low_pass_filter PUBLIC cxx_std_17)
target_include_directories(low_pass_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/control_toolbox>
)
target_link_libraries(low_pass_filter PUBLIC
  low_pass_filter_parameters
  Eigen3::Eigen
)
ament_target_dependencies(low_pass_filter PUBLIC ${CONTROL_FILTERS_INCLUDE_DEPENDS})

generate_parameter_library(
  rate_limiter_parameters
  src/control_filters/rate_limiter_parameters.yaml
  include/control_filters/custom_validators.hpp
)

add_library(rate_limiter SHARED
  src/control_filters/rate_limiter.cpp
)
target_compile_features(rate_limiter PUBLIC cxx_std_17)
target_include_directories(rate_limiter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/control_toolbox>
)
target_link_libraries(rate_limiter PUBLIC
  rate_limiter_parameters
  Eigen3::Eigen
)
ament_target_dependencies(rate_limiter PUBLIC ${CONTROL_FILTERS_INCLUDE_DEPENDS})

generate_parameter_library(
  exponential_filter_parameters
  src/control_filters/exponential_filter_parameters.yaml
)

add_library(exponential_filter SHARED
  src/control_filters/exponential_filter.cpp
)
target_compile_features(exponential_filter PUBLIC cxx_std_17)
target_include_directories(exponential_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/control_toolbox>
)
target_link_libraries(exponential_filter PUBLIC
  exponential_filter_parameters
)
ament_target_dependencies(exponential_filter PUBLIC ${CONTROL_FILTERS_INCLUDE_DEPENDS})

# Install pluginlib xml
pluginlib_export_plugin_description_file(filters control_filters.xml)

##########
# Testing
##########
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  ament_add_gmock(pid_tests test/pid_tests.cpp)
  target_link_libraries(pid_tests control_toolbox)

  ament_add_gmock(rate_limiter_tests test/rate_limiter.cpp)
  target_link_libraries(rate_limiter_tests control_toolbox)

  ament_add_gtest(pid_parameters_tests test/pid_parameters_tests.cpp)
  target_link_libraries(pid_parameters_tests control_toolbox)

  ament_add_gtest(pid_publisher_tests test/pid_publisher_tests.cpp)
  target_link_libraries(pid_publisher_tests control_toolbox)
  ament_target_dependencies(pid_publisher_tests rclcpp_lifecycle)

  ## Control Filters
  # exponential_filter
  add_rostest_with_parameters_gmock(test_exponential_filter test/control_filters/test_exponential_filter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/control_filters/test_exponential_filter_parameters.yaml
  )
  target_link_libraries(test_exponential_filter exponential_filter exponential_filter_parameters)
  ament_target_dependencies(test_exponential_filter ${CONTROL_FILTERS_INCLUDE_DEPENDS})
  set_tests_properties(test_exponential_filter PROPERTIES TIMEOUT 2)

  ament_add_gmock(test_load_exponential_filter test/control_filters/test_load_exponential_filter.cpp)
  target_link_libraries(test_load_exponential_filter exponential_filter exponential_filter_parameters)
  ament_target_dependencies(test_load_exponential_filter ${CONTROL_FILTERS_INCLUDE_DEPENDS})

  # low_pass_filter
  add_rostest_with_parameters_gmock(test_low_pass_filter test/control_filters/test_low_pass_filter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/control_filters/test_low_pass_filter_parameters.yaml
  )
  target_link_libraries(test_low_pass_filter low_pass_filter low_pass_filter_parameters)
  ament_target_dependencies(test_low_pass_filter ${CONTROL_FILTERS_INCLUDE_DEPENDS})
  set_tests_properties(test_low_pass_filter PROPERTIES TIMEOUT 2)

  ament_add_gmock(test_load_low_pass_filter test/control_filters/test_load_low_pass_filter.cpp)
  target_link_libraries(test_load_low_pass_filter low_pass_filter low_pass_filter_parameters)
  ament_target_dependencies(test_load_low_pass_filter ${CONTROL_FILTERS_INCLUDE_DEPENDS})

  # rate_limiter
  add_rostest_with_parameters_gmock(test_rate_limiter test/control_filters/test_rate_limiter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/control_filters/test_rate_limiter_parameters.yaml
  )
  target_link_libraries(test_rate_limiter rate_limiter rate_limiter_parameters)
  ament_target_dependencies(test_rate_limiter ${CONTROL_FILTERS_INCLUDE_DEPENDS})
  set_tests_properties(test_rate_limiter PROPERTIES TIMEOUT 2)

  ament_add_gmock(test_load_rate_limiter test/control_filters/test_load_rate_limiter.cpp)
  target_link_libraries(test_load_rate_limiter rate_limiter rate_limiter_parameters)
  ament_target_dependencies(test_load_rate_limiter ${CONTROL_FILTERS_INCLUDE_DEPENDS})
endif()

# Install
install(
  DIRECTORY include/
  DESTINATION include/control_toolbox
)
install(TARGETS control_toolbox
  low_pass_filter low_pass_filter_parameters
  rate_limiter rate_limiter_parameters
  exponential_filter exponential_filter_parameters
  EXPORT export_control_toolbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_control_toolbox HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} ${CONTROL_FILTERS_INCLUDE_DEPENDS})
ament_package()
