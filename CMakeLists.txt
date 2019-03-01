cmake_minimum_required(VERSION 2.8.3)
project(control_toolbox)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  control_msgs
  dynamic_reconfigure
  message_generation
  realtime_tools
  roscpp
  std_msgs
  )

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(TinyXML REQUIRED)

# Dynamics reconfigure
generate_dynamic_reconfigure_options(
  cfg/Parameters.cfg
  )

# Add services and generate them
add_service_files(
  FILES
  SetPidGains.srv
  )

generate_messages(
  DEPENDENCIES std_msgs
)

# Declare catkin package
catkin_package(
  DEPENDS TinyXML
  CATKIN_DEPENDS
    control_msgs
    dynamic_reconfigure
    message_runtime
    realtime_tools
    roscpp
    std_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${TinyXML_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/dither.cpp
  src/limited_proxy.cpp
  src/pid.cpp
  src/pid_gains_setter.cpp
  src/sine_sweep.cpp
  src/sinusoid.cpp
  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # Tests
  catkin_add_gtest(pid_tests test/pid_tests.cpp)
  target_link_libraries(pid_tests ${catkin_LIBRARIES} ${PROJECT_NAME})

  # add_executable(test_linear test/linear.cpp)
endif()

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
