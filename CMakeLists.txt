cmake_minimum_required(VERSION 2.8.3)
project(control_toolbox)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  #rosbuild_genmsg()
  rosbuild_gensrv()

  #dynamic reconfigure
  rosbuild_find_ros_package(dynamic_reconfigure)
  include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
  gencfg()

  rosbuild_add_library(${PROJECT_NAME}
    src/pid.cpp
    src/pid_gains_setter.cpp
    src/sine_sweep.cpp
    src/dither.cpp
    src/sinusoid.cpp
    src/limited_proxy.cpp
    )
  target_link_libraries(${PROJECT_NAME} tinyxml)

  # rosbuild_add_executable(test_linear test/linear.cpp)

  # Tests
  rosbuild_add_gtest(test/pid_tests test/pid_tests.cpp)
  target_link_libraries(test/pid_tests ${PROJECT_NAME})

else()

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS
    rosconsole
    message_generation
    tf
    roscpp
    angles
    dynamic_reconfigure
    realtime_tools
    )

  find_package(Boost REQUIRED COMPONENTS system thread)

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    )

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
    DEPENDS tinyxml
    CATKIN_DEPENDS 
      rosconsole 
      tf 
      roscpp 
      angles 
      dynamic_reconfigure
      realtime_tools
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    )

  add_library(${PROJECT_NAME}
    src/pid.cpp
    src/pid_gains_setter.cpp
    src/sine_sweep.cpp
    src/dither.cpp
    src/sinusoid.cpp
    src/limited_proxy.cpp
    )
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg) # wait for dynamic reconfigure

  target_link_libraries(${PROJECT_NAME} tinyxml ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp) # wait for msgs

  if(CATKIN_ENABLE_TESTING)
    # Tests
    catkin_add_gtest(pid_tests test/pid_tests.cpp)
    target_link_libraries(pid_tests ${catkin_LIBRARIES} ${PROJECT_NAME})

    # add_executable(test_linear test/linear.cpp)
  endif()

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()
