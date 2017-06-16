cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME msf_localization_core)
project(${PROJECT_NAME})


# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    #set(CMAKE_CXX_FLAGS "-std=c++11")
    # http://stackoverflow.com/questions/37621342/cmake-will-not-compile-to-c-11-standard
    # CMake below 3.1
    add_definitions(-std=c++11)
    # CMake 3.1 and above
    set(CMAKE_CXX_STANDARD 11) # C++11...
    set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
    set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
elseif(COMPILER_SUPPORTS_CXX0X)
    #set(CMAKE_CXX_FLAGS "-std=c++0x")
    add_definitions(-std=c++0x)
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

 
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
#set(DCMAKE_BUILD_TYPE Release)


#MSF_LOCALIZATION
set(MSF_LOCALIZATION_CORE_SOURCE_DIR
	src/source) 
	
set(MSF_LOCALIZATION_CORE_INCLUDE_DIR
	src/include)

set(MSF_LOCALIZATION_CORE_SOURCE_FILES
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/msf_element_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/input_command_component.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_measurement_component.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/state_component.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/world_reference_frame_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/world_reference_frame_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/absolute_pose_sensor_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/absolute_pose_sensor_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/absolute_pose_sensor_measurement_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/input_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_input_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/input_command_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_input_command_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/robot_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/free_model_robot_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_driven_robot_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_sensor_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/input_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_input_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/absolute_pose_input_command_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/absolute_pose_input_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/absolute_pose_input_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/absolute_pose_driven_robot_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/absolute_pose_driven_robot_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/global_parameters_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/global_parameters_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/robot_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/free_model_robot_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_driven_robot_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/map_element_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/map_element_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_basics.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_sensor_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_measurement_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_sensor_measurement_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/coded_visual_marker_eye_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/coded_visual_marker_eye_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/coded_visual_marker_measurement_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/coded_visual_marker_landmark_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/coded_visual_marker_landmark_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/px4flow_sensor_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/px4flow_sensor_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/px4flow_sensor_measurement_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/msf_storage_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/state_estimation_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/msfLocalization.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/block_matrix.cpp
)
 
set(MSF_LOCALIZATION_CORE_HEADER_FILES
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/msf_element_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/input_command_component.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/sensor_measurement_component.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/state_component.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/world_reference_frame_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/world_reference_frame_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/absolute_pose_sensor_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/absolute_pose_sensor_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/absolute_pose_sensor_measurement_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/input_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/imu_input_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/input_command_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/imu_input_command_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/absolute_pose_input_command_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/absolute_pose_input_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/absolute_pose_input_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/absolute_pose_driven_robot_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/absolute_pose_driven_robot_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/robot_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/free_model_robot_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/imu_driven_robot_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/sensor_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/imu_sensor_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/input_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/imu_input_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/global_parameters_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/global_parameters_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/robot_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/free_model_robot_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/imu_driven_robot_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/map_element_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/map_element_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/sensor_basics.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/sensor_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/imu_sensor_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/sensor_measurement_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/imu_sensor_measurement_core.h


  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/coded_visual_marker_eye_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/coded_visual_marker_eye_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/coded_visual_marker_measurement_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/coded_visual_marker_landmark_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/coded_visual_marker_landmark_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/px4flow_sensor_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/px4flow_sensor_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/px4flow_sensor_measurement_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/msf_storage_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/state_estimation_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/msfLocalization.h



  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/block_matrix.h

  #${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/ring_buffer.h
  #${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/time_stamp.h
  #${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/stamped_ring_buffer.h
)




find_package(catkin REQUIRED
                COMPONENTS pugixml quaternion_algebra time_stamp buffer)


find_package(Boost REQUIRED
                COMPONENTS filesystem)


find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
  message(WARN "Using Eigen2!")
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

#add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)


find_package(Threads REQUIRED)



catkin_package(
        INCLUDE_DIRS ${MSF_LOCALIZATION_CORE_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES}
        DEPENDS Boost EIGEN3 Threads
        CATKIN_DEPENDS pugixml quaternion_algebra time_stamp buffer
  )


include_directories(${MSF_LOCALIZATION_CORE_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME} ${MSF_LOCALIZATION_CORE_SOURCE_FILES} ${MSF_LOCALIZATION_CORE_HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

