cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME msf_localization_core)
project(${PROJECT_NAME})


# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    #set(CMAKE_CXX_FLAGS "-std=c++11")
    add_definitions(-std=c++11)
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

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/msf_storage_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/state_estimation_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/msfLocalization.cpp


  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/block_matrix.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/quaternion_algebra.cpp
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

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/msf_storage_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/state_estimation_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/msfLocalization.h



  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/block_matrix.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/quaternion_algebra.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/ring_buffer.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/time_stamp.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_localization_core/stamped_ring_buffer.h
)




find_package(catkin REQUIRED
                COMPONENTS cmake_modules pugixml)


find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen REQUIRED)


catkin_package(
        INCLUDE_DIRS ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME}
        DEPENDS Boost Eigen
        CATKIN_DEPENDS pugixml
  )


include_directories(${MSF_LOCALIZATION_CORE_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME} ${MSF_LOCALIZATION_CORE_SOURCE_FILES} ${MSF_LOCALIZATION_CORE_HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Eigen_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

