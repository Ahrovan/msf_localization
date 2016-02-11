cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME msf_localization_core)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11)
add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

 


#MSF_LOCALIZATION
set(MSF_LOCALIZATION_CORE_SOURCE_DIR
	src/source) 
	
set(MSF_LOCALIZATION_CORE_INCLUDE_DIR
	src/include)

set(MSF_LOCALIZATION_CORE_SOURCE_FILES
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/robot_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/free_model_robot_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_state_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_sensor_state_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/robot_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/free_model_robot_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_basics.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_sensor_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/sensor_measurement_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/imu_sensor_measurement_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/msf_storage_core.cpp
  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/state_estimation_core.cpp

  ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/msfLocalization.cpp
)
 
set(MSF_LOCALIZATION_CORE_HEADER_FILES
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/robot_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/free_model_robot_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/sensor_state_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/imu_sensor_state_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/robot_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/free_model_robot_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/sensor_basics.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/sensor_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/imu_sensor_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/sensor_measurement_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/imu_sensor_measurement_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msf_storage_core.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/state_estimation_core.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msfLocalization.h

  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/ring_buffer.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/time_stamp.h
  ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/stamped_ring_buffer.h
)




find_package(catkin REQUIRED
                COMPONENTS roscpp cmake_modules lib_pugixml)


find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen REQUIRED)


catkin_package(
        INCLUDE_DIRS ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME}
        DEPENDS Boost Eigen
        CATKIN_DEPENDS roscpp cmake_modules lib_pugixml
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


add_executable(${PROJECT_NAME}_test_buffer ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/test_ring_buffer.cpp)
add_dependencies(${PROJECT_NAME}_test_buffer ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_buffer ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_test_buffer ${catkin_LIBRARIES})
