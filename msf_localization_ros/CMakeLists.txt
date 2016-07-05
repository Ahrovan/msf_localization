cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME msf_localization_ros)
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
 


set(MSF_LOCALIZATION_ROS_SOURCE_DIR
	src/source) 
	
set(MSF_LOCALIZATION_ROS_INCLUDE_DIR
	src/include)

set(MSF_LOCALIZATION_ROS_SOURCE_FILES
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_interface.cpp

	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_sensor_interface.cpp
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_imu_sensor_interface.cpp
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_aruco_eye_interface.cpp
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_absolute_pose_sensor_interface.cpp

	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_input_interface.cpp
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_imu_input_interface.cpp
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_absolute_pose_input_interface.cpp

	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_robot_interface.cpp
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_free_model_robot_interface.cpp
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_imu_driven_robot_interface.cpp
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_absolute_pose_driven_robot_interface.cpp

	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/droneMsfLocalizationROSModule.cpp 

)
 
set(MSF_LOCALIZATION_ROS_HEADER_FILES
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_interface.h

	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_sensor_interface.h
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_imu_sensor_interface.h
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_aruco_eye_interface.h
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_absolute_pose_sensor_interface.h

	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_input_interface.h
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_imu_input_interface.h
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_absolute_pose_input_interface.h

	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_robot_interface.h
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_free_model_robot_interface.h
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_imu_driven_robot_interface.h
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_absolute_pose_driven_robot_interface.h

	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/droneMsfLocalizationROSModule.h
)




find_package(catkin REQUIRED
                COMPONENTS cmake_modules roscpp std_srvs std_msgs sensor_msgs geometry_msgs tf pugixml msf_localization_core msf_localization_ros_srvs aruco_eye_msgs)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen REQUIRED)


catkin_package(
    INCLUDE_DIRS ${MSF_LOCALIZATION_ROS_INCLUDE_DIR}
    LIBRARIES ${PROJECT_NAME}_lib
    DEPENDS Boost Eigen
    CATKIN_DEPENDS roscpp std_srvs std_msgs sensor_msgs geometry_msgs tf pugixml msf_localization_core msf_localization_ros_srvs aruco_eye_msgs
  )


include_directories(${MSF_LOCALIZATION_ROS_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME}_lib ${MSF_LOCALIZATION_ROS_SOURCE_FILES} ${MSF_LOCALIZATION_ROS_HEADER_FILES})
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${Eigen_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test ${MSF_LOCALIZATION_ROS_SOURCE_DIR}/test.cpp)
add_dependencies(${PROJECT_NAME}_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} ${MSF_LOCALIZATION_ROS_SOURCE_DIR}/droneMsfLocalizationROSModuleNode.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

