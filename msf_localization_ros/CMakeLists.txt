cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME msf_localization_ros)
project(${PROJECT_NAME})

add_definitions(-std=c++11)


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
	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_sensor_interface.cpp

	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/ros_sensor_imu_interface.cpp

	${MSF_LOCALIZATION_ROS_SOURCE_DIR}/droneMsfLocalizationROSModule.cpp 

)
 
set(MSF_LOCALIZATION_ROS_HEADER_FILES
	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_sensor_interface.h

	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/ros_sensor_imu_interface.h

	${MSF_LOCALIZATION_ROS_INCLUDE_DIR}/msf_localization_ros/droneMsfLocalizationROSModule.h
)




find_package(catkin REQUIRED
                COMPONENTS roscpp std_srvs std_msgs sensor_msgs geometry_msgs tf pugixml msf_localization_core msf_localization_ros_srvs)

find_package(Boost REQUIRED COMPONENTS filesystem)


catkin_package(
    DEPENDS Boost
    CATKIN_DEPENDS roscpp std_srvs std_msgs sensor_msgs geometry_msgs tf pugixml msf_localization_core msf_localization_ros_srvs
  )


include_directories(${MSF_LOCALIZATION_ROS_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME}_lib ${MSF_LOCALIZATION_ROS_SOURCE_FILES} ${MSF_LOCALIZATION_ROS_HEADER_FILES})
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME} ${MSF_LOCALIZATION_ROS_SOURCE_DIR}/droneMsfLocalizationROSModuleNode.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

