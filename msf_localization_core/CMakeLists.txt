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
        ${MSF_LOCALIZATION_CORE_SOURCE_DIR}/msfLocalization.cpp
)
 
set(MSF_LOCALIZATION_CORE_HEADER_FILES
        ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}/msfLocalization.h
)




find_package(catkin REQUIRED
		COMPONENTS lib_pugixml)


find_package(Boost REQUIRED COMPONENTS filesystem)


catkin_package(
        INCLUDE_DIRS ${MSF_LOCALIZATION_CORE_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME}
        DEPENDS Boost
        CATKIN_DEPENDS lib_pugixml
  )


include_directories(${MSF_LOCALIZATION_CORE_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME} ${MSF_LOCALIZATION_CORE_SOURCE_FILES} ${MSF_LOCALIZATION_CORE_HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

