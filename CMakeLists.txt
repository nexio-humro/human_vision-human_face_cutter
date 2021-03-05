cmake_minimum_required(VERSION 3.0.2)
project(human_face_cutter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#############################################
# Human_localization adding libraries START #
#############################################

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*.c*)
FILE(GLOB_RECURSE TEST_SRC_FILES test_src/*.c*)
FILE(GLOB_RECURSE HDR_FILES include/*.h*)

###########################################
# Human_localization adding libraries END #
###########################################

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  cv_bridge
  human_vision_exchange
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS cv_bridge human_vision_exchange roscpp roslib std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

############################
# List INCLUDE_DIRECTORIES # 
############################

#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message(STATUS "dir='${dir}'")
#endforeach()

##################################
# Human_localization build START #
##################################

add_executable(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES} main_src/main.cpp)
#add_executable(${PROJECT_NAME}_tests ${HDR_FILES} ${SRC_FILES} ${TEST_SRC_FILES} )
add_executable(${PROJECT_NAME}_cutFaceMock ${HDR_FILES} mocks/cutFaceMock.cpp)
add_definitions(-std=c++14)

target_link_libraries(${PROJECT_NAME}
                        ${SPECIAL_OS_LIBS}
                        ${OpenCV_LIBRARIES}
                        ${catkin_LIBRARIES})

#target_link_libraries(${PROJECT_NAME}_tests
#                        ${SPECIAL_OS_LIBS}
#                        ${ZED_LIBS}
#                        ${OpenCV_LIBRARIES}
#                        ${catkin_LIBRARIES}
#                        dlib::dlib)

target_link_libraries(${PROJECT_NAME}_cutFaceMock
                        ${SPECIAL_OS_LIBS}
                        ${OpenCV_LIBRARIES}
                        ${catkin_LIBRARIES})

################################
# Human_localization build END #
################################
