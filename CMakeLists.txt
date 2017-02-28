cmake_minimum_required(VERSION 2.8.3)
project(velodyne_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  rosbag  

)

set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)

find_package(Qt5Core)
find_package (Qt5Widgets)
find_package (Qt5Gui)
find_package (Qt5OpenGL)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(PCL REQUIRED)

set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set (Velodyne_Detector_QTLib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/glwidget.cpp ${SRC_DIR}/ROSThread.cpp)
set (Velodyne_Detector_QTLib_hdr ${SRC_DIR}/mainwindow.h ${SRC_DIR}/glwidget.h ${SRC_DIR}/ROSThread.h)
set (Velodyne_Detector_QTLib_ui  ${SRC_DIR}/mainwindow.ui)
set (Velodyne_Detector_QTBin_src ${SRC_DIR}/main.cpp)

qt5_wrap_cpp(Velodyne_Detector_QTLib_hdr_moc ${Velodyne_Detector_QTLib_hdr})
qt5_wrap_ui (Velodyne_Detector_QTLib_ui_moc  ${Velodyne_Detector_QTLib_ui})
qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)

set (CMAKE_C_FLAGS "-u MAIN__ -std=gnu99 -fopenmp ${CMAKE_C_FLAGS}")
set (CMAKE_CXX_FLAGS "-u MAIN__ -std=gnu++11 -fopenmp ${CMAKE_CXX_FLAGS}")

include_directories (${PROJECT_BINARY_DIR})
include_directories(${PCL_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



#catkin_python_setup() #needed for qt setup.py to install plugin


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  test.msg
  VelodynePacket.msg
  VelodyneScan.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInits.srv  
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES velodyne_detector
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SRC_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
)

#add_definitions(${Qt5Widgets_DEFINITIONS})

## Declare a C++ library

add_library(Velodyne_Detector_QTLib SHARED

 ${Velodyne_Detector_QTLib_src}
 ${Velodyne_Detector_QTLib_hdr_moc}
 ${Velodyne_Detector_QTLib_ui_moc}
)

target_link_libraries (Velodyne_Detector_QTLib Qt5::Widgets Qt5::Gui Qt5::OpenGL)

## Specify libraries to link a library or executable target against

add_executable(velodyne_detector ${Velodyne_Detector_QTLib_src} ${Velodyne_Detector_QTLib_hdr} ${Velodyne_Detector_QTBin_src} ${SHADER_RSC_ADDED})     


target_link_libraries(velodyne_detector

  ${catkin_LIBRARIES}
  ${QT_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${GLEW_LIBRARIES}
  Velodyne_Detector_QTLib
  GL
  ${BOOST_CUSTOM_LIBS}
  ${PCL_LIBRARIES} 
  

)



