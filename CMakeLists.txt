include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
    set(CMAKE_C_FLAGS "-cflags libmodbus")

elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

cmake_minimum_required(VERSION 2.8.3)
project(leddarone C CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  roslaunch
)

roslaunch_add_file_check(launch)
generate_messages(
  DEPENDENCIES
std_msgs
  sensor_msgs

)
catkin_package(
  CATKIN_DEPENDS  roscpp std_msgs sensor_msgs  
  INCLUDE_DIRS src
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(LeddarOneRos src/LeddarOneRos.cpp)
target_link_libraries(LeddarOneRos  ${catkin_LIBRARIES})

#Theese are important, you might have to change this and put the path of your
#File although i believe this will suffice
#Din lib libmodbus.so

target_link_libraries(LeddarOneRos
                      libmodbus.so)
