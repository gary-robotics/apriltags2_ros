cmake_minimum_required(VERSION 2.8.3)
project(apriltags2_ros)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -O2 -DNDEBUG")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  tf
  cmake_modules
  pnc_msgs
  center_msgs
  localization_msgs
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

add_definitions (-Wall -O3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funsafe-loop-optimizations -fsee -funroll-loops -fno-math-errno -funsafe-math-optimizations -ffinite-math-only -fno-signed-zeros ")

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
  message("enabling msse2 for x86_64 or x86_32 architecture")
  #  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse2 ")
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm)
  message("enabling -mfpu=neon -mfloat-abi=softfp for ARM architecture")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mcpu=cortex-a9 -mfpu=neon -mtune=cortex-a9 -mvectorize-with-neon-quad -ffast-math ")
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm)

add_message_files(
  FILES
  AprilTagDetectionArray.msg
  AprilTagDetection.msg
  AprilTagPosition.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS apriltags2 geometry_msgs image_transport roscpp sensor_msgs std_msgs message_runtime cv_bridge tf
  DEPENDS Eigen OpenCV ceres
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

find_library(
        GTEST_LIBRARY
        gtest
        PATHS ./lib
    )

find_library(
      TEST_UTIL_LIBRARY
      test_util
      PATHS ./lib
  )

find_library(
      APRILTAGS2_LIBRARY
      apriltags2
      PATHS ./lib
  )    


add_library(ippe src/ippe.cpp)
add_dependencies(ippe ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ippe
        ${catkin_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
        )


add_library(common src/common_functions.cpp)
add_dependencies(common ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(common
  ippe
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GTEST_LIBRARY}
  ${TEST_UTIL_LIBRARY}
  ${APRILTAGS2_LIBRARY}
  #glog
  #cxsparse
  #cholmod
  #gomp
)

add_library(continuous_detector src/continuous_detector.cpp)
add_dependencies(continuous_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(continuous_detector
  common
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_continuous_node src/${PROJECT_NAME}_continuous_node.cpp)
add_dependencies(${PROJECT_NAME}_continuous_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_continuous_node
  continuous_detector
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ippe common continuous_detector ${PROJECT_NAME}_continuous_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY lib/
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
