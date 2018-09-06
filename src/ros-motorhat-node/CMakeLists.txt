cmake_minimum_required(VERSION 2.8.3)
project(motor_hat)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motor_hat
#  CATKIN_DEPENDS roscpp
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
)

## Declare a C++ library
# add_library(motor_hat
#   src/${PROJECT_NAME}/motor_hat.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(motor_hat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(motor_hat_node src/motor_hat_node.cpp)
add_executable(encoder_node src/rotaryEncoder.cpp)
add_executable(temp_humi_node src/rht03.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(motor_hat_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(motor_hat_node
  ${catkin_LIBRARIES}
  wiringPi
)

target_link_libraries(encoder_node
  ${catkin_LIBRARIES}
  wiringPi
)

target_link_libraries(temp_humi_node
  ${catkin_LIBRARIES}
  wiringPi
  wiringPiDev
  crypt
  pthread
  m
  rt
)


