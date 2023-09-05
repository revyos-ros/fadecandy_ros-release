cmake_minimum_required(VERSION 3.5)
project(fadecandy_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
##  Build setup ##
##################

set(DEPENDENCIES
  diagnostic_updater
  fadecandy_msgs
  rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb libusb-1.0)

##############
##   Build  ##
##############

include_directories(
  ${libusb_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/util.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${libusb_LIBRARIES})

add_library(${PROJECT_NAME}_ros SHARED
  src/ros/${PROJECT_NAME}_ros.cpp
)
ament_target_dependencies(${PROJECT_NAME}_ros)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME})

add_executable(fadecandy_node src/ros/${PROJECT_NAME}_ros_node.cpp)
target_link_libraries(fadecandy_node ${PROJECT_NAME}_ros)

#############
## Install ##
#############

install(
  TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
  fadecandy_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  install(
    PROGRAMS test/example_client
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
