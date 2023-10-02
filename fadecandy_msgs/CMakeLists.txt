cmake_minimum_required(VERSION 3.5)
project(fadecandy_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(dependency IN ITEMS ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

set(msg_files
  "msg/LEDStrip.msg"
  "msg/LEDArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${DEPENDENCIES}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
