cmake_minimum_required(VERSION 3.5)
project(mmi_keyboard_package)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(
publisher_node_1
"src/TestNode.cpp" 
)

# Add the include directory
# All files in the include/my_project/ directory will be included
target_include_directories(publisher_node_1 PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Set the path to the .lib file depending on the build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(SERIAL_LIB "${CMAKE_SOURCE_DIR}/lib/debug/serial.lib")
else()
    set(SERIAL_LIB "${CMAKE_SOURCE_DIR}/lib/release/serial.lib")
endif()

# Link the executable with the appropriate serial.lib file
target_link_libraries(publisher_node_1 ${SERIAL_LIB})


ament_target_dependencies(publisher_node_1 rclcpp std_msgs)

install(TARGETS
  publisher_node_1
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
