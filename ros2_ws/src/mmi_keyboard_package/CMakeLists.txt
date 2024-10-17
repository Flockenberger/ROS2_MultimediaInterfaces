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
mmi_project
# cpp source Files
 "src/Main.cpp"
 "src/PublisherNode.cpp" 
 "src/SerialNode.cpp" 
 "src/SerialInterface.cpp"

# h include Files
 "include/PublisherNode.h"
 "include/SerialNode.h" 
 "include/MMIPredefine.h"
 "include/SerialInterface.h" 
 )

# Add the include directory
# All files in the include/my_project/ directory will be included
target_include_directories(mmi_project PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_include_directories(mmi_project PRIVATE
    ${CMAKE_SOURCE_DIR}/vendor/include
)

# Set the path to the .lib file depending on the build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(SERIAL_LIB "${CMAKE_SOURCE_DIR}/vendor/lib/debug/serial.lib")
else()
    set(SERIAL_LIB "${CMAKE_SOURCE_DIR}/vendor/lib/release/serial.lib")
endif()

# Link the executable with the appropriate serial.lib file
target_link_libraries(mmi_project ${SERIAL_LIB})


ament_target_dependencies(mmi_project rclcpp std_msgs)

install(TARGETS
  mmi_project
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
