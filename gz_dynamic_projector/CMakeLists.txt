cmake_minimum_required(VERSION 3.5)
project(gz_dynamic_projector)
set(CMAKE_PREFIX_PATH "$HOME/.local" ${CMAKE_PREFIX_PATH})
# Find necessary Gazebo/ROS packages
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-sensors8 REQUIRED COMPONENTS depth_camera)
find_package(gz-common5 REQUIRED)  
find_package(gz-msgs10 REQUIRED)   

# Include directories
include_directories(include)

# Build the plugin library
add_library(DynamicProjector SHARED
  src/DynamicProjector.cpp
)

# Correct include dirs
target_include_directories(DynamicProjector PRIVATE
  include
  ${gz-common5_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
)

# Link against Gazebo libraries
target_link_libraries(DynamicProjector
  PRIVATE
    -Wl,--no-as-needed
    gz-plugin2::gz-plugin2
    gz-sim8::gz-sim8
    gz-rendering8::gz-rendering8
    gz-sensors8::depth_camera         # <-- key fix for DepthCameraSensor
    gz-common5::gz-common5
    gz-msgs10::gz-msgs10
)

# Install the plugin and resources
install(TARGETS DynamicProjector
  LIBRARY DESTINATION lib
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

target_include_directories(DynamicProjector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_package()
