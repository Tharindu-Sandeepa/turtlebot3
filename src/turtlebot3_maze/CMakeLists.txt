cmake_minimum_required(VERSION 3.8)
project(turtlebot3_maze)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Include Python files
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install models
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
