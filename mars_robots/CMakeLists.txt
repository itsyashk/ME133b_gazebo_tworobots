cmake_minimum_required(VERSION 3.8)
project(mars_robots)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF and related files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 