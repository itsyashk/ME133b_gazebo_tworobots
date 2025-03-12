cmake_minimum_required(VERSION 3.8)
project(mars_world)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 