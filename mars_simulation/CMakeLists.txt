cmake_minimum_required(VERSION 3.8)
project(mars_simulation)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 