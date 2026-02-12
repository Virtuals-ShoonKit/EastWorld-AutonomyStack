cmake_minimum_required(VERSION 3.8)
project(eastworld_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files, configs, and systemd unit
install(DIRECTORY
  launch
  config
  scripts
  systemd
  DESTINATION share/${PROJECT_NAME}
)

# Install wrapper script as an executable
install(PROGRAMS
  scripts/start_autonomy_stack.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
